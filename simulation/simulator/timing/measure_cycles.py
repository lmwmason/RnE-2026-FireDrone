#!/usr/bin/env python3
"""Run the real flight stages on an emulated Cortex-M4F and report the cycle budget.

Needs:  pip install unicorn capstone   and   arm-none-eabi-gcc  (make -C timing)
The image is built from the firmware sources, so this is the actual machine code
(-Og by default, the same optimisation level as the firmware Makefile).
"""
import argparse
import json
import os
import re
import subprocess
import sys

try:
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UcError
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC
    import capstone
except ImportError:
    print('unicorn / capstone not installed:  pip install unicorn capstone')
    sys.exit(2)

HERE     = os.path.dirname(os.path.abspath(__file__))
FLASH    = 0x08000000
RAM      = 0x20000000
RAM_SIZE = 128 * 1024
CPU_HZ   = 168_000_000
TICK_CYC = CPU_HZ // 1000           # 1 kHz control tick

# Cortex-M4 TRM instruction timing (core only; flash wait states are covered by the ART cache)
def cycles_of(insn):
    m = insn.mnemonic
    ops = insn.op_str
    n_regs = ops.count(',') + 1 if '{' in ops else 0
    if m.startswith(('vdiv', 'vsqrt')):
        return 14
    if m.startswith(('vmla', 'vmls', 'vnmla', 'vnmls', 'vfma', 'vfms', 'vfnma', 'vfnms')):
        return 3
    if m.startswith(('vldr', 'vstr')):
        return 2
    if m.startswith(('vldm', 'vstm', 'vpop', 'vpush')):
        return 1 + n_regs
    if m.startswith(('push', 'pop', 'ldm', 'stm')):
        return 1 + max(n_regs, 1)
    if m.startswith(('ldr', 'str')):
        return 2
    if m.startswith(('sdiv', 'udiv')):
        return 6
    return 1


BRANCH_PENALTY = 2            # pipeline refill when a branch is taken (TRM: P = 1..3)


def is_branch(insn):
    m = insn.mnemonic
    if m.startswith(('bic', 'bfi', 'bfc', 'bkpt')):
        return False
    if m == 'pop':
        return 'pc' in insn.op_str
    return m.startswith(('b', 'cbz', 'cbnz', 'tbb', 'tbh'))


def symbols(elf):
    out = subprocess.run(['arm-none-eabi-nm', elf], capture_output=True, text=True).stdout
    return {m.group(2): int(m.group(1), 16) for m in re.finditer(r'^([0-9a-f]+) [TtDdBb] (\S+)$', out, re.M)}


class Cpu:
    def __init__(self, image, syms):
        self.syms = syms
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.mem_map(FLASH, 1024 * 1024)
        self.uc.mem_map(RAM, RAM_SIZE)
        self.uc.mem_map(0xE0000000, 0x100000)
        self.uc.mem_write(0xE000ED88, (0xF << 20).to_bytes(4, 'little'))     # CPACR: enable FPU
        self.uc.mem_write(FLASH, image)
        self.cs = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB | capstone.CS_MODE_MCLASS)
        self.cost = {}
        self.prev = None
        self.cycles = 0
        self.instrs = 0
        self.ret = syms['bench_ret']
        self.uc.hook_add(UC_HOOK_CODE, self._hook)

    def _hook(self, uc, addr, size, _):
        if self.prev is not None and self.prev[2] and addr != self.prev[0] + self.prev[1]:
            self.cycles += BRANCH_PENALTY                 # previous branch was taken
        c = self.cost.get(addr)
        if c is None:
            insn = next(self.cs.disasm(bytes(uc.mem_read(addr, size)), addr))
            c = (cycles_of(insn), is_branch(insn))
            self.cost[addr] = c
        self.cycles += c[0]
        self.instrs += 1
        self.prev = (addr, size, c[1])

    def call(self, name):
        self.cycles = self.instrs = 0
        self.prev = None
        self.uc.reg_write(UC_ARM_REG_SP, RAM + RAM_SIZE)
        self.uc.reg_write(UC_ARM_REG_LR, self.ret | 1)
        self.uc.emu_start(self.syms[name] | 1, self.ret, count=5_000_000)
        return self.cycles, self.instrs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--elf', default=os.path.join(HERE, 'fw_bench.elf'))
    ap.add_argument('--iters', type=int, default=200)
    ap.add_argument('--json', help='write the per-tick cost here (consumed by the SITL)')
    args = ap.parse_args()

    if not os.path.exists(args.elf):
        print(f'{args.elf} missing - run:  make -C timing')
        sys.exit(1)
    image = open(args.elf.replace('.elf', '.bin'), 'rb').read()
    syms = symbols(args.elf)
    cpu = Cpu(image, syms)

    try:
        cpu.call('bench_boot')
        cpu.call('bench_init')
    except UcError as e:
        print('emulation error during init:', e)
        sys.exit(1)

    stages = [('ukf_predict', 1.0), ('ukf_update', 0.5), ('indi_update', 1.0), ('adrc_update', 1.0),
              ('mrac_update', 1.0), ('fault_update', 1.0), ('cam_solve', 0.0)]
    names = {'ukf_predict': 'bench_ukf_predict', 'ukf_update': 'bench_ukf_update',
             'indi_update': 'bench_indi', 'adrc_update': 'bench_adrc', 'mrac_update': 'bench_mrac',
             'fault_update': 'bench_fault', 'cam_solve': 'bench_cam'}

    print(f'{"stage":14s} {"instr":>9s} {"cycles":>9s} {"us @168MHz":>11s}  calls/tick')
    cost = {}
    for stage, rate in stages:
        runs = [cpu.call(names[stage]) for _ in range(args.iters if stage != 'cam_solve' else 3)]
        c = max(x[0] for x in runs); n = max(x[1] for x in runs)
        cost[stage] = c
        print(f'{stage:14s} {n:9d} {c:9d} {c / CPU_HZ * 1e6:11.1f}  {rate}')

    base = sum(cost[k] for k in ('ukf_predict', 'indi_update', 'adrc_update', 'mrac_update', 'fault_update'))
    even = base                               # tick without the UKF measurement update
    odd  = base + cost['ukf_update']          # every second tick also runs ukf_update
    avg  = 0.5 * (even + odd)
    print()
    print(f'even tick           : {even:9d} cycles = {even / TICK_CYC * 100:6.1f} % of the {TICK_CYC} cycle budget')
    print(f'odd tick (+update)  : {odd:9d} cycles = {odd / TICK_CYC * 100:6.1f} %   <- worst case')
    print(f'average             : {avg:9.0f} cycles = {avg / TICK_CYC * 100:6.1f} %')
    print('(1 kHz task only: baro / GPS / CRSF / SPI transfer time is not included)')

    if args.json:
        with open(args.json, 'w') as f:
            json.dump({'even_us': even / CPU_HZ * 1e6, 'odd_us': odd / CPU_HZ * 1e6,
                       'even_pct': even / TICK_CYC * 100, 'odd_pct': odd / TICK_CYC * 100}, f, indent=2)


if __name__ == '__main__':
    main()
