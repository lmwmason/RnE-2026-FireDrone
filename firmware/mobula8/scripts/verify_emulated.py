#!/usr/bin/env python3
"""Execute candidate ARM CLI in Unicorn; no serial/hardware access.
Peripheral/runtime reinitializers are stubbed; configuration parser and storage
execute the actual candidate ELF. This is not a flight/hardware simulation.
"""
from pathlib import Path
import struct,json
from elftools.elf.elffile import ELFFile
from unicorn import Uc,UC_ARCH_ARM,UC_MODE_THUMB,UC_MODE_MCLASS,UC_HOOK_CODE
from unicorn.arm_const import *
R=Path(__file__).resolve().parents[1]
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
for addr,size in [(0x08000000,0x100000),(0x20000000,0x20000),(0x10000000,0x10000),(0x1fff0000,0x10000),(0xe0000000,0x100000),(0x40000000,0x30000),(0x30000000,0x10000)]:u.mem_map(addr,size)
u.mem_write(0xe000ed88,struct.pack('<I',0xf<<20))
with (R/'betaflight/obj/main/betaflight_STM32F405.elf').open('rb') as f:
 e=ELFFile(f)
 sy={s.name:s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
 for seg in e.iter_segments():
  if seg['p_type']=='PT_LOAD' and seg['p_filesz']:
   u.mem_write(seg['p_vaddr'],seg.data())
   if seg['p_paddr']!=seg['p_vaddr']:u.mem_write(seg['p_paddr'],seg.data())
def sym(n):
 if n in sy:return sy[n]
 return next(v for k,v in sy.items() if k.startswith(n+'.'))
def cstr(a):
 out=bytearray()
 while a and len(out)<1000:
  c=u.mem_read(a,1)[0]
  if not c:break
  out.append(c);a+=1
 return out.decode(errors='replace')
output=bytearray();errors=[]
def ret(uc):uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
def hook(n,fn):
 try:a=sym(n)&~1
 except StopIteration:return
 u.hook_add(UC_HOOK_CODE,lambda uc,ad,sz,data:fn(uc),begin=a,end=a)
hook('bufWriterAppend',lambda uc:(output.append(uc.reg_read(UC_ARM_REG_R1)&255),ret(uc)))
for n in ['bufWriterFlush','pidInit','initEscEndpoints','mixerInitProfile','initRcProcessing','beeperConfirmationBeeps','delay','schedulerIgnoreTaskExecTime','pinioBoxTaskControl']:
 hook(n,ret)
hook('cliPrintErrorVa',lambda uc:errors.append({'command':cstr(uc.reg_read(UC_ARM_REG_R0)),'format':cstr(uc.reg_read(UC_ARM_REG_R1))}))
STOP=0x30000000
u.mem_write(STOP,b'\x00\xbe')
def call(n,a=0,b=0):
 u.reg_write(UC_ARM_REG_SP,0x1000fff0);u.reg_write(UC_ARM_REG_LR,STOP|1)
 u.reg_write(UC_ARM_REG_R0,a);u.reg_write(UC_ARM_REG_R1,b)
 try:u.emu_start(sym(n)|1,STOP,count=100000000)
 except Exception:
  print('failed',n,hex(u.reg_read(UC_ARM_REG_PC)),errors[-8:]);raise
 if u.reg_read(UC_ARM_REG_PC)!=STOP:raise RuntimeError('instruction budget exhausted '+n)
def writer():
 for n in ['cliWriter','cliErrorWriter']:u.mem_write(sym(n),struct.pack('<I',0x30001000))
u.mem_write(0x30002000,b'defaults\0');u.mem_write(0x30002100,b'nosave\0')
writer();call('cliDefaults',0x30002000,0x30002100)
print('Import errors:',errors)
call('validateAndFixConfig')
# Populate runtime availability for every compiled port; parser/storage are unchanged.
for i,identifier in enumerate(u.mem_read(sym('serialPortIdentifiers'),9)):
 u.mem_write(sym('serialPortUsageList')+i*12+8,bytes([identifier]))
writer();output.clear()
u.mem_write(0x30002000,b'dump\0');u.mem_write(0x30002100,b'all\0')
call('cliDump',0x30002000,0x30002100)
(R/'build/emulated-dump-all.txt').write_bytes(output)
(R/'build/emulated-errors.json').write_text(json.dumps(errors,indent=2))
print('Dump bytes',len(output))
if errors:raise SystemExit(1)
