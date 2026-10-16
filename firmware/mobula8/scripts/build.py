#!/usr/bin/env python3
"""Build a candidate HEX and record source/config/toolchain/artifact hashes."""
from pathlib import Path
import hashlib,json,subprocess,sys,datetime,shutil
import prepare
R=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    (R/"dist/config-verification.json").unlink(missing_ok=True)
    prepare.main();b=R/"betaflight"
    sdk=R/"tools/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gcc"
    if not sdk.exists():raise SystemExit("먼저 betaflight에서 make arm_sdk_install을 실행하세요.")
    target="obj/betaflight_4.4.2_STM32F405.hex"
    (R/"logs").mkdir(exist_ok=True);(R/"dist").mkdir(exist_ok=True)
    with (R/"logs/build.log").open("w") as f:
        p=subprocess.run(["make","TARGET=STM32F405","ARM_SDK_DIR=../tools/gcc-arm-none-eabi-10.3-2021.10","-j8","./"+target],cwd=b,stdout=f,stderr=subprocess.STDOUT)
    if p.returncode:
        print((R/"logs/build.log").read_text()[-6000:]);raise SystemExit(p.returncode)
    dest=R/"dist/FireDrone_Mobula8_STM32F405_BF4.4.2_CANDIDATE.hex"
    shutil.copy2(b/target,dest)
    sources={str(p.relative_to(R)):sha(p) for folder in ("research","patches","configs","scripts","cli") for p in sorted((R/folder).rglob("*")) if p.is_file() and "__pycache__" not in p.parts}
    manifest=dict(status="BUILT_NOT_HARDWARE_VALIDATED",board="BETAFLIGHTF4",manufacturer="FPVM",mcu="STM32F405",betaflight=prepare.REV,
        config_base=sha(R/"configs/stock-dump-all.txt"),config_changes="Full captured FC settings embedded; research OFF",
        compiler=subprocess.check_output([str(sdk),"--version"],text=True).splitlines()[0],
        utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),artifact=dest.name,sha256=sha(dest),sources=sources,
        defaults=dict(fd_mode=0,fd_blend=0,fd_ukf=0,effectiveness="unidentified; all zero"),
        hardware_connected=False,flashed=False)
    (R/"dist/manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print((R/"logs/build.log").read_text()[-1600:]);print(dest)
    emulator=R/".venv/bin/python"
    if emulator.exists():
        (R/"build").mkdir(exist_ok=True)
        run=subprocess.run
        run([str(emulator),str(R/"scripts/verify_emulated.py")],check=True)
        run([sys.executable,str(R/"scripts/verify_snapshot.py")],check=True)
    else:
        print("Built only: install verification dependencies and run verify_emulated.py / verify_snapshot.py.")
if __name__=="__main__":main()
