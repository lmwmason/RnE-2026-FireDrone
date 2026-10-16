#!/usr/bin/env python3
"""Fetch pinned upstream and install the reviewable local patch; no hardware IO."""
from pathlib import Path
import subprocess, shutil, json
ROOT=Path(__file__).resolve().parents[1]
REV="23d066d0824ecdb2cd0986a31eb6603a89d15a3e"
def run(*args,**kw):return subprocess.run(args,check=True,**kw)
def main():
    b=ROOT/"betaflight"
    if not b.exists():run("git","clone","--depth","1","--branch","4.4.2","https://github.com/betaflight/betaflight.git",str(b))
    actual=subprocess.check_output(["git","-C",str(b),"rev-parse","HEAD"],text=True).strip()
    if actual!=REV:raise SystemExit("Unexpected upstream revision; refusing to overwrite your checkout")
    patch=ROOT/"patches/0001-firedrone-betaflight.patch"
    check=subprocess.run(["git","-C",str(b),"apply","--reverse","--check",str(patch)],capture_output=True)
    if check.returncode:
        run("git","-C",str(b),"apply","--check",str(patch))
        run("git","-C",str(b),"apply",str(patch))
    # research/ is the canonical source. Do not edit its generated copies under betaflight/.
    shutil.copytree(ROOT/"research",b/"src/main/research",dirs_exist_ok=True)
    shutil.copy2(ROOT/"research/firedrone_pg.c",b/"src/main/pg/firedrone.c")
    (b/"src/main/research/firedrone_pg.c").unlink()
    full=(ROOT/"configs/full-defaults.txt").read_text()
    (b/"src/main/config/captured_defaults.c").write_text('#pragma GCC diagnostic ignored "-Woverlength-strings"\n__attribute__((used,section(".captured_defaults")))\nconst char capturedDefaults[] =\n'+'\n'.join(json.dumps(line+'\n') for line in full.splitlines())+';\n')
    print("Pinned source + research patch prepared:",actual)
if __name__=="__main__":main()
