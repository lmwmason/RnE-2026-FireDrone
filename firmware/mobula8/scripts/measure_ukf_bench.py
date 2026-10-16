#!/usr/bin/env python3
"""Read-only disarmed UKF MSP sampling; debug mode must be set separately."""
import argparse,time,json,struct
from pathlib import Path
from download_blackbox import MSP
p=argparse.ArgumentParser();p.add_argument("--seconds",type=float,default=10);p.add_argument("--out",type=Path,required=True);a=p.parse_args()
m=None
for attempt in range(15):
    try:
        m=MSP("/dev/ttyACM0");m.s.reset_input_buffer();m.get(101);break
    except (OSError,RuntimeError,TimeoutError):
        if m:m.s.close()
        time.sleep(1)
else:raise RuntimeError("MSP unavailable after reboot")
rows=[];start=time.monotonic()
try:
    while time.monotonic()-start<a.seconds:
        status=m.get(101);cycle,i2c,sensors,flags,profile=struct.unpack_from("<HHHIB",status)
        if flags&1:raise RuntimeError("Aircraft armed; bench sampling stopped")
        debug=struct.unpack("<hhhh",m.get(254));att=struct.unpack("<hhh",m.get(108))
        rows.append(dict(t=time.monotonic()-start,cycle_us=cycle,cpu=struct.unpack_from("<H",status,11)[0],debug=debug,attitude=att))
        time.sleep(.04)
finally:
    m.s.close();a.out.parent.mkdir(parents=True,exist_ok=True);a.out.write_text(json.dumps(rows,indent=2)+"\n")
print(json.dumps(dict(samples=len(rows),cycle_max=max(r["cycle_us"] for r in rows),cpu_max=max(r["cpu"] for r in rows),last=rows[-1]),indent=2))
