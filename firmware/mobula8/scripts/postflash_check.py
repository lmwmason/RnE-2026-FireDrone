#!/usr/bin/env python3
"""Read and compare the flashed aircraft. Does not change settings or drive motors."""
from pathlib import Path
import sys,json,re,time
import serial
from verify_snapshot import state
R=Path(__file__).resolve().parents[1]
d=R/'backups/postflash';d.mkdir(exist_ok=True)
with serial.Serial('/dev/ttyACM0',115200,timeout=.2) as s:
 def response():
  out=bytearray();end=time.monotonic()+20
  while time.monotonic()<end:
   out.extend(s.read(max(1,s.in_waiting)))
   if out.endswith(b'\n# '):
    more=s.read(1)
    if not more:return out.decode().replace('\r\n','\n')
    out.extend(more)
  raise RuntimeError('CLI timeout')
 while s.read(4096):pass
 s.write(b'\nversion\n');time.sleep(.3)
 initial=s.read(s.in_waiting)
 if b'Betaflight /' not in initial:
  s.write(b'#');response()
 for cmd in ['version','status','dump all']:
  s.write((cmd+'\n').encode());out=response();(d/(cmd.replace(' ','-')+'.txt')).write_text(out)
  if cmd!='dump all':print(out)
a=state((R/'configs/stock-dump-all.txt').read_text());b=state((d/'dump-all.txt').read_text())
expected=dict(a);expected['master:set blackbox_device']='set blackbox_device = NONE'
diff={k:{'expected':v,'actual':b.get(k)} for k,v in expected.items() if b.get(k)!=v}
for key in ['fd_mode','fd_blend','fd_ukf']:
 if b.get('master:set '+key)!='set '+key+' = 0':diff[key]=b.get('master:set '+key)
report={'settings_match_except_requested_logging_off':not diff,'mismatches':diff,
 'flashed':True,'blackbox_device':'NONE','research_control':'OFF','ukf':'OFF',
 'motors_tested':False,'receiver_inputs_tested':False,'flight_tested':False,
 'active_pid_profile':b.get('active:profile'),'active_rate_profile':b.get('active:rateprofile')}
(R/'dist/postflash-verification.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({**report,'mismatches':dict(list(diff.items())[:8])},indent=2))
if diff:raise SystemExit(1)
