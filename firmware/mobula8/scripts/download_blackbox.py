#!/usr/bin/env python3
"""Read-only MSPv2 Blackbox download. Never erases, arms, or changes settings."""
import serial,struct,time,json,hashlib,argparse
from pathlib import Path
def crc8(data):
 crc=0
 for v in data:
  crc^=v
  for _ in range(8):crc=((crc<<1)^0xd5 if crc&128 else crc<<1)&255
 return crc
class MSP:
 def __init__(self,port):self.s=serial.Serial(port,115200,timeout=.2)
 def exact(self,n):
  out=bytearray();end=time.monotonic()+10
  while len(out)<n and time.monotonic()<end:out.extend(self.s.read(n-len(out)))
  if len(out)!=n:raise TimeoutError('MSP reply truncated')
  return bytes(out)
 def get(self,cmd,payload=b''):
  h=struct.pack('<BHH',0,cmd,len(payload));self.s.write(b'$X<'+h+payload+bytes([crc8(h+payload)]))
  prefix=self.exact(3)
  if prefix!=b'$X>':raise RuntimeError('Unexpected reply '+repr(prefix))
  h=self.exact(5);flags,c,n=struct.unpack('<BHH',h);data=self.exact(n);crc=self.exact(1)[0]
  if c!=cmd or crc8(h+data)!=crc:raise RuntimeError('MSP checksum/command mismatch')
  return data
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--port',default='/dev/ttyACM0');ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 a.out.mkdir(parents=True,exist_ok=False);m=MSP(a.port)
 st=m.get(101);assert not struct.unpack_from('<I',st,6)[0]&1,'Aircraft is armed'
 flags,sectors,total,used=struct.unpack('<BIII',m.get(70));assert flags&1 and flags&2 and used<=total
 print('Flash bytes:',used,'/',total,flush=True)
 info={'flags':flags,'total_bytes':total,'used_bytes':used,'armed':False,'downloaded_utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}
 (a.out/'flash-summary.json').write_text(json.dumps(info,indent=2)+'\n')
 pos=0;next_progress=0
 with (a.out/'flight.bbl').open('xb') as f:
  while pos<used:
   n=min(4096,used-pos);reply=m.get(71,struct.pack('<IHB',pos,n,0));address,length,compression=struct.unpack_from('<IHB',reply)
   assert address==pos and compression==0 and length==len(reply)-7 and 0<length<=n
   f.write(reply[7:]);pos+=length
   if pos>=next_progress:print('Read',pos,'/',used,flush=True);next_progress=pos+1048576
 raw=(a.out/'flight.bbl').read_bytes();info['sha256']=hashlib.sha256(raw).hexdigest();info['log_headers']=raw.count(b'H Product:Blackbox')
 (a.out/'flash-summary.json').write_text(json.dumps(info,indent=2)+'\n');m.s.close();print(info)
