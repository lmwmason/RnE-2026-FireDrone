#!/usr/bin/env python3
"""Compare every captured FC setting against actual ARM CLI round-trip output."""
from pathlib import Path
import hashlib,json,collections
R=Path(__file__).resolve().parents[1]
def commands(text):
 return [s for line in text.splitlines() if (s:=line.strip()) and not s.startswith('#') and s not in ('dump all','save','batch start','batch end')]
def state(text):
 result={};scope='master'
 for s in commands(text):
  t=s.split();cmd=t[0]
  if cmd in ('profile','rateprofile'):
   scope=cmd+':'+t[1];result['active:'+cmd]=t[1];continue
  if cmd=='set':key=scope+':'+s.split(' = ')[0]
  elif cmd in ('resource','dma'):key=' '.join(t[:3])
  elif cmd=='timer':key=' '.join(t[:2])
  elif cmd in ('feature','beeper','beacon'):key=cmd+':'+t[1].lstrip('-')
  elif cmd in ('serial','aux','adjrange','rxrange','rxfail','led','color','vtx','vtxtable'):key=' '.join(t[:2])
  elif cmd=='mode_color':key=' '.join(t[:3])
  else:key=cmd
  result[key]=s
 return result

def verify():
 stock=(R/'configs/stock-dump-all.txt').read_text()
 embedded=(R/'configs/full-defaults.txt').read_text()
 restore=(R/'cli/00-restore-full-stock.cli').read_text()
 assert commands(stock)==commands(embedded),'Embedded command sequence changed'
 assert commands(stock)==[s for s in commands(restore) if s!='defaults bare nosave'],'Restore sequence changed'
 a=state(stock);b=state((R/'build/emulated-dump-all.txt').read_text())
 # MCU UID is immutable silicon, not writable FC configuration. Unicorn has no MCU UID.
 mismatches={k:{'captured':v,'candidate':b.get(k)} for k,v in a.items() if k!='mcu_id' and b.get(k)!=v}
 errors=json.loads((R/'build/emulated-errors.json').read_text())
 # Independently check bytes and address ranges in the deliverable Intel HEX.
 memory={};upper=0
 for line in (R/'dist/FireDrone_Mobula8_STM32F405_BF4.4.2_CANDIDATE.hex').read_text().splitlines():
  raw=bytes.fromhex(line[1:]);assert sum(raw)%256==0
  size=raw[0];addr=int.from_bytes(raw[1:3],'big');kind=raw[3];data=raw[4:4+size]
  if kind==4:upper=int.from_bytes(data,'big')<<16
  elif kind==0:
   for i,v in enumerate(data):memory[upper+addr+i]=v
 expected=embedded.encode()+b'\0'
 assert bytes(memory[0x080f0000+i] for i in range(len(expected)))==expected
 assert not any(0x08004000<=a<0x08008000 for a in memory),'HEX writes existing EEPROM sector'
 assert all(0x08000000<=a<0x08100000 for a in memory)
 report={'status':'PASS' if not mismatches and not errors else 'FAIL','captured_commands':len(commands(stock)),
 'compared_state_entries':len(a)-1,'set_commands':sum(s.startswith('set ') for s in commands(stock)),
 'pid_profiles':4,'rate_profiles':4,'active_pid_profile':a['active:profile'],'active_rate_profile':a['active:rateprofile'],
 'mismatches':mismatches,'cli_errors':errors,'extra_candidate_entries':len(b.keys()-a.keys()),
 'embedded_bytes':len(expected),'embedded_address':'0x080F0000','hex_config_sector_untouched':True,
 'scope':'FC CLI settings; immutable MCU UID excluded; ESC internal EEPROM, RX firmware/binding, fonts, flash logs not cloned',
 'method':'Actual candidate ARM CLI emulation including validateAndFixConfig, with hardware/runtime initializers stubbed; all captured values compared by profile and key. Not flight validation.',
 'hex_sha256':hashlib.sha256((R/'dist/FireDrone_Mobula8_STM32F405_BF4.4.2_CANDIDATE.hex').read_bytes()).hexdigest(),
 'elf_sha256':hashlib.sha256((R/'betaflight/obj/main/betaflight_STM32F405.elf').read_bytes()).hexdigest(),
 'stock_sha256':hashlib.sha256((R/'configs/stock-dump-all.txt').read_bytes()).hexdigest()}
 (R/'dist/config-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps(report,ensure_ascii=False,indent=2))
 if mismatches or errors:raise SystemExit(1)
if __name__=='__main__':verify()
