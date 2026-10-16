#!/usr/bin/env python3
"""Read-only board inspection; never flash, arm, save, reset, or set resources."""
import argparse, datetime, json, pathlib, re, time

def inspect(text):
    versions=re.findall(r"Betaflight / ([A-Za-z0-9_]+)\s+\([^)]*\)\s+([\d.]+)",text)
    boards=re.findall(r"(?:board_name\s+|Board:\s*FPVM/)([A-Za-z0-9_]+)",text)
    mcu=versions[0][0] if versions else None
    board=boards[0] if boards else None
    def setting(name):
        m=re.findall(r"^set "+re.escape(name)+r"\s*=\s*(.+)$",text,re.M)
        return m[-1].strip() if m else None
    errors=[]
    if mcu != "STM32F405": errors.append("STM32F405 확인 필요: AT32F435에는 이 HEX를 사용하지 마세요.")
    if board != "BETAFLIGHTF4": errors.append("보드 타깃이 BETAFLIGHTF4인지 확인되지 않았습니다.")
    if versions and versions[0][1] != "4.4.2": errors.append("캡처 기준 Betaflight 4.4.2와 버전이 다릅니다.")
    if len(set(versions))>1 or len(set(boards))>1: errors.append("여러 보드/버전의 출력을 섞지 마세요.")
    return dict(mcu=mcu,board=board,version=versions[0][1] if versions else None,
        receiver=setting("serialrx_provider"),board_roll=setting("align_board_roll"),
        motor_order=setting("motor_output_reordering"),gyro=setting("gyro_1_sensor_align"),
        identity_matches=not errors,errors=errors)

def capture(port,dest):
    import serial
    dest.mkdir(parents=True,exist_ok=False)
    def response(s,timeout=15):
        data=bytearray(); end=time.monotonic()+timeout
        while time.monotonic()<end:
            data.extend(s.read(max(s.in_waiting,1)))
            if re.search(rb"(?:\r?\n)#\s*$",data): return data.decode(errors="replace")
        raise TimeoutError("CLI prompt timeout; no configuration was written")
    with serial.Serial(port,115200,timeout=.2) as s:
        s.write(b"#"); response(s)
        for command in ("version","status","diff all","dump all","resource","timer","dma","tasks"):
            s.write((command+"\n").encode())
            (dest/(command.replace(" ","-")+".txt")).write_text(response(s))
    # Deliberately no exit/save/reboot command. Unplug USB or leave CLI in Configurator.
    return "\n".join(p.read_text() for p in sorted(dest.glob("*.txt")))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    group=ap.add_mutually_exclusive_group(required=True)
    group.add_argument("--port"); group.add_argument("--transcript",type=pathlib.Path)
    ap.add_argument("--out",type=pathlib.Path)
    a=ap.parse_args()
    if a.port:
        dest=a.out or pathlib.Path(__file__).resolve().parents[1]/"backups"/datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        text=capture(a.port,dest)
    else: text=a.transcript.read_text(errors="replace")
    report=inspect(text); print(json.dumps(report,ensure_ascii=False,indent=2))
    if not report["identity_matches"]: raise SystemExit(2)
    print("보드 종류 일치. 이는 플래시/비행 검증 결과가 아닙니다. README의 설정·방향·수신기 확인이 필요합니다.")
if __name__=="__main__":main()
