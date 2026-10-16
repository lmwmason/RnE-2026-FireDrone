#!/usr/bin/env python3
"""CLI transport for supervised bench operations. Does not issue motor commands."""
import time
import serial
class CLI:
    def __init__(self, port="/dev/ttyACM0", already=False):
        self.s=serial.Serial(port,115200,timeout=.15)
        self.s.reset_input_buffer()
        if already:
            self.s.write(b"\n")
        else:
            self.s.write(b"#");self.response()
    def response(self, timeout=30):
        out=bytearray();end=time.monotonic()+timeout
        while time.monotonic()<end:
            out.extend(self.s.read(max(1,self.s.in_waiting)))
            if out.endswith(b"\n# "):
                more=self.s.read(1)
                if not more:return out.decode().replace("\r\n","\n")
                out.extend(more)
        raise TimeoutError("CLI prompt timeout")
    def command(self, text):
        self.s.write((text+"\n").encode());result=self.response()
        if "###ERROR" in result:raise RuntimeError(result)
        return result
    def reboot(self, command="save"):
        self.s.write((command+"\n").encode());self.s.flush();time.sleep(.5);self.s.close()
    def close(self):self.s.close()
