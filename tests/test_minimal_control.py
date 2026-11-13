"""Run the minimal motion example against an emulated STS3215, never hardware."""
import os
import pty
import select
import subprocess
import sys
import threading
import unittest

BINARY = sys.argv.pop(1)

def packet(error=0, data=b""):
    value = bytearray((255, 255, 1, len(data)+2, error)) + data
    value.append(255-(sum(value[2:]) & 255))
    return value

class MinimalExample(unittest.TestCase):
    def run_example(self, speed_mode, reply_level, fail_phase=False):
        master, slave = pty.openpty()
        regs = bytearray(256)
        regs[8] = reply_level
        regs[0x21] = 0 if speed_mode else 1
        regs[0x12] = 0x0c # Unit bit 2 must survive; stop/max-speed bit 3 may change.
        regs[0x28] = regs[0x37] = 1
        regs[0x38:0x3c] = bytes((0,4,0x64,0x80)) # Position 1024, speed -100.
        stop, errors, writes = threading.Event(), [], []
        def emulate():
            pending = bytearray()
            try:
                while not stop.is_set():
                    if not select.select([master], [], [], 0.02)[0]:
                        continue
                    pending += os.read(master,512)
                    while len(pending)>=4:
                        self.assertEqual(pending[:2],b"\xff\xff")
                        size=pending[3]+4
                        if len(pending)<size:
                            break
                        frame=bytes(pending[:size]); del pending[:size]
                        self.assertEqual(sum(frame[2:]) & 255,255)
                        self.assertEqual(frame[2],1)
                        command,address=frame[4],frame[5]
                        if command==2:
                            response=packet(0,regs[address:address+frame[6]])
                        else:
                            self.assertEqual(command,3)
                            data=frame[6:-1]
                            writes.append((address,bytes(data)))
                            if address in (0x21,0x12):
                                self.assertEqual(regs[0x28],0,"mode writes must precede torque enable")
                                self.assertEqual(regs[0x37],0,"EEPROM must be unlocked")
                            if address in (0x2a,0x2e):
                                self.assertEqual(regs[0x28],0,"load the target before torque enable")
                            if address==0x28 and data==b"\x01":
                                self.assertEqual(regs[0x21],int(speed_mode))
                                self.assertEqual(regs[0x2e:0x30],bytes((0xf4,1)))
                                if not speed_mode:
                                    self.assertEqual(regs[0x2a:0x2c],bytes((0,8)))
                            error=1 if fail_phase and address==0x12 else 0
                            if not error:
                                regs[address:address+len(data)]=data
                            response=packet(error) if reply_level else b""
                        if response:
                            os.write(master,response)
            except BaseException as exc:
                errors.append(exc)
        thread=threading.Thread(target=emulate,daemon=True); thread.start()
        try:
            result=subprocess.run([BINARY,os.ttyname(slave),"speed" if speed_mode else "position"],
                                  capture_output=True,text=True,timeout=5)
        finally:
            stop.set(); thread.join(1); os.close(master); os.close(slave)
        self.assertFalse(errors,errors)
        self.assertEqual(regs[0x37],1,"EEPROM must be re-locked, including on failure")
        if fail_phase:
            self.assertEqual(result.returncode,1)
            self.assertEqual(regs[0x28],0,"failed configuration must leave torque disabled")
            self.assertFalse(any(address in (0x2a,0x2e) for address,_ in writes))
        else:
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertIn("position=1024 speed=-100",result.stdout)
            self.assertEqual(regs[0x28],1)
            self.assertEqual(regs[0x12],4 if speed_mode else 12)
            self.assertEqual(regs[0x21],int(speed_mode))

    def test_configuration_feedback_and_both_control_modes(self):
        for speed in (False,True):
            for level in (0,1):
                with self.subTest(speed=speed,reply_level=level):
                    self.run_example(speed,level)

    def test_configuration_failure_relocks_and_keeps_torque_off(self):
        self.run_example(True,1,fail_phase=True)

if __name__ == "__main__":
    unittest.main()
