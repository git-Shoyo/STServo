"""Real Linux termios/PTY integration; no servo hardware required."""
import errno
import os
import pty
import select
import subprocess
import sys
import termios
import threading
import time
import unittest

BINARY = sys.argv.pop(1)

def status(servo_id, error=0, data=b""):
    packet = bytearray((255, 255, servo_id, len(data) + 2, error)) + data
    packet.append(255 - (sum(packet[2:]) & 255))
    return bytes(packet)

class PosixIntegration(unittest.TestCase):
    def run_serial(self, *, echo=False, echo_only=False, error=0, missing=None, noise=False):
        master, slave = pty.openpty()
        original = termios.tcgetattr(slave)
        device = os.ttyname(slave)
        stop = threading.Event()
        failures = []
        requests = []

        def emulate():
            pending = bytearray()
            try:
                while not stop.is_set():
                    if not select.select([master], [], [], 0.02)[0]:
                        continue
                    pending += os.read(master, 512)
                    while len(pending) >= 4:
                        if pending[:2] != b"\xff\xff":
                            del pending[0]
                            continue
                        size = pending[3] + 4
                        if len(pending) < size:
                            break
                        frame = bytes(pending[:size])
                        del pending[:size]
                        self.assertEqual(sum(frame[2:]) & 255, 255)
                        servo_id, instruction = frame[2], frame[4]
                        self.assertIn(instruction, (1, 2), "CLI must never send a motion/write command")
                        requests.append((servo_id, instruction))
                        response = frame if echo else b""
                        if not echo_only and servo_id != missing:
                            payload = bytes((servo_id,)) if instruction == 2 else b""
                            if instruction == 2:
                                self.assertEqual(frame[5:7], bytes((5, 1)))
                            if noise:
                                other = (servo_id + 10) % 254
                                response += b"\x00\x7f" + status(other, 0, bytes((other,)) if payload else b"")
                            response += status(servo_id, error, b"" if error else payload)
                        # Exercise real serial reads of partial status packets.
                        for index in range(0, len(response), 2):
                            os.write(master, response[index:index + 2])
                            time.sleep(0.001)
            except OSError as exc:
                if exc.errno not in (errno.EIO, errno.EBADF):
                    failures.append(exc)
            except BaseException as exc:
                failures.append(exc)

        thread = threading.Thread(target=emulate, daemon=True)
        thread.start()
        try:
            command = [BINARY, "--device", device, "--timeout", "100"]
            if echo:
                command.append("--echo")
            command += ["1", "2"]
            result = subprocess.run(command, capture_output=True, text=True, timeout=5)
            self.assertEqual(termios.tcgetattr(slave), original, "terminal settings must be restored")
        finally:
            stop.set()
            thread.join(timeout=1)
            os.close(master)
            os.close(slave)
        self.assertFalse(failures, failures)
        return result, requests

    def test_two_ids_with_fragmented_replies_and_noise(self):
        result, requests = self.run_serial(noise=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("id=1 read_id=OK status=1 err=0x00 value=1", result.stdout)
        self.assertIn("id=2 read_id=OK status=1 err=0x00 value=2", result.stdout)
        self.assertEqual(requests, [(1, 1), (1, 2), (2, 1), (2, 2)])

    def test_explicit_local_echo(self):
        result, _ = self.run_serial(echo=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("responding=2 queried=2", result.stdout)

    def test_echo_is_not_a_success(self):
        result, _ = self.run_serial(echo=True, echo_only=True)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout.count("result=ECHO_ONLY"), 2)

    def test_real_servo_error_one_is_preserved(self):
        result, requests = self.run_serial(error=1)
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout.count("result=OK status=1 err=0x01"), 2)
        self.assertEqual(requests, [(1, 1), (2, 1)])

    def test_missing_second_id_does_not_reuse_first_result(self):
        result, _ = self.run_serial(missing=2)
        self.assertEqual(result.returncode, 2)
        self.assertIn("id=2 result=TIMEOUT status=0", result.stdout)
        self.assertIn("responding=1 queried=2", result.stdout)

    def test_invalid_cli_and_missing_device(self):
        for args in (["--device", "/dev/does-not-exist", "1"],
                     ["--device", "/dev/null", "254"],
                     ["--device", "/dev/null", "--timeout", "0", "1"]):
            result = subprocess.run([BINARY, *args], capture_output=True, text=True, timeout=2)
            self.assertEqual(result.returncode, 1)

if __name__ == "__main__":
    unittest.main()
