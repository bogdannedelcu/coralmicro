"""Reusable serial-REPL helpers for s236 board work (modeled on run_s234.py)."""
import time
import serial

PORT = "/dev/ttyACM0"
BAUD = 115200


def drain(ser, quiet_s=0.3):
    end = time.monotonic() + quiet_s
    buf = bytearray()
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
            end = time.monotonic() + quiet_s
        else:
            time.sleep(0.02)
    return bytes(buf)


def open_repl():
    ser = serial.Serial(PORT, BAUD, timeout=0.3)
    ser.write(b"\r\x03\x03")
    time.sleep(0.3)
    drain(ser, 0.4)
    ser.write(b"\r\n")
    time.sleep(0.2)
    drain(ser, 0.4)
    return ser


def cmd(ser, line, t=8.0):
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    ser.write(line.encode() + b"\r\n")
    dl = time.monotonic() + t
    buf = bytearray()
    while time.monotonic() < dl:
        c = ser.read(4096)
        if c:
            buf.extend(c)
            if b"\r\n>>> " in bytes(buf):
                break
        else:
            time.sleep(0.02)
    return bytes(buf).decode(errors="replace")


def run_driver(ser, path, timeout_s=180.0, log=None):
    """exec a board-side driver file over REPL, stream until '=== done ==='."""
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    ser.write(('exec(sentai.fs.read_str("%s"))\r\n' % path).encode())
    dl = time.monotonic() + timeout_s
    acc = bytearray()
    saw_done = False
    while time.monotonic() < dl:
        c = ser.read(4096)
        if c:
            acc.extend(c)
            if log:
                log.write(c.decode(errors="replace"))
                log.flush()
            if b"=== done ===" in acc:
                saw_done = True
                tail = drain(ser, 1.0)
                if log:
                    log.write(tail.decode(errors="replace"))
                break
        else:
            time.sleep(0.05)
    return saw_done, acc.decode(errors="replace")
