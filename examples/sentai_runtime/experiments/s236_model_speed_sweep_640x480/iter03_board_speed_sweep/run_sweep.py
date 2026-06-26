"""Host runner for _t_s236_speed.py: exec the board driver, resume after each
sys.reset() until '=== done ==='. Pulls results.csv at the end.
"""
import sys, os, time, subprocess
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import board_serial as B

DRIVER = "/lib/diag/_t_s236_speed.py"
OUT = os.path.dirname(__file__)
LOG = os.path.join(OUT, "sweep_stream.log")
MAX_SEGMENTS = 12  # 8 models + margin


def wait_enum():
    for _ in range(25):
        if b"1fc9:c0a1" in subprocess.run(["lsusb"], capture_output=True).stdout:
            break
        time.sleep(1)
    time.sleep(4)


def open_retry():
    for _ in range(12):
        try:
            return B.open_repl()
        except Exception:
            time.sleep(2)
    raise RuntimeError("cannot open REPL")


def exec_segment(logf):
    """exec driver; stream until done or connection drop (reset)."""
    ser = open_retry()
    B.cmd(ser, "import sentai", 4)
    while ser.in_waiting:
        ser.read(ser.in_waiting)
    ser.write(('exec(sentai.fs.read_str("%s"))\r\n' % DRIVER).encode())
    dl = time.monotonic() + 120
    acc = bytearray()
    while time.monotonic() < dl:
        try:
            c = ser.read(4096)
        except Exception:
            # serial dropped => board reset mid-run
            logf.write("\n[host] serial drop (reset)\n")
            logf.flush()
            try:
                ser.close()
            except Exception:
                pass
            return "reset"
        if c:
            acc.extend(c)
            logf.write(c.decode(errors="replace"))
            logf.flush()
            if b"=== done ===" in acc:
                try:
                    ser.close()
                except Exception:
                    pass
                return "done"
        else:
            time.sleep(0.05)
    try:
        ser.close()
    except Exception:
        pass
    return "timeout"


def main():
    logf = open(LOG, "w")
    status = None
    for seg in range(MAX_SEGMENTS):
        logf.write("\n===== segment %d =====\n" % seg)
        logf.flush()
        print("segment", seg)
        status = exec_segment(logf)
        print("  ->", status)
        if status == "done":
            break
        if status == "reset":
            wait_enum()
            continue
        if status == "timeout":
            print("  timeout, retrying")
            wait_enum()
            continue
    logf.close()
    print("final status:", status)


if __name__ == "__main__":
    main()
