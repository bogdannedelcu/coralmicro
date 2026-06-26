"""Probe which of the 8 models invoke on the board. Reset between each model
to clear any TPU wedge (experiment.md: a bad model wedges the TPU for the boot).
Run from repo. Writes results to invoke_probe.csv.
"""
import sys, time, os
sys.path.insert(0, os.path.dirname(__file__) or ".")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import board_serial as B

MODELS = ["msblock", "c3", "c2f", "c2f_pan2", "gelan", "gelan_pan2",
          "c2f_deep", "c2f_thick"]
OUT = os.path.join(os.path.dirname(__file__), "invoke_probe.csv")


def wait_enum():
    import subprocess
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


def line_of(out, key):
    for l in out.split("\r\n"):
        if key in l:
            return l.strip()
    return ""


def reset(ser):
    try:
        ser.write(b"sentai.sys.reset()\r\n")
        time.sleep(1)
        ser.close()
    except Exception:
        pass
    wait_enum()


rows = ["model,size_bytes,load_rc,invoke_ms,i_bytes,in_bytes,fails,note"]
ser = open_retry()
B.cmd(ser, "import sentai", 4)
# camera once
B.cmd(ser, "sentai.camera.set_resolution(640,480)", 4)
B.cmd(ser, "sentai.camera.init(1)", 15)
B.cmd(ser, "sentai.tpu.chunk_size(64*1024)", 4)
time.sleep(1)

for m in MODELS:
    path = "/%s.tflite" % m
    print("\n=== %s ===" % m)
    ld = B.cmd(ser, "print('LD', sentai.tpu.load('%s'))" % path, 20)
    load_rc = line_of(ld, "LD")
    B.cmd(ser, "sentai.camera.to_tensor()", 15)
    B.cmd(ser, "sentai.tpu.urb_stats(1)", 4)  # reset urb counters
    fails = 0
    inv_ms = []
    for k in range(5):
        o = B.cmd(ser, "print('INV', sentai.tpu.invoke())", 15)
        v = line_of(o, "INV")
        print("  ", v)
        try:
            n = int(v.split("INV")[1].strip())
            if n < 0:
                fails += 1
            else:
                inv_ms.append(n)
        except Exception:
            fails += 1
    urb = B.cmd(ser, "hz,r=sentai.tpu.urb_stats(0); print('IB', r[0][3], 'INB', r[1][3])", 6)
    ib = line_of(urb, "IB")
    sz = B.cmd(ser, "print('SZ', sentai.fs.size('%s'))" % path, 6)
    med = sorted(inv_ms)[len(inv_ms)//2] if inv_ms else -1
    note = "OK" if fails == 0 else ("PARTIAL" if inv_ms else "ALL_FAIL")
    rows.append("%s,%s,%s,%d,%s,,%d,%s" % (
        m, line_of(sz, "SZ").replace("SZ", "").strip(),
        load_rc.replace("LD", "").strip(), med, ib, fails, note))
    print("  -> %s med=%dms fails=%d" % (note, med, fails))
    # reset between models to clear any wedge
    reset(ser)
    ser = open_retry()
    B.cmd(ser, "import sentai", 4)
    B.cmd(ser, "sentai.camera.set_resolution(640,480)", 4)
    B.cmd(ser, "sentai.camera.init(1)", 15)
    B.cmd(ser, "sentai.tpu.chunk_size(64*1024)", 4)
    time.sleep(1)

ser.close()
open(OUT, "w").write("\n".join(rows) + "\n")
print("\n=== SUMMARY ===")
print("\n".join(rows))
print("csv:", OUT)
