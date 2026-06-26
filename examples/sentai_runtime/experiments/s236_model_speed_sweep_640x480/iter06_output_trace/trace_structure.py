"""Per-model: capture load message (parameter_caching_exe?) + invoke hint
structure (sequence of N=instruction / O=output / P=param / I=input hints).
Determines which models are MULTI-PASS (an N instruction hint appears AFTER
an O output hint) vs single-pass. Resets between models to clear TPU wedge.
"""
import sys, os, time, subprocess
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import board_serial as B

MODELS = ["msblock", "c3", "c2f", "c2f_pan2", "gelan", "gelan_pan2",
          "c2f_deep", "c2f_thick"]
OUT = os.path.join(os.path.dirname(__file__), "structure.txt")


def wait_enum():
    for _ in range(25):
        if b"1fc9:c0a1" in subprocess.run(["lsusb"], capture_output=True).stdout:
            break
        time.sleep(1)
    time.sleep(4)


def fresh():
    for _ in range(12):
        try:
            return B.open_repl()
        except Exception:
            time.sleep(2)
    raise RuntimeError("no repl")


def raw(ser, line, t=18):
    try:
        while ser.in_waiting:
            ser.read(ser.in_waiting)
    except Exception:
        pass
    ser.write(line.encode() + b"\r\n")
    dl = time.time() + t
    buf = bytearray()
    while time.time() < dl:
        try:
            c = ser.read(4096)
        except Exception:
            return buf.decode(errors="replace")
        if c:
            buf.extend(c)
            if b"\r\n>>> " in bytes(buf):
                break
    return buf.decode(errors="replace")


def reset(ser):
    try:
        ser.write(b"sentai.sys.reset()\r\n")
        time.sleep(1)
        ser.close()
    except Exception:
        pass
    wait_enum()


lines_out = []
ser = fresh()
raw(ser, "import sentai", 4)
raw(ser, "sentai.camera.set_resolution(640,480)", 4)
raw(ser, "sentai.camera.init(1)", 15)
time.sleep(1)
for m in MODELS:
    ld = raw(ser, "sentai.tpu.load('/%s.tflite')" % m, 18)
    pc = "param_caching=YES" if "parameter_caching_exe=present" in ld else "param_caching=no"
    raw(ser, "sentai.camera.to_tensor()", 10)
    raw(ser, "sentai.tpu.trace(1)", 4)
    inv = raw(ser, "print('INV', sentai.tpu.invoke())", 22)
    raw(ser, "sentai.tpu.trace(0)", 4)
    # extract the [tpu] hint sequence (N/O/P/I), compress to a structure string
    seq = []
    for ln in inv.split("\r\n"):
        ln = ln.strip()
        if ln.startswith("[tpu] "):
            tok = ln.split()[1]  # N/O/P/I/E
            if tok in ("N", "O", "P", "I"):
                seq.append(tok)
    seqs = "".join(seq)
    # multi-pass = an N occurs after the first O
    first_o = seqs.find("O")
    multipass = first_o >= 0 and "N" in seqs[first_o:]
    rc = "FAIL" if "INV -2" in inv or "0B6" in inv else "OK"
    row = "%-11s %-17s seq=%-14s multipass=%s invoke=%s" % (
        m, pc, seqs, "YES" if multipass else "no", rc)
    print(row)
    lines_out.append(row)
    reset(ser)
    ser = fresh()
    raw(ser, "import sentai", 4)
    raw(ser, "sentai.camera.set_resolution(640,480)", 4)
    raw(ser, "sentai.camera.init(1)", 15)
    time.sleep(1)
ser.close()
open(OUT, "w").write("\n".join(lines_out) + "\n")
print("\nsaved", OUT)
