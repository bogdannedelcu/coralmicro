# _t_load_order.py — isolate "first model loaded after boot pays
# extra latency" hypothesis.
#
# Method: 3 models × 3 boot orderings, each rotation puts a different
# model first.  Across reboots, persist [order_idx, csv_path] in
# /diags/.load_order_state and resume.  Per boot we time:
#   - load #1 (the FIRST tpu.load() of this boot)
#   - load #2 (second model, with EdgeTPU already opened)
#   - load #3 (third model)
#
# If the hypothesis is correct, MSBlock-first / C2f-first / GELAN-first
# should ALL show the same "load #1 is anomalously slow" pattern,
# regardless of which model is in slot 1.  If MSBlock specifically is
# slow, we'd see it slow even from slot 2 / slot 3.
#
# Self-contained per agent.md §5.1.2 + §5.1.5.  Drives the entire
# experiment internally; host only pushes/execs/downloads.
import sentai
sentai.verbose(1)

MODELS = [
    ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite"),
    ("C2f",     "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite"),
    ("GELAN",   "/models/iarna_p3p4_GELAN_1ep_uint8_480x640_int8_edgetpu.tflite"),
]
# 3 rotations: (MSBlock, C2f, GELAN), (C2f, GELAN, MSBlock), (GELAN, MSBlock, C2f).
ORDERS = [(0,1,2), (1,2,0), (2,0,1)]
STATE_PATH = "/diags/.load_order_state"


def _sd(name):
    try: sentai.fs.mkdir("/diags")
    except Exception: pass
    c = "/diags/.counter"
    sid = 1
    try: sid = int(sentai.fs.read_str(c).strip()) + 1
    except Exception: pass
    try: sentai.fs.write(c, str(sid))
    except Exception: pass
    d = "/diags/s%03d_%s" % (sid, name)
    try: sentai.fs.mkdir(d)
    except Exception: pass
    return d


def _time_load(label, path):
    t0 = sentai.rtos.ticks_ms()
    rc = sentai.tpu.load(path)
    ms = sentai.rtos.ticks_ms() - t0
    print("  [load slot] %-8s %5d ms rc=%s" % (label, ms, rc))
    # One warmup invoke to confirm model actually runs (rc>0 = elapsed ms).
    t0 = sentai.rtos.ticks_ms()
    try: rci = sentai.tpu.invoke()
    except Exception as e: rci = -1
    w0 = sentai.rtos.ticks_ms() - t0
    print("  [warm0]                %5d ms rc=%s" % (w0, rci))
    return ms, w0


# Resume across reboots.
sess = None
order_idx = 0
try:
    raw = sentai.fs.read_str(STATE_PATH).strip()
    parts = raw.split("|")
    if len(parts) == 2:
        sess = parts[0]
        order_idx = int(parts[1])
        if not sentai.fs.exists(sess):
            sess = None; order_idx = 0
except Exception:
    pass

if sess is None:
    sess = _sd("load_order")
    csv_path = sess + "/results.csv"
    sentai.fs.write(csv_path,
        "order_idx,slot,label,load_ms,warm0_ms\r\n")
    print("=== fresh session:", sess, "===")
else:
    csv_path = sess + "/results.csv"
    print("=== resuming session:", sess, "@ order_idx", order_idx, "===")

# Stop pipeline if running.
try:
    if sentai.pipeline.running(): sentai.pipeline.stop()
except Exception: pass

if order_idx < len(ORDERS):
    order = ORDERS[order_idx]
    print("\n--- order #%d: %s -> %s -> %s ---" %
          (order_idx,
           MODELS[order[0]][0], MODELS[order[1]][0], MODELS[order[2]][0]))
    for slot, midx in enumerate(order):
        label, path = MODELS[midx]
        ms, w0 = _time_load(label, path)
        sentai.fs.append(csv_path,
            "%d,%d,%s,%d,%d\r\n" % (order_idx, slot, label, ms, w0))
    order_idx += 1
    if order_idx < len(ORDERS):
        sentai.fs.write(STATE_PATH, "%s|%d" % (sess, order_idx))
        print("--- order %d/%d done; sys.reset() for next ordering ---" %
              (order_idx, len(ORDERS)))
        sentai.rtos.sleep_ms(500)
        sentai.sys.reset()

# All done.
try: sentai.fs.remove(STATE_PATH)
except Exception: pass
print("\nCSV: %s" % csv_path)
print("=== done ===")
