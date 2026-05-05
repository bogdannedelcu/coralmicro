# _t_twins.py — TWINS: alt 1:1 with a DIFFERENT model per camera.
#
# Goal: bench the production scenario for the multi-slot dispatcher.
# cam0 runs model A on slot 0, cam1 runs model B on slot 1, alt 1:1
# means InferTask alternates between the two models every frame.
# This is the path that exercises the EdgeTPU on-chip cache-context
# switch (~3-5 ms/swap on Coral USB) AND the heap-allocated slot 1
# arena AND the per-cam slot dispatcher all at once.
#
# Sweep: every UNORDERED PAIR from {MSBlock, C2f, GELAN}.  With 3
# candidates that is 3 pairs × 2 directions = 6 (cam0=A, cam1=B and
# cam0=B, cam1=A so we can see if direction matters), but in practice
# the slot/cam mapping is symmetric so we collapse to 3 unique pairs.
#
# Self-contained per agent.md §5.1.2 + §5.1.5.  Persistent flash
# REQUIRED — driver calls sentai.sys.reset() between pairs to reset
# the on-chip parameter cache and start each pair from a clean slate.
import sentai
sentai.verbose(1)

A = ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite")
B = ("C2f",     "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite")
C = ("GELAN",   "/models/iarna_p3p4_GELAN_1ep_uint8_480x640_int8_edgetpu.tflite")

# Pairs to bench.  Each entry: (cam0_model, cam1_model).
PAIRS = [
    (A, B),    # MSBlock vs C2f
    (A, C),    # MSBlock vs GELAN
    (B, C),    # C2f vs GELAN
]

FPS = 30   # VGA30 sweep this run; 2026-04-28 already collected VGA45 in s007.
NB  = 100
STATE_PATH = "/diags/.twins_state"


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


# Resume across reboots: state file holds "session_dir|next_pair_idx".
sess = None
idx  = 0
try:
    raw = sentai.fs.read_str(STATE_PATH).strip()
    parts = raw.split("|")
    if len(parts) == 2:
        sess = parts[0]
        idx  = int(parts[1])
        if not sentai.fs.exists(sess):
            sess = None; idx = 0
except Exception:
    pass

if sess is None:
    sess = _sd("twins")
    csv_path = sess + "/results.csv"
    sentai.fs.write(csv_path,
        "pair_idx,cam0_model,cam1_model,frames,cam0,cam1,"
        "slot0_invokes,slot1_invokes,"
        "invoke_avg,invoke_min,invoke_max,total_avg,fps_x100\r\n")
    print("=== fresh session:", sess, "===")
else:
    csv_path = sess + "/results.csv"
    print("=== resuming session:", sess, "@ pair_idx", idx, "===")


def _bench_pair(pair_idx, cam0_model, cam1_model, csv_path):
    a_label, a_path = cam0_model
    b_label, b_path = cam1_model
    print("\n=== PAIR %d/%d: cam0=%s slot0 | cam1=%s slot1 ===" %
          (pair_idx + 1, len(PAIRS), a_label, b_label))

    # Stop pipeline if running.
    try:
        if sentai.pipeline.running(): sentai.pipeline.stop()
    except Exception: pass

    # Load both slots.
    t0 = sentai.rtos.ticks_ms()
    rc0 = sentai.tpu.load_slot(0, a_path)
    print("  slot 0 %s rc=%s in %d ms" %
          (a_label, rc0, sentai.rtos.ticks_ms() - t0))
    if rc0 != 0:
        sentai.fs.append(csv_path,
            "%d,%s,%s,LOAD0_FAIL,0,0,0,0,0,0,0,0,0\r\n" %
            (pair_idx, a_label, b_label))
        return
    t0 = sentai.rtos.ticks_ms()
    rc1 = sentai.tpu.load_slot(1, b_path)
    print("  slot 1 %s rc=%s in %d ms" %
          (b_label, rc1, sentai.rtos.ticks_ms() - t0))
    if rc1 != 0:
        sentai.fs.append(csv_path,
            "%d,%s,%s,LOAD1_FAIL,0,0,0,0,0,0,0,0,0\r\n" %
            (pair_idx, a_label, b_label))
        return

    # Configure cam → slot routing for the twin scenario.
    sentai.pipeline.set_slot_for_cam(0, 0)   # cam0 → slot 0 (model A)
    sentai.pipeline.set_slot_for_cam(1, 1)   # cam1 → slot 1 (model B)
    print("  routing: cam0->slot %d, cam1->slot %d" %
          (sentai.pipeline.get_slot_for_cam(0),
           sentai.pipeline.get_slot_for_cam(1)))

    # Camera up at VGA45.
    rc = sentai.camera.init(1, FPS)
    print("  camera.init(1, %d) =" % FPS, rc)
    if rc == -11:
        sentai.rtos.sleep_ms(200); sentai.sys.reset()
    elif rc != 0:
        sentai.fs.append(csv_path,
            "%d,%s,%s,INIT_FAIL,0,0,0,0,0,0,0,0,0\r\n" %
            (pair_idx, a_label, b_label))
        return
    sentai.rtos.sleep_ms(800)
    sentai.camera.ratio(1, 1)
    sentai.camera.switch_drain(1)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)

    # Reset slot counters before the timed window.
    sentai.pipeline.slot_stats_reset()

    # The model-path arg of pipeline.calibrate is cosmetic in this
    # multi-slot world: load_slot already populated both slots and
    # the dispatcher routes per cam_id.  Pass a_path so the calibrate
    # progress messages are deterministic.
    print("  pipeline.calibrate (NB=%d, alt 1:1) ..." % NB)
    res = sentai.pipeline.calibrate(a_path, NB, 5000)
    n = res.get("frames", 0) or 1
    cam0 = res.get("cam0", 0)
    cam1 = res.get("cam1", 0)
    ia   = res.get("invoke_ms_sum", 0) // n
    imn  = res.get("invoke_ms_min", 0)
    imx  = res.get("invoke_ms_max", 0)
    ta   = res.get("total_ms_sum", 0) // n
    f100 = res.get("fps_x100", 0)

    slots = sentai.pipeline.slot_stats()
    print("  frames=%d cam0=%d cam1=%d invoke=%d/%d/%d total=%d fps=%d.%02d" %
          (n, cam0, cam1, ia, imn, imx, ta, f100 // 100, f100 % 100))
    print("  slot_stats: s0=%d s1=%d s2=%d (sum=%d)" %
          (slots[0], slots[1], slots[2], sum(slots)))

    sentai.fs.append(csv_path,
        "%d,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n" %
        (pair_idx, a_label, b_label, n, cam0, cam1,
         slots[0], slots[1], ia, imn, imx, ta, f100))

    try: sentai.pipeline.stop()
    except Exception: pass


while idx < len(PAIRS):
    cam0_model, cam1_model = PAIRS[idx]
    _bench_pair(idx, cam0_model, cam1_model, csv_path)
    idx += 1
    if idx < len(PAIRS):
        sentai.fs.write(STATE_PATH, "%s|%d" % (sess, idx))
        print("--- pair %d/%d done; sys.reset() to clear TPU cache ---" %
              (idx, len(PAIRS)))
        sentai.rtos.sleep_ms(500)
        sentai.sys.reset()

try: sentai.fs.remove(STATE_PATH)
except Exception: pass
print("\nCSV: %s" % csv_path)
print("=== done ===")
