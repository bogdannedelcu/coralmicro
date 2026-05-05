# _t_two_slot_pipeline.py — validate Phase 2 per-camera TPU slot dispatch.
#
# Hypothesis to validate:
#   In pipeline mode, with cam0 routed to slot 0 (MSBlock) and cam1
#   routed to slot 1 (C2f) at alt 1:1 ratio, both slots' invoke
#   counters MUST advance ~50/50.  Pipeline FPS is the main throughput
#   metric.  Per-slot detection result publication is Phase 2b — for
#   now we only verify dispatch correctness via slot_stats() counters.
#
# Self-contained per agent.md §5.1.2 + §5.1.5.
import sentai
sentai.verbose(1)

A = ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite")
B = ("C2f",     "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite")
FPS = 45
NB  = 100
RATIO_A, RATIO_B = 1, 1   # alt 1:1


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


sess = _sd("two_slot_pipeline")
csv_path = sess + "/results.csv"
sentai.fs.write(csv_path,
    "phase,key,value\r\n")

print("=== session:", sess, " slot_count:", sentai.tpu.slot_count(), "===")

# Stop pipeline if running.
try:
    if sentai.pipeline.running(): sentai.pipeline.stop()
except Exception: pass

# 1) Load slot 0 (MSBlock) — sets up the TPU device + pipeline target.
t0 = sentai.rtos.ticks_ms()
rc = sentai.tpu.load_slot(0, A[1])
print("slot 0 load %s rc=%s in %d ms" % (A[0], rc, sentai.rtos.ticks_ms() - t0))
sentai.fs.append(csv_path, "load,slot0_%s,%s\r\n" % (A[0], rc))

# 2) Load slot 1 (C2f) — heap-allocated arena.
t0 = sentai.rtos.ticks_ms()
rc = sentai.tpu.load_slot(1, B[1])
print("slot 1 load %s rc=%s in %d ms" % (B[0], rc, sentai.rtos.ticks_ms() - t0))
sentai.fs.append(csv_path, "load,slot1_%s,%s\r\n" % (B[0], rc))

# Sanity: both slots ready?
if not sentai.tpu.slot_ready(0) or not sentai.tpu.slot_ready(1):
    print("FAIL: slots not ready (s0=%s s1=%s)" %
          (sentai.tpu.slot_ready(0), sentai.tpu.slot_ready(1)))
    print("=== done ==="); raise SystemExit

# 3) Configure cam → slot routing.
sentai.pipeline.set_slot_for_cam(0, 0)   # cam0 → MSBlock
sentai.pipeline.set_slot_for_cam(1, 1)   # cam1 → C2f
print("routing: cam0->slot %d, cam1->slot %d" %
      (sentai.pipeline.get_slot_for_cam(0),
       sentai.pipeline.get_slot_for_cam(1)))
sentai.fs.append(csv_path, "route,cam0->slot,%d\r\n" %
                 sentai.pipeline.get_slot_for_cam(0))
sentai.fs.append(csv_path, "route,cam1->slot,%d\r\n" %
                 sentai.pipeline.get_slot_for_cam(1))

# 4) Bring camera up at VGA45.
rc = sentai.camera.init(1, FPS)
print("camera.init(1, %d) =" % FPS, rc)
if rc == -11:
    sentai.rtos.sleep_ms(200); sentai.sys.reset()
elif rc != 0:
    print("FAIL camera.init=%d" % rc); print("=== done ==="); raise SystemExit
sentai.rtos.sleep_ms(800)
sentai.camera.ratio(RATIO_A, RATIO_B)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(300)

# 5) Reset slot counters, run pipeline.calibrate(slot0_model, NB).
sentai.pipeline.slot_stats_reset()
print("--- pipeline.calibrate (model=%s, N=%d frames, alt %d:%d) ---" %
      (A[0], NB, RATIO_A, RATIO_B))
res = sentai.pipeline.calibrate(A[1], NB, 5000)
n = res.get("frames", 0) or 1
ia = res.get("invoke_ms_sum", 0) // n
ta = res.get("total_ms_sum", 0) // n
f100 = res.get("fps_x100", 0)
print("  frames=%d cam0=%d cam1=%d invoke=%d/%d/%d total=%d fps=%d.%02d" %
      (n, res.get("cam0", 0), res.get("cam1", 0),
       ia, res.get("invoke_ms_min", 0), res.get("invoke_ms_max", 0),
       ta, f100 // 100, f100 % 100))
sentai.fs.append(csv_path,
    "pipe,frames,%d\r\npipe,cam0,%d\r\npipe,cam1,%d\r\npipe,invoke_avg,%d\r\npipe,fps_x100,%d\r\n" %
    (n, res.get("cam0", 0), res.get("cam1", 0), ia, f100))

# 6) Read per-slot invoke counters.
slot_inv = sentai.pipeline.slot_stats()
print("slot_stats: slot0=%d slot1=%d slot2=%d (sum=%d)" %
      (slot_inv[0], slot_inv[1], slot_inv[2],
       slot_inv[0] + slot_inv[1] + slot_inv[2]))
sentai.fs.append(csv_path,
    "slot,slot0_invokes,%d\r\nslot,slot1_invokes,%d\r\nslot,slot2_invokes,%d\r\n" %
    (slot_inv[0], slot_inv[1], slot_inv[2]))

# 7) Verdict.
total_inv = sum(slot_inv)
print("\n=== Verdict ===")
if total_inv == 0:
    print("  FAIL: no invokes recorded")
elif slot_inv[0] > 0 and slot_inv[1] > 0:
    pct1 = (100 * slot_inv[1]) // total_inv
    print("  PASS: both slots invoked, slot1 share=%d%%" % pct1)
    if 30 <= pct1 <= 70:
        print("  Distribution healthy (target 50%% at alt 1:1)")
    else:
        print("  Distribution skewed; expected ~50%% at alt 1:1")
elif slot_inv[0] > 0:
    print("  PARTIAL: only slot 0 invoked (cam1 frames not arriving?)")
elif slot_inv[1] > 0:
    print("  PARTIAL: only slot 1 invoked (cam0 frames not arriving?)")

try: sentai.pipeline.stop()
except Exception: pass
print("\nCSV:", csv_path)
print("=== done ===")
