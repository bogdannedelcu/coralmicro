# _t_twins3.py — TWINS3: alt 2:1 with TWO MODELS per cam1 frame.
#
# Scenario: at alt 2:1 ratio cam0 frames are common (~67%), cam1
# frames rare (~33%).  Treat the rarer cam1 frame as "important" —
# run TWO model interpretations on it back-to-back, while cam0 keeps
# running its single model.
#
# Layout of TPU slots:
#   slot 0 = cam0 single model (e.g. MSBlock)
#   slot 1 = cam1 model A      (e.g. C2f)
#   slot 2 = cam1 model B      (e.g. GELAN)
#
# Per-frame invocation logic in the driver (Python pre-pipeline):
# Since pipeline.calibrate currently invokes ONE slot per frame via
# the cam_id dispatcher, the "two interpretations on cam1" part can't
# be free; we need to run two passes over the same frame.  The
# simplest approach is a HOST-DRIVEN per-frame loop instead of
# pipeline.calibrate: grab a frame, feed it to slot N, invoke,
# optionally invoke a second slot on the same frame.  This loses
# the V22 PrepTask/InferTask parallelism but exposes the cost of
# the dual-invoke directly.
#
# Self-contained per agent.md §5.1.2 + §5.1.5.  Persistent flash
# REQUIRED (driver uses sys.reset() between sweeps).
import sentai
sentai.verbose(1)

CAM0_MODEL = ("MSBlock", "/models/iarna_p3p4_MSBlock_1ep_uint8_480x640_int8_edgetpu.tflite")
CAM1_MODEL_A = ("C2f",   "/models/iarna_p3p4_C2f_1ep_uint8_480x640_int8_edgetpu.tflite")
CAM1_MODEL_B = ("GELAN", "/models/iarna_p3p4_GELAN_1ep_uint8_480x640_int8_edgetpu.tflite")

FPS = 30        # VGA30 per the request
RATIO_A = 2     # cam0 every 2 frames
RATIO_B = 1     # cam1 every 3rd frame
N_FRAMES = 100  # total grabs


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


sess = _sd("twins3")
csv_path = sess + "/results.csv"
sentai.fs.write(csv_path,
    "label,frames,cam0,cam1,slot0_inv,slot1_inv,slot2_inv,"
    "wall_ms,invokes_total,avg_invoke_ms,fps_x100\r\n")
print("=== session:", sess, " VGA%d alt %d:%d ===" %
      (FPS, RATIO_A, RATIO_B))

# Stop pipeline.
try:
    if sentai.pipeline.running(): sentai.pipeline.stop()
except Exception: pass

# Load all 3 slots up-front (~1.2 sec total).
print("\n--- loading 3 slots ---")
t0 = sentai.rtos.ticks_ms()
sentai.tpu.load_slot(0, CAM0_MODEL[1])
print("  slot 0 %s in %d ms" % (CAM0_MODEL[0], sentai.rtos.ticks_ms() - t0))
t0 = sentai.rtos.ticks_ms()
sentai.tpu.load_slot(1, CAM1_MODEL_A[1])
print("  slot 1 %s in %d ms" % (CAM1_MODEL_A[0], sentai.rtos.ticks_ms() - t0))
t0 = sentai.rtos.ticks_ms()
sentai.tpu.load_slot(2, CAM1_MODEL_B[1])
print("  slot 2 %s in %d ms" % (CAM1_MODEL_B[0], sentai.rtos.ticks_ms() - t0))

if not (sentai.tpu.slot_ready(0) and sentai.tpu.slot_ready(1)
        and sentai.tpu.slot_ready(2)):
    print("FAIL: slots not all ready")
    print("=== done ==="); raise SystemExit

# Camera up at VGA30.
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


# ==========================================================================
# We can't reuse pipeline.calibrate because it invokes exactly one slot
# per frame via the cam_id dispatcher.  For the "two-models-on-cam1"
# scenario we do two calibrate runs and let the user compare:
#
#   Sweep A (BASELINE):  cam0->slot0, cam1->slot1.  alt 2:1.  Pipeline-
#                        scheduled, V22 fast path, full PrepTask/InferTask
#                        parallelism.  Single invoke per cam1 frame.
#   Sweep B (DUAL):      cam0->slot0, cam1->slot1.  alt 2:1.  After EACH
#                        cam1 frame's first invoke (slot 1), the driver
#                        immediately calls invoke_slot(2) on the same
#                        input buffer.  This is OUTSIDE pipeline.calibrate
#                        — done as a top-level per-frame loop using
#                        camera.grabbed_id() + tpu.invoke_slot* directly.
#
# Sweep B can't go through pipeline.calibrate (which doesn't have a
# "post-invoke hook"), so the driver runs its own grab loop.  This loses
# the parallelism advantage but isolates the cost of the dual invoke
# pattern: cam0 invokes at slot 0, cam1 invokes at slot 1 + slot 2.
# ==========================================================================


def _sweep_baseline():
    print("\n--- Sweep A: BASELINE (cam1 = 1 model, alt %d:%d) ---" %
          (RATIO_A, RATIO_B))
    # Use pipeline.calibrate with cam0->slot0, cam1->slot1.
    sentai.pipeline.set_slot_for_cam(0, 0)
    sentai.pipeline.set_slot_for_cam(1, 1)
    sentai.pipeline.slot_stats_reset()
    t0 = sentai.rtos.ticks_ms()
    res = sentai.pipeline.calibrate(CAM0_MODEL[1], N_FRAMES, 8000)
    wall = sentai.rtos.ticks_ms() - t0
    n = res.get("frames", 0) or 1
    cam0 = res.get("cam0", 0); cam1 = res.get("cam1", 0)
    ia = res.get("invoke_ms_sum", 0) // n
    f100 = res.get("fps_x100", 0)
    slots = sentai.pipeline.slot_stats()
    total_inv = slots[0] + slots[1] + slots[2]
    avg_ms = (ia * n) // total_inv if total_inv else 0
    print("  frames=%d cam0=%d cam1=%d  slot0=%d slot1=%d slot2=%d  "
          "invoke_avg=%d  wall=%d ms  fps=%d.%02d" %
          (n, cam0, cam1, slots[0], slots[1], slots[2], ia, wall,
           f100 // 100, f100 % 100))
    sentai.fs.append(csv_path,
        "baseline,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n" %
        (n, cam0, cam1, slots[0], slots[1], slots[2],
         wall, total_inv, avg_ms, f100))
    try: sentai.pipeline.stop()
    except Exception: pass


def _sweep_dual_offline():
    """Pure-TPU offline measurement of the alt 2:1 invoke pattern.

    No pipeline, no camera grabs.  Just sequential `invoke_slot(N)`
    calls in the cadence the production pipeline would produce:
    - Pattern SINGLE: per cycle of 3 frames, invoke slot0 twice + slot1 once.
    - Pattern DUAL:   per cycle of 3 frames, invoke slot0 twice + slot1 once
                      + slot2 once (on the same conceptual cam1 frame).

    Both patterns share the on-chip parameter cache state with what the
    pipeline would have, so per-invoke cost matches reality.  Wall time
    of the loop is the reciprocal of the maximum FPS the pipeline could
    achieve at that workload (camera + scheduling overhead would only
    make real FPS lower).
    """
    print("\n--- Sweep B: PURE-TPU offline measurement of alt %d:%d cadence ---" %
          (RATIO_A, RATIO_B))
    # Pre-warm all slots so the on-chip parameter cache is populated.
    sentai.tpu.invoke_slot(0)
    sentai.tpu.invoke_slot(1)
    sentai.tpu.invoke_slot(2)
    sentai.diag.repl_kick()

    CYCLES = N_FRAMES // (RATIO_A + RATIO_B)   # 100 // 3 = 33 cycles ≈ 99 frames
    print("  cycles=%d total_frames=%d" % (CYCLES, CYCLES * (RATIO_A + RATIO_B)))

    # SINGLE pattern (baseline equivalent at TPU level).
    sum_ms = 0; n_inv = 0
    t0 = sentai.rtos.ticks_ms()
    for c in range(CYCLES):
        ms = sentai.tpu.invoke_slot(0); sum_ms += max(ms, 0); n_inv += 1
        ms = sentai.tpu.invoke_slot(0); sum_ms += max(ms, 0); n_inv += 1
        ms = sentai.tpu.invoke_slot(1); sum_ms += max(ms, 0); n_inv += 1
        if (c & 0x07) == 0: sentai.diag.repl_kick()
    wall_single = sentai.rtos.ticks_ms() - t0
    avg_single = sum_ms // n_inv
    fps_single = CYCLES * (RATIO_A + RATIO_B) * 100000 // wall_single
    print("  SINGLE pattern: %d invokes, wall=%d ms, avg=%d ms/invoke, "
          "implied FPS=%d.%02d" %
          (n_inv, wall_single, avg_single, fps_single // 100, fps_single % 100))
    sentai.fs.append(csv_path,
        "tpu_single,%d,0,0,%d,%d,0,%d,%d,%d,%d\r\n" %
        (CYCLES * (RATIO_A + RATIO_B), 2 * CYCLES, CYCLES,
         wall_single, n_inv, avg_single, fps_single))

    # DUAL pattern (slot 2 added on every cam1 cycle).
    sum_ms = 0; n_inv = 0
    t0 = sentai.rtos.ticks_ms()
    for c in range(CYCLES):
        ms = sentai.tpu.invoke_slot(0); sum_ms += max(ms, 0); n_inv += 1
        ms = sentai.tpu.invoke_slot(0); sum_ms += max(ms, 0); n_inv += 1
        ms = sentai.tpu.invoke_slot(1); sum_ms += max(ms, 0); n_inv += 1
        ms = sentai.tpu.invoke_slot(2); sum_ms += max(ms, 0); n_inv += 1
        if (c & 0x07) == 0: sentai.diag.repl_kick()
    wall_dual = sentai.rtos.ticks_ms() - t0
    avg_dual = sum_ms // n_inv
    fps_dual = CYCLES * (RATIO_A + RATIO_B) * 100000 // wall_dual
    print("  DUAL pattern  : %d invokes, wall=%d ms, avg=%d ms/invoke, "
          "implied FPS=%d.%02d" %
          (n_inv, wall_dual, avg_dual, fps_dual // 100, fps_dual % 100))
    sentai.fs.append(csv_path,
        "tpu_dual,%d,0,0,%d,%d,%d,%d,%d,%d,%d\r\n" %
        (CYCLES * (RATIO_A + RATIO_B), 2 * CYCLES, CYCLES, CYCLES,
         wall_dual, n_inv, avg_dual, fps_dual))

    # Cost summary.
    extra_ms = wall_dual - wall_single
    extra_per_cycle = extra_ms // CYCLES
    print("  → DUAL adds %d ms total over %d cycles = %d ms / cam1 frame "
          "(slot2 invoke cost on cache-hot path)" %
          (extra_ms, CYCLES, extra_per_cycle))


_sweep_baseline()
_sweep_dual_offline()

print("\nCSV:", csv_path)
print("=== done ===")
