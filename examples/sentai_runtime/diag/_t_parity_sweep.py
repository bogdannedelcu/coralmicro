# _t_parity_sweep.py — sweep loop_delay until cam0/cam1 parity breaks.
# Goal: prove the alternation / parity tracking is real by finding the
# delay range where the natural 50/50 cadence collapses.
import sentai
sentai.verbose(1)

print("=== boot ===")
print("version=", sentai.version())

sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
sentai.camera.init(1)
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(500)

# Sweep delays in steps tight enough to spot the transition.
# At VGA45 a sensor period is ~22 ms; queue/drain dynamics react
# around multiples and fractions of that.
DELAYS = [0, 2, 5, 8, 11, 14, 17, 20, 22, 25, 28, 33, 40, 50, 66, 80]
N = 60  # frames per measurement -- enough to see clear bias

print("--- parity OFF sweep ---")
sentai.pipeline.force_parity(0)
print("  delay  cam0  cam1  unk  fps     bias%")
for d in DELAYS:
    r = sentai.pipeline.calibrate(None, N, 2000, d)
    tot = r['cam0'] + r['cam1']
    if tot == 0:
        bias = 0
    else:
        bias = (100 * (r['cam0'] - r['cam1'])) // tot
    print("  %3d    %3d   %3d   %3d  %5d.%02d  %+d" %
          (d, r['cam0'], r['cam1'], r['unknown'],
           r['fps_x100'] // 100, r['fps_x100'] % 100, bias))

# Repeat with parity ON: should clamp the bias toward 0 (cam0/cam1
# balanced) at the cost of bumping skipped/timeout counters.
print("--- parity ON sweep ---")
sentai.pipeline.force_parity(1)
print("  delay  cam0  cam1  unk  fps     skip  to    bias%")
for d in DELAYS:
    sentai.pipeline.force_parity_reset()
    r = sentai.pipeline.calibrate(None, N, 2000, d)
    ps = sentai.pipeline.force_parity_stats()
    tot = r['cam0'] + r['cam1']
    bias = (100 * (r['cam0'] - r['cam1'])) // tot if tot else 0
    print("  %3d    %3d   %3d   %3d  %5d.%02d  %4d  %4d  %+d" %
          (d, r['cam0'], r['cam1'], r['unknown'],
           r['fps_x100'] // 100, r['fps_x100'] % 100,
           ps['skipped'], ps['timeout'], bias))

sentai.pipeline.force_parity(0)
print("=== done ===")
