# _t_parity_on.py — does force_parity rescue alternation when natural
# cadence collapses?  Tests at the delays where parity-OFF collapsed
# (14, 17, 20, 22 ms) — should see skipped > 0 and bias closer to 0.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
sentai.camera.init(1)
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(500)

DELAYS = [0, 8, 14, 17, 20, 22]
N = 60

sentai.pipeline.force_parity(1)
print("--- parity ON ---")
print("  delay  cam0  cam1  fps     skipped  timeout  bias%")
for d in DELAYS:
    sentai.pipeline.force_parity_reset()
    r = sentai.pipeline.calibrate(None, N, 2000, d)
    ps = sentai.pipeline.force_parity_stats()
    tot = r['cam0'] + r['cam1']
    bias = (100 * (r['cam0'] - r['cam1'])) // tot if tot else 0
    print("  %3d    %3d   %3d   %5d.%02d   %5d   %5d   %+d" %
          (d, r['cam0'], r['cam1'],
           r['fps_x100'] // 100, r['fps_x100'] % 100,
           ps['skipped'], ps['timeout'], bias))

sentai.pipeline.force_parity(0)
print("=== done ===")
