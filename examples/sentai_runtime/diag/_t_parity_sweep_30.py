# _t_parity_sweep_30.py — same as _t_parity_sweep but at VGA30 sensor.
# Sensor period 33.3 ms vs 22.2 ms at VGA45 — collapse threshold should
# shift higher.  Combined parity-OFF + parity-ON in one pass to keep
# total wall time inside one REPL window.
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

DELAYS = [0, 5, 11, 17, 22, 28, 33, 40, 50, 66]
N = 50

print("--- parity OFF sweep (VGA30) ---")
sentai.pipeline.force_parity(0)
print("  delay  cam0  cam1  fps     bias%")
for d in DELAYS:
    r = sentai.pipeline.calibrate(None, N, 2000, d)
    tot = r['cam0'] + r['cam1']
    bias = (100 * (r['cam0'] - r['cam1'])) // tot if tot else 0
    print("  %3d    %3d   %3d   %5d.%02d  %+d" %
          (d, r['cam0'], r['cam1'],
           r['fps_x100'] // 100, r['fps_x100'] % 100, bias))

print("--- parity ON sweep (VGA30) ---")
sentai.pipeline.force_parity(1)
print("  delay  cam0  cam1  fps     skip  to    bias%")
for d in DELAYS:
    sentai.pipeline.force_parity_reset()
    r = sentai.pipeline.calibrate(None, N, 2000, d)
    ps = sentai.pipeline.force_parity_stats()
    tot = r['cam0'] + r['cam1']
    bias = (100 * (r['cam0'] - r['cam1'])) // tot if tot else 0
    print("  %3d    %3d   %3d   %5d.%02d  %4d  %4d  %+d" %
          (d, r['cam0'], r['cam1'],
           r['fps_x100'] // 100, r['fps_x100'] % 100,
           ps['skipped'], ps['timeout'], bias))

sentai.pipeline.force_parity(0)
print("=== done ===")
