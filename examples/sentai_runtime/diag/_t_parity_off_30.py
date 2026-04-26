# Parity OFF only @ VGA30. Smaller N + fewer delays so the run fits
# inside one stable session (the previous all-in-one wedged after ~1
# calibrate at high delay).
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

DELAYS = [0, 11, 22, 33, 50]
N = 30

sentai.pipeline.force_parity(0)
print("--- parity OFF VGA30 ---")
print("  delay  cam0  cam1  fps     bias%")
for d in DELAYS:
    r = sentai.pipeline.calibrate(None, N, 2000, d)
    tot = r['cam0'] + r['cam1']
    bias = (100 * (r['cam0'] - r['cam1'])) // tot if tot else 0
    print("  %3d    %3d   %3d   %5d.%02d  %+d" %
          (d, r['cam0'], r['cam1'],
           r['fps_x100'] // 100, r['fps_x100'] % 100, bias))
    sentai.rtos.sleep_ms(500)
print("=== done ===")
