# Verify new default (drain=1) delivers ~20 FPS per camera
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed, drain default =", sentai.camera.switch_drain())

def ms(): return sentai.rtos.ticks_ms()

# Baseline
print("--- baseline cam0 (no alternation) ---")
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
t0 = ms(); sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(5000)
istats = sentai.pipeline.infer_stats()
sentai.pipeline.stop(); dt = ms()-t0
print("  infer=%.1f FPS fail=%d" % (1000*istats['ok']/dt, istats['fail']))

# Alternating 1:1 with default (should be drain=1 now)
print("--- alt 1:1 with new default ---")
sentai.camera.ratio(1, 1)
sentai.rtos.sleep_ms(300)
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
t0 = ms(); sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(5000)
istats = sentai.pipeline.infer_stats()
sentai.pipeline.stop(); dt = ms()-t0
per_cam = 1000*istats['ok']/dt / 2.0
print("  total_infer=%.1f FPS per_cam=%.1f FPS fail=%d" % (
    1000*istats['ok']/dt, per_cam, istats['fail']))

sentai.camera.ratio(0, 0); sentai.camera.select(0)
print("=== done ===")
