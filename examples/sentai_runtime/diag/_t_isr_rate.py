# Does CSI ISR fire 1x or 2x per sensor frame?
# Measures frame_count delta in multiple scenarios with fresh reference.
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

print("=== boot ===")
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.rtos.sleep_ms(1500)  # fully settle CSI
print("settled")

def sample(label, duration_ms, work_fn=None):
    c0 = sentai.camera.frame_count()
    t0 = sentai.rtos.ticks_ms()
    if work_fn is None:
        sentai.rtos.sleep_ms(duration_ms)
    else:
        while sentai.rtos.ticks_ms() - t0 < duration_ms:
            work_fn()
    c1 = sentai.camera.frame_count()
    dt = sentai.rtos.ticks_ms() - t0
    fps = 1000.0 * (c1 - c0) / dt
    print("  %-25s %d frames / %d ms = %.1f FPS" % (label, c1-c0, dt, fps))
    return fps

print("--- A: idle camera (no consumer) ---")
for i in range(3): sample("idle %d" % i, 2000)

print("--- B: with to_tensor drain ---")
for i in range(3): sample("to_tensor %d" % i, 2000, work_fn=sentai.camera.to_tensor)

print("--- C: during pipeline ---")
rc = sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(500)  # settle
for i in range(3): sample("pipeline %d" % i, 2000)
istats = sentai.pipeline.infer_stats()
pstats = sentai.pipeline.prep_stats()
print("  infer ok=%d fail=%d, prep=%d" % (
    istats['ok'], istats['fail'], pstats['frames']))
sentai.pipeline.stop()

print("=== done ===")
