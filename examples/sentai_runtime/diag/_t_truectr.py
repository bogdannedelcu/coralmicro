# Compare frame_count (ISR-entry) vs true_frame_count (DMA-done-gated)
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.rtos.sleep_ms(1500)
print("settled")

def sample(label, duration_ms):
    f0 = sentai.camera.frame_count()
    t0 = sentai.camera.true_frame_count()
    sentai.rtos.sleep_ms(duration_ms)
    f1 = sentai.camera.frame_count()
    t1 = sentai.camera.true_frame_count()
    print("  %-20s ISR=%d TRUE=%d ratio=%.2f" % (
        label, f1-f0, t1-t0, (f1-f0)/max(t1-t0, 1)))

print("--- idle ---")
for i in range(3): sample("idle %d" % i, 2000)

print("--- pipeline ---")
rc = sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(500)
for i in range(3): sample("pipeline %d" % i, 2000)
sentai.pipeline.stop()

print("=== done ===")
