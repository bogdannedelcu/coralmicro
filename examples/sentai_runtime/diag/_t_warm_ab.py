# A/B: cold-start vs warmup + V22-style pipeline.start args
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"
sentai.tpu.load(MODEL)
sentai.camera.init()

print("=== TEST A: cold start, no to_tensor / no warmup, my pipeline.start(0.5) ===")
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
sentai.pipeline.start(0.5)
sentai.rtos.sleep_ms(5000)
ia = sentai.pipeline.infer_stats()
sentai.pipeline.stop()
print("COLD:", ia['ok'], "invokes,", ia['fail'], "fails, avg", ia['ms_sum']/max(ia['ok'],1), "ms,", ia['ok']/5.0, "FPS")

# Drain TPU state
sentai.rtos.sleep_ms(500)

print("=== TEST B: V22 driver style — to_tensor + warmup + start(0.25, 0.45, 50) ===")
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(100)
for i in range(3): sentai.tpu.invoke()
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(5000)
ib = sentai.pipeline.infer_stats()
sentai.pipeline.stop()
print("WARM:", ib['ok'], "invokes,", ib['fail'], "fails, avg", ib['ms_sum']/max(ib['ok'],1), "ms,", ib['ok']/5.0, "FPS")

print("=== done ===")
