# Long-duration early_release stability test
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed")

sentai.pipeline.early_release(1)
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
sentai.pipeline.start(0.25, 0.45, 50)

# Sample every 2s for 20s total
for checkpoint in range(10):
    sentai.rtos.sleep_ms(2000)
    i = sentai.pipeline.infer_stats()
    p = sentai.pipeline.prep_stats()
    print("t=%d s: infer ok=%d fail=%d | prep=%d" % (
        (checkpoint+1)*2, i['ok'], i['fail'], p['frames']))

sentai.pipeline.stop()
sentai.pipeline.early_release(0)
print("=== done ===")
