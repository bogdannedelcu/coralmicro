# Setup only: load model, init camera. Run once after fresh flash.
import sentai
sentai.verbose(1)
print("=== setup begin ===")
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(200)
for i in range(3): sentai.tpu.invoke()
print("=== setup done ===")
