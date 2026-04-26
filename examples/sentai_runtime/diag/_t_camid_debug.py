# Debug: query both current_id AND grabbed_id continuously while pipeline runs.
# Goal: see if MUX is actually flipping during alt mode.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()

# Check 1: single cam (no alt) - cam_id should be stable
print("--- single cam, no alt, ratio(0,0) ---")
sentai.camera.ratio(0, 0)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(800)
for i in range(5):
    sentai.camera.save_jpeg("/diag/cs/dbg.jpg", 30)  # triggers grab
    print("  i=%d cur=%d grabbed=%d" % (i, sentai.camera.current_id(), sentai.camera.grabbed_id()))
    sentai.rtos.sleep_ms(200)

# Check 2: alt 1:1 - should see alternation
print("--- alt 1:1, raw grab loop, no pipeline ---")
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(1)
sentai.rtos.sleep_ms(500)
for i in range(10):
    sentai.camera.save_jpeg("/diag/cs/dbg.jpg", 30)
    print("  i=%d cur=%d grabbed=%d" % (i, sentai.camera.current_id(), sentai.camera.grabbed_id()))
    sentai.rtos.sleep_ms(80)

sentai.camera.ratio(0, 0)
sentai.camera.select(0)
print("=== done ===")
