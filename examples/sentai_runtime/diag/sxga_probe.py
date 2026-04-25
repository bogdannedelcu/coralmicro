import sentai
sentai.verbose(0)
res = []
res.append('hw_before=%s' % (sentai.camera.hw_config(),))

# Try SXGA @ 15 first (safer)
rc = sentai.camera.init(1, 1280, 960, 15)
res.append('init(1280,960,15)=%d' % rc)
sentai.rtos.sleep_ms(800)
res.append('hw_after=%s fc=%d' % (sentai.camera.hw_config(), sentai.camera.frame_count()))

# Measure CSI frame rate to confirm it streams
fc0 = sentai.camera.frame_count()
sentai.rtos.sleep_ms(1000)
fc1 = sentai.camera.frame_count()
res.append('CSI frames in 1s: %d' % (fc1 - fc0))

# Quick pure TPU sanity at SXGA (camera on)
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
for _ in range(3): sentai.tpu.invoke()
import time
t0 = sentai.rtos.ticks_ms()
for _ in range(30): sentai.tpu.invoke()
t1 = sentai.rtos.ticks_ms()
res.append('pure_invoke@SXGA15 fps=%.1f avg_ms=%.2f' % (30000/(t1-t0), (t1-t0)/30))

sentai.fs.write('/sxga_probe.txt', '\n'.join(res) + '\n')
print('=== DONE ===')
