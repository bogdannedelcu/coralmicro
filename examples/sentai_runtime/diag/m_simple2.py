import sentai
sentai.verbose(0)
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
sentai.camera.init(1)
sentai.rtos.sleep_ms(500)
for _ in range(3): sentai.tpu.invoke()

# V45 quick
sentai.camera.ratio(1,1)
sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(3000)
s1 = sentai.pipeline.infer_stats()
sentai.pipeline.stop()
sentai.rtos.sleep_ms(300)

# Reinit to V30
sentai.camera.ratio(0,0)
sentai.camera.stop()
sentai.rtos.sleep_ms(400)
rc = sentai.camera.set_hw(640, 480, 30)
sentai.camera.init(1)
sentai.rtos.sleep_ms(500)
hw = sentai.camera.hw_config()
fc = sentai.camera.frame_count()
for _ in range(3): sentai.tpu.invoke()

# V30 quick
sentai.camera.ratio(1,1)
sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(3000)
s2 = sentai.pipeline.infer_stats()
sentai.pipeline.stop()
sentai.camera.ratio(0,0)

out = 'V45: ok=%d fps=%.1f\nV30(rc=%d, hw=%s, fc=%d): ok=%d fps=%.1f\n' % (
    s1['ok'], s1['ok']/3, rc, hw, fc, s2['ok'], s2['ok']/3)
sentai.fs.write('/m_s2.txt', out)
print('=== DONE ===')
