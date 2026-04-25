import sentai
sentai.verbose(0)
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
sentai.camera.init(1, 640, 480, 45)
sentai.rtos.sleep_ms(1000)
for _ in range(3): sentai.tpu.invoke()
fc0 = sentai.camera.frame_count()
sentai.rtos.sleep_ms(1000)
cam_rate = sentai.camera.frame_count() - fc0
res = []
res.append('hw=%s cam_rate=%d/s' % (sentai.camera.hw_config(), cam_rate))

# Just ONE pipeline test
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(3000)
s = sentai.pipeline.infer_stats()
sentai.pipeline.stop()
sentai.rtos.sleep_ms(500)
res.append('V45 single: ok=%d fail=%d fps=%.1f inv_ms=%d' % (
    s['ok'], s['fail'], s['ok']/3, s['ms_sum']//max(s['ok'],1)))

sentai.fs.write('/v45.txt', '\n'.join(res) + '\n')
print('=== DONE ===')
