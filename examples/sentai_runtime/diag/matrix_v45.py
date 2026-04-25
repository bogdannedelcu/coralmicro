import sentai
sentai.verbose(0)
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
sentai.camera.init(1, 640, 480, 45)
sentai.rtos.sleep_ms(800)
for _ in range(3): sentai.tpu.invoke()
fc0 = sentai.camera.frame_count(); sentai.rtos.sleep_ms(1000); cam_rate = sentai.camera.frame_count()-fc0
res = ['hw=%s cam_rate=%d/s' % (sentai.camera.hw_config(), cam_rate)]

for pf, ipf in [(0,1), (30,1), (15,2), (15,4)]:
    sentai.pipeline.prep_fps(pf); sentai.pipeline.invokes_per_frame(ipf)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    sentai.pipeline.start(0.25, 0.45, 50)
    sentai.rtos.sleep_ms(3000)
    s = sentai.pipeline.infer_stats()
    p = sentai.pipeline.prep_stats()
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(400)
    res.append('V45 pf=%d ipf=%d inf=%.1f prep=%.1f fail=%d ms=%d' % (
        pf, ipf, s['ok']/3, p.get('frames',0)/3, s['fail'],
        s['ms_sum']//max(s['ok'],1)))
sentai.pipeline.prep_fps(0); sentai.pipeline.invokes_per_frame(1)
sentai.fs.write('/m_v45.txt', '\n'.join(res)+'\n')
print('=== DONE ===')
