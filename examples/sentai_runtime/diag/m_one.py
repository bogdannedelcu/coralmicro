# Matrix at whatever FPS camera was init'd at.  Single init per boot.
import sentai
sentai.verbose(0)
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
res = []
hw = sentai.camera.hw_config()
fc = sentai.camera.frame_count()
res.append('hw=%s fc=%d' % (hw, fc))

sentai.camera.ratio(1, 1)
for pf, ipf in [(0,1), (30,1), (30,2), (15,4)]:
    sentai.pipeline.prep_fps(pf); sentai.pipeline.invokes_per_frame(ipf)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    sentai.pipeline.start(0.25, 0.45, 50)
    sentai.rtos.sleep_ms(3000)
    s = sentai.pipeline.infer_stats()
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(400)
    res.append('pf=%d ipf=%d fps=%.1f fail=%d inv_ms=%d' %
               (pf, ipf, s['ok']/3, s['fail'], s['ms_sum']//max(s['ok'],1)))
sentai.camera.ratio(0, 0)
sentai.pipeline.prep_fps(0); sentai.pipeline.invokes_per_frame(1)
sentai.fs.write('/m_one.txt', '\n'.join(res) + '\n')
print('=== DONE ===')
