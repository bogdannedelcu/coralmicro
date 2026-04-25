# Runs a 4-point pipeline matrix at the CURRENT camera config.
# Call from REPL after init.
import sentai
sentai.verbose(0)
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
res = []
hw = sentai.camera.hw_config()
fc0 = sentai.camera.frame_count()
sentai.rtos.sleep_ms(1000)
fc1 = sentai.camera.frame_count()
cam_rate = fc1 - fc0
res.append('hw=%s cam_rate=%d/s' % (hw, cam_rate))
for _ in range(3): sentai.tpu.invoke()

sentai.camera.ratio(0, 0)  # single camera, no alt
for pf, ipf in [(0,1), (30,1), (15,2), (15,4)]:
    sentai.pipeline.prep_fps(pf); sentai.pipeline.invokes_per_frame(ipf)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    sentai.pipeline.start(0.25, 0.45, 50)
    sentai.rtos.sleep_ms(3000)
    s = sentai.pipeline.infer_stats()
    p = sentai.pipeline.prep_stats()
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(300)
    res.append('pf=%d ipf=%d cam=%.1f prep=%.1f inf=%.1f fail=%d inv_ms=%d pxp_ms=%d' % (
        pf, ipf, cam_rate,
        p.get('frames',0)/3,
        s['ok']/3,
        s['fail'],
        s['ms_sum']//max(s['ok'],1),
        p.get('pxp_ms_sum',0)//max(p.get('frames',1),1)))
sentai.pipeline.prep_fps(0); sentai.pipeline.invokes_per_frame(1)
sentai.fs.write('/matrix.txt', '\n'.join(res) + '\n')
print('=== DONE ===')
