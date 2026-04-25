import sentai
sentai.verbose(0)
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
res = []

def run_matrix(tag):
    sentai.camera.ratio(1, 1)
    for pf, ipf in [(0,1), (30,1), (30,2), (15,4)]:
        sentai.pipeline.prep_fps(pf)
        sentai.pipeline.invokes_per_frame(ipf)
        sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
        sentai.pipeline.start(0.25, 0.45, 50)
        sentai.rtos.sleep_ms(3000)
        s = sentai.pipeline.infer_stats()
        sentai.pipeline.stop()
        sentai.rtos.sleep_ms(300)
        res.append('%s pf=%d ipf=%d fps=%.1f fail=%d' % (tag, pf, ipf, s['ok']/3, s['fail']))
    sentai.camera.ratio(0, 0)
    sentai.pipeline.prep_fps(0); sentai.pipeline.invokes_per_frame(1)

def reinit(w, h, fps):
    if sentai.pipeline.running(): sentai.pipeline.stop()
    sentai.camera.stop()
    sentai.rtos.sleep_ms(400)
    rc = sentai.camera.set_hw(w, h, fps)
    res.append('set_hw(%d,%d,%d)=%d' % (w, h, fps, rc))
    if rc != 0: return False
    sentai.camera.init(1)
    sentai.rtos.sleep_ms(500)
    hw = sentai.camera.hw_config()
    fc = sentai.camera.frame_count()
    res.append('after_init hw=%s fc=%d' % (hw, fc))
    for _ in range(3): sentai.tpu.invoke()
    return True

# VGA 45 (default)
sentai.camera.init(1)
sentai.rtos.sleep_ms(500)
for _ in range(3): sentai.tpu.invoke()
res.append('boot hw=%s' % (sentai.camera.hw_config(),))
run_matrix('V45')

# VGA 30
if reinit(640, 480, 30):
    run_matrix('V30')

# VGA 15
if reinit(640, 480, 15):
    run_matrix('V15')

sentai.fs.write('/m_all.txt', '\n'.join(res) + '\n')
print('=== DONE ===')
