import sentai
sentai.verbose(0)
results = []
def log(m): results.append(str(m))

sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
log("hw_default=" + str(sentai.camera.hw_config()))

def bench(tag, prep_fps, ipf, dur=3000):
    try:
        sentai.pipeline.prep_fps(prep_fps)
        sentai.pipeline.invokes_per_frame(ipf)
        sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
        sentai.pipeline.start(0.25, 0.45, 50)
        sentai.rtos.sleep_ms(dur)
        s = sentai.pipeline.infer_stats()
        sentai.pipeline.stop()
        sentai.rtos.sleep_ms(400)
        log("%s pf=%d ipf=%d  infFPS=%.1f fail=%d inv_ms=%d" % (
            tag, prep_fps, ipf, s['ok']/(dur/1000), s['fail'],
            s['ms_sum']//max(s['ok'],1)))
    except Exception as e:
        log("%s EXC=%s" % (tag, e))
        try: sentai.pipeline.stop()
        except: pass

def run_all(tag_prefix):
    for pf, ipf in [(0,1), (0,2), (30,1), (30,2), (15,1), (15,4)]:
        bench(tag_prefix, pf, ipf)

def reinit_at(w, h, fps):
    try:
        if sentai.pipeline.running(): sentai.pipeline.stop()
        sentai.camera.stop()
        sentai.rtos.sleep_ms(300)
        rc = sentai.camera.set_hw(w, h, fps)
        log("set_hw(%d,%d,%d)=%d" % (w, h, fps, rc))
        if rc != 0: return False
        sentai.rtos.sleep_ms(200)
        sentai.camera.init(1)
        sentai.rtos.sleep_ms(500)
        log("after_init hw=" + str(sentai.camera.hw_config()) + " fc=" + str(sentai.camera.frame_count()))
        for _ in range(3): sentai.tpu.invoke()
        return True
    except Exception as e:
        log("reinit EXC=%s" % e); return False

# Test ALT 1:1 at each FPS (resolution stays VGA)
sentai.camera.init(1)
sentai.rtos.sleep_ms(500)
for _ in range(3): sentai.tpu.invoke()
log("warmed @ default VGA45")
sentai.camera.ratio(1, 1)

log("=== alt11 VGA@45 ===")
run_all("V45")

# Reinit to VGA@30
if reinit_at(640, 480, 30):
    sentai.camera.ratio(1, 1)
    log("=== alt11 VGA@30 ===")
    run_all("V30")

# Reinit to VGA@15
if reinit_at(640, 480, 15):
    sentai.camera.ratio(1, 1)
    log("=== alt11 VGA@15 ===")
    run_all("V15")

# Cleanup
sentai.camera.ratio(0, 0)
sentai.pipeline.prep_fps(0)
sentai.pipeline.invokes_per_frame(1)

sentai.fs.write('/alt_fps.txt', "\n".join(results) + "\n")
print("=== RESULTS ===")
for r in results: print(r)
print("=== END ===")
