# Continuous 1:1 camera alternation during pipeline
# Uses sentai.camera.ratio(1,1) to let the CSI ISR auto-flip the MUX.
# Measures total pipeline FPS + per-camera frame split.
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

print("=== boot ===")
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def ms(): return sentai.rtos.ticks_ms()

def run(label, ratio_a, ratio_b, duration_ms):
    print("--- %s: ratio=(%d,%d) ---" % (label, ratio_a, ratio_b))
    sentai.camera.ratio(ratio_a, ratio_b)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    fc0 = sentai.camera.frame_count()
    t0 = ms()
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    if rc != 0:
        print("  start FAIL rc=%s" % rc); sentai.camera.ratio(0,0); return
    sentai.rtos.sleep_ms(duration_ms)
    fc1 = sentai.camera.frame_count()
    t1 = ms()
    istats = sentai.pipeline.infer_stats()
    pstats = sentai.pipeline.prep_stats()
    sentai.pipeline.stop()
    dt = t1 - t0
    try:
        cs = sentai.diag.cam_stats()
    except:
        cs = {}
    print("  duration=%d ms" % dt)
    print("    camera frames    : %d  (%.1f FPS)" % (fc1-fc0, 1000*(fc1-fc0)/dt))
    print("    PrepTask         : %d  (%.1f FPS)" % (pstats['frames'], 1000*pstats['frames']/dt))
    print("    InferTask ok     : %d  (%.1f FPS)" % (istats['ok'], 1000*istats['ok']/dt))
    print("    InferTask fail   : %d" % istats['fail'])
    if istats['ok']:
        print("    avg invoke       : %d ms" % (istats['ms_sum']//istats['ok']))
    if cs:
        print("    cam_stats        : switch_ok_eof=%d fallback=%d drain_to=%d" % (
            cs.get('switch_ok_eof',0), cs.get('switch_fallback',0),
            cs.get('drain_timeout',0)))

# Baseline: single camera, no switching
run("baseline cam0 only", 0, 0, 5000)

# Continuous 1:1 alternation
run("alternating 1:1", 1, 1, 5000)

# 2:1 ratio (cam0 gets 2 frames for every 1 of cam1)
run("biased 2:1", 2, 1, 5000)

# Reset
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
print("=== done ===")
