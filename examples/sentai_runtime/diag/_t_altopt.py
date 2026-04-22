# Optimize 1:1 camera alternation — sweep drain thresholds
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

def run(label, ratio_a, ratio_b, drain, duration_ms):
    print("--- %s: ratio=(%d,%d) drain=%d ---" % (label, ratio_a, ratio_b, drain))
    sentai.camera.switch_drain(drain)
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
    print("  cam=%.1f prep=%.1f infer=%.1f fail=%d avg=%dms" % (
        1000*(fc1-fc0)/dt, 1000*pstats['frames']/dt, 1000*istats['ok']/dt,
        istats['fail'],
        istats['ms_sum']//max(istats['ok'],1)))

# Baseline - single camera reference
run("single cam0 baseline", 0, 0, 2, 4000)

# 1:1 alternation with various drain thresholds
run("alt 1:1 drain=2 (default)", 1, 1, 2, 4000)
run("alt 1:1 drain=1", 1, 1, 1, 4000)
# Note: 0 is clamped to 1 by the setter; smallest effective is 1

# Also test single-cam with drain=1 (no switches but verify no regression)
run("single cam0 drain=1", 0, 0, 1, 4000)

sentai.camera.ratio(0,0); sentai.camera.select(0); sentai.camera.switch_drain(2)
print("=== done ===")
