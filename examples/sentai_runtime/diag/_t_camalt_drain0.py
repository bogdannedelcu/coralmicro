# Test alt 1:1 with switch_drain=0 (bypass drain wait) to see if
# drain is actually necessary for correctness or just a false bottleneck.
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
    print("  duration=%d ms  cam=%d (%.1f tick/s)" % (dt, fc1-fc0, 1000.0*(fc1-fc0)/dt))
    print("  PrepTask=%d (%.1f FPS)" % (pstats['frames'], 1000.0*pstats['frames']/dt))
    print("  infer ok=%d (%.1f FPS)  fail=%d  avg_invoke=%d ms" %
          (istats['ok'], 1000.0*istats['ok']/dt, istats['fail'],
           istats['ms_sum']//max(istats['ok'],1)))
    sentai.rtos.sleep_ms(400)

# Compare drain=1 (default) vs drain=0 (bypass)
run("alt 1:1 drain=1", 1, 1, 1, 5000)
run("alt 1:1 drain=0", 1, 1, 0, 5000)
run("alt 1:1 drain=2", 1, 1, 2, 5000)

sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.switch_drain(1)
print("=== done ===")
