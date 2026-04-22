# Test serialize_prep toggle — isolate whether bus contention is the issue
import sentai, gc

sentai.verbose(1)

def _ms(): return sentai.rtos.ticks_ms()

def run_pipeline(label, serialize, direct):
    print("=== %s: serialize=%d direct=%d ===" % (label, serialize, direct))
    sentai.pipeline.prep_reset()
    sentai.pipeline.infer_reset()
    sentai.pipeline.debug_prep_mode(0)  # FULL prep
    sentai.pipeline.debug_no_invoke(0)  # REAL invoke
    sentai.pipeline.serialize_prep(serialize)
    sentai.pipeline.direct_tensor(direct)
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    print("  start rc=%s" % rc)
    sentai.rtos.sleep_ms(5000)
    istats = sentai.pipeline.infer_stats()
    pstats = sentai.pipeline.prep_stats()
    print("  @5s: infer ok=%d fail=%d rc=%d" % (
        istats['ok'], istats['fail'], istats['last_rc']))
    print("  @5s: prep frames=%d total_ms=%d" % (
        pstats['frames'], pstats['total_ms_sum']))
    if istats['ok'] > 0:
        avg_ms = istats['ms_sum'] / max(istats['ok'], 1)
        fps = (istats['ok'] * 1000) / 5000
        print("  avg_invoke=%.1f ms, infer FPS=%.1f" % (avg_ms, fps))
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(300)
    # Post-check: can standalone invoke still work?
    fails = 0
    for _ in range(5):
        ok = sentai.tpu.invoke()
        if ok < 0: fails += 1
    print("  POST 5 invokes: fails=%d" % fails)

print("=== boot ===")
sentai.tpu.load('/yolo26n.edgetpu_1.tflite')
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(100)
for i in range(3):
    sentai.tpu.invoke()
print("warmed")

# Baseline: full pipeline direct+parallel (expect fail)
run_pipeline("DIRECT-PARA",  serialize=0, direct=1)

# Legacy path (binary sem, 1-frame pipeline)
run_pipeline("LEGACY-PARA",  serialize=0, direct=0)

# Legacy path + serialize (true single-frame serialization)
run_pipeline("LEGACY-SERIAL", serialize=1, direct=0)

print("=== done ===")
