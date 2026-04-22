# Test cam_skip_dcache hypothesis: does skipping 615KB dcache invalidate
# stop the TPU wedging in the full pipeline path?
import sentai, gc

sentai.verbose(1)

def run(label, skip_dcache):
    print("=== %s: skip_dcache=%d ===" % (label, skip_dcache))
    sentai.diag.cam_skip_dcache(skip_dcache)
    sentai.pipeline.prep_reset()
    sentai.pipeline.infer_reset()
    sentai.pipeline.debug_prep_mode(0)  # FULL
    sentai.pipeline.debug_no_invoke(0)  # real invoke
    sentai.pipeline.serialize_prep(0)   # parallel
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    print("  start rc=%s" % rc)
    sentai.rtos.sleep_ms(5000)
    istats = sentai.pipeline.infer_stats()
    pstats = sentai.pipeline.prep_stats()
    print("  @5s infer ok=%d fail=%d rc=%d | prep=%d" % (
        istats['ok'], istats['fail'], istats['last_rc'], pstats['frames']))
    if istats['ok']:
        avg = istats['ms_sum'] // istats['ok']
        fps = istats['ok'] / 5.0
        print("  avg=%d ms  infer_fps=%.1f" % (avg, fps))
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(300)
    fails = 0
    for _ in range(5):
        r = sentai.tpu.invoke()
        if r < 0: fails += 1
    print("  POST 5 invokes fails=%d" % fails)

print("=== boot ===")
sentai.tpu.load('/yolo26n.edgetpu_1.tflite')
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(100)
for i in range(3):
    sentai.tpu.invoke()
print("warmed")

# Baseline (current behavior)
run("WITH_DCACHE",    skip_dcache=0)
# Hypothesis test
run("SKIP_DCACHE",    skip_dcache=1)

print("=== done ===")
