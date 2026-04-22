# Staged isolation — start from mock, add components one by one
import sentai, gc

sentai.verbose(1)

def _ms(): return sentai.rtos.ticks_ms()

def warmup():
    for i in range(3):
        sentai.tpu.invoke()

def test_stage(prep_mode, no_invoke, duration_ms, label):
    print("--- %s: prep_mode=%d no_invoke=%d ---" % (label, prep_mode, no_invoke))
    sentai.pipeline.prep_reset()
    sentai.pipeline.infer_reset()
    sentai.pipeline.debug_prep_mode(prep_mode)
    sentai.pipeline.debug_no_invoke(no_invoke)
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    if rc != 0:
        print("  start FAILED rc=%s" % rc)
        return False
    # Sample at 1/4, 1/2, 3/4, end
    for frac in (4, 2, 4, 1):
        sentai.rtos.sleep_ms(duration_ms // frac)
        istats = sentai.pipeline.infer_stats()
        pstats = sentai.pipeline.prep_stats()
        print("  @%d ms: infer ok=%d fail=%d rc=%d | prep frames=%d" % (
            _ms() & 0xFFFFFFF,
            istats['ok'], istats['fail'], istats['last_rc'],
            pstats['frames']))
    sentai.pipeline.stop()
    # Post-stop: can we still invoke standalone?
    sentai.rtos.sleep_ms(200)
    fails_post = 0
    t0 = _ms()
    for _ in range(5):
        ok = sentai.tpu.invoke()
        if not ok or ok < 0: fails_post += 1
    t1 = _ms()
    print("  POST-STOP 5 invokes: %d ms, fails=%d" % (t1 - t0, fails_post))
    return fails_post == 0

print("=== boot ===")
sentai.tpu.load('/yolo26n.edgetpu_1.tflite')
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(100)
warmup()
print("warmed")

# Stage S1: MOCK prep, no invoke.  Pure Task timing + sems.
test_stage(prep_mode=1, no_invoke=1, duration_ms=2000, label="S1-MOCK")

# Stage S2: +InferTask memcpy only (mock prep delivers zero buf).
# Skipped — in mock mode InferTask does no TPU work anyway; this is
# effectively the same as S1.

# Stage S3: CAM only prep + no invoke (test CSI DMA)
test_stage(prep_mode=2, no_invoke=1, duration_ms=2000, label="S3-CAM")

# Stage S4: CAM + PXP + no invoke (test PXP)
test_stage(prep_mode=3, no_invoke=1, duration_ms=2000, label="S4-PXP")

# Stage S5: FULL prep + no invoke (test quant)
test_stage(prep_mode=0, no_invoke=1, duration_ms=2000, label="S5-FULL-NOINVOKE")

# Stage S6: MOCK prep + REAL invoke — no camera/PXP/quant load
test_stage(prep_mode=1, no_invoke=0, duration_ms=2000, label="S6-MOCK-INVOKE")

# Stage S7: CAM prep + REAL invoke
test_stage(prep_mode=2, no_invoke=0, duration_ms=2000, label="S7-CAM-INVOKE")

# Stage S8: CAM+PXP prep + REAL invoke
test_stage(prep_mode=3, no_invoke=0, duration_ms=2000, label="S8-PXP-INVOKE")

# Stage S9: FULL prep + REAL invoke (the real pipeline)
test_stage(prep_mode=0, no_invoke=0, duration_ms=2000, label="S9-FULL-FULL")

print("=== done ===")
