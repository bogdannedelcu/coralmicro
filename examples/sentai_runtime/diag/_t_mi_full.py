# Full A/B test for multi_invoke modes at SXGA30 alt 1:1.
# Single-shot: load + setup + run all configs.
import sentai
sentai.verbose(1)  # per agent.md rule 8: verbose(0) silences Python prints

print("=== boot ===")
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(200)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def run(label, n, mode, ra, rb, drain, dur):
    print("--- %s ---" % label)
    sentai.pipeline.invokes_per_frame(n)
    sentai.pipeline.multi_invoke_mode(mode)
    sentai.camera.switch_drain(drain)
    sentai.camera.ratio(ra, rb)
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    fc0 = sentai.camera.frame_count()
    t0 = sentai.rtos.ticks_ms()
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    if rc != 0:
        print("  start FAIL rc=%s" % rc); return
    sentai.rtos.sleep_ms(dur)
    fc1 = sentai.camera.frame_count()
    t1 = sentai.rtos.ticks_ms()
    istats = sentai.pipeline.infer_stats()
    pstats = sentai.pipeline.prep_stats()
    sentai.pipeline.stop()
    dt = t1 - t0
    print("  prep=%d/%.1f cam=%d/%.1f infer ok=%d (%.1f/s) fail=%d rc=%d" % (
        pstats['frames'], 1000.0*pstats['frames']/dt,
        fc1-fc0, 2000.0*(fc1-fc0)/dt,
        istats['ok'], 1000.0*istats['ok']/dt,
        istats['fail'], istats['last_rc']))
    if istats['ok']: print("  avg=%dms" % (istats['ms_sum']//istats['ok']))
    sentai.rtos.sleep_ms(400)

DUR=5000
run("ref n=1 single",       1, 0, 0, 0, 1, DUR)
run("ref n=1 alt 1:1",      1, 0, 1, 1, 1, DUR)
run("A n=2 alt 1:1 DEFER",  2, 1, 1, 1, 1, DUR)
run("B n=2 alt 1:1 REARM",  2, 2, 1, 1, 1, DUR)

sentai.pipeline.invokes_per_frame(1); sentai.pipeline.multi_invoke_mode(0)
sentai.camera.ratio(0,0); sentai.camera.select(0); sentai.camera.switch_drain(1)
print("=== done ===")
