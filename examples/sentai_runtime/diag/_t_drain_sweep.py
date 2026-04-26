# Sweep drain={0,1,2} at SXGA30 alt 1:1, measure pipeline FPS + invoke fail rate.
# Goal: see if drain=0 is safe (TPU still works) and how much faster.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(200)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def run(label, drain, n, mode, ratio, dur):
    sentai.pipeline.invokes_per_frame(n)
    sentai.pipeline.multi_invoke_mode(mode)
    rc_drain = sentai.camera.switch_drain(drain)
    sentai.camera.ratio(ratio[0], ratio[1])
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    fc0 = sentai.camera.frame_count()
    t0 = sentai.rtos.ticks_ms()
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    if rc != 0:
        print("--- %s START FAIL rc=%s drain_set=%s ---" % (label, rc, rc_drain)); return
    sentai.rtos.sleep_ms(dur)
    fc1 = sentai.camera.frame_count()
    t1 = sentai.rtos.ticks_ms()
    istats = sentai.pipeline.infer_stats()
    pstats = sentai.pipeline.prep_stats()
    sentai.pipeline.stop()
    dt = t1 - t0
    print("--- %s drain=%d (set rc=%s) ---" % (label, drain, rc_drain))
    print("  cam=%d ticks (~%.1f sensor FPS)  prep=%d (%.1f FPS)" % (
        fc1-fc0, 2000.0*(fc1-fc0)/dt, pstats['frames'], 1000.0*pstats['frames']/dt))
    print("  infer ok=%d (%.1f /s) fail=%d last_rc=%d" % (
        istats['ok'], 1000.0*istats['ok']/dt, istats['fail'], istats['last_rc']))
    if istats['ok']: print("  avg_invoke=%dms" % (istats['ms_sum']//istats['ok']))
    sentai.rtos.sleep_ms(400)

DUR=5000

# Baseline single-cam (no switch, drain irrelevant)
run("REF single n=1",   1, 1, 0, (0,0), DUR)

# Switch alt 1:1 with drain sweep, n=1 (single invoke per frame)
run("alt 1:1 n=1",      1, 1, 0, (1,1), DUR)
run("alt 1:1 n=1",      0, 1, 0, (1,1), DUR)  # NEW: drain=0
run("alt 1:1 n=1",      2, 1, 0, (1,1), DUR)

# Switch alt 1:1 with DEFER n=2
run("alt 1:1 n=2 DEFER",1, 2, 1, (1,1), DUR)
run("alt 1:1 n=2 DEFER",0, 2, 1, (1,1), DUR)  # NEW: drain=0 + DEFER n=2

sentai.pipeline.invokes_per_frame(1); sentai.pipeline.multi_invoke_mode(0)
sentai.camera.ratio(0,0); sentai.camera.select(0); sentai.camera.switch_drain(1)
print("=== done ===")
