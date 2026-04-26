# A/B test: multi_invoke_mode 0 vs 1 (DEFER) vs 2 (REARM) at SXGA30 alt 1:1.
# Goal: see how many invokes/sec we get when running 2 invokes per camera frame.
# At SXGA30 alt 1:1, frame period = drain(33ms) + grab+PXP(35ms) = ~68 ms.
# 2 invokes x 35ms each = 70 ms, fits inside the frame period.
import sentai
sentai.verbose(0)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

print("=== boot ===")
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def ms(): return sentai.rtos.ticks_ms()

def run_pipe(label, n_invokes, mode, ratio, drain, dur):
    print("--- %s: n=%d mode=%d ratio=%s drain=%d ---" %
          (label, n_invokes, mode, ratio, drain))
    sentai.pipeline.invokes_per_frame(n_invokes)
    sentai.pipeline.multi_invoke_mode(mode)
    sentai.camera.switch_drain(drain)
    sentai.camera.ratio(ratio[0], ratio[1])
    sentai.camera.select(0)
    sentai.rtos.sleep_ms(300)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    fc0 = sentai.camera.frame_count()
    t0 = ms()
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    if rc != 0:
        print("  start FAIL rc=%s" % rc); sentai.camera.ratio(0,0); return
    sentai.rtos.sleep_ms(dur)
    fc1 = sentai.camera.frame_count()
    t1 = ms()
    istats = sentai.pipeline.infer_stats()
    pstats = sentai.pipeline.prep_stats()
    sentai.pipeline.stop()
    dt = t1 - t0
    print("  duration   : %d ms" % dt)
    print("  cam ticks  : %d (~%.1f sensor FPS)" % (fc1-fc0, 2000.0*(fc1-fc0)/dt))
    print("  PrepTask   : %d frames (%.1f FPS)" % (pstats['frames'], 1000.0*pstats['frames']/dt))
    print("  infer ok   : %d (%.1f /s) fail=%d last_rc=%d" %
          (istats['ok'], 1000.0*istats['ok']/dt, istats['fail'], istats['last_rc']))
    if istats['ok']:
        print("  avg invoke : %d ms" % (istats['ms_sum']//istats['ok']))
    sentai.rtos.sleep_ms(400)

DUR = 5000

# === SXGA30 single-cam reference ===
run_pipe("ref n=1 mode=0 single",     1, 0, (0,0), 1, DUR)

# === SXGA30 alt 1:1 baseline (n=1) ===
run_pipe("alt n=1 mode=0",            1, 0, (1,1), 1, DUR)

# === SXGA30 alt 1:1 with n=2 — three sync modes ===
run_pipe("alt n=2 mode=0 LEGACY",     2, 0, (1,1), 1, DUR)
run_pipe("alt n=2 mode=1 DEFER",      2, 1, (1,1), 1, DUR)
run_pipe("alt n=2 mode=2 REARM",      2, 2, (1,1), 1, DUR)

# Reset to defaults
sentai.pipeline.invokes_per_frame(1)
sentai.pipeline.multi_invoke_mode(0)
sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.switch_drain(1)
print("=== done ===")
