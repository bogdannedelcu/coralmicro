# Measure the real impact of camera-switch drain threshold on pipeline FPS
# when running with 1:1 camera alternation.
#
# Key question: counter `g_camera_frame_seq` ticks on FB2-done only → every
# 2 sensor frames (per camera_support.c:147).  So threshold=1 waits for
# ~44 ms (2 sensor frames at 45 FPS) before accepting post-switch frames.
# If a smaller wait produces correct images, pipeline FPS should improve.
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

def run_pipe(label, ratio_a, ratio_b, drain, duration_ms):
    print("--- %s: ratio=(%d,%d) drain=%d ---" % (label, ratio_a, ratio_b, drain))
    # Set switch-drain threshold BEFORE start
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
    cam_ticks = fc1-fc0
    # Counter ticks every 2 sensor frames → real sensor FPS = ticks*2
    cam_fps = 1000 * cam_ticks / dt
    sensor_fps = cam_fps * 2  # accounting for FB2 gate
    print("  duration     : %d ms" % dt)
    print("  cam counter  : %d ticks  (%.1f tick/s, ~%.1f sensor FPS)" %
          (cam_ticks, cam_fps, sensor_fps))
    print("  PrepTask     : %d frames (%.1f FPS)" % (pstats['frames'], 1000*pstats['frames']/dt))
    print("  infer ok     : %d (%.1f FPS) fail=%d" %
          (istats['ok'], 1000*istats['ok']/dt, istats['fail']))
    if istats['ok']:
        print("  avg invoke   : %d ms" % (istats['ms_sum']//istats['ok']))
    sentai.rtos.sleep_ms(400)

# Single camera baseline (no switching) — reference
run_pipe("cam0 only baseline",  0, 0, 1, 5000)

# Alternating 1:1 with different drain thresholds
run_pipe("alt 1:1 drain=1 (default)", 1, 1, 1, 5000)
run_pipe("alt 1:1 drain=2 (1 full tick = 2 sensor frames extra)", 1, 1, 2, 5000)
# NB: drain_set() has min=1 in C so drain=0 is not testable without C change;
# what we'd WANT to test is "wait for 1 sensor frame only" but that needs
# counter semantics fix.

sentai.camera.ratio(0, 0)
sentai.camera.select(0)
sentai.camera.switch_drain(1)
print("=== done ===")
