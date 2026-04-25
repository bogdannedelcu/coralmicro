# Audit: camera FPS vs invoke FPS during pipeline. Measures whether
# invoke rate exceeds camera frame rate (= we're inferring stale
# frames) or falls below (= we're dropping camera frames).
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed")

sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()

# Probe 1: camera alone (no pipeline running — raw CSI FPS)
print("\n--- Probe 1: Camera alone (no pipeline) ---")
c0 = sentai.camera.frame_count()
t0 = sentai.rtos.ticks_ms()
sentai.rtos.sleep_ms(3000)
c1 = sentai.camera.frame_count()
dt = sentai.rtos.ticks_ms() - t0
print("camera: %d frames in %d ms = %.1f FPS (sensor rate)" %
      (c1-c0, dt, (c1-c0) * 1000.0 / dt))

# Probe 2: during running pipeline — compare camera FPS vs infer FPS
print("\n--- Probe 2: Pipeline running — camera vs infer ---")
sentai.pipeline.start(0.25, 0.45, 50)

c0 = sentai.camera.frame_count()
i0 = sentai.pipeline.infer_stats()['ok']
p0 = sentai.pipeline.prep_stats()['frames']
t0 = sentai.rtos.ticks_ms()

sentai.rtos.sleep_ms(10000)

c1 = sentai.camera.frame_count()
istats = sentai.pipeline.infer_stats()
pstats = sentai.pipeline.prep_stats()
dt = sentai.rtos.ticks_ms() - t0

cam_fps = (c1-c0) * 1000.0 / dt
prep_fps = (pstats['frames']-p0) * 1000.0 / dt
infer_fps = (istats['ok']-i0) * 1000.0 / dt

print("  camera frames: %d in %d ms = %.2f FPS" % (c1-c0, dt, cam_fps))
print("  prep frames:   %d = %.2f FPS" % (pstats['frames']-p0, prep_fps))
print("  infer OK:      %d = %.2f FPS" % (istats['ok']-i0, infer_fps))
print("  infer FAIL:    %d" % (istats['fail']))
print()
if infer_fps > cam_fps:
    print("  VERDICT: invoke_fps > camera_fps -> %d%% of invokes re-use stale frames"
          % int((infer_fps - cam_fps) * 100.0 / infer_fps))
    print("  Action: either raise camera FPS, or accept some invokes re-process")
elif cam_fps > infer_fps * 1.05:
    print("  VERDICT: camera_fps > invoke_fps -> we drop %d%% of camera frames"
          % int((cam_fps - infer_fps) * 100.0 / cam_fps))
    print("  Action: pipeline is the bottleneck; every cam frame gets a unique infer")
else:
    print("  VERDICT: camera and infer are balanced (within 5%)")

sentai.pipeline.stop()
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
print("\n=== done ===")
