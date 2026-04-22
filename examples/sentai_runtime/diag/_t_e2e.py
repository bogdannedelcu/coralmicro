# E2E pipeline check — ReadEvent static fix + priority 2
import sentai, gc

sentai.verbose(1)

def _ms(): return sentai.rtos.ticks_ms()

print("=== boot ===")
sentai.tpu.load('/yolo26n.edgetpu_1.tflite')
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(100)

print("--- TPU warmup ---")
for i in range(3):
    ok = sentai.tpu.invoke()
    print("  warm %d: %s" % (i, ok))
sentai.rtos.sleep_ms(200)

print("--- sustained pipeline: 10 s ---")
rc = sentai.pipeline.start(0.25, 0.45, 50)
print("start rc=%s" % rc)
sentai.rtos.sleep_ms(1000)
s1 = sentai.pipeline.prep_stats()
print("  prep_stats @1s:", s1)
sentai.rtos.sleep_ms(5000)
s2 = sentai.pipeline.prep_stats()
print("  prep_stats @6s:", s2)
sentai.rtos.sleep_ms(4000)
s3 = sentai.pipeline.prep_stats()
print("  prep_stats @10s:", s3)
sentai.pipeline.stop()
print("stopped")

# End-to-end: frames per second over full run
dt_frames = s3['frames'] - s1['frames']
print("delta_frames=%d over ~9s  = %.1f FPS end-to-end" % (dt_frames, dt_frames/9.0))

# Pure TPU after pipeline (confirm no corruption)
print("--- pure TPU after pipeline ---")
sentai.rtos.sleep_ms(300)
gc.collect()
fails = 0
t0 = _ms()
for i in range(20):
    ok = sentai.tpu.invoke()
    if not ok: fails += 1
t1 = _ms()
print("20 invokes: %d ms total, fails=%d, avg=%d ms" % (t1-t0, fails, (t1-t0)//20))

print("=== done ===")
