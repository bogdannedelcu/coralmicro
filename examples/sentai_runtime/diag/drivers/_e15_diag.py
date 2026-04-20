# Diagnose where the 200 ms/frame is spent.
# Strategy: run pipeline, keep verbose=1 briefly to catch [frame] prints,
# then count frames and report stall distribution.
import sentai
sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
sentai.camera.select(0)
sentai.camera.set_resolution(512, 512)
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.camera.to_tensor()

# Keep prints ON to see what's happening per frame.
rc = sentai.pipeline.start(0.25, 0.45, 50)
_ = sentai.pipeline.get(2000)  # warmup

N = 10
samples = []
t_prev = sentai.rtos.ticks_ms()
for i in range(N):
    d = sentai.pipeline.get(2000)
    t_now = sentai.rtos.ticks_ms()
    ps, is_ = sentai.pipeline.task_health()
    samples.append((t_now - t_prev, ps, is_))
    t_prev = t_now

sentai.pipeline.stop()
p, dr, f = sentai.pipeline.stats()
print()
print("idx | wall_interval | prep_stall | infer_stall")
for i, (iv, ps, is_) in enumerate(samples):
    print("  %d |  %6d ms   |  %4d ms   |  %4d ms" % (i, iv, ps, is_))
print("fw: processed=%d dropped=%d avg_fps=%.1f" % (p, dr, f))
