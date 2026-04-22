# Test async_input toggle effect on pipeline
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def bench_pure(n):
    t0 = sentai.rtos.ticks_ms()
    fails = 0
    for i in range(n):
        r = sentai.tpu.invoke()
        if r < 0: fails += 1
    t1 = sentai.rtos.ticks_ms()
    return (t1-t0, fails)

def bench_pipe(duration_ms):
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    sentai.pipeline.start(0.25, 0.45, 50)
    sentai.rtos.sleep_ms(duration_ms)
    istats = sentai.pipeline.infer_stats()
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(200)
    return istats

print("--- async_input OFF (default) ---")
sentai.diag.tpu_async_input(0)
dt, f = bench_pure(30)
print("  PURE TPU: %.1f ms/invoke = %.1f FPS fails=%d" % (dt/30, 30000/dt, f))
p = bench_pipe(4000)
print("  PIPELINE @4s: ok=%d fail=%d avg=%d ms  fps=%.1f" % (
    p['ok'], p['fail'], p['ms_sum']//max(p['ok'],1), p['ok']/4.0))

print("--- async_input ON ---")
sentai.diag.tpu_async_input(1)
dt, f = bench_pure(30)
print("  PURE TPU: %.1f ms/invoke = %.1f FPS fails=%d" % (dt/30, 30000/dt, f))
p = bench_pipe(4000)
print("  PIPELINE @4s: ok=%d fail=%d avg=%d ms  fps=%.1f" % (
    p['ok'], p['fail'], p['ms_sum']//max(p['ok'],1), p['ok']/4.0))

sentai.diag.tpu_async_input(0)  # restore default
print("=== done ===")
