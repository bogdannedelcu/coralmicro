# T1+T3+T4 isolation — equal priority 2 on Prep/Infer tasks
import sentai, gc

sentai.verbose(1)

def _ms():
    return sentai.rtos.ticks_ms()

def run(n, tag):
    fails = 0
    t0 = _ms()
    mx = 0
    for i in range(n):
        ta = _ms()
        ok = sentai.tpu.invoke()
        tb = _ms()
        d = tb - ta
        if d > mx: mx = d
        if not ok: fails += 1
        if i < 3 or i == n-1:
            print("  [%s] i=%d ok=%s d=%d ms" % (tag, i, ok, d))
    t1 = _ms()
    return (t1 - t0, fails, (t1 - t0) // max(n,1), mx)

print("=== boot ===")
sentai.tpu.load('/yolo26n.edgetpu_1.tflite')
sentai.camera.init()
print("camera init ok")
sentai.camera.to_tensor()
print("to_tensor ok")
sentai.rtos.sleep_ms(50)

print("--- T1: pure TPU ---")
gc.collect()
tot, fails, avg, mx = run(10, "T1")
print("T1: tot=%d ms fails=%d avg=%d mx=%d" % (tot, fails, avg, mx))

print("--- T3: pipeline.start+stop then invoke ---")
rc = sentai.pipeline.start(0.25, 0.45, 50)
print("  start rc=%s" % rc)
sentai.rtos.sleep_ms(300)
sentai.pipeline.stop()
print("  stop done")
sentai.rtos.sleep_ms(300)
gc.collect()
tot, fails, avg, mx = run(10, "T3")
print("T3: tot=%d ms fails=%d avg=%d mx=%d" % (tot, fails, avg, mx))

print("--- T4: pipeline RUNNING ---")
rc = sentai.pipeline.start(0.25, 0.45, 50)
print("  start rc=%s" % rc)
sentai.rtos.sleep_ms(500)
try:
    print("prep_stats pre:", sentai.pipeline.prep_stats())
except Exception as e:
    print("err:", e)
gc.collect()
tot, fails, avg, mx = run(10, "T4")
print("T4: tot=%d ms fails=%d avg=%d mx=%d" % (tot, fails, avg, mx))
try:
    print("prep_stats post:", sentai.pipeline.prep_stats())
except Exception as e:
    print("err:", e)
sentai.pipeline.stop()
print("=== done ===")
