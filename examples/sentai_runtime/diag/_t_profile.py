import sentai
sentai.verbose(1)
print("=== profile TPU ===")
sentai.tpu.load('/yolo26n.edgetpu_1.tflite')
print("chunk_size =", sentai.diag.tpu_chunk_size())
print("zero_copy  =", sentai.diag.tpu_zero_copy())
print("desc_cache =", sentai.diag.tpu_desc_cache())
print("urb_timeout=", sentai.diag.tpu_urb_timeout())

# Warmup
for i in range(3): sentai.tpu.invoke()

# Reset perf counters
sentai.diag.tpu_perf(1)

# 30 invokes
t0 = sentai.rtos.ticks_ms()
for i in range(30):
    sentai.tpu.invoke()
t1 = sentai.rtos.ticks_ms()
print("30 invokes: %d ms total = %.1f ms/invoke = %.1f FPS" % (
    t1-t0, (t1-t0)/30.0, 30000.0/(t1-t0)))

perf = sentai.diag.tpu_perf()
print("perf:", perf)

# Sweep chunk size — find if 33KB sweet spot applies to yolo26n
for n in (16*1024, 33*1024, 48*1024, 64*1024, 96*1024):
    sentai.diag.tpu_chunk_size(n)
    for i in range(3): sentai.tpu.invoke()  # warm
    t0 = sentai.rtos.ticks_ms()
    for i in range(20):
        sentai.tpu.invoke()
    t1 = sentai.rtos.ticks_ms()
    print("chunk=%d KB : %.1f ms/invoke = %.1f FPS" % (
        n/1024, (t1-t0)/20.0, 20000.0/(t1-t0)))

# Reset to default for next tests
sentai.diag.tpu_chunk_size(33*1024)
print("=== done ===")
