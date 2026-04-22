import sentai
sentai.verbose(1)
print("=== desc_cache test ===")
sentai.tpu.load('/yolo26n.edgetpu_1.tflite')

# Warmup
for i in range(3): sentai.tpu.invoke()

print("--- BASELINE (no cache) ---")
sentai.diag.tpu_desc_cache(0)
sentai.diag.tpu_chunk_size(16*1024)
for i in range(3): sentai.tpu.invoke()  # warm
t0 = sentai.rtos.ticks_ms()
fails = 0
for i in range(30):
    r = sentai.tpu.invoke()
    if r < 0: fails += 1
t1 = sentai.rtos.ticks_ms()
print("baseline: %.1f ms/invoke %.1f FPS fails=%d" % ((t1-t0)/30.0, 30000.0/(t1-t0), fails))

print("--- DESC_CACHE ON ---")
sentai.diag.tpu_desc_cache(1)
for i in range(3): sentai.tpu.invoke()  # warm
t0 = sentai.rtos.ticks_ms()
fails = 0
for i in range(30):
    r = sentai.tpu.invoke()
    if r < 0: fails += 1
t1 = sentai.rtos.ticks_ms()
print("desc_cache: %.1f ms/invoke %.1f FPS fails=%d" % ((t1-t0)/30.0, 30000.0/(t1-t0), fails))

sentai.diag.tpu_desc_cache(0)  # restore default
print("=== done ===")
