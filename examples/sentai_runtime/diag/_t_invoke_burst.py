# N invokes in a tight loop on the same input — measure per-invoke
# timing + diff Send* call counters before/after.
# Goal: see if invoke 1 is "different" from invokes 2..N (caching path)
# and confirm exactly how many SendInputs/SendInstructions/SendParameters
# each invoke triggers.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(200)

print("--- warmup 3 ---")
for i in range(3):
    t0 = sentai.rtos.ticks_ms()
    sentai.tpu.invoke()
    t1 = sentai.rtos.ticks_ms()
    print("warmup #%d: %d ms" % (i, t1-t0))

# Reset call counters
sentai.diag.tpu_call_stats(1)
print("--- counters reset ---")

# 8 invokes back-to-back, time each one.
print("--- burst 8 invokes ---")
for i in range(8):
    s_pre = sentai.diag.tpu_call_stats()
    t0 = sentai.rtos.ticks_ms()
    rc = sentai.tpu.invoke()
    t1 = sentai.rtos.ticks_ms()
    s_post = sentai.diag.tpu_call_stats()
    dp = s_post['p_calls'] - s_pre['p_calls']
    di = s_post['i_calls'] - s_pre['i_calls']
    din = s_post['in_calls'] - s_pre['in_calls']
    db_p = s_post['p_bytes'] - s_pre['p_bytes']
    db_i = s_post['i_bytes'] - s_pre['i_bytes']
    db_in = s_post['in_bytes'] - s_pre['in_bytes']
    print("inv #%d: %dms rc=%d  p=%dx/%dB i=%dx/%dB in=%dx/%dB" %
          (i, t1-t0, rc, dp, db_p, di, db_i, din, db_in))

print("=== done ===")
