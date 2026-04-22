# Chunk size sweep for yolo_1 with OCRAM tensor
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()
print("warmed")

def bench(n_kb, n_invokes):
    sentai.diag.tpu_chunk_size(n_kb * 1024)
    for i in range(3): sentai.tpu.invoke()  # warm
    t0 = sentai.rtos.ticks_ms()
    fails = 0
    for i in range(n_invokes):
        r = sentai.tpu.invoke()
        if r < 0: fails += 1
    t1 = sentai.rtos.ticks_ms()
    dt = t1 - t0
    return (dt, fails)

print("--- PURE TPU chunk sweep ---")
for kb in (8, 16, 24, 32, 36, 40, 48, 64, 96, 128, 160):
    dt, f = bench(kb, 30)
    print("  %3d KB : %.1f ms/invoke = %.1f FPS  fails=%d" % (
        kb, dt/30.0, 30000.0/dt, f))

# Pipeline test at best 3 candidates
print("--- PIPELINE chunk sweep (best candidates) ---")
for kb in (24, 33, 48):
    sentai.diag.tpu_chunk_size(kb * 1024)
    sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
    rc = sentai.pipeline.start(0.25, 0.45, 50)
    sentai.rtos.sleep_ms(3000)
    istats = sentai.pipeline.infer_stats()
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(200)
    if istats['ok']:
        print("  %d KB: %.1f FPS, avg=%d ms, fails=%d" % (
            kb, istats['ok']/3.0, istats['ms_sum']//istats['ok'], istats['fail']))
    else:
        print("  %d KB: 0 FPS, fails=%d" % (kb, istats['fail']))

sentai.diag.tpu_chunk_size(33*1024)  # restore default
print("=== done ===")
