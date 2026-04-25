import sentai, time
sentai.verbose(0)
results = []

def log(msg):
    results.append(str(msg))

# Check knobs
log("ring_init=%d mode_init=%d wait_init=%d" % (
    sentai.pipeline.async_ring(),
    sentai.pipeline.ar_mode(),
    sentai.pipeline.ar_infer_wait()))

# Load model
try:
    rc = sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
    log("tpu_load_rc=%s" % rc)
except Exception as e:
    log("tpu_load_exc=%s" % e)

# Init camera
try:
    if sentai.camera.frame_count() == 0:
        sentai.camera.init(1)
    log("cam_fc=%d" % sentai.camera.frame_count())
except Exception as e:
    log("cam_exc=%s" % e)

sentai.rtos.sleep_ms(500)

# Warmup single invokes
for i in range(3):
    rc = sentai.tpu.invoke()
    log("warmup%d_rc=%d" % (i, rc))

def bench(tag, mode, wait, dur=3000):
    try:
        sentai.pipeline.async_ring(1)
        sentai.pipeline.ar_mode(mode)
        sentai.pipeline.ar_infer_wait(wait)
        sentai.pipeline.prep_reset()
        sentai.pipeline.infer_reset()
        sentai.pipeline.start(0.25, 0.45, 50)
        sentai.rtos.sleep_ms(dur)
        stats = sentai.pipeline.infer_stats()
        ar = sentai.pipeline.ar_stats()
        sentai.pipeline.stop()
        sentai.pipeline.async_ring(0)
        sentai.rtos.sleep_ms(300)
        fps = stats['ok'] / (dur/1000)
        log("%s ok=%d fail=%d fps=%.1f prep=%d infer=%d skip=%d ovw=%d" % (
            tag, stats['ok'], stats['fail'], fps, ar[0], ar[1], ar[4], ar[5]))
    except Exception as e:
        log("%s EXC=%s" % (tag, e))
        try:
            sentai.pipeline.stop()
            sentai.pipeline.async_ring(0)
        except: pass

# Test legacy baseline (async_ring OFF)
try:
    sentai.pipeline.async_ring(0)
    sentai.pipeline.prep_reset()
    sentai.pipeline.infer_reset()
    sentai.pipeline.start(0.25, 0.45, 50)
    sentai.rtos.sleep_ms(3000)
    stats = sentai.pipeline.infer_stats()
    sentai.pipeline.stop()
    sentai.rtos.sleep_ms(300)
    fps = stats['ok'] / 3.0
    log("LEGACY ok=%d fail=%d fps=%.1f" % (stats['ok'], stats['fail'], fps))
except Exception as e:
    log("LEGACY EXC=%s" % e)

# V23 matrix
bench("V23_m0_w1", 0, 1)
bench("V23_m0_w5", 0, 5)
bench("V23_m0_w10", 0, 10)
bench("V23_m0_w20", 0, 20)
bench("V23_m2_w10", 2, 10)

# Save results to LFS
out = "\n".join(results) + "\n"
sentai.fs.write('/test_results.txt', out)
print("\n=== TEST RESULTS ===")
for r in results:
    print(r)
print("=== END ===")
