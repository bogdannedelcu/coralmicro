import sentai
sentai.verbose(0)
results = []
def log(m): results.append(str(m))

sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.rtos.sleep_ms(500)
log("cam_fc=%d" % sentai.camera.frame_count())

for _ in range(3): sentai.tpu.invoke()
log("warmup done")

# Test 1: pure invoke (no pipeline, baseline)
N = 30
t0 = sentai.rtos.ticks_ms()
for _ in range(N): sentai.tpu.invoke()
t1 = sentai.rtos.ticks_ms()
log("pure_invoke_with_cam: %.1f FPS avg=%.2fms" % (1000*N/(t1-t0), (t1-t0)/N))

# Test 2: Full legacy pipeline (now with RGB565 camera)
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(3000)
s = sentai.pipeline.infer_stats()
p = sentai.pipeline.prep_stats()
sentai.pipeline.stop()
sentai.rtos.sleep_ms(300)
log("LEGACY_RGB565 ok=%d fail=%d fps=%.1f prep=%d pxp_ms=%d cam_ms=%d" % (
    s['ok'], s['fail'], s['ok']/3.0,
    p.get('frames',0), p.get('pxp_ms_sum',0), p.get('cam_grab_ms_sum',0)))

# Test 3: Infer-only to confirm 68 FPS ceiling still reachable
sentai.pipeline.infer_only(1)
sentai.pipeline.infer_reset()
sentai.pipeline.start(0.25, 0.45, 50)
sentai.rtos.sleep_ms(3000)
s2 = sentai.pipeline.infer_stats()
sentai.pipeline.stop()
sentai.pipeline.infer_only(0)
log("INFER_ONLY (cam RGB565 running) ok=%d fps=%.1f fail=%d" % (
    s2['ok'], s2['ok']/3.0, s2['fail']))

sentai.fs.write('/rgb565_test.txt', "\n".join(results) + "\n")
print("=== RGB565 RESULTS ===")
for r in results: print(r)
print("=== END ===")
