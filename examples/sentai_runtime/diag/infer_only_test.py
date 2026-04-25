import sentai
sentai.verbose(0)
results = []
def log(m): results.append(str(m))

sentai.tpu.load('/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite')
# Camera init (needed for pipeline.start to succeed), but InferTask ignores it
if sentai.camera.frame_count() == 0:
    sentai.camera.init(1)
sentai.rtos.sleep_ms(300)
log("cam_fc=%d" % sentai.camera.frame_count())

# Warmup via REPL
for _ in range(3): sentai.tpu.invoke()

# Enable infer-only mode
sentai.pipeline.infer_only(1)
sentai.pipeline.infer_reset()
# Start pipeline — InferTask enters infer-only branch, PrepTask sleeps
sentai.pipeline.start(0.25, 0.45, 50)
log("started pipeline, infer_only=1")

# Run 3 sec, measure
sentai.rtos.sleep_ms(3000)
s = sentai.pipeline.infer_stats()
log("t=3s: ok=%d fail=%d avg_ms=%d" % (
    s['ok'], s['fail'], s['ms_sum']//max(s['ok'],1)))
fps = s['ok']/3.0
log("infer_only FPS=%.1f" % fps)

# Run 5 more seconds for stability
sentai.rtos.sleep_ms(5000)
s2 = sentai.pipeline.infer_stats()
extra = s2['ok'] - s['ok']
fps2 = extra/5.0
log("t=5s_more: +ok=%d fps=%.1f fails_delta=%d" % (
    extra, fps2, s2['fail']-s['fail']))

sentai.pipeline.stop()
sentai.pipeline.infer_only(0)
sentai.rtos.sleep_ms(200)

sentai.fs.write('/infer_only_task.txt', "\n".join(results) + "\n")
print("=== INFER-ONLY-TASK RESULTS ===")
for r in results: print(r)
print("=== END ===")
