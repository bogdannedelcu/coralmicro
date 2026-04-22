# Pure TPU + Pipeline with yolo_1 (512x512)
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

print("=== boot ===")
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(100)
for i in range(3): sentai.tpu.invoke()
print("warmed")
print("chunk=%d urb_to=%d desc_cache=%s" % (
    sentai.diag.tpu_chunk_size(),
    sentai.diag.tpu_urb_timeout(),
    sentai.diag.tpu_desc_cache()))

# Pure TPU
t0 = sentai.rtos.ticks_ms()
fails = 0
for i in range(50):
    r = sentai.tpu.invoke()
    if r < 0: fails += 1
t1 = sentai.rtos.ticks_ms()
print("PURE-TPU: %.1f ms/invoke = %.1f FPS (fails=%d)" % (
    (t1-t0)/50.0, 50000.0/(t1-t0), fails))

# Pipeline end-to-end
print("--- PIPELINE ---")
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
rc = sentai.pipeline.start(0.25, 0.45, 50)
print("start rc=%s" % rc)
sentai.rtos.sleep_ms(5000)
istats = sentai.pipeline.infer_stats()
pstats = sentai.pipeline.prep_stats()
print("@5s infer ok=%d fail=%d rc=%d | prep=%d" % (
    istats['ok'], istats['fail'], istats['last_rc'], pstats['frames']))
if istats['ok']:
    print("  avg=%.1f ms, end2end=%.1f FPS" % (
        istats['ms_sum']/istats['ok'], istats['ok']/5.0))
sentai.pipeline.stop()
print("=== done ===")
