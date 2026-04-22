import sentai
sentai.verbose(1)
print("=== boot A: WITH_DCACHE ===")
sentai.tpu.load('/yolo26n.edgetpu_1.tflite')
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(100)
for i in range(3): sentai.tpu.invoke()
print("warmed")
sentai.diag.cam_skip_dcache(0)  # DEFAULT
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
rc = sentai.pipeline.start(0.25, 0.45, 50)
print("start rc=%s" % rc)
sentai.rtos.sleep_ms(5000)
istats = sentai.pipeline.infer_stats()
pstats = sentai.pipeline.prep_stats()
print("@5s infer ok=%d fail=%d rc=%d | prep=%d" % (
    istats['ok'], istats['fail'], istats['last_rc'], pstats['frames']))
if istats['ok']:
    print("avg=%d ms fps=%.1f" % (istats['ms_sum']//istats['ok'], istats['ok']/5.0))
sentai.pipeline.stop()
print("=== done ===")
