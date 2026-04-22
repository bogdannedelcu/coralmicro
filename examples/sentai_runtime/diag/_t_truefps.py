# True end-to-end FPS: compare invoke count vs unique camera frames
import sentai
sentai.verbose(1)

MODEL = "/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite"

print("=== boot ===")
sentai.tpu.load(MODEL)
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(500)  # let camera settle
print("warmed")

# Camera alone — measure FPS over 2s
fc0 = sentai.camera.frame_count()
sentai.rtos.sleep_ms(2000)
fc1 = sentai.camera.frame_count()
cam_fps = (fc1 - fc0) / 2.0
print("CAMERA alone: %d frames / 2s = %.1f FPS" % (fc1 - fc0, cam_fps))

# Pipeline — measure camera frames AND inferences over 5s
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
fc0 = sentai.camera.frame_count()
t0 = sentai.rtos.ticks_ms()
rc = sentai.pipeline.start(0.25, 0.45, 50)
print("start rc=%s" % rc)
sentai.rtos.sleep_ms(5000)
fc1 = sentai.camera.frame_count()
t1 = sentai.rtos.ticks_ms()
istats = sentai.pipeline.infer_stats()
pstats = sentai.pipeline.prep_stats()
sentai.pipeline.stop()

cam_frames = fc1 - fc0
elapsed_ms = t1 - t0
print("--- results over %d ms ---" % elapsed_ms)
print("  camera frames produced:  %d  (%.1f FPS)" % (
    cam_frames, 1000.0 * cam_frames / elapsed_ms))
print("  PrepTask iterations:     %d  (%.1f FPS)" % (
    pstats['frames'], 1000.0 * pstats['frames'] / elapsed_ms))
print("  InferTask successes:     %d  (%.1f FPS)" % (
    istats['ok'], 1000.0 * istats['ok'] / elapsed_ms))
print("  InferTask fails:         %d" % istats['fail'])
print("  avg invoke: %d ms" % (istats['ms_sum'] // max(istats['ok'], 1)))
if istats['ok'] > cam_frames:
    print("  NOTE: %d invokes > %d cam frames => %d duplicated frames" % (
        istats['ok'], cam_frames, istats['ok'] - cam_frames))
else:
    print("  UNIQUE inferences = min(invokes, cam_frames) = %d (%.1f FPS)" % (
        istats['ok'], 1000.0 * istats['ok'] / elapsed_ms))
print("=== done ===")
