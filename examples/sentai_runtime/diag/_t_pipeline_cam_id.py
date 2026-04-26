# Pipeline cam_id propagation test: at alt 1:1, run pipeline + read
# DetectionFrame.cam_id (= per-buffer tag) for each detection.
# Verifies that the ISR-set tag flows through PrepTask → InferTask → DetectionFrame.
import sentai
sentai.verbose(1)

print("=== boot ===")
sentai.tpu.load("/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite")
sentai.camera.init()
sentai.camera.to_tensor()
sentai.rtos.sleep_ms(300)
for i in range(3): sentai.tpu.invoke()

# Alt 1:1 with default drain so MUX flips between cameras
sentai.camera.ratio(1, 1)
sentai.camera.switch_drain(1)
sentai.camera.select(0)
sentai.rtos.sleep_ms(500)

print("--- pipeline alt 1:1, sample cam_id per detection ---")
sentai.pipeline.prep_reset(); sentai.pipeline.infer_reset()
rc = sentai.pipeline.start(0.25, 0.45, 50)
print("start rc=%d" % rc)

cam0 = 0; cam1 = 0; unknown = 0
n = 24
for i in range(n):
    out = sentai.pipeline.get_ex(2000)
    if out is None:
        print("  i=%d TIMEOUT" % i); continue
    dets, invoke_ms, total_ms, frame_seq, memcpy_ms, nms_ms, cam_id = out
    if cam_id == 0: cam0 += 1
    elif cam_id == 1: cam1 += 1
    else: unknown += 1
    print("  i=%d cam_id=%d frame_seq=%d invoke=%dms" %
          (i, cam_id, frame_seq, invoke_ms))

sentai.pipeline.stop()
sentai.camera.ratio(0, 0)
sentai.camera.select(0)

print("--- summary ---")
print("  cam0=%d cam1=%d unknown=%d (of %d)" % (cam0, cam1, unknown, n))
print("=== done ===")
