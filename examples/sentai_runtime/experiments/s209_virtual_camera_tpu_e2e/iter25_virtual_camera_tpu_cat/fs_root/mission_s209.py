import sentai


def run():
    print("IMG_SIZE", sentai.fs.size("/images/cat_640x480.bmp"))
    print("MODEL_SIZE", sentai.fs.size(
        "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"))
    print("CAM_SELECT", sentai.camera.select(
        -1, "/images/cat_640x480.bmp"))
    print("TPU_LOAD", sentai.tpu.load(
        "/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"))
    print("TPU_READY", sentai.tpu.ready())
    print("PIPE_START", sentai.pipeline.start(0.1, 0.45, 50, False))
    frame = None
    for _ in range(40):
        frame = sentai.pipeline.get_ex(0)
        if frame is not None:
            break
        sentai.rtos.sleep_ms(50)
    print("PIPE_FRAME", frame is not None)
    print("TPU_OUTPUTS", sentai.tpu.num_outputs())
    dets = sentai.pipeline.detections(100)
    print("DETECTIONS_COUNT", len(dets))
    print("DETECTIONS_WRITTEN",
          sentai.fs.write("/detections.txt", repr(dets)))
    print("OVERLAY_WRITTEN",
          sentai.pipeline.save("/images/detected_overlay.bmp"))
    print("PIPE_STOP", sentai.pipeline.stop())
    return len(dets)
