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
    print("PIPE_STEP", sentai.pipeline.step())
    print("TPU_OUTPUTS", sentai.tpu.num_outputs())
    dets = sentai.pipeline.detections(100)
    print("DETECTIONS_COUNT", len(dets))
    print("DETECTIONS_WRITTEN",
          sentai.fs.write("/detections.txt", repr(dets)))
    return len(dets)
