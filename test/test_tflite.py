#!/usr/bin/env python3
"""test_tflite.py — Run yolo26n.tflite (CPU float32).

Requires: source ../venv/bin/activate   (Python 3.12, tensorflow)
Usage:    python test_tflite.py [conf_threshold] [image_path]

Output:   [1, 300, 6] post-NMS, coords normalized 0-1.
"""

import os, sys, time
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_common import parse_post_nms, draw_detections, print_detections

TEST_DIR   = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(TEST_DIR, "yolo26n.tflite")
CONF_THR   = 0.05


def main():
    conf = float(sys.argv[1]) if len(sys.argv) > 1 else CONF_THR
    image_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(TEST_DIR, "input.jpeg")
    out_path = os.path.join(TEST_DIR, "det_tflite.jpg")

    import tensorflow as tf

    print(f"Model:  {os.path.basename(MODEL_PATH)}")
    print(f"Image:  {image_path}")
    print(f"Conf:   {conf}")
    print()

    interp = tf.lite.Interpreter(model_path=MODEL_PATH)
    interp.allocate_tensors()

    inp = interp.get_input_details()[0]
    out = interp.get_output_details()[0]
    in_h, in_w = int(inp['shape'][1]), int(inp['shape'][2])
    print(f"  Input:  {list(inp['shape'])}  {inp['dtype'].__name__}")
    print(f"  Output: {list(out['shape'])}  {out['dtype'].__name__}")

    # Preprocess: float32 normalized 0-1
    img = Image.open(image_path).convert('RGB').resize((in_w, in_h), Image.BILINEAR)
    pixels = np.array(img, dtype=np.float32) / 255.0
    interp.set_tensor(inp['index'], pixels.reshape(inp['shape']))

    t0 = time.perf_counter()
    interp.invoke()
    dt = (time.perf_counter() - t0) * 1000
    print(f"  Inference: {dt:.1f} ms")

    result = interp.get_tensor(out['index'])
    dets = parse_post_nms(result, in_w, in_h, conf, coords_normalized=True)
    print_detections(dets, conf)

    draw_img = draw_detections(img, dets)
    draw_img.save(out_path, quality=90)
    print(f"  Saved: {out_path}")


if __name__ == "__main__":
    main()
