#!/usr/bin/env python3
"""test_edgetpu.py — Run yolo26n.edgetpu.tflite on Coral USB Accelerator.

Requires: source ../venv_coral/bin/activate   (Python 3.9, pycoral + tflite-runtime)
          + libedgetpu1-std installed system-wide
          + Coral USB plugged in

Usage:    python test_edgetpu.py [conf_threshold] [iou_threshold] [image_path]

Output:   [1, 84, 2100] int8 raw YOLO — NMS done here (port of sentai_tpu_detect).
"""

import os, sys, time
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_common import yolo_nms, draw_detections, print_detections

TEST_DIR   = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(TEST_DIR, "yolo26n.edgetpu.tflite")
CONF_THR   = 0.05
IOU_THR    = 0.45


def main():
    conf = float(sys.argv[1]) if len(sys.argv) > 1 else CONF_THR
    iou = float(sys.argv[2]) if len(sys.argv) > 2 else IOU_THR
    image_path = sys.argv[3] if len(sys.argv) > 3 else os.path.join(TEST_DIR, "input.jpeg")
    out_path = os.path.join(TEST_DIR, "det_edgetpu.jpg")

    from pycoral.utils.edgetpu import make_interpreter

    print(f"Model:  {os.path.basename(MODEL_PATH)}")
    print(f"Image:  {image_path}")
    print(f"Conf:   {conf}")
    print(f"IoU:    {iou}")
    print()

    interp = make_interpreter(MODEL_PATH)
    interp.allocate_tensors()

    inp = interp.get_input_details()[0]
    out = interp.get_output_details()[0]
    in_h, in_w = int(inp['shape'][1]), int(inp['shape'][2])

    in_sc = float(inp['quantization_parameters']['scales'][0])
    in_zp = int(inp['quantization_parameters']['zero_points'][0])
    out_sc = float(out['quantization_parameters']['scales'][0])
    out_zp = int(out['quantization_parameters']['zero_points'][0])

    print(f"  Input:  {list(inp['shape'])}  {inp['dtype'].__name__}  scale={in_sc:.6f} zp={in_zp}")
    print(f"  Output: {list(out['shape'])}  {out['dtype'].__name__}  scale={out_sc:.6f} zp={out_zp}")

    # Preprocess: uint8 pixels → int8 via (pixel + zero_point)
    # Matches sentai_runtime.cc to_tensor(): data[i] = clamp(pixel + zp, -128, 127)
    img = Image.open(image_path).convert('RGB').resize((in_w, in_h), Image.BILINEAR)
    pixels = np.array(img, dtype=np.uint8)
    input_data = (pixels.astype(np.int16) + in_zp).clip(-128, 127).astype(np.int8)
    interp.set_tensor(inp['index'], input_data.reshape(inp['shape']))

    # Warmup (EdgeTPU may overwrite the input buffer, so re-set before real run)
    interp.invoke()

    interp.set_tensor(inp['index'], input_data.reshape(inp['shape']))
    t0 = time.perf_counter()
    interp.invoke()
    dt = (time.perf_counter() - t0) * 1000
    print(f"  Inference: {dt:.1f} ms")

    # Dequantize: float = scale * (int8 - zero_point)
    raw = interp.get_tensor(out['index'])[0]  # [84, 2100] int8
    data = out_sc * (raw.astype(np.float32) - out_zp)
    print(f"  Raw shape: {list(raw.shape)}")
    print(f"  Dequantized: bbox=[{data[:4].min():.3f}..{data[:4].max():.3f}]  "
          f"cls=[{data[4:].min():.3f}..{data[4:].max():.3f}]")

    # NMS (port of sentai_tpu_detect)
    dets = yolo_nms(data, in_w, in_h, conf, iou)
    print_detections(dets, conf)

    draw_img = draw_detections(img, dets)
    draw_img.save(out_path, quality=90)
    print(f"  Saved: {out_path}")


if __name__ == "__main__":
    main()
