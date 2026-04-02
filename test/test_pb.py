#!/usr/bin/env python3
"""test_pb.py — Run yolo26n.pb (TF Frozen GraphDef).

Requires: source ../venv/bin/activate   (Python 3.12, tensorflow)
Usage:    python test_pb.py [conf_threshold] [image_path]

Output:   [1, 300, 6] post-NMS, coords in PIXELS (not normalized).
"""

import os, sys, time
import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_common import parse_post_nms, draw_detections, print_detections

TEST_DIR   = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(TEST_DIR, "yolo26n.pb")
CONF_THR   = 0.05


def main():
    conf = float(sys.argv[1]) if len(sys.argv) > 1 else CONF_THR
    image_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(TEST_DIR, "input.jpeg")
    out_path = os.path.join(TEST_DIR, "det_pb.jpg")

    import tensorflow as tf

    print(f"Model:  {os.path.basename(MODEL_PATH)}")
    print(f"Image:  {image_path}")
    print(f"Conf:   {conf}")
    print()

    with open(MODEL_PATH, 'rb') as f:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(f.read())
    print(f"  Nodes: {len(graph_def.node)}")

    # Find input/output nodes
    input_name = output_name = None
    for n in graph_def.node:
        if n.op == 'Placeholder' and input_name is None:
            input_name = n.name
        if n.name == 'Identity':
            output_name = n.name
    print(f"  Input:  {input_name}")
    print(f"  Output: {output_name}")

    with tf.compat.v1.Graph().as_default() as graph:
        tf.import_graph_def(graph_def, name='')
        input_tensor = graph.get_tensor_by_name(f'{input_name}:0')
        output_tensor = graph.get_tensor_by_name(f'{output_name}:0')

        shape = input_tensor.shape.as_list()
        in_h = shape[1] if shape[1] is not None else 320
        in_w = shape[2] if shape[2] is not None else 320

        # Preprocess: float32 normalized 0-1 (output coords are in pixels)
        img = Image.open(image_path).convert('RGB').resize((in_w, in_h), Image.BILINEAR)
        pixels = np.array(img, dtype=np.float32) / 255.0

        with tf.compat.v1.Session(graph=graph) as sess:
            t0 = time.perf_counter()
            result = sess.run(output_tensor, {input_tensor: pixels.reshape(1, in_h, in_w, 3)})
            dt = (time.perf_counter() - t0) * 1000
            print(f"  Inference: {dt:.1f} ms")

    dets = parse_post_nms(result, in_w, in_h, conf, coords_normalized=False)
    print_detections(dets, conf)

    draw_img = draw_detections(img, dets)
    draw_img.save(out_path, quality=90)
    print(f"  Saved: {out_path}")


if __name__ == "__main__":
    main()
