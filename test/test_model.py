#!/usr/bin/env python3
"""
test_model.py — PC-side YOLO detection test
Runs yolo26n on all 3 formats: .tflite, .edgetpu.tflite, .pb
Mirrors the sentai draw() style for annotated output.

Usage:
  source venv/bin/activate
  python test/test_model.py [conf_threshold]

Outputs in test/:
  det_tflite.jpg        — from yolo26n.tflite (CPU float32)
  det_pb.jpg            — from yolo26n.pb (frozen graph)
  det_edgetpu.jpg       — from yolo26n.edgetpu.tflite (needs Edge TPU)

Model output: [1, 300, 6] = [x1, y1, x2, y2, conf, class_id]
  Coords normalized 0-1.  NMS already done inside the model.
"""

import os, sys, time
import numpy as np
from PIL import Image, ImageDraw

# ─── paths ───
TEST_DIR  = os.path.dirname(os.path.abspath(__file__))
IMAGE     = os.path.join(TEST_DIR, "input.jpeg")
MODEL_TFL = os.path.join(TEST_DIR, "yolo26n.tflite")
MODEL_PB  = os.path.join(TEST_DIR, "yolo26n.pb")
MODEL_EDG = os.path.join(TEST_DIR, "yolo26n.edgetpu.tflite")

# ─── COCO 80 class names (same as sentai_runtime.cc kCocoNames) ───
COCO_NAMES = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack",
    "umbrella","handbag","tie","suitcase","frisbee","skis","snowboard","sports ball",
    "kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket",
    "bottle","wine glass","cup","fork","knife","spoon","bowl","banana","apple",
    "sandwich","orange","broccoli","carrot","hot dog","pizza","donut","cake","chair",
    "couch","potted plant","bed","dining table","toilet","tv","laptop","mouse",
    "remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier",
    "toothbrush",
]

# ─── 10 box colors (same as sentai_runtime.cc kBoxColors) ───
BOX_COLORS = [
    (255,  56,  56), ( 56, 255,  56), ( 56,  56, 255),
    (255, 255,  56), (255,  56, 255), ( 56, 255, 255),
    (255, 128,   0), (  0, 128, 255), (255,   0, 128),
    (128, 255,   0),
]

CONF_THR = 0.05


# ─── Parse [N, 6] post-NMS output → pixel-space detections ───
def parse_detections(output, img_w, img_h, conf_thr=None, coords_normalized=True):
    """
    output: (1, 300, 6) or (300, 6)
            each row = [x1, y1, x2, y2, conf, class_id]
    coords_normalized: True  → x/y in 0-1, multiply by img size
                       False → x/y already in pixel space
    Returns list of (x1, y1, x2, y2, conf, class_id) in pixel coords.
    """
    if conf_thr is None:
        conf_thr = CONF_THR
    if output.ndim == 3:
        output = output[0]
    dets = []
    for row in output:
        x1r, y1r, x2r, y2r, conf, cls_f = row
        if conf < conf_thr:
            continue
        cls = int(cls_f)
        if coords_normalized:
            x1 = max(0, int(x1r * img_w + 0.5))
            y1 = max(0, int(y1r * img_h + 0.5))
            x2 = min(img_w, int(x2r * img_w + 0.5))
            y2 = min(img_h, int(y2r * img_h + 0.5))
        else:
            x1 = max(0, int(x1r + 0.5))
            y1 = max(0, int(y1r + 0.5))
            x2 = min(img_w, int(x2r + 0.5))
            y2 = min(img_h, int(y2r + 0.5))
        if x2 > x1 and y2 > y1:
            dets.append((x1, y1, x2, y2, float(conf), cls))
    return dets


# ─── Draw — mirrors sentai_tpu_draw() style ───
def draw_detections(img, detections):
    """Draw bboxes + labels on a PIL Image. Returns new image."""
    img = img.copy()
    draw = ImageDraw.Draw(img)
    for (x1, y1, x2, y2, score, cls) in detections:
        ci = cls % 10
        color = BOX_COLORS[ci]
        draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
        name = COCO_NAMES[cls] if cls < len(COCO_NAMES) else f"cls{cls}"
        label = f"{name} {int(score*100)}%"
        lw = len(label) * 7 + 4
        lh = 12
        ly = y1 - lh - 1 if y1 - lh - 1 >= 0 else y1
        draw.rectangle([x1, ly, x1 + lw, ly + lh], fill=color)
        draw.text((x1 + 2, ly + 1), label, fill=(0, 0, 0))
    return img


def print_dets(dets):
    print(f"  {len(dets)} detections (conf>{CONF_THR*100:.0f}%)")
    for d in dets:
        x1, y1, x2, y2, score, cls = d
        name = COCO_NAMES[cls] if cls < len(COCO_NAMES) else f"cls{cls}"
        print(f"    [{x1:3d},{y1:3d},{x2:3d},{y2:3d}] {score:.3f} {name}")


# ═══════════════════════════════════════════════════════════
#  1. TFLite (CPU, float32)
# ═══════════════════════════════════════════════════════════
def run_tflite(model_path, image_path, out_path):
    import tensorflow as tf

    print(f"Model: {os.path.basename(model_path)}")
    interp = tf.lite.Interpreter(model_path=model_path)
    interp.allocate_tensors()

    inp = interp.get_input_details()[0]
    out = interp.get_output_details()[0]
    in_h, in_w = inp['shape'][1], inp['shape'][2]
    print(f"  Input:  {list(inp['shape'])}  dtype={inp['dtype'].__name__}")
    print(f"  Output: {list(out['shape'])}  dtype={out['dtype'].__name__}")

    # Preprocess: resize + float32 normalized 0-1
    img = Image.open(image_path).convert('RGB').resize((in_w, in_h), Image.BILINEAR)
    pixels = np.array(img, dtype=np.float32) / 255.0
    input_data = pixels.reshape(inp['shape'])

    # Invoke
    interp.set_tensor(inp['index'], input_data)
    t0 = time.perf_counter()
    interp.invoke()
    dt = (time.perf_counter() - t0) * 1000
    print(f"  Inference: {dt:.1f} ms")

    result = interp.get_tensor(out['index'])
    print(f"  Output shape: {list(result.shape)}")
    dets = parse_detections(result, in_w, in_h)
    print_dets(dets)

    draw_img = draw_detections(img, dets)
    draw_img.save(out_path, quality=90)
    print(f"  Saved: {out_path}")
    return dets


# ═══════════════════════════════════════════════════════════
#  2. Frozen GraphDef (.pb)
# ═══════════════════════════════════════════════════════════
def run_pb(model_path, image_path, out_path):
    import tensorflow as tf

    print(f"Model: {os.path.basename(model_path)}")

    with open(model_path, 'rb') as f:
        graph_def = tf.compat.v1.GraphDef()
        graph_def.ParseFromString(f.read())
    print(f"  Nodes: {len(graph_def.node)}")

    # Find input/output nodes
    input_name = None
    output_name = None
    for n in graph_def.node:
        if n.op == 'Placeholder' and input_name is None:
            input_name = n.name
        if n.name == 'Identity':
            output_name = n.name
    print(f"  Input node:  {input_name}")
    print(f"  Output node: {output_name}")

    with tf.compat.v1.Graph().as_default() as graph:
        tf.import_graph_def(graph_def, name='')
        input_tensor = graph.get_tensor_by_name(f'{input_name}:0')
        output_tensor = graph.get_tensor_by_name(f'{output_name}:0')

        shape = input_tensor.shape.as_list()
        in_h = shape[1] if shape[1] is not None else 320
        in_w = shape[2] if shape[2] is not None else 320
        print(f"  Input shape: {shape}")

        # Preprocess: float32 normalized 0-1 (PB outputs pixel-space coords)
        img = Image.open(image_path).convert('RGB').resize((in_w, in_h), Image.BILINEAR)
        pixels = np.array(img, dtype=np.float32) / 255.0
        input_data = pixels.reshape([1, in_h, in_w, 3])

        with tf.compat.v1.Session(graph=graph) as sess:
            t0 = time.perf_counter()
            result = sess.run(output_tensor, {input_tensor: input_data})
            dt = (time.perf_counter() - t0) * 1000
            print(f"  Inference: {dt:.1f} ms")
            print(f"  Output shape: {list(result.shape)}")

    dets = parse_detections(result, in_w, in_h, coords_normalized=False)
    print_dets(dets)

    draw_img = draw_detections(img, dets)
    draw_img.save(out_path, quality=90)
    print(f"  Saved: {out_path}")
    return dets


# ═══════════════════════════════════════════════════════════
#  3. EdgeTPU TFLite (needs libedgetpu + USB/PCI Coral)
# ═══════════════════════════════════════════════════════════
def run_edgetpu(model_path, image_path, out_path):
    import tensorflow as tf

    print(f"Model: {os.path.basename(model_path)}")

    # Try EdgeTPU delegate
    delegate = None
    for lib in ['libedgetpu.so.1', 'libedgetpu.1.dylib', 'edgetpu.dll']:
        try:
            delegate = tf.lite.experimental.load_delegate(lib)
            print(f"  EdgeTPU delegate: {lib}")
            break
        except (ValueError, OSError):
            continue

    if delegate is None:
        print("  WARNING: EdgeTPU delegate not found — will try CPU fallback")
        print("  Install: https://coral.ai/docs/accelerator/get-started/")

    try:
        if delegate:
            interp = tf.lite.Interpreter(
                model_path=model_path,
                experimental_delegates=[delegate])
        else:
            interp = tf.lite.Interpreter(model_path=model_path)
        interp.allocate_tensors()
    except RuntimeError as e:
        print(f"  ERROR: {e}")
        print("  Skipping (no EdgeTPU hardware or delegate)")
        return None

    inp = interp.get_input_details()[0]
    out = interp.get_output_details()[0]
    in_h, in_w = inp['shape'][1], inp['shape'][2]
    in_dtype = inp['dtype']
    print(f"  Input:  {list(inp['shape'])}  dtype={in_dtype.__name__}")
    print(f"  Output: {list(out['shape'])}  dtype={out['dtype'].__name__}")

    # Preprocess
    img = Image.open(image_path).convert('RGB').resize((in_w, in_h), Image.BILINEAR)
    pixels = np.array(img, dtype=np.uint8)

    # Quantize if needed (matches sentai to_tensor)
    if in_dtype == np.int8:
        zp = inp['quantization_parameters']['zero_points'][0]
        scale = inp['quantization_parameters']['scales'][0]
        print(f"  Input quant: scale={scale:.6f} zp={zp}")
        input_data = (pixels.astype(np.int16) + zp).clip(-128, 127).astype(np.int8)
    elif in_dtype == np.uint8:
        input_data = pixels
    else:
        input_data = pixels.astype(np.float32) / 255.0
    input_data = input_data.reshape(inp['shape'])

    # Invoke
    interp.set_tensor(inp['index'], input_data)
    t0 = time.perf_counter()
    interp.invoke()
    dt = (time.perf_counter() - t0) * 1000
    print(f"  Inference: {dt:.1f} ms")

    raw = interp.get_tensor(out['index'])
    print(f"  Output shape: {list(raw.shape)}")

    # Dequantize if needed
    out_scales = out['quantization_parameters']['scales']
    out_zps = out['quantization_parameters']['zero_points']
    if len(out_scales) > 0 and out_scales[0] != 0:
        print(f"  Output quant: scale={out_scales[0]:.6f} zp={out_zps[0]}")
        result = (raw.astype(np.float32) - out_zps[0]) * out_scales[0]
    else:
        result = raw.astype(np.float32)

    dets = parse_detections(result, in_w, in_h)
    print_dets(dets)

    draw_img = draw_detections(img, dets)
    draw_img.save(out_path, quality=90)
    print(f"  Saved: {out_path}")
    return dets


# ═══════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════
def main():
    global CONF_THR
    if len(sys.argv) > 1:
        CONF_THR = float(sys.argv[1])

    print(f"Image:      {IMAGE}")
    print(f"Confidence: {CONF_THR}")
    print()

    # 1. TFLite float32 (CPU)
    if os.path.exists(MODEL_TFL):
        print("=" * 60)
        print("1. TFLite (CPU, float32)")
        print("=" * 60)
        run_tflite(MODEL_TFL, IMAGE, os.path.join(TEST_DIR, "det_tflite.jpg"))
        print()
    else:
        print(f"SKIP: {MODEL_TFL} not found\n")

    # 2. Frozen GraphDef (.pb)
    if os.path.exists(MODEL_PB):
        print("=" * 60)
        print("2. Frozen GraphDef (.pb)")
        print("=" * 60)
        run_pb(MODEL_PB, IMAGE, os.path.join(TEST_DIR, "det_pb.jpg"))
        print()
    else:
        print(f"SKIP: {MODEL_PB} not found\n")

    # 3. EdgeTPU
    if os.path.exists(MODEL_EDG):
        print("=" * 60)
        print("3. EdgeTPU TFLite")
        print("=" * 60)
        run_edgetpu(MODEL_EDG, IMAGE, os.path.join(TEST_DIR, "det_edgetpu.jpg"))
        print()
    else:
        print(f"SKIP: {MODEL_EDG} not found\n")

    print("Done.")


if __name__ == "__main__":
    main()
