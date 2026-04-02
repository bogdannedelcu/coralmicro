"""yolo_common.py — shared constants and helpers for YOLO26n test scripts."""

import numpy as np
from PIL import ImageDraw

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


def class_name(cls_id):
    return COCO_NAMES[cls_id] if 0 <= cls_id < len(COCO_NAMES) else f"cls{cls_id}"


def draw_detections(img, detections):
    """Draw bboxes + labels on a PIL Image (sentai_tpu_draw style). Returns copy."""
    img = img.copy()
    draw = ImageDraw.Draw(img)
    for (x1, y1, x2, y2, score, cls) in detections:
        color = BOX_COLORS[cls % 10]
        draw.rectangle([x1, y1, x2, y2], outline=color, width=2)
        label = f"{class_name(cls)} {int(score * 100)}%"
        lw = len(label) * 7 + 4
        lh = 12
        ly = y1 - lh - 1 if y1 - lh - 1 >= 0 else y1
        draw.rectangle([x1, ly, x1 + lw, ly + lh], fill=color)
        draw.text((x1 + 2, ly + 1), label, fill=(0, 0, 0))
    return img


def print_detections(dets, conf_thr):
    print(f"  {len(dets)} detections (conf>{conf_thr*100:.0f}%)")
    for x1, y1, x2, y2, score, cls in dets:
        print(f"    [{x1:3d},{y1:3d},{x2:3d},{y2:3d}] {score:.3f} {class_name(cls)}")


def parse_post_nms(output, img_w, img_h, conf_thr, coords_normalized=True):
    """Parse [1,300,6] or [300,6] post-NMS output → pixel-space detections.

    Each row: [x1, y1, x2, y2, conf, class_id].
    coords_normalized=True  → multiply by img size
    coords_normalized=False → coords already in pixel space
    """
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


def yolo_nms(data, img_w, img_h, conf_thr, iou_thr=0.45, max_dets=200):
    """NMS for raw YOLO output [C, N] — port of sentai_tpu_detect().

    data: float32 [C, N] where C = 4 + num_classes, N = candidate anchors.
          Rows 0-3: cx, cy, w, h (normalized or pixel).
          Rows 4+:  class confidence scores (after sigmoid).

    Returns list of (x1, y1, x2, y2, conf, class_id) in pixel coords.
    """
    C, N = data.shape
    num_classes = C - 4

    # Phase 1: confidence filter
    candidates = []
    max_coord = 0.0
    for j in range(N):
        scores = data[4:, j]
        best_cls = int(np.argmax(scores))
        best_score = float(scores[best_cls])
        if best_score < conf_thr:
            continue
        cx, cy, w, h = float(data[0, j]), float(data[1, j]), float(data[2, j]), float(data[3, j])
        x1 = cx - w / 2
        y1 = cy - h / 2
        x2 = cx + w / 2
        y2 = cy + h / 2
        mc = max(abs(x1), abs(y1), abs(x2), abs(y2))
        if mc > max_coord:
            max_coord = mc
        candidates.append((best_score, best_cls, x1, y1, x2, y2))

    if not candidates:
        return []

    # Auto-detect normalized vs pixel coords (same as sentai)
    normalized = max_coord < 2.0

    # Phase 2: sort by score descending
    candidates.sort(key=lambda c: -c[0])

    # Phase 3: greedy class-aware NMS
    suppressed = [False] * len(candidates)
    dets = []

    for i, (score_i, cls_i, x1i, y1i, x2i, y2i) in enumerate(candidates):
        if suppressed[i]:
            continue
        # Scale to pixels
        if normalized:
            px1 = max(0, int(x1i * img_w + 0.5))
            py1 = max(0, int(y1i * img_h + 0.5))
            px2 = min(img_w, int(x2i * img_w + 0.5))
            py2 = min(img_h, int(y2i * img_h + 0.5))
        else:
            px1 = max(0, min(img_w, int(x1i + 0.5)))
            py1 = max(0, min(img_h, int(y1i + 0.5)))
            px2 = max(0, min(img_w, int(x2i + 0.5)))
            py2 = max(0, min(img_h, int(y2i + 0.5)))

        if px2 <= px1 or py2 <= py1:
            continue
        dets.append((px1, py1, px2, py2, score_i, cls_i))
        if len(dets) >= max_dets:
            break

        # Suppress same-class overlapping boxes
        area_i = (x2i - x1i) * (y2i - y1i)
        for j in range(i + 1, len(candidates)):
            if suppressed[j] or candidates[j][1] != cls_i:
                continue
            _, _, x1j, y1j, x2j, y2j = candidates[j]
            ix1 = max(x1i, x1j)
            iy1 = max(y1i, y1j)
            ix2 = min(x2i, x2j)
            iy2 = min(y2i, y2j)
            inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
            area_j = (x2j - x1j) * (y2j - y1j)
            iou = inter / (area_i + area_j - inter + 1e-6)
            if iou > iou_thr:
                suppressed[j] = True

    return dets
