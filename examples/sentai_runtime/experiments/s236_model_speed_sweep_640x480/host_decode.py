"""Shared host-side decoder for the headless YOLOv5n p3p4 detectors.

Model I/O (verified by introspection of the cover_v1 int8/edgetpu tflite):
  IN  : uint8 [1,480,640,3], quant scale 1/255, zp 0  (plain RGB image)
  OUT0: int8  [1,30,40,6]  stride 16 (P4), anchor [32,32]
  OUT1: int8  [1,60,80,6]  stride  8 (P3), anchor [16,16]
  last dim 6 = [tx,ty,tw,th,obj,cls] raw logits (pre-sigmoid)

YOLOv5 decode per grid cell (gy,gx):
  x = (sig(tx)*2 - 0.5 + gx) * stride
  y = (sig(ty)*2 - 0.5 + gy) * stride
  w = (sig(tw)*2)^2 * anchor_w
  h = (sig(th)*2)^2 * anchor_h
  conf = sig(obj) * sig(cls)

Runs the EdgeTPU .tflite on a connected Coral USB Accelerator via
ai-edge-litert + libedgetpu delegate. Pure-CPU int8 fallback if no delegate.
"""
import numpy as np

IN_H, IN_W = 480, 640
# (stride, (anchor_w, anchor_h)) keyed by grid height so we map regardless of
# output ordering. 30 rows -> stride 16/P4; 60 rows -> stride 8/P3.
HEAD_BY_GH = {
    30: (16, (32.0, 32.0)),
    60: (8,  (16.0, 16.0)),
}


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def make_interpreter(model_path, use_edgetpu=True):
    from ai_edge_litert.interpreter import Interpreter, load_delegate
    if use_edgetpu:
        dl = load_delegate("libedgetpu.so.1")
        it = Interpreter(model_path=model_path, experimental_delegates=[dl])
    else:
        it = Interpreter(model_path=model_path)
    it.allocate_tensors()
    return it


def preprocess(img_rgb_u8):
    """img_rgb_u8: HxWx3 uint8 at 640x480 -> NHWC uint8 input tensor."""
    assert img_rgb_u8.shape == (IN_H, IN_W, 3), img_rgb_u8.shape
    return img_rgb_u8[None, ...].astype(np.uint8)


def decode_head(arr, scale, zp, stride, anchor):
    """arr: [gh,gw,6] int8 -> list of (x1,y1,x2,y2,conf) in input pixels."""
    gh, gw = arr.shape[0], arr.shape[1]
    real = scale * (arr.astype(np.int32) - zp)
    s = _sigmoid(real)
    gx = np.arange(gw).reshape(1, gw, 1)
    gy = np.arange(gh).reshape(gh, 1, 1)
    x = (s[..., 0:1] * 2.0 - 0.5 + gx) * stride
    y = (s[..., 1:2] * 2.0 - 0.5 + gy) * stride
    w = (s[..., 2:3] * 2.0) ** 2 * anchor[0]
    h = (s[..., 3:4] * 2.0) ** 2 * anchor[1]
    conf = (s[..., 4:5] * s[..., 5:6]).reshape(-1)
    x = x.reshape(-1); y = y.reshape(-1); w = w.reshape(-1); h = h.reshape(-1)
    x1 = x - w / 2; y1 = y - h / 2; x2 = x + w / 2; y2 = y + h / 2
    return np.stack([x1, y1, x2, y2, conf], axis=1)


def _nms(boxes, iou_thr):
    if len(boxes) == 0:
        return boxes
    order = boxes[:, 4].argsort()[::-1]
    keep = []
    b = boxes[order]
    x1, y1, x2, y2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    area = (x2 - x1).clip(0) * (y2 - y1).clip(0)
    sup = np.zeros(len(b), bool)
    for i in range(len(b)):
        if sup[i]:
            continue
        keep.append(b[i])
        xx1 = np.maximum(x1[i], x1); yy1 = np.maximum(y1[i], y1)
        xx2 = np.minimum(x2[i], x2); yy2 = np.minimum(y2[i], y2)
        iw = (xx2 - xx1).clip(0); ih = (yy2 - yy1).clip(0)
        inter = iw * ih
        iou = inter / (area[i] + area - inter + 1e-6)
        sup |= (iou > iou_thr)
        sup[i] = True
    return np.array(keep)


def infer(interp, img_rgb_u8, conf_thr=0.25, iou_thr=0.45):
    """Returns Nx5 array [x1,y1,x2,y2,conf] in 640x480 input pixels."""
    inp = interp.get_input_details()[0]
    interp.set_tensor(inp["index"], preprocess(img_rgb_u8))
    interp.invoke()
    allb = []
    for od in interp.get_output_details():
        arr = interp.get_tensor(od["index"])[0]  # [gh,gw,6]
        gh = arr.shape[0]
        stride, anchor = HEAD_BY_GH[gh]
        scale, zp = od["quantization"]
        allb.append(decode_head(arr, scale, zp, stride, anchor))
    boxes = np.concatenate(allb, axis=0)
    boxes = boxes[boxes[:, 4] >= conf_thr]
    return _nms(boxes, iou_thr)
