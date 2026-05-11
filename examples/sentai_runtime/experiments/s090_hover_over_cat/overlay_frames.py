#!/usr/bin/env python3
"""Post-run overlay: read PPM frames dumped by camera_bridge_recv +
HOVER_LOGIC_DETS lines from hover_sim.log, draw bboxes with
class+conf labels.  Output PNG to same dir.

Usage:
  python3 overlay_frames.py <frames_dir> <hover_sim.log>
"""
from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

# COCO classes (raw indices from MobileNet V2 COCO17 EdgeTPU output)
COCO = {0: "person", 1: "bicycle", 2: "car", 14: "bird", 15: "cat",
        16: "dog", 17: "horse", 19: "cow", 20: "elephant", 21: "bear",
        56: "chair", 60: "dining_table", 61: "toilet", 62: "tv",
        63: "laptop", 64: "mouse", 67: "cell_phone", 71: "tv",
        72: "laptop", 73: "mouse", 79: "toaster", 80: "sink",
        81: "refrigerator", 84: "book", 85: "clock"}


def parse_dets(payload: str) -> list[tuple]:
    """Parse HOVER_LOGIC_DETS= ((x1,y1,x2,y2,conf,cls), ...)."""
    try:
        return list(ast.literal_eval(payload))
    except Exception as e:
        print(f"parse err: {e}", file=sys.stderr)
        return []


def overlay(ppm_path: Path, dets: list[tuple], out_path: Path):
    img = Image.open(ppm_path).convert("RGB")
    W, H = img.size
    # Detections are in 300x300 image-coord space (TPU input).  Scale to
    # source 640x480.
    sx, sy = W / 300, H / 300
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
    except Exception:
        font = ImageFont.load_default()
    for d in dets:
        if len(d) < 6:
            continue
        x1, y1, x2, y2, conf, cls = d
        # bbox in 300x300 → scale
        x1s, y1s = x1 * sx, y1 * sy
        x2s, y2s = x2 * sx, y2 * sy
        label = f"{COCO.get(cls, f'cls={cls}')} {conf/10:.1f}%"
        # Color: green for class 16 (cat/dog), red otherwise
        color = (0, 255, 0) if cls == 16 else (255, 80, 80)
        draw.rectangle([(x1s, y1s), (x2s, y2s)], outline=color, width=3)
        # Label background + text
        tw, th = draw.textbbox((0, 0), label, font=font)[2:4]
        draw.rectangle([(x1s, y1s - th - 2), (x1s + tw + 6, y1s)], fill=color)
        draw.text((x1s + 3, y1s - th - 2), label, fill=(0, 0, 0), font=font)
    img.save(out_path)


def main():
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <frames_dir> <hover_sim.log>", file=sys.stderr)
        return 1
    frames_dir = Path(sys.argv[1])
    log = Path(sys.argv[2])
    if not frames_dir.is_dir():
        print(f"no dir: {frames_dir}", file=sys.stderr)
        return 1
    # Scan log for HOVER_LOGIC_DETS — we don't have per-frame timestamps
    # so just collect the latest DETS at each STATE iter, then use them
    # for the nearest dumped frame in time order.
    dets_by_iter = {}
    state_iter = -1
    with log.open() as f:
        for line in f:
            if line.startswith("STATE="):
                try:
                    t = ast.literal_eval(line.split("STATE=", 1)[1].strip())
                    state_iter = t[0]
                except Exception:
                    pass
            elif "HOVER_LOGIC_DETS=" in line:
                payload = line.split("HOVER_LOGIC_DETS=", 1)[1].strip()
                dets_by_iter[state_iter] = parse_dets(payload)
    if not dets_by_iter:
        print("no DETS in log — running with no overlay", file=sys.stderr)
    # Walk dumped frames sorted by seq.
    ppms = sorted(frames_dir.glob("frame_*.ppm"))
    print(f"{len(ppms)} dumped frames, {len(dets_by_iter)} DETS snapshots", file=sys.stderr)
    iters_sorted = sorted(dets_by_iter.keys())
    n_overlaid = 0
    for i, ppm in enumerate(ppms):
        # Use Nth-fraction-through DETS for Nth-fraction-through frames.
        if iters_sorted:
            idx = min(len(iters_sorted) - 1, int(i * len(iters_sorted) / max(1, len(ppms))))
            dets = dets_by_iter[iters_sorted[idx]]
        else:
            dets = []
        out = ppm.with_suffix(".png")
        overlay(ppm, dets, out)
        n_overlaid += 1
    print(f"wrote {n_overlaid} annotated PNGs to {frames_dir}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
