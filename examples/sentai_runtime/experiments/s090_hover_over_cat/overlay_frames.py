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

# COCO classes — Coral's official mapping from
# /home/bogdan/work/edge/edgetpu/test_data/coco_labels.txt.  pycoral
# `id` field is 0-indexed.  16 == CAT (not dog as initially mis-labelled
# 2026-05-11; user caught the bug on frame_000855 visual check).
COCO = {0: "person", 1: "bicycle", 2: "car", 14: "bench", 15: "bird",
        16: "cat", 17: "dog", 18: "horse", 19: "sheep", 20: "cow",
        21: "elephant", 22: "bear", 56: "chair", 60: "dining_table",
        61: "toilet", 62: "tv", 63: "laptop", 64: "mouse",
        65: "remote", 67: "cell_phone", 71: "tv", 72: "laptop",
        73: "mouse", 74: "remote", 78: "microwave", 79: "oven",
        80: "toaster", 81: "sink", 82: "refrigerator", 84: "book",
        85: "clock"}


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


def parse_hover_log(log: Path):
    """Return:
      dets_by_iter: {iter: [(x1,y1,x2,y2,conf,cls), ...]}  — from HOVER_LOGIC_DETS
      state_by_fseq: {gz_frame_seq: (tid, cls, conf, cx, cy, x1, y1, x2, y2)}
                       from STATE= (only tid>0, 17-field with fseq).  Falls back
                       to keying by iter for older 16-field STATE.
    """
    dets = {}
    state_by_fseq = {}
    state_by_iter = {}   # legacy fallback
    last_iter = -1
    with log.open() as f:
        for line in f:
            if line.startswith("STATE="):
                try:
                    t = ast.literal_eval(line.split("STATE=", 1)[1].strip())
                    last_iter = t[0]
                    if t[1] > 0:
                        if len(t) >= 17:
                            # NEW: gz frame seq at t[16] — keys by seq.
                            entry = (t[1], t[2], t[3], t[4], t[5],
                                     t[12], t[13], t[14], t[15])
                            state_by_fseq[t[16]] = entry
                        elif len(t) >= 16:
                            state_by_iter[t[0]] = (t[1], t[2], t[3], t[4], t[5],
                                                   t[12], t[13], t[14], t[15])
                        else:
                            state_by_iter[t[0]] = (t[1], t[2], t[3], t[4], t[5],
                                                   None, None, None, None)
                except Exception:
                    pass
            elif "HOVER_LOGIC_DETS=" in line:
                payload = line.split("HOVER_LOGIC_DETS=", 1)[1].strip()
                dets[last_iter] = parse_dets(payload)
    return dets, state_by_fseq, state_by_iter


def main():
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <frames_dir> <hover_sim.log>", file=sys.stderr)
        return 1
    frames_dir = Path(sys.argv[1])
    log = Path(sys.argv[2])
    if not frames_dir.is_dir():
        print(f"no dir: {frames_dir}", file=sys.stderr)
        return 1
    dets_by_iter, state_by_fseq, state_by_iter = parse_hover_log(log)
    ppms = sorted(frames_dir.glob("frame_*.ppm"))
    print(f"{len(ppms)} dumped frames, {len(dets_by_iter)} DETS, "
          f"{len(state_by_fseq)} STATE-by-fseq (preferred), "
          f"{len(state_by_iter)} STATE-by-iter (fallback)", file=sys.stderr)
    fseq_sorted = sorted(state_by_fseq.keys()) if state_by_fseq else []
    iter_sorted = sorted(state_by_iter.keys()) if state_by_iter else []
    n_overlaid = n_with_track = 0
    for ppm in ppms:
        # Parse gz frame seq from filename frame_NNNNNN.ppm
        try:
            frame_seq = int(ppm.stem.replace("frame_", ""))
        except Exception:
            frame_seq = -1
        chosen_dets = []
        if fseq_sorted and frame_seq >= 0:
            # Match by FSEQ — find STATE entry whose fseq is closest.
            best_fseq = min(fseq_sorted, key=lambda s: abs(s - frame_seq))
            # Only use if within reasonable window (±15 frames at 30 fps = 0.5 s).
            if abs(best_fseq - frame_seq) <= 30:
                tid, cls, conf, cx, cy, x1, y1, x2, y2 = state_by_fseq[best_fseq]
                if x1 is not None:
                    chosen_dets = [(x1, y1, x2, y2, conf, cls)]
                    n_with_track += 1
        elif iter_sorted:
            # Legacy fallback: index-ratio match (known buggy on misaligned rates).
            i = ppms.index(ppm)
            idx = min(len(iter_sorted) - 1, int(i * len(iter_sorted) / max(1, len(ppms))))
            it = iter_sorted[idx]
            tid, cls, conf, cx, cy, x1, y1, x2, y2 = state_by_iter[it]
            if x1 is not None:
                chosen_dets = [(x1, y1, x2, y2, conf, cls)]
                n_with_track += 1
        out = ppm.with_suffix(".png")
        overlay(ppm, chosen_dets, out)
        n_overlaid += 1
    print(f"wrote {n_overlaid} annotated PNGs to {frames_dir} ({n_with_track} with track-bbox)",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
