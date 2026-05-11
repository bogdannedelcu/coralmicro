#!/usr/bin/env python3
"""Make an MP4 from a SIM run's per-experiment dir.

Reads:
  <exp_dir>/frame_*.ppm   — 640×480 RGB camera frames (every 15th gz frame)
  <exp_dir>/state.tsv     — per-tick tracker + flow + cmd (5 Hz)
  <exp_dir>/flight.tsv    — drone XYZ + RPY from cf2 EKF (50 Hz)
  <exp_dir>/hover_sim.log — fallback if state.tsv missing (older runs)

Each video frame gets:
  - The cat-detection bbox (green for cls=16, red otherwise) scaled from
    300×300 SSD space to 640×480 capture space.
  - HUD in top-left: gz seq, drone XYZ, roll/pitch/yaw, bbox centroid, err.

Usage:
  python3 make_video.py <exp_dir> [<hover_sim.log_path>] [<out.mp4>]

If you omit the log path, defaults to <exp_dir>/state.tsv (the new format)
or falls back to <exp_dir>/hover_sim.log.  Default output:
<exp_dir>/run.mp4 at 5 fps.  Requires: PIL, imageio-ffmpeg.
"""
from __future__ import annotations

import ast
import os
import re
import subprocess
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

COCO = {0: "person", 14: "bench", 15: "bird", 16: "cat", 17: "dog",
        18: "horse", 19: "sheep", 20: "cow", 21: "elephant", 22: "bear",
        56: "chair", 60: "dining_table", 61: "toilet", 62: "tv",
        63: "laptop", 71: "tv", 72: "laptop", 79: "toaster",
        80: "sink", 81: "refrigerator", 85: "clock"}


def parse_state_tsv(tsv: Path) -> tuple[dict[int, dict], list[dict]]:
    """Parse state.tsv into (state_by_fseq, state_list)."""
    by_fseq = {}
    in_order = []
    if not tsv.exists():
        return by_fseq, in_order
    with tsv.open() as f:
        header = f.readline().strip().split("\t")
        for line in f:
            cols = line.strip().split("\t")
            if len(cols) != len(header):
                continue
            try:
                row = {h: int(c) for h, c in zip(header, cols)}
            except ValueError:
                continue
            if row.get("tid", 0) <= 0:
                continue
            entry = dict(it=row["iter"], tid=row["tid"], cls=row["cls"],
                          conf=row["conf"], cx=row["cx"], cy=row["cy"],
                          err_x=row["err_x"], err_y=row["err_y"],
                          x1=row["x1"], y1=row["y1"], x2=row["x2"], y2=row["y2"])
            in_order.append(entry)
            if row.get("fseq", -1) >= 0:
                by_fseq[row["fseq"]] = entry
    return by_fseq, in_order


def parse_states_from_log(log: Path) -> tuple[dict[int, dict], list[dict]]:
    """Fallback: parse hover_sim.log STATE= lines (older runs without state.tsv)."""
    by_fseq = {}
    in_order = []
    with log.open() as f:
        for line in f:
            if not line.startswith("STATE="):
                continue
            try:
                t = ast.literal_eval(line.split("STATE=", 1)[1].strip())
            except Exception:
                continue
            if t[1] <= 0 or len(t) < 16:
                continue
            it, tid, cls, conf, cx, cy, ex, ey = t[0], t[1], t[2], t[3], t[4], t[5], t[6], t[7]
            x1, y1, x2, y2 = t[12], t[13], t[14], t[15]
            entry = dict(it=it, tid=tid, cls=cls, conf=conf,
                          cx=cx, cy=cy, err_x=ex, err_y=ey,
                          x1=x1, y1=y1, x2=x2, y2=y2)
            in_order.append(entry)
            if len(t) >= 17:
                by_fseq[t[16]] = entry
    return by_fseq, in_order


def parse_flight_tsv(tsv: Path) -> list[dict]:
    """Return list of {ts, roll, pitch, yaw, x, y, z} sorted by ts."""
    rows = []
    if not tsv.exists():
        return rows
    with tsv.open() as f:
        header = f.readline().strip().split("\t")
        for line in f:
            cols = line.strip().split("\t")
            if len(cols) != len(header):
                continue
            d = {h: float(c) for h, c in zip(header, cols)}
            rows.append(d)
    rows.sort(key=lambda r: r["ts"])
    return rows


def latest_flight_state_for_frame(flight: list[dict], frame_idx: int, n_frames: int) -> dict | None:
    """Map frame index → closest flight row by linear interpolation of index.
    Flight rows are spaced ~20 ms apart; frames every 15th gz frame (~0.5s).
    Approximation: use proportional indexing across both lists.
    """
    if not flight:
        return None
    idx = min(len(flight) - 1, int(frame_idx * len(flight) / max(1, n_frames)))
    return flight[idx]


def render_frame(ppm: Path, state: dict | None, flight: dict | None, out_path: Path,
                  frame_seq: int):
    img = Image.open(ppm).convert("RGB")
    W, H = img.size
    draw = ImageDraw.Draw(img)
    try:
        font_big = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
        font_med = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
    except Exception:
        font_big = ImageFont.load_default()
        font_med = ImageFont.load_default()

    # bbox overlay (300×300 → 640×480 scaling)
    if state:
        sx, sy = W / 300, H / 300
        x1s, y1s = state["x1"] * sx, state["y1"] * sy
        x2s, y2s = state["x2"] * sx, state["y2"] * sy
        color = (0, 255, 0) if state["cls"] == 16 else (255, 80, 80)
        cls_name = COCO.get(state['cls'], 'cls=' + str(state['cls']))
        label = f"{cls_name} {state['conf']/10:.1f}%"
        draw.rectangle([(x1s, y1s), (x2s, y2s)], outline=color, width=3)
        tw, th = draw.textbbox((0, 0), label, font=font_big)[2:4]
        draw.rectangle([(x1s, y1s - th - 2), (x1s + tw + 6, y1s)], fill=color)
        draw.text((x1s + 3, y1s - th - 2), label, fill=(0, 0, 0), font=font_big)

    # HUD top-left — semi-transparent black panel
    panel_h = 110
    panel = Image.new("RGBA", (260, panel_h), (0, 0, 0, 180))
    img.paste(panel, (5, 5), panel)
    draw = ImageDraw.Draw(img)
    y = 8
    draw.text((10, y), f"frame seq={frame_seq}", fill=(255, 255, 255), font=font_med); y += 16
    if flight:
        draw.text((10, y),
                  f"drone XYZ ({flight['x']:+.2f}, {flight['y']:+.2f}, {flight['z']:+.2f}) m",
                  fill=(120, 255, 120), font=font_med); y += 16
        draw.text((10, y),
                  f"att rpy ({flight['roll']:+5.1f}, {flight['pitch']:+5.1f}, {flight['yaw']:+5.1f})°",
                  fill=(180, 180, 255), font=font_med); y += 16
    if state:
        draw.text((10, y),
                  f"bbox c=({state['cx']:3d},{state['cy']:3d}) err=({state['err_x']:+4d},{state['err_y']:+4d})",
                  fill=(255, 200, 80), font=font_med); y += 16
        draw.text((10, y),
                  f"cls={state['cls']} ({COCO.get(state['cls'],'?')})  conf={state['conf']/10:.1f}%",
                  fill=(255, 200, 80), font=font_med); y += 16
    else:
        draw.text((10, y), "no track", fill=(255, 80, 80), font=font_med); y += 16
    img.save(out_path)


def main():
    if len(sys.argv) < 2:
        print(f"usage: {sys.argv[0]} <exp_dir> [<hover_sim.log>] [<out.mp4>]",
              file=sys.stderr)
        return 1
    frames_dir = Path(sys.argv[1])
    out_mp4 = Path(sys.argv[3]) if len(sys.argv) > 3 else (frames_dir / "run.mp4")
    # Prefer state.tsv (clean disk-replay artifact, Sim.md §10i).  Fall back
    # to legacy hover_sim.log if state.tsv is missing.
    state_tsv = frames_dir / "state.tsv"
    if state_tsv.exists():
        state_by_fseq, state_list = parse_state_tsv(state_tsv)
        print(f"[states] read {state_tsv}", file=sys.stderr)
    else:
        log = Path(sys.argv[2]) if len(sys.argv) > 2 else (frames_dir / "hover_sim.log")
        if not log.exists():
            log = Path("/tmp/hover_sim.log")
        state_by_fseq, state_list = parse_states_from_log(log)
        print(f"[states] fallback read {log}", file=sys.stderr)
    flight = parse_flight_tsv(frames_dir / "flight.tsv")
    print(f"{len(state_by_fseq)} STATE entries with fseq, "
          f"{len(state_list)} total STATE-with-track, "
          f"{len(flight)} flight log rows", file=sys.stderr)
    ppms = sorted(frames_dir.glob("frame_*.ppm"))
    if not ppms:
        print(f"no frame_*.ppm files in {frames_dir}", file=sys.stderr)
        return 1
    # Output frames go to a temp subdir as 0000.png / 0001.png for ffmpeg.
    out_seq_dir = frames_dir / "_vid_seq"
    out_seq_dir.mkdir(exist_ok=True)
    for old in out_seq_dir.glob("*.png"):
        old.unlink()
    # Output filename uses the SAME seq number as the source PPM so
    # `frame_000255.png` always comes from `frame_000255.ppm` — no
    # index-vs-seq confusion possible.  Per embeded.md "replace implicit
    # conventions with explicit APIs".  ffmpeg consumes via -pattern_type
    # glob so non-contiguous numbers work.
    fseq_list = sorted([s for s in state_by_fseq.keys() if s >= 0])
    fseq_match_tolerance = 15   # ±0.5 s at 30 fps
    n_frames = len(ppms)
    n_with_bbox = 0
    n_no_bbox = 0
    for i, ppm in enumerate(ppms):
        m = re.match(r"frame_(\d+)\.ppm", ppm.name)
        frame_seq = int(m.group(1)) if m else i
        state = None
        if fseq_list:
            closest = min(fseq_list, key=lambda s: abs(s - frame_seq))
            if abs(closest - frame_seq) <= fseq_match_tolerance:
                state = state_by_fseq[closest]
        # STRICT: no fallback to proportional index match.  If no fseq
        # within tolerance, the frame gets the HUD but NO bbox — better
        # than showing a wrong bbox that doesn't match the image content.
        if state:
            n_with_bbox += 1
        else:
            n_no_bbox += 1
        flight_row = latest_flight_state_for_frame(flight, i, n_frames)
        # SAME seq number in render filename as source PPM.
        out_png = out_seq_dir / f"frame_{frame_seq:06d}.png"
        render_frame(ppm, state, flight_row, out_png, frame_seq)
    print(f"[overlay] {n_with_bbox} frames with bbox (fseq matched within ±{fseq_match_tolerance}), "
          f"{n_no_bbox} without (no STATE for this frame)", file=sys.stderr)
    print(f"rendered {n_frames} HUD-overlaid PNGs to {out_seq_dir}", file=sys.stderr)
    # Stitch with ffmpeg.  Prefer imageio-ffmpeg's bundled binary so we
    # don't depend on a system-wide install.
    try:
        import imageio_ffmpeg
        ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        ffmpeg_exe = "ffmpeg"
    # Baseline profile + Level 3.0 + faststart for max compatibility
    # (VLC, browser HTML5 video, ffplay, mobile players).
    # Use glob input pattern so frame_NNNNNN.png with arbitrary jumps in
    # seq numbers all get included in seq order.
    cmd = [ffmpeg_exe, "-y",
           "-framerate", "5",
           "-pattern_type", "glob",
           "-i", str(out_seq_dir / "frame_*.png"),
           "-c:v", "libx264",
           "-profile:v", "baseline",
           "-level", "3.0",
           "-pix_fmt", "yuv420p",
           "-preset", "medium",
           "-crf", "23",
           "-movflags", "+faststart",
           str(out_mp4)]
    print("[ffmpeg]", " ".join(cmd), file=sys.stderr)
    rc = subprocess.run(cmd).returncode
    if rc == 0:
        print(f"\n✓ wrote {out_mp4} ({out_mp4.stat().st_size} bytes)", file=sys.stderr)
    return rc


if __name__ == "__main__":
    sys.exit(main())
