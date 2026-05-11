"""Build MP4 from captured PPMs with ArUco bbox overlay + telemetry HUD.

Reads frames from $SENTAI_DUMP_FRAMES_DIR (or arg), detects ArUco on each,
draws marker contours + IDs + center crosshair, adds a HUD with EKF pose
from hover_log.json (matched by frame seq if available), writes MP4.

Usage:
    python3 make_video_aruco.py [<frames_dir>]
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "s090_hover_over_cat"))
from aruco_detector import detect_markers, read_ppm, KNOWN_POSITIONS_M

# Color per marker id (BGR for cv2)
COLORS = {
    0: (0, 255, 0),     # green   NE
    1: (255, 200, 0),   # cyan    NW
    2: (0, 200, 255),   # orange  SW
    3: (255, 0, 255),   # magenta SE
}


def draw_marker(img, m):
    color = COLORS.get(m.id, (200, 200, 200))
    pts = m.corners.astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(img, [pts], isClosed=True, color=color, thickness=2)
    cx, cy = int(m.cx), int(m.cy)
    cv2.circle(img, (cx, cy), 4, color, -1)
    cv2.putText(img, f"id={m.id}", (cx + 8, cy - 8),
                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def draw_hud(img, fseq, n_det, ekf=None, pnp_z=None):
    h, w = img.shape[:2]
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, 60), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, img, 0.45, 0, img)
    txt1 = f"fseq={fseq}  n_det={n_det}"
    cv2.putText(img, txt1, (10, 22),
                 cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    if ekf is not None:
        txt2 = f"EKF x={ekf[0]:+.2f} y={ekf[1]:+.2f} z={ekf[2]:.2f}m"
        if pnp_z is not None:
            txt2 += f"   PnP z={pnp_z:.2f}m"
        cv2.putText(img, txt2, (10, 48),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 220, 255), 1, cv2.LINE_AA)
    # image-center crosshair
    cx, cy = w // 2, h // 2
    cv2.line(img, (cx - 8, cy), (cx + 8, cy), (80, 80, 80), 1)
    cv2.line(img, (cx, cy - 8), (cx, cy + 8), (80, 80, 80), 1)


def main(argv):
    frames_dir = Path(argv[1] if len(argv) > 1
                      else __import__("os").environ.get(
                          "SENTAI_DUMP_FRAMES_DIR", "/tmp"))
    ppms = sorted(frames_dir.glob("frame_*.ppm"))
    if not ppms:
        print(f"no PPMs in {frames_dir}", file=sys.stderr)
        return 1
    print(f"[video] {len(ppms)} frames in {frames_dir}", file=sys.stderr)

    # Optional: pose log for HUD
    log_path = Path(__file__).parent / "hover_log.json"
    samples_by_fseq = {}
    if log_path.exists():
        log = json.loads(log_path.read_text())
        for s in log.get("samples", []):
            samples_by_fseq[s["fseq"]] = s

    # Probe first frame for size
    first = read_ppm(ppms[0])
    h, w = first.shape[:2]
    out_path = frames_dir / "aruco_overlay.mp4"
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    # Camera publishes ~16 fps but we keep video at 10 fps for watchability
    vw = cv2.VideoWriter(str(out_path), fourcc, 10.0, (w, h))
    if not vw.isOpened():
        print(f"[video] FAIL open writer", file=sys.stderr)
        return 1

    for i, ppm in enumerate(ppms):
        try:
            rgb = read_ppm(ppm)
        except Exception as e:
            print(f"[video] skip {ppm.name}: {e}", file=sys.stderr)
            continue
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        try:
            dets = detect_markers(rgb, estimate_pose=False)
        except Exception:
            dets = {}
        for m in dets.values():
            draw_marker(bgr, m)
        try:
            fseq = int(ppm.stem.replace("frame_", ""))
        except Exception:
            fseq = i
        sample = samples_by_fseq.get(fseq)
        ekf = sample.get("ekf") if sample else None
        pnp_z = sample.get("pnp", [None, None, None])[2] if sample else None
        draw_hud(bgr, fseq, len(dets), ekf=ekf, pnp_z=pnp_z)
        vw.write(bgr)
        if i % 30 == 0:
            print(f"[video]   {i+1}/{len(ppms)} frames", file=sys.stderr)
    vw.release()
    print(f"[video] wrote {out_path}  ({out_path.stat().st_size} bytes)",
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
