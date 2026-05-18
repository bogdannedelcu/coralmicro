#!/usr/bin/env python3
"""rotate_pgm.py — produce a rotated copy of a 320x240 PGM around its
image centre.  Used by s175 to test if sentai_aruco's PnP gives the
same tvec_cam.z for the same physical scene viewed under different
camera in-plane rotations (drone yaw).

If PnP is rotation-invariant on Z (correct), the two PGMs should
yield identical per-marker tvec_cam[2].  If they differ → planar-
marker ambiguity confirmed.

Anti-cheat: pure offline image manipulation, no Gazebo touched.
"""
from __future__ import annotations
import argparse
import sys

import numpy as np
import cv2


def load_pgm(path: str) -> np.ndarray:
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"could not load {path}")
    return img


def save_pgm(img: np.ndarray, path: str) -> None:
    cv2.imwrite(path, img)


def rotate_around_centre(img: np.ndarray, angle_deg: float) -> np.ndarray:
    h, w = img.shape
    M = cv2.getRotationMatrix2D((w / 2.0, h / 2.0), angle_deg, 1.0)
    # Constant background fill = mean of image (so PnP detector doesn't
    # see weird borders).
    bg = int(img.mean())
    return cv2.warpAffine(img, M, (w, h),
                            flags=cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_CONSTANT,
                            borderValue=bg)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input",  required=True,
                     help="input PGM (e.g. frame_original.pgm)")
    ap.add_argument("--angle",  type=float, default=35.0,
                     help="rotation angle deg (CCW positive)")
    ap.add_argument("--output", required=True,
                     help="output PGM")
    args = ap.parse_args()

    img = load_pgm(args.input)
    rot = rotate_around_centre(img, args.angle)
    save_pgm(rot, args.output)
    print(f"wrote {args.output} ({rot.shape}, rotation={args.angle:+.1f}°)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
