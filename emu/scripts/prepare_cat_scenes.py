#!/usr/bin/env python3
"""Pre-process cat_640x480.bmp into tiny grayscale frames for the B8.7c
Flow-offset emulator test (sentai_emu_flowest).

Renode injects each scene into guest SDRAM via `sysbus LoadBinary`, so we
need the bytes that land at `g_frame_buffer` to be exactly `FRAME_W *
FRAME_H` 8-bit grayscale pixels with no header.

Strategy:
  - read the B7 cat reference BMP from s209;
  - center-crop to a square, downsample to FRAME_W x FRAME_H grayscale by
    averaging (no PIL bilinear so the byte-level result is deterministic
    and reproducible across hosts);
  - generate N shifted variants in the small-frame coordinate space (NOT
    the original 640x480 space — a 2-pixel shift at 640x480 is < 0.1 px
    after downsampling and would be undetectable).
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np
from PIL import Image

REPO = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_SRC = (
    REPO
    / "examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e"
    / "iter78_virtual_camera_flow_static_shift/fs_root/images/cat_640x480.bmp"
)
DEFAULT_OUT = REPO / "emu/output/scenes"

FRAME_W = 32
FRAME_H = 32


def load_grayscale_centered(path: pathlib.Path) -> np.ndarray:
    img = Image.open(path).convert("L")
    w, h = img.size
    s = min(w, h)
    left = (w - s) // 2
    top = (h - s) // 2
    crop = img.crop((left, top, left + s, top + s))
    small = crop.resize((FRAME_W, FRAME_H), Image.BOX)
    return np.asarray(small, dtype=np.uint8)


def shift_x(base: np.ndarray, dx: int) -> np.ndarray:
    """Shift the image by `dx` pixels along X (positive = right).  Vacated
    pixels are filled with the BASE edge column so the apparent motion is a
    clean translation rather than a black-band injection that would dominate
    SAD with `dx=0`."""
    h, w = base.shape
    shifted = np.empty_like(base)
    if dx == 0:
        return base.copy()
    if dx > 0:
        shifted[:, dx:] = base[:, : w - dx]
        shifted[:, :dx] = base[:, :1]  # repeat left edge
    else:
        n = -dx
        shifted[:, : w - n] = base[:, n:]
        shifted[:, w - n :] = base[:, -1:]  # repeat right edge
    return shifted


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", type=pathlib.Path, default=DEFAULT_SRC)
    parser.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    if not args.src.exists():
        print(f"missing source BMP: {args.src}", file=sys.stderr)
        return 2

    args.out.mkdir(parents=True, exist_ok=True)

    base = load_grayscale_centered(args.src)
    assert base.shape == (FRAME_H, FRAME_W), base.shape

    # B8.7c operator model: a single cat photo, panned by 1 px per frame.
    # Six frames means a clean constant-velocity test: frame 1 is prime
    # (no prev), frames 2..6 should each detect dx=+1.
    scenes = {
        f"scene_off_{i}.bin": shift_x(base, i) for i in range(6)
    }
    # Keep a couple of named variants on disk too, for ad-hoc tests that
    # want known larger jumps (history: B7 cat-flow test used these).
    scenes["scene_base.bin"] = base
    scenes["scene_shift_x_p2.bin"] = shift_x(base, +2)
    scenes["scene_shift_x_p4.bin"] = shift_x(base, +4)
    scenes["scene_shift_x_n2.bin"] = shift_x(base, -2)

    for name, arr in scenes.items():
        out_path = args.out / name
        out_path.write_bytes(arr.tobytes())
        # Also write a tiny PNG preview for human inspection.
        png_path = out_path.with_suffix(".png")
        Image.fromarray(arr, mode="L").resize((128, 128), Image.NEAREST).save(
            png_path
        )
        print(f"wrote {out_path}  ({arr.shape}, mean={arr.mean():.1f})")

    return 0


if __name__ == "__main__":
    sys.exit(main())
