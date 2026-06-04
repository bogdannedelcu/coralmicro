#!/usr/bin/env python3
"""Create the 80x60 grayscale cat frame used by S214."""

from __future__ import annotations

import argparse
import pathlib
import sys

try:
    from PIL import Image
except ImportError as exc:  # pragma: no cover - runner reports this clearly.
    raise SystemExit("Pillow is required: python3 -m pip install pillow") from exc


ROOT = pathlib.Path(__file__).resolve().parents[4]
EXP = pathlib.Path(__file__).resolve().parent
DEFAULT_SOURCE = (
    ROOT
    / "examples/sentai_runtime/experiments/s211_tfl_cpu_sim_smoke"
    / "iter03_coco_cat_tfl_cpu/fs_root/images/cat_640x480.bmp"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=pathlib.Path, default=DEFAULT_SOURCE)
    parser.add_argument("--out-dir", type=pathlib.Path, default=EXP / "assets")
    args = parser.parse_args()

    src = args.source
    if not src.exists():
        print(f"missing source image: {src}", file=sys.stderr)
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)
    image = Image.open(src).convert("L").resize((80, 60), Image.Resampling.BOX)
    raw = image.tobytes()
    (args.out_dir / "cat80_base.bin").write_bytes(raw)
    image.save(args.out_dir / "cat80_base.png")
    print(f"wrote {args.out_dir / 'cat80_base.bin'} ({len(raw)} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
