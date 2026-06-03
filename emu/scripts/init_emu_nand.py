#!/usr/bin/env python3
"""Initialize the B8 ARM-emulator raw NAND image.

The guest firmware still runs production FileX/LevelX.  This script only
creates the host-backed raw NAND bytes that Renode's nand_bridge serves to the
guest at the page/program/erase boundary.
"""

from __future__ import annotations

import argparse
import pathlib


ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_IMAGE = ROOT / "emu" / "output" / "sentai_emu_nand.bin"
PAGE_RAW_BYTES = 2112
PAGES_PER_BLOCK = 64
TOTAL_BLOCKS = 1024
IMAGE_SIZE = PAGE_RAW_BYTES * PAGES_PER_BLOCK * TOTAL_BLOCKS
CHUNK_SIZE = 1024 * 1024


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", type=pathlib.Path, default=DEFAULT_IMAGE)
    parser.add_argument("--reset", action="store_true")
    args = parser.parse_args()

    image = args.image
    image.parent.mkdir(parents=True, exist_ok=True)
    if image.exists() and not args.reset and image.stat().st_size == IMAGE_SIZE:
        print(f"emu NAND image already present: {image} ({IMAGE_SIZE} bytes)")
        return 0

    ff = bytes([0xFF]) * CHUNK_SIZE
    remaining = IMAGE_SIZE
    with image.open("wb") as f:
        while remaining:
            n = CHUNK_SIZE if remaining >= CHUNK_SIZE else remaining
            f.write(ff[:n])
            remaining -= n
    print(f"initialized emu NAND image: {image} ({IMAGE_SIZE} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
