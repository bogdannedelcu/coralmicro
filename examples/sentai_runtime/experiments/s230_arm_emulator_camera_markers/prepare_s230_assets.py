#!/usr/bin/env python3
"""Generate deterministic WhyCon assets for s230 camera/markers emu smoke."""

from __future__ import annotations

import math
from pathlib import Path


ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets"
PGM = ASSETS / "whycon_320x240.pgm"
BMP = ASSETS / "whycon_640x480.bmp"


def draw_whycon(w: int = 320, h: int = 240) -> bytearray:
    img = bytearray([220] * (w * h))
    cx, cy = 160.0, 120.0
    outer = 34.0
    white_inner = 23.0
    black_dot = 9.0
    for y in range(h):
        for x in range(w):
            d = math.hypot(x + 0.5 - cx, y + 0.5 - cy)
            v = 220
            if d <= outer:
                v = 25
            if d <= white_inner:
                v = 245
            if d <= black_dot:
                v = 25
            img[y * w + x] = v
    return img


def write_pgm(path: Path, gray: bytes, w: int, h: int) -> None:
    path.write_bytes(f"P5\n{w} {h}\n255\n".encode("ascii") + gray)


def write_bmp_2x(path: Path, gray: bytes, w: int, h: int) -> None:
    bw, bh = w * 2, h * 2
    row_bytes = ((bw * 3 + 3) // 4) * 4
    pixels = bytearray(row_bytes * bh)
    for y in range(bh):
        src_y = y // 2
        out_y = bh - 1 - y
        for x in range(bw):
            v = gray[src_y * w + (x // 2)]
            off = out_y * row_bytes + x * 3
            pixels[off + 0] = v
            pixels[off + 1] = v
            pixels[off + 2] = v

    file_size = 14 + 40 + len(pixels)
    header = bytearray()
    header += b"BM"
    header += file_size.to_bytes(4, "little")
    header += (0).to_bytes(4, "little")
    header += (14 + 40).to_bytes(4, "little")
    header += (40).to_bytes(4, "little")
    header += bw.to_bytes(4, "little", signed=True)
    header += bh.to_bytes(4, "little", signed=True)
    header += (1).to_bytes(2, "little")
    header += (24).to_bytes(2, "little")
    header += (0).to_bytes(4, "little")
    header += len(pixels).to_bytes(4, "little")
    header += (2835).to_bytes(4, "little", signed=True)
    header += (2835).to_bytes(4, "little", signed=True)
    header += (0).to_bytes(4, "little")
    header += (0).to_bytes(4, "little")
    path.write_bytes(header + pixels)


def main() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    gray = draw_whycon()
    write_pgm(PGM, gray, 320, 240)
    write_bmp_2x(BMP, gray, 320, 240)
    print(f"wrote {PGM} ({PGM.stat().st_size} bytes)")
    print(f"wrote {BMP} ({BMP.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
