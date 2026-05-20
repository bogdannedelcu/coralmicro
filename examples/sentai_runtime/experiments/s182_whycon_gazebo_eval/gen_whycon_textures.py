#!/usr/bin/env python3
"""Generate Krajník/WhyCon PNG textures + copy them into the Gazebo
materials/textures directory (both the repo copy and the live
CrazySim install).  Output one canonical texture (Krajník-pattern
is ID-less — WhyCode would add bits but we're not doing that here).

Geometry — must match the WhyCon detector + W3 sample geometry in
sentai_aruco.cc:

  r ≤ 0.20 R           → DARK   (centre localiser dot, fits Phase W3 3x3 mean)
  0.20 R < r ≤ 0.60 R  → WHITE  (inner disc, 0.55 R radial samples)
  0.60 R < r ≤ R       → DARK   (outer annulus, 0.95 R radial samples)
  r > R                → WHITE  (background)

Geometry locked by:
  - examples/sentai_runtime/sentai_aruco.cc :: WHYCON_W3_INNER_RADIUS_FRAC = 0.55
  - examples/sentai_runtime/sentai_aruco.cc :: WHYCON_W3_OUTER_RADIUS_FRAC = 0.95
  - examples/sentai_runtime/sentai_aruco.cc :: WHYCON_PNP_ANNULUS_FACTOR = 1.16619
    (assumes r_inner/r_outer = 0.6)

Texture size: 512×512, white background, marker fills the texture
(R_pix = 232, leaves 24 px margin on each side so the marker doesn't
clip when Gazebo applies non-zero edge blending).

Run: python3 examples/sentai_runtime/experiments/s182_whycon_gazebo_eval/gen_whycon_textures.py
"""
from __future__ import annotations

import os
import shutil
from pathlib import Path

from PIL import Image, ImageDraw


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT  = SCRIPT_DIR.parents[3]
REPO_TEXTURE_DIR = REPO_ROOT / "sim" / "gazebo" / "materials" / "textures"
CRAZYSIM_TEXTURE_DIR = Path.home() / (
    "work/crazyflie/CrazySim/crazyflie-firmware/tools/"
    "crazyflie-simulation/simulator_files/gazebo/materials/textures"
)

TEXTURE_SIZE = 512
MARGIN = 24
R_OUTER = (TEXTURE_SIZE // 2) - MARGIN     # 232 px
R_WHITE = int(R_OUTER * 0.60)               # 139 px
R_DOT   = int(R_OUTER * 0.20)               # 46 px

DARK    = (20, 20, 20)
WHITE   = (235, 235, 235)
BG      = (255, 255, 255)


def draw_krajnik_marker(out_path: Path) -> None:
    img = Image.new("RGB", (TEXTURE_SIZE, TEXTURE_SIZE), BG)
    draw = ImageDraw.Draw(img)
    cx = cy = TEXTURE_SIZE // 2

    def disc(r, color):
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=color)

    disc(R_OUTER, DARK)
    disc(R_WHITE, WHITE)
    disc(R_DOT,   DARK)

    img.save(out_path, "PNG", optimize=True)
    print(f"  {out_path}  ({R_OUTER}px outer / {R_WHITE}px white / {R_DOT}px centre)")


def main() -> int:
    # Single canonical PNG.  All 4 world markers use the same texture
    # (Krajník is ID-less; multi-marker constellation pose disambiguates
    # which is which from geometric layout — see W19-T3).
    for dest in (REPO_TEXTURE_DIR, CRAZYSIM_TEXTURE_DIR):
        if not dest.exists():
            print(f"[gen_whycon] skip {dest} (not present)")
            continue
        dest.mkdir(parents=True, exist_ok=True)
        out = dest / "whycon_krajnik.png"
        draw_krajnik_marker(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
