#!/usr/bin/env python3
"""render_corners.py — montage of (color JPEG | 40×30 gray) per corner.
One row per WAIT corner, shows what the cam sees vs what the SAD
algorithm gets after step-16 decimation.
"""
import pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


HERE = pathlib.Path(__file__).parent
SHOTS = HERE / "corners"


def read_pgm(p):
    raw = p.read_bytes()
    assert raw.startswith(b"P5\n")
    nl1 = raw.index(b"\n", 3)
    nl2 = raw.index(b"\n", nl1 + 1)
    w, h = (int(x) for x in raw[3:nl1].split())
    data = np.frombuffer(raw[nl2 + 1:], dtype=np.uint8).reshape(h, w)
    return data


def main():
    corners = ["WAIT_start", "WAIT_c1", "WAIT_c2", "WAIT_c3", "WAIT_end"]
    fig, axes = plt.subplots(len(corners), 2, figsize=(11, 3.2 * len(corners)))
    for i, name in enumerate(corners):
        jpg = SHOTS / f"{name}_cam0.jpg"
        pgm = SHOTS / f"{name}_gray40x30.pgm"
        if jpg.exists():
            img = Image.open(jpg)
            axes[i, 0].imshow(np.array(img))
            axes[i, 0].set_title(f"{name} — cam0 640×480 JPEG")
            axes[i, 0].set_xticks([]); axes[i, 0].set_yticks([])
        if pgm.exists():
            gray = read_pgm(pgm)
            axes[i, 1].imshow(gray, cmap="gray", vmin=0, vmax=255,
                              interpolation="nearest")
            axes[i, 1].set_title(f"{name} — M4 gray 40×30 (SAD input)")
            axes[i, 1].set_xticks([]); axes[i, 1].set_yticks([])
    fig.suptitle("E34 — square-walk corners: color cam vs M4 flow gray "
                 "(both captured at same corner)", fontsize=12)
    fig.tight_layout()
    out = HERE / "corners_montage.png"
    fig.savefig(out, dpi=100); plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
