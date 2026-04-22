#!/usr/bin/env python3
"""brighten.py — apply four brightness corrections to each JPEG and
emit a 5-wide comparison strip (original + 4 methods) per image.

Methods:
  gamma   : y = 255 * (x/255)^(1/g)  — brightens shadows, keeps highlights
  linear  : scale up mean to target, clip — simple but blows highlights
  stretch : map (p5..p95) → (0..255) — boosts local contrast
  hist_eq : per-channel histogram equalization — max detail, may color shift
"""
import pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image


HERE = pathlib.Path(__file__).parent
CORNERS = HERE / "corners"


def gamma(arr, g=2.2):
    x = arr.astype(np.float32) / 255.0
    return np.clip(np.power(x, 1.0 / g) * 255.0, 0, 255).astype(np.uint8)


def linear_to(arr, target_mean=140):
    mean = arr.mean()
    if mean < 1: return arr
    scale = target_mean / mean
    return np.clip(arr.astype(np.float32) * scale, 0, 255).astype(np.uint8)


def stretch(arr, low_pct=2, high_pct=98):
    lo, hi = np.percentile(arr, [low_pct, high_pct])
    if hi - lo < 1: return arr
    x = (arr.astype(np.float32) - lo) * (255.0 / (hi - lo))
    return np.clip(x, 0, 255).astype(np.uint8)


def hist_eq(arr):
    # per-channel equalisation — keeps colour roughly intact, max detail
    out = np.empty_like(arr)
    for c in range(arr.shape[2]):
        ch = arr[..., c]
        hist, _ = np.histogram(ch, 256, [0, 256])
        cdf = hist.cumsum()
        cdf_masked = np.ma.masked_equal(cdf, 0)
        cdf_masked = (cdf_masked - cdf_masked.min()) * 255 / \
                     (cdf_masked.max() - cdf_masked.min())
        cdf_final = np.ma.filled(cdf_masked, 0).astype(np.uint8)
        out[..., c] = cdf_final[ch]
    return out


def process(jpg_path):
    arr = np.array(Image.open(jpg_path))
    out = {
        "original": arr,
        "gamma 2.2": gamma(arr, 2.2),
        "linear mean=140": linear_to(arr, 140),
        "stretch 2-98%": stretch(arr),
        "hist eq": hist_eq(arr),
    }
    return out


def main():
    # Process all corner JPEGs, one row per corner.
    jpgs = sorted(CORNERS.glob("*_cam0.jpg"))
    if not jpgs:
        print(f"No JPEGs in {CORNERS}")
        return
    print(f"Processing {len(jpgs)} images")

    fig, axes = plt.subplots(len(jpgs), 5,
                             figsize=(20, 3.2 * len(jpgs)))
    if len(jpgs) == 1: axes = [axes]
    for i, p in enumerate(jpgs):
        variants = process(p)
        for j, (name, arr) in enumerate(variants.items()):
            ax = axes[i][j]
            ax.imshow(arr)
            title = f"{p.stem}\n{name}  L̄={arr.mean():.0f}" if j == 0 \
                    else f"{name}  L̄={arr.mean():.0f}"
            ax.set_title(title, fontsize=9)
            ax.set_xticks([]); ax.set_yticks([])
            # Save each corrected image as a separate PNG for easy
            # download.  Keep originals untouched.
            if name != "original":
                out = p.parent / f"{p.stem}_{name.replace(' ', '_').replace('%', 'p')}.png"
                Image.fromarray(arr).save(out)

    fig.suptitle("E34 corners — brightness correction comparison",
                 fontsize=14)
    fig.tight_layout()
    out = HERE / "corners_brightness_comparison.png"
    fig.savefig(out, dpi=90); plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
