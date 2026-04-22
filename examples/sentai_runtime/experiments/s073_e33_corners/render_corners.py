#!/usr/bin/env python3
"""inspect.py — render the 40×30 gray PGMs as 8×-upsampled PNGs for
visual inspection of the SAD algorithm's input.

Also emit a single combined montage showing all 5 corners side by
side with phase labels, so we can eyeball whether each corner has
distinct vertical/horizontal texture the block-match can lock onto.
"""
import pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


HERE = pathlib.Path(__file__).parent
SHOTS = HERE / "shots"


def read_pgm(p):
    """Simple P5 PGM reader — the on-device emitter hand-writes the
    header, so we can't assume a general parser.  Expect exactly
    'P5\\n40 30\\n255\\n' + 1200 bytes."""
    raw = p.read_bytes()
    # Parse header.
    assert raw.startswith(b"P5\n")
    nl1 = raw.index(b"\n", 3)
    nl2 = raw.index(b"\n", nl1 + 1)
    w, h = (int(x) for x in raw[3:nl1].split())
    assert (w, h) == (40, 30)
    data = np.frombuffer(raw[nl2 + 1:], dtype=np.uint8).reshape(h, w)
    return data


def main():
    corners = ["WAIT_start", "WAIT_c1", "WAIT_c2", "WAIT_c3", "WAIT_end"]
    imgs = {}
    for name in corners:
        p = SHOTS / f"{name}_gray40x30.pgm"
        if p.exists():
            imgs[name] = read_pgm(p)
            print(f"{name}: min={imgs[name].min():3d} "
                  f"max={imgs[name].max():3d} "
                  f"std={imgs[name].std():.1f}")
        else:
            print(f"{name}: missing")

    # ── Write per-corner PNGs (nearest-neighbour upscale ×20 so
    #    the viewer can see the pixel structure easily).
    from PIL import Image
    for name, arr in imgs.items():
        im = Image.fromarray(arr, mode="L")
        im = im.resize((40 * 20, 30 * 20), resample=Image.NEAREST)
        out = HERE / f"{name}_gray_upscaled.png"
        im.save(out); print(f"wrote {out}")

    # ── Combined montage as one PNG for side-by-side comparison.
    fig, axes = plt.subplots(1, len(imgs), figsize=(len(imgs) * 3.5, 3.5))
    for ax, (name, arr) in zip(axes, imgs.items()):
        ax.imshow(arr, cmap="gray", interpolation="nearest",
                  vmin=0, vmax=255)
        ax.set_title(name, fontsize=10)
        ax.set_xticks([]); ax.set_yticks([])
    fig.suptitle("E33 — 40×30 gray frames the M4 SAD algorithm uses")
    fig.tight_layout()
    out = HERE / "corners_montage.png"
    fig.savefig(out, dpi=120); plt.close(fig)
    print(f"wrote {out}")

    # ── Horizontal / vertical gradient check: if no vertical texture
    # the algorithm can't lock onto vertical motion.  Sobel + stats.
    from scipy.ndimage import sobel  # only if scipy available
    print("\nPer-corner texture / gradient energy:")
    for name, arr in imgs.items():
        gx = sobel(arr.astype(np.float32), axis=1)   # horizontal edges → left-right variation
        gy = sobel(arr.astype(np.float32), axis=0)   # vertical   edges → up-down  variation
        e_x = (gx ** 2).sum() / arr.size
        e_y = (gy ** 2).sum() / arr.size
        print(f"  {name}: grad-energy_x={e_x:7.1f}  "
              f"grad-energy_y={e_y:7.1f}  "
              f"ratio y/x={e_y / max(e_x, 1e-6):.2f}")


if __name__ == "__main__":
    main()
