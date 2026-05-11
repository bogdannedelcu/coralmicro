"""Reproduce the 3-level pyramid downsampling in Python, dump each
level as PGM (and upscaled PNG for easy viewing) so we can verify
visually that the C box-filters are producing sane content.

Replicates EXACTLY the C functions:
  - L0 wide:  PXP shim 8× area-average (640×480 → 80×60)
  - L1 mid:   boxfilter_4x_rgb_to_gray (center 320×240 → 80×60)
  - L2 fine:  boxfilter_2x_rgb_to_gray (center 160×120 → 80×60)

BT.601 luma weights: 77*R + 150*G + 29*B, then >>8 (or >>10/>>12 for 4/16-pixel averages).
"""
import sys
from pathlib import Path
import numpy as np
from PIL import Image


def read_ppm(path: Path) -> np.ndarray:
    with open(path, "rb") as f:
        magic = f.readline().strip()
        assert magic == b"P6"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline().strip())
        assert maxval == 255
        data = f.read(w * h * 3)
    return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)


def luma_bt601(R, G, B):
    """77R + 150G + 29B, mimicking integer math >>8."""
    return ((77 * R.astype(np.uint32) + 150 * G.astype(np.uint32)
             + 29 * B.astype(np.uint32)) >> 8).astype(np.uint8)


def l0_wide(rgb):
    """640x480 RGB → 80x60 gray.  PXP-style 8× area average."""
    h, w, _ = rgb.shape
    assert h == 480 and w == 640
    # Mean over 8x8 blocks
    rgb_reshaped = rgb.reshape(60, 8, 80, 8, 3).astype(np.uint32)
    mean = rgb_reshaped.mean(axis=(1, 3)).astype(np.uint8)
    return luma_bt601(mean[:, :, 0], mean[:, :, 1], mean[:, :, 2])


def crop_center(rgb, out_h, out_w):
    h, w, _ = rgb.shape
    y0 = (h - out_h) // 2
    x0 = (w - out_w) // 2
    return rgb[y0:y0 + out_h, x0:x0 + out_w]


def l1_mid(rgb):
    """Center 320x240 RGB → 80x60 gray, 4× box-filter."""
    patch = crop_center(rgb, 240, 320)
    # 4×4 box-average per output pixel
    patch_resh = patch.reshape(60, 4, 80, 4, 3).astype(np.uint32)
    mean = patch_resh.mean(axis=(1, 3)).astype(np.uint8)
    return luma_bt601(mean[:, :, 0], mean[:, :, 1], mean[:, :, 2])


def l2_fine(rgb):
    """Center 160x120 RGB → 80x60 gray, 2× box-filter."""
    patch = crop_center(rgb, 120, 160)
    patch_resh = patch.reshape(60, 2, 80, 2, 3).astype(np.uint32)
    mean = patch_resh.mean(axis=(1, 3)).astype(np.uint8)
    return luma_bt601(mean[:, :, 0], mean[:, :, 1], mean[:, :, 2])


def save_gray(gray: np.ndarray, path: Path, upscale_to: int = 8):
    """Save as PNG, with optional nearest-neighbour upscale for viewing."""
    img = Image.fromarray(gray, mode="L")
    if upscale_to > 1:
        img = img.resize((gray.shape[1] * upscale_to,
                           gray.shape[0] * upscale_to),
                          Image.NEAREST)
    img.save(path)


def main():
    if len(sys.argv) < 2:
        # Find latest captured PPM
        cands = sorted(Path("/tmp").glob("sentai_frames_arucohover_*"),
                       key=lambda p: p.stat().st_mtime, reverse=True)
        ppm = None
        for d in cands:
            ppms = sorted(d.glob("frame_*.ppm"))
            if ppms:
                ppm = ppms[-1]   # last frame in latest run
                break
        if ppm is None:
            print("no PPMs found in /tmp/sentai_frames_arucohover_*", file=sys.stderr)
            return 1
    else:
        ppm = Path(sys.argv[1])

    print(f"[dump] source PPM: {ppm}")
    rgb = read_ppm(ppm)
    print(f"[dump] shape: {rgb.shape}")

    out_dir = ppm.parent / "pyramid_levels"
    out_dir.mkdir(exist_ok=True)

    L0 = l0_wide(rgb)
    L1 = l1_mid(rgb)
    L2 = l2_fine(rgb)

    save_gray(L0, out_dir / "L0_wide_8x.png", upscale_to=4)
    save_gray(L1, out_dir / "L1_mid_4x.png",  upscale_to=4)
    save_gray(L2, out_dir / "L2_fine_2x.png", upscale_to=4)

    # Also save the source PPM as PNG for reference
    Image.fromarray(rgb).save(out_dir / "source_640x480.png")

    print(f"[dump] L0 stats: shape={L0.shape}  mean={L0.mean():.1f}  "
          f"min={L0.min()} max={L0.max()}  std={L0.std():.1f}")
    print(f"[dump] L1 stats: shape={L1.shape}  mean={L1.mean():.1f}  "
          f"min={L1.min()} max={L1.max()}  std={L1.std():.1f}")
    print(f"[dump] L2 stats: shape={L2.shape}  mean={L2.mean():.1f}  "
          f"min={L2.min()} max={L2.max()}  std={L2.std():.1f}")
    print(f"[dump] PNGs written to {out_dir}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
