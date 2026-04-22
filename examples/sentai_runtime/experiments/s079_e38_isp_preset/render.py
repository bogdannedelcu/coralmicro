import pathlib, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from PIL import Image
HERE = pathlib.Path(__file__).parent
PRESETS = ["nxp_stock","bright_indoor","daylight","low_light"]
fig, axes = plt.subplots(len(PRESETS), 2, figsize=(14, 4.5*len(PRESETS)))
for i,p in enumerate(PRESETS):
    # JPEG column
    jp = HERE/"frames"/f"{p}_cam0.jpg"
    if jp.exists():
        arr = np.array(Image.open(jp))
        axes[i,0].imshow(arr)
        axes[i,0].set_title(f"{p} — JPEG 640×480 (L̄={arr.mean():.0f})", fontsize=11)
    axes[i,0].set_xticks([]); axes[i,0].set_yticks([])
    # Gray column
    pg = HERE/"frames"/f"{p}_gray40x30.pgm"
    if pg.exists():
        raw = pg.read_bytes()
        nl1=raw.index(b"\n",3); nl2=raw.index(b"\n",nl1+1)
        g = np.frombuffer(raw[nl2+1:], dtype=np.uint8).reshape(30,40)
        axes[i,1].imshow(g, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
        axes[i,1].set_title(f"{p} — 40×30 gray (min={g.min()} max={g.max()} mean={g.mean():.0f})", fontsize=11)
    axes[i,1].set_xticks([]); axes[i,1].set_yticks([])
fig.suptitle("E38 — ISP preset comparison (JPEG vs SAD-input gray)", fontsize=13)
fig.tight_layout()
fig.savefig(HERE/"e38_grid.png", dpi=100); plt.close(fig)
print("wrote", HERE/"e38_grid.png")
