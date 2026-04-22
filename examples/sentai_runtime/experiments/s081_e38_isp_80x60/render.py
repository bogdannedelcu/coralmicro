import pathlib, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from PIL import Image
HERE = pathlib.Path(__file__).parent
PRESETS = ["nxp_stock","bright_indoor","daylight","low_light"]
fig,axes = plt.subplots(len(PRESETS), 2, figsize=(16, 5*len(PRESETS)))
for i,p in enumerate(PRESETS):
    jp = HERE/"frames"/f"{p}_cam0.jpg"
    pg = HERE/"frames"/f"{p}_gray80x60.pgm"
    if jp.exists():
        a = np.array(Image.open(jp))
        axes[i,0].imshow(a); axes[i,0].set_title(f"{p} — JPEG (L̄={a.mean():.0f})", fontsize=12)
    axes[i,0].axis("off")
    if pg.exists():
        raw = pg.read_bytes()
        nl1=raw.index(b"\n",3); nl2=raw.index(b"\n",nl1+1)
        w,h = [int(x) for x in raw[3:nl1].split()]
        g = np.frombuffer(raw[nl2+1:], dtype=np.uint8).reshape(h,w)
        # 8× upscale for visibility
        im = Image.fromarray(g, mode="L").resize((w*8, h*8), Image.NEAREST)
        axes[i,1].imshow(np.array(im), cmap="gray", vmin=0, vmax=255)
        axes[i,1].set_title(f"{p} — 80×60 gray (min={g.min()} max={g.max()} mean={g.mean():.0f})", fontsize=12)
        # Save upscaled PNG too
        im.save(HERE/"frames"/f"{p}_gray80x60_x8.png")
        Image.fromarray(g, mode="L").save(HERE/"frames"/f"{p}_gray80x60.png")
    axes[i,1].axis("off")
fig.suptitle("E38 — ISP preset comparison @ 80×60 flow resolution  (group-write atomic apply)", fontsize=14)
fig.tight_layout()
fig.savefig(HERE/"e38_80x60_grid.png", dpi=100); plt.close(fig)
print("wrote", HERE/"e38_80x60_grid.png")
