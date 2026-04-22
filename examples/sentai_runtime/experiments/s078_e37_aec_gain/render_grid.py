import pathlib, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from PIL import Image
HERE = pathlib.Path(__file__).parent
GAINS = ["07C","1F0","3FF"]
GAIN_LABELS = ["15.5×","31×","62.9×"]
AECS = [("30","28"),("78","68"),("A8","98")]
fig,axes=plt.subplots(3,3,figsize=(15,11))
for gi,g in enumerate(GAINS):
    for ai,(h,l) in enumerate(AECS):
        p=HERE/"frames"/f"g{g}_aec_{h}_{l}.jpg"
        ax=axes[gi][ai]
        if p.exists():
            arr=np.array(Image.open(p))
            ax.imshow(arr)
            ax.set_title(f"gain≤{GAIN_LABELS[gi]}  AEC 0x{h}/0x{l}  (L̄={arr.mean():.0f})")
        ax.set_xticks([]);ax.set_yticks([])
fig.suptitle("E37 — AEC target × gain ceiling  (NXP stock = top-left)",fontsize=13)
fig.tight_layout()
fig.savefig(HERE/"e37_grid.png",dpi=100);plt.close(fig)
print("wrote",HERE/"e37_grid.png")
