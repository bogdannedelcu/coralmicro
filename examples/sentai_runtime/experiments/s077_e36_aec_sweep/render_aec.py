import pathlib, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from PIL import Image
HERE = pathlib.Path(__file__).parent
pairs = [(0x30,0x28),(0x48,0x38),(0x60,0x50),(0x78,0x68),(0x90,0x80),(0xA8,0x98)]
fig, axes = plt.subplots(2, 3, figsize=(15, 10))
for i,(hi,lo) in enumerate(pairs):
    path = HERE/"frames"/f"aec_{hi:02X}_{lo:02X}_cam0.jpg"
    ax = axes[i//3][i%3]
    if path.exists():
        arr = np.array(Image.open(path))
        ax.imshow(arr)
        mean = arr.mean()
        ax.set_title(f"target 0x{hi:02X}/0x{lo:02X}  (mean L = {mean:.0f})")
    ax.set_xticks([]); ax.set_yticks([])
fig.suptitle("E36 — AEC target sweep  (0x30/0x28 = NXP stock → 0xA8/0x98)")
fig.tight_layout()
fig.savefig(HERE/"aec_montage.png", dpi=100); plt.close(fig)
print("wrote", HERE/"aec_montage.png")
