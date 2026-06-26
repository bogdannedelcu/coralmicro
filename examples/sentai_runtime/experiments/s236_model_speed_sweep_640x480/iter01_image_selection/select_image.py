"""Pick a test-split image that detects well on ALL 8 cover_v1 models.

Runs each model's EdgeTPU .tflite on the Coral USB Accelerator (pycoral),
decodes via host_decode, matches detections to GT (IoU center-distance),
and scores each candidate by cross-model agreement.

Run: venv-coral39/bin/python select_image.py
"""
import sys, glob, os, re, json
import numpy as np
from PIL import Image
from pycoral.utils.edgetpu import make_interpreter

SD = "/home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/s236_model_speed_sweep_640x480"
sys.path.insert(0, SD)
import host_decode as H

TEST = "/run/media/bogdan/3E4252C842528495/Data/sentai_human/review/splits/curated_clean_cover_v1/test.txt"
OUT = os.path.join(SD, "iter01_image_selection")
CONF = 0.25
N_CAND = 60

ARCHS = [a for a in sorted(os.listdir(os.path.join(SD, "models")))
         if os.path.isdir(os.path.join(SD, "models", a))]


def nboxes(p):
    m = re.search(r"boxes_(\d+)", p)
    return int(m.group(1)) if m else 0


def category(p):
    b = os.path.basename(p)
    return b.split("_batch")[0]


def load_gt(p):
    lbl = p.replace("/images/", "/labels/").replace(".png", ".txt")
    gt = []
    try:
        for ln in open(lbl):
            c, cx, cy, w, h = ln.split()
            gt.append((float(cx) * 640, float(cy) * 480,
                       float(w) * 640, float(h) * 480))
    except Exception:
        pass
    return gt


def matched(dets, gt):
    """Count GT boxes whose center is inside some detection box."""
    m = 0
    for (cx, cy, w, h) in gt:
        for d in dets:
            if d[0] <= cx <= d[2] and d[1] <= cy <= d[3]:
                m += 1
                break
    return m


def pick_candidates():
    imgs = [l.strip() for l in open(TEST) if l.strip()]
    # diverse categories, moderate GT count, prefer some larger persons
    by_cat = {}
    for p in imgs:
        n = nboxes(p)
        if 2 <= n <= 6:
            by_cat.setdefault(category(p), []).append(p)
    cands = []
    cats = sorted(by_cat)
    i = 0
    while len(cands) < N_CAND and any(by_cat.values()):
        c = cats[i % len(cats)]
        if by_cat[c]:
            cands.append(by_cat[c].pop(0))
        i += 1
        if i > 100000:
            break
    return cands


def main():
    os.makedirs(OUT, exist_ok=True)
    cands = pick_candidates()
    print("categories:", sorted(set(category(p) for p in cands)))
    print("candidates:", len(cands))

    interps = {}
    for a in ARCHS:
        mp = glob.glob(os.path.join(SD, "models", a, "edgetpu", "*edgetpu.tflite"))[0]
        it = make_interpreter(mp)
        it.allocate_tensors()
        interps[a] = it
    print("loaded models:", list(interps))

    rows = []
    for p in cands:
        img = np.array(Image.open(p).convert("RGB"))
        if img.shape != (480, 640, 3):
            continue
        gt = load_gt(p)
        if not gt:
            continue
        per = {}
        for a, it in interps.items():
            dets = H.infer(it, img, conf_thr=CONF, iou_thr=0.45)
            per[a] = {"det": int(len(dets)), "matched": int(matched(dets, gt))}
        n_models_detect = sum(1 for a in ARCHS if per[a]["det"] >= 1)
        min_matched = min(per[a]["matched"] for a in ARCHS)
        sum_matched = sum(per[a]["matched"] for a in ARCHS)
        rows.append({
            "image": p, "gt": len(gt),
            "n_models_detect": n_models_detect,
            "min_matched": min_matched, "sum_matched": sum_matched,
            "per_model": per,
        })

    # rank: all 8 detect, then highest min_matched, then sum_matched
    rows.sort(key=lambda r: (r["n_models_detect"], r["min_matched"],
                             r["sum_matched"]), reverse=True)
    with open(os.path.join(OUT, "selection.json"), "w") as f:
        json.dump(rows, f, indent=2)

    print("\n=== TOP 10 ===")
    print("%-3s %-3s %-5s %-5s  %s" % ("md", "gt", "minM", "sumM", "image"))
    for r in rows[:10]:
        print("%-3d %-3d %-5d %-5d  %s" % (
            r["n_models_detect"], r["gt"], r["min_matched"],
            r["sum_matched"], os.path.basename(r["image"])))
    best = rows[0]
    print("\nBEST:", best["image"])
    print("per-model:", json.dumps(best["per_model"]))
    with open(os.path.join(OUT, "chosen_image.txt"), "w") as f:
        f.write(best["image"] + "\n")


if __name__ == "__main__":
    main()
