#!/usr/bin/env python3
"""1:1 comparison harness: WhyCon detector vs cv2.findContours on
random PGM frames from s191.

Approach: copy each frame into sim fs_root as `_test_frame.pgm`,
spawn sentai_sim per frame with a single-shot import of detect_pgm
module that reads via sentai.fs.read + detect_buffer.

Outputs JSON summary + side-by-side annotated PNGs.

Run:
  /home/bogdan/work/coralmicro/venv/bin/python \
      examples/sentai_runtime/experiments/s191_calib_classic_only/\
      marker_analysis/compare_v2.py
"""
import os, sys, glob, json, subprocess, random, re, shutil
import cv2
import numpy as np

REPO   = "/home/bogdan/work/coralmicro"
EXP    = f"{REPO}/examples/sentai_runtime/experiments/s191_calib_classic_only"
SIM    = f"{REPO}/build-sim/sim/sentai_sim"
FS     = f"{REPO}/build-sim/sentai_fs_root"
FRAMES = f"{EXP}/fr_current/frames"
OUT    = f"{EXP}/marker_analysis/compare_v2"
os.makedirs(OUT, exist_ok=True)


REPL_SCRIPT = "import detect_pgm\ndetect_pgm.run()\n"


def run_our(pgm):
    """Copy pgm into fs_root and run sentai_sim REPL detection."""
    shutil.copy(pgm, f"{FS}/_test_frame.pgm")
    try:
        r = subprocess.run([SIM], input=REPL_SCRIPT, capture_output=True,
                            text=True, timeout=30)
    except subprocess.TimeoutExpired:
        return None
    n = None
    markers = []
    for line in r.stdout.split('\n'):
        line = line.strip().lstrip('>').strip()
        if line.startswith('DETECT_N '):
            try:
                n = int(line.split()[1])
            except ValueError:
                pass
        elif line.startswith('MARKER '):
            parts = line.split()
            if len(parts) >= 4:
                try:
                    markers.append({"cx": float(parts[2]),
                                    "cy": float(parts[3])})
                except ValueError:
                    pass
    return {"n": n, "markers": markers}


def run_cv(pgm):
    img = cv2.imread(pgm, cv2.IMREAD_GRAYSCALE)
    if img is None: return None, None
    _, thr = cv2.threshold(img, 80, 255, cv2.THRESH_BINARY_INV)
    cnts, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL,
                                 cv2.CHAIN_APPROX_NONE)
    out = []
    for c in cnts:
        a = cv2.contourArea(c)
        if a < 40 or a > 4000: continue
        p = cv2.arcLength(c, True)
        if p < 1: continue
        circ = 4 * np.pi * a / (p * p)
        if circ < 0.55: continue
        M = cv2.moments(c)
        if M['m00'] < 1: continue
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        x, y, w, h = cv2.boundingRect(c)
        if x == 0 or y == 0 or x+w == img.shape[1] or y+h == img.shape[0]:
            continue
        out.append({"cx": float(cx), "cy": float(cy),
                    "area": float(a), "circ": float(circ)})
    return out, img


def match(ours, cv_list, tol_px=2.5):
    pairs = []
    used = set()
    for o in ours:
        best_d, best_i = 1e9, -1
        for i, c in enumerate(cv_list):
            if i in used: continue
            d = ((o['cx']-c['cx'])**2 + (o['cy']-c['cy'])**2) ** 0.5
            if d < best_d:
                best_d, best_i = d, i
        if best_i >= 0 and best_d < tol_px:
            pairs.append((o, cv_list[best_i], best_d))
            used.add(best_i)
    o_extra = [o for o in ours
               if not any(o is p[0] for p in pairs)]
    c_miss  = [c for i, c in enumerate(cv_list) if i not in used]
    return pairs, o_extra, c_miss


def main():
    all_frames = sorted(glob.glob(f"{FRAMES}/*.pgm"))
    # Filter to frames with some markers (n>=1 in filename)
    candidates = [p for p in all_frames
                  if not re.search(r"_n0_", os.path.basename(p))]
    if len(candidates) < 5:
        candidates = all_frames
    random.seed(123)
    n_samples = min(10, len(candidates))
    sample = random.sample(candidates, n_samples)
    sample.sort()

    print(f"Comparing {n_samples} frames\n")
    print(f"{'frame':<40} {'alg_n':>5} {'cv_n':>5} {'match':>5} "
          f"{'extra':>5} {'we_miss':>7}")
    print("-" * 75)
    summary = []
    for pgm in sample:
        name = os.path.basename(pgm)
        ours_res = run_our(pgm)
        cv_list, img = run_cv(pgm)
        if ours_res is None or cv_list is None:
            print(f"{name[:40]:40}  ERROR")
            continue
        ours = ours_res['markers']
        pairs, extra, miss = match(ours, cv_list)
        print(f"{name[:40]:40} {ours_res['n']:>5} {len(cv_list):>5} "
              f"{len(pairs):>5} {len(extra):>5} {len(miss):>7}")
        summary.append({"frame": name,
                        "alg_n": ours_res['n'],
                        "cv_n": len(cv_list),
                        "matched": len(pairs),
                        "alg_extra": len(extra),
                        "we_missed_cv": len(miss)})

        # Annotated comparison PNG.
        annot = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        # green = matched
        for o, c, _d in pairs:
            cv2.circle(annot, (int(o['cx']), int(o['cy'])),
                        7, (0, 255, 0), 1)
        # yellow = ours-extra (false pos in our detector?)
        for o in extra:
            cv2.circle(annot, (int(o['cx']), int(o['cy'])),
                        10, (0, 255, 255), 2)
        # red = cv-only (we missed)
        for c in miss:
            cv2.circle(annot, (int(c['cx']), int(c['cy'])),
                        12, (0, 0, 255), 2)
        out_png = f"{OUT}/{name.replace('.pgm','_diff.png')}"
        cv2.imwrite(out_png, annot)

    # totals
    total_alg   = sum(s['alg_n']        for s in summary)
    total_cv    = sum(s['cv_n']         for s in summary)
    total_match = sum(s['matched']      for s in summary)
    total_miss  = sum(s['we_missed_cv'] for s in summary)
    total_extra = sum(s['alg_extra']    for s in summary)
    print(f"\nTOTAL alg={total_alg}  cv={total_cv}  match={total_match} "
          f"  alg_extra={total_extra}  we_missed_cv={total_miss}")

    with open(f"{OUT}/summary.json", "w") as f:
        json.dump({"summary": summary,
                   "totals": {"alg": total_alg, "cv": total_cv,
                              "match": total_match,
                              "we_missed_cv": total_miss,
                              "alg_extra": total_extra}}, f, indent=2)

if __name__ == "__main__":
    main()
