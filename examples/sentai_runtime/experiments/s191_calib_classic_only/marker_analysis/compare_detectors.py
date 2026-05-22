#!/usr/bin/env python3
"""OP-S10-W21-T13: 1:1 comparison of our WhyCon detector vs cv2 on
saved iter-18 PGM frames.

Pipeline:
  1. Sample N random in-flight frames from s191/iter-18 fr_current.
  2. Push each frame to sentai_sim via REPL: `sentai.whycon.test_pgm(p)`
     → returns n_dets, then `sentai.whycon.get_markers()` → list of
     (cx, cy, axis_a, axis_b, angle).
  3. Run cv2 detection on the same frame with cv2.findContours +
     circularity filter (mirroring our detector's logic).
  4. Compare: count, centroid distance, OK if Δ < 1.5 px.

Outputs JSON report + side-by-side annotated PNGs.

Run from repo root:
    /home/bogdan/work/coralmicro/venv/bin/python \
        examples/sentai_runtime/experiments/s191_calib_classic_only/\
        marker_analysis/compare_detectors.py
"""
import os, sys, glob, json, subprocess, re, random, tempfile
import cv2
import numpy as np

REPO = "/home/bogdan/work/coralmicro"
EXP  = f"{REPO}/examples/sentai_runtime/experiments/s191_calib_classic_only"
SIM  = f"{REPO}/build-sim/sim/sentai_sim"
FRAMES = f"{EXP}/fr_current/frames"
OUT  = f"{EXP}/marker_analysis/compare_v2"
os.makedirs(OUT, exist_ok=True)


def cv_detect(pgm_path):
    """OpenCV reference detector mirroring our pipeline:
       threshold → findContours → filter by area/circularity → centroid."""
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return [], None
    # We use simple fixed threshold; our detector uses adaptive Bradley.
    # Both should detect dark blobs on light background.
    _, thr = cv2.threshold(img, 80, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(thr, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_NONE)
    out = []
    for c in contours:
        a = cv2.contourArea(c)
        if a < 40 or a > 4000:     # same as our min/max_area
            continue
        p = cv2.arcLength(c, True)
        if p < 1: continue
        circ = 4.0 * np.pi * a / (p*p)
        if circ < 0.55:             # same as our min_circularity
            continue
        # Centroid via moments.
        M = cv2.moments(c)
        if M['m00'] < 1: continue
        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        # Reject border-touching (matches our touches_border check).
        x, y, w, h = cv2.boundingRect(c)
        if x == 0 or y == 0 or x+w == img.shape[1] or y+h == img.shape[0]:
            continue
        out.append({"cx": float(cx), "cy": float(cy),
                    "area": float(a), "perimeter": float(p),
                    "circularity": float(circ)})
    return out, img


def our_detect(pgm_path):
    """Push frame through sentai_sim REPL, get back marker list."""
    script = f"""
import sentai
n = sentai.whycon.test_pgm('{pgm_path}')
ms = sentai.whycon.get_markers() if n > 0 else []
import json
print('DETECT_RESULT', json.dumps({{'n': n, 'markers': ms}}))
"""
    try:
        r = subprocess.run([SIM], input=script, capture_output=True,
                            text=True, timeout=15)
    except subprocess.TimeoutExpired:
        return None
    # Find DETECT_RESULT line in stdout.
    for line in r.stdout.split('\n'):
        if line.startswith('DETECT_RESULT '):
            try:
                return json.loads(line[len('DETECT_RESULT '):])
            except json.JSONDecodeError:
                return None
    return None


def match_markers(ours, cv_list, tol_px=2.0):
    """Greedy nearest-centroid matching. Returns matched pairs + unmatched."""
    used_cv = set()
    pairs = []
    for o in ours:
        best_d, best_i = 1e9, -1
        for i, c in enumerate(cv_list):
            if i in used_cv: continue
            d = ((o['cx']-c['cx'])**2 + (o['cy']-c['cy'])**2) ** 0.5
            if d < best_d:
                best_d, best_i = d, i
        if best_i >= 0 and best_d < tol_px:
            pairs.append((o, cv_list[best_i], best_d))
            used_cv.add(best_i)
    unmatched_ours = [o for o in ours
                      if not any(o is p[0] for p in pairs)]
    unmatched_cv   = [c for i, c in enumerate(cv_list)
                      if i not in used_cv]
    return pairs, unmatched_ours, unmatched_cv


def main():
    # Pick 12 random in-flight frames (skip first 40 = takeoff)
    all_frames = sorted(glob.glob(f"{FRAMES}/*.pgm"))
    if len(all_frames) < 50:
        print("Not enough frames in", FRAMES)
        return 1
    inflight = all_frames[40:min(200, len(all_frames))]
    random.seed(123)
    sample = random.sample(inflight, min(12, len(inflight)))

    summary = []
    print(f"{'frame':<38} {'alg':>3} {'cv':>3} {'match':>5} {'extra':>5} {'miss':>4}")
    print("-" * 68)
    for pgm in sorted(sample):
        cv_list, img = cv_detect(pgm)
        our_res = our_detect(pgm)
        if our_res is None:
            print(f"{os.path.basename(pgm)[:38]:38}  TIMEOUT")
            continue
        ours = our_res['markers']
        pairs, our_extra, cv_miss = match_markers(ours, cv_list)
        name = os.path.basename(pgm)
        print(f"{name[:38]:38} {len(ours):>3} {len(cv_list):>3} "
              f"{len(pairs):>5} {len(our_extra):>5} {len(cv_miss):>4}")
        summary.append({
            "frame": name, "our_n": len(ours), "cv_n": len(cv_list),
            "matched": len(pairs), "our_extra": len(our_extra),
            "cv_missed_by_us": len(cv_miss),
        })

        # Save annotated PNG side by side.
        if img is not None:
            annot = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            # Red = cv-only (we missed), Green = matched, Yellow = ours-only
            for o in ours:
                cv2.circle(annot, (int(o['cx']), int(o['cy'])),
                            6, (0, 255, 255), 1)
            for c in cv_list:
                cv2.circle(annot, (int(c['cx']), int(c['cy'])),
                            8, (0, 200, 0), 1)
            for c in cv_miss:
                cv2.circle(annot, (int(c['cx']), int(c['cy'])),
                            12, (0, 0, 255), 2)
            outp = f"{OUT}/{name.replace('.pgm','_diff.png')}"
            cv2.imwrite(outp, annot)

    # Aggregate stats
    total_our   = sum(s['our_n']             for s in summary)
    total_cv    = sum(s['cv_n']              for s in summary)
    total_match = sum(s['matched']           for s in summary)
    total_miss  = sum(s['cv_missed_by_us']   for s in summary)
    total_extra = sum(s['our_extra']         for s in summary)
    print(f"\nTOTAL: ours={total_our} cv={total_cv} match={total_match} "
          f"our_extra={total_extra} we_miss={total_miss}")

    with open(f"{OUT}/summary.json", "w") as f:
        json.dump({"summary": summary,
                   "totals": {"our": total_our, "cv": total_cv,
                              "match": total_match,
                              "we_miss": total_miss,
                              "our_extra": total_extra}}, f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
