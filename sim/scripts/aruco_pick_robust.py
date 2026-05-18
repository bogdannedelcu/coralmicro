#!/usr/bin/env python3
"""aruco_pick_robust.py — offline scoring of rotation-robustness for
the OpenCV DICT_4X4_50 dictionary.

OP-S10-W14 T16 prerequisite (operator 2026-05-18: "nu poti folosi
doar markeri care nu au problema asta?  Doar ne trebuie 4 markeri
bine alesi").

For each marker M in the dictionary:
  - extract its 4x4 bit pattern
  - compute the 3 rotations (90, 180, 270 degrees)
  - for each rotation, find the minimum Hamming distance to ANY
    OTHER marker pattern in the dictionary
  - score(M) = min(min_ham_90, min_ham_180, min_ham_270)

High score = unique under rotation (no alias to another marker
even when seen rotated by the camera).  Low score = rotation
of M is close to some other marker → orientation flip risk.

Output: ranked list, top 4 picked + JSON-ish key=value lines for
easy parse by downstream scripts (per operator 2026-05-18 "nu
prea vreau sa tinem configurari in json, un format mai simplu
cheie valoare e mai potrivit pentru MCU").

Anti-cheat compliant: pure offline tool, no Gazebo / GT touched.
Run inside the project venv:  venv/bin/python3 sim/scripts/aruco_pick_robust.py
"""
from __future__ import annotations

import sys
import numpy as np

try:
    import cv2  # noqa: E402
except ImportError:
    sys.exit("cv2 not installed in venv — `source venv/bin/activate; pip install opencv-python`")


def marker_bits(d: "cv2.aruco.Dictionary", marker_id: int) -> np.ndarray:
    """Return 4x4 0/1 bit pattern of marker_id (inside its black border)."""
    # OpenCV's drawMarker / generateImageMarker produces a 6x6 byte
    # image when sidePixels = 6 — the center 4x4 is the data area.
    img = d.generateImageMarker(marker_id, 6)
    # Center 4x4 from the 6x6 image (drop 1-px black border each side).
    data = img[1:5, 1:5]
    # Black bit = 0, white bit = 1 (OpenCV convention).
    return (data > 0).astype(np.uint8)


def rot90(bits: np.ndarray) -> np.ndarray:
    """numpy.rot90 returns CCW rotation; we want CW for "marker rotated
    in image by +90° from sensor POV", which is the same numerically
    but conceptually clearer."""
    return np.rot90(bits, k=-1)


def hamming(a: np.ndarray, b: np.ndarray) -> int:
    return int(np.sum(a != b))


def main() -> int:
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    N = 50
    # All markers' bit patterns + their 4 rotations.
    base = [marker_bits(d, i) for i in range(N)]
    rot = {
        "0":   base,
        "90":  [rot90(b)            for b in base],
        "180": [rot90(rot90(b))     for b in base],
        "270": [rot90(rot90(rot90(b))) for b in base],
    }

    # score[i] = min over rotations r in {90, 180, 270} of
    #            min over j != i of hamming(rot[r][i], base[j])
    # i.e. for each non-zero rotation of marker i, how close is it
    # to ANY other marker?  Larger = more uniquely identifiable.
    print("# aruco_pick_robust.py — DICT_4X4_50 rotation-robustness ranking")
    print("# format: id=<int> score=<min_hamming> ham90=<n> ham180=<n> ham270=<n>")
    scored = []
    for i in range(N):
        worst = 16  # upper bound: 16 bits total
        per_rot = {}
        for rname in ("90", "180", "270"):
            rotated = rot[rname][i]
            nearest = 16
            for j in range(N):
                if j == i:
                    continue
                h = hamming(rotated, base[j])
                if h < nearest:
                    nearest = h
            per_rot[rname] = nearest
            if nearest < worst:
                worst = nearest
        scored.append((worst, i, per_rot))

    # Sort by score desc (most robust first), tie-break by id asc.
    scored.sort(key=lambda t: (-t[0], t[1]))

    for worst, i, per_rot in scored:
        print(f"id={i} score={worst} "
              f"ham90={per_rot['90']} "
              f"ham180={per_rot['180']} "
              f"ham270={per_rot['270']}")

    # Pick top 4 by score (with tie-break by ID).
    chosen = [(i, worst, per_rot) for (worst, i, per_rot) in scored[:4]]
    print()
    print("# === chosen 4 robust markers (greedy top-4 by score) ===")
    for i, worst, per_rot in chosen:
        print(f"chosen={i} score={worst}")
    chosen_ids = [i for (i, _, _) in chosen]
    print(f"chosen_ids={' '.join(str(i) for i in chosen_ids)}")

    # In-scene cross-check: for the chosen 4, find the minimum
    # Hamming distance between any rotation of one marker and any
    # of the OTHER 3 markers in the same set.  This is the
    # detector-relevant metric: confusion only matters if rot(M)
    # matches a marker we actually use.
    def evaluate_set(ids):
        worst = 16
        worst_case = None
        for i in ids:
            for rname in ("90", "180", "270"):
                rotated = rot[rname][i]
                for j in ids:
                    if j == i:
                        continue
                    h = hamming(rotated, base[j])
                    if h < worst:
                        worst = h
                        worst_case = (i, rname, j, h)
        return worst, worst_case

    in_scene_score, wc = evaluate_set(chosen_ids)
    print(f"in_scene_min_hamming={in_scene_score}")
    if wc is not None:
        print(f"in_scene_worst_pair=id{wc[0]}_rot{wc[1]}_vs_id{wc[2]}_ham={wc[3]}")

    # Compare to LEGACY scene {0, 1, 2, 3}:
    legacy_ids = [0, 1, 2, 3]
    legacy_score, wc_l = evaluate_set(legacy_ids)
    print(f"legacy_ids={' '.join(str(i) for i in legacy_ids)}")
    print(f"legacy_in_scene_min_hamming={legacy_score}")
    if wc_l is not None:
        print(f"legacy_worst_pair=id{wc_l[0]}_rot{wc_l[1]}_vs_id{wc_l[2]}_ham={wc_l[3]}")

    # Brute-force search for the GLOBAL OPTIMUM in-scene 4-set.
    # 50C4 = 230 300 combos × 12 rotation×other pairs each = quick.
    print()
    print("# === brute-force optimum (in-scene min Hamming) ===")
    from itertools import combinations
    best_score = -1
    best_set = None
    for combo in combinations(range(N), 4):
        s, _ = evaluate_set(list(combo))
        if s > best_score:
            best_score = s
            best_set = combo
    print(f"optimum_ids={' '.join(str(i) for i in best_set)}")
    print(f"optimum_in_scene_min_hamming={best_score}")
    # If multiple sets tie for the optimum, find them all (up to 5).
    ties = []
    for combo in combinations(range(N), 4):
        if len(ties) >= 5: break
        s, _ = evaluate_set(list(combo))
        if s == best_score:
            ties.append(combo)
    print(f"optimum_ties_first_5={ties}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
