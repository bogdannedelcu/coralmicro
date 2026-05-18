#!/usr/bin/env python3
"""s175 verdict — compare per-marker tvec_cam.z between original and
rotated PGM detections.

Parses lines like:
  PGM_RESULT id=0 tvec=(+0.123,-0.456,+0.789) rvec=(...) reproj=0.5 path=/.../frame_X.pgm

For each marker id present in BOTH files, compute Δz = z_rot - z_orig.
Threshold: if max |Δz| > 1 cm → planar ambiguity confirmed.
"""
from __future__ import annotations
import re, sys

PGM_RX = re.compile(
    r"PGM_RESULT id=(\d+) tvec=\(([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+)\) "
    r"rvec=\(([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+)\) "
    r"reproj=([\d.]+) path=(\S+)"
)

DELTA_Z_LIMIT = 0.010   # 1 cm


def parse(path: str):
    by_file: dict[str, dict[int, tuple]] = {}
    with open(path) as f:
        for line in f:
            m = PGM_RX.search(line)
            if not m: continue
            mid = int(m.group(1))
            tvec = tuple(float(m.group(i)) for i in (2, 3, 4))
            rvec = tuple(float(m.group(i)) for i in (5, 6, 7))
            reproj = float(m.group(8))
            fpath = m.group(9)
            by_file.setdefault(fpath, {})[mid] = (tvec, rvec, reproj)
    return by_file


def main() -> int:
    if len(sys.argv) < 2:
        sys.exit("usage: verdict.py <run.log>")
    by_file = parse(sys.argv[1])
    if len(by_file) < 2:
        print("[verdict] FAIL — fewer than 2 files in log")
        sys.exit(1)
    paths = sorted(by_file.keys())
    a = next(p for p in paths if "original" in p)
    b = next(p for p in paths if "rot35" in p)
    print(f"[verdict] orig: {a}")
    print(f"[verdict] rot:  {b}")
    common = sorted(set(by_file[a]) & set(by_file[b]))
    if not common:
        print("[verdict] FAIL — no common marker IDs detected")
        sys.exit(1)

    print(f"[verdict] common marker IDs: {common}")
    print()
    print(f"  {'id':>3} | {'z_orig (m)':>11} | {'z_rot (m)':>11} | "
          f"{'Δz (mm)':>9} | {'reproj_o':>9} | {'reproj_r':>9}")
    print("  " + "-" * 70)
    max_dz = 0.0
    for mid in common:
        tv_o, rv_o, rp_o = by_file[a][mid]
        tv_r, rv_r, rp_r = by_file[b][mid]
        dz = tv_r[2] - tv_o[2]
        if abs(dz) > abs(max_dz):
            max_dz = dz
        print(f"  {mid:>3} | {tv_o[2]:>+11.4f} | {tv_r[2]:>+11.4f} | "
              f"{dz*1000:>+9.2f} | {rp_o:>9.3f} | {rp_r:>9.3f}")

    print()
    print(f"[verdict] max |Δz| across markers: {abs(max_dz)*1000:.2f} mm")
    if abs(max_dz) > DELTA_Z_LIMIT:
        print(f"[verdict] FAIL — Δz exceeds {DELTA_Z_LIMIT*1000:.0f} mm")
        print("[verdict] → planar-marker ambiguity CONFIRMED")
        print("[verdict]   (rotating image plane changes pose solution choice)")
        sys.exit(1)
    print("[verdict] PASS — Δz within tolerance; PnP is rotation-invariant")
    sys.exit(0)


if __name__ == "__main__":
    main()
