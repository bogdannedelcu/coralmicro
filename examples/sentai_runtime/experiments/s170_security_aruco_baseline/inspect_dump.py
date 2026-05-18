#!/usr/bin/env python3
"""inspect_dump.py — post-process dumped frames for operator review.

Reads frame_NNNNNN.ppm files from SENTAI_DUMP_FRAMES_DIR, runs cv2.aruco
detection on each, and hardlinks to `<workdir>/inspect_<ts>/` with a
descriptive name:

    t<ms>_n<dets>_f<fseq>.ppm

where:
  - ms     = milliseconds from first frame dumped (proxy for mission time)
  - n      = number of ArUco markers detected by cv2
  - fseq   = the camera frame_seq (from filename)

Host-side, post-mortem ONLY — anti-cheat compliant
([[sentai-sim-air-gapped-from-truth]]: GT/post-mortem on host OK; never
inject into firmware).  cv2.aruco here is the OPERATOR's verification
tool; the firmware's own detection happens via sentai_aruco_detect
inside SafetyTask (independent path).

Env:
  SENTAI_DUMP_FRAMES_DIR  source of frame_NNNNNN.ppm files
  INSPECT_DIR             destination for hardlinks (default
                          /tmp/s170_security_aruco_baseline/inspect)
"""
from __future__ import annotations
import os, re, sys
from pathlib import Path

import cv2  # only used host-side


def main() -> int:
    src = Path(os.environ.get("SENTAI_DUMP_FRAMES_DIR", ""))
    if not src.is_dir():
        print(f"FAIL — SENTAI_DUMP_FRAMES_DIR not a dir: {src}",
              file=sys.stderr)
        return 1
    dst = Path(os.environ.get(
        "INSPECT_DIR",
        f"/tmp/s170_security_aruco_baseline/inspect_{src.name.split('_')[-1]}"))
    dst.mkdir(parents=True, exist_ok=True)
    print(f"[inspect] src: {src}")
    print(f"[inspect] dst: {dst}")

    rx = re.compile(r'^frame_(\d+)\.ppm$')
    frames = sorted(src.glob("frame_*.ppm"),
                     key=lambda p: int(rx.match(p.name).group(1))
                                    if rx.match(p.name) else 0)
    if not frames:
        print("[inspect] no frame files; nothing to do")
        return 0

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    # Estimate per-frame ms from camera dump rate.  Bridge publishes
    # ~30 Hz raw and sentai_sim dumps every N-th frame.  Use fseq
    # spacing in the filename + assumed 30 fps to derive ms:
    #     ms = (fseq - first_fseq) * (1000 / 30)
    first_fseq = int(rx.match(frames[0].name).group(1))
    counts = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
    n_link = 0
    n_fail = 0
    for f in frames:
        m = rx.match(f.name)
        if m is None:
            continue
        fseq = int(m.group(1))
        ms = int((fseq - first_fseq) * (1000.0 / 30.0))
        try:
            img = cv2.imread(str(f))
            if img is None:
                n_fail += 1
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            n_dets = int(ids.size) if ids is not None else 0
        except Exception as e:
            n_dets = 0
            n_fail += 1
        if n_dets not in counts:
            counts[n_dets] = 0
        counts[n_dets] += 1
        out = dst / f"t{ms:08d}_n{n_dets}_f{fseq:06d}.ppm"
        try:
            if out.exists():
                out.unlink()
            os.link(f, out)
            n_link += 1
        except OSError:
            try:
                import shutil
                shutil.copy(f, out)
                n_link += 1
            except Exception:
                pass

    print(f"[inspect] processed {len(frames)} frames, {n_link} hardlinks, "
          f"{n_fail} detect-fails")
    print(f"[inspect] n_det distribution: "
          + ", ".join(f"n={k}: {v}" for k, v in sorted(counts.items())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
