#!/usr/bin/env python3
"""s170 verdict — SecurityArucoBaseline.

Reads mission summary + journal + GT JSONL + inspect dir n_dets
distribution.  Reports clearly whether sentai.safety actually FIRED.
"""
import json, math, re, sys
from collections import Counter
from pathlib import Path

WORKDIR = Path("/tmp/s170_security_aruco_baseline")
FS_ROOT = Path("/home/bogdan/work/coralmicro/build-sim/sentai_fs_root")
SUMMARY = FS_ROOT / "mission_security_aruco_summary.json"
JOURNAL = FS_ROOT / "mission_security_aruco_journal.txt"
GT      = WORKDIR / "gt_poses.jsonl"
REPL_LOG = WORKDIR / "repl.log"
LAND_RADIUS_M = 0.30


def _load_jsonl(p):
    if not p.is_file(): return []
    out = []
    for line in p.read_text().splitlines():
        line = line.strip()
        if not line: continue
        try: out.append(json.loads(line))
        except Exception: pass
    return out


def main() -> int:
    print(f"[verdict] s170 SecurityArucoBaseline")
    print(f"          summary: {SUMMARY}")
    print(f"          journal: {JOURNAL}")

    s = None
    if SUMMARY.is_file():
        try: s = json.loads(SUMMARY.read_text())
        except Exception as e: print(f"          summary parse FAIL: {e}")
    if s:
        print(f"\n[verdict] mission status     : {s.get('status')}")
        print(f"          phases_done          : {s.get('phases_done')}")
        print(f"          aborted              : {s.get('aborted')}")
        print(f"          abort_reason         : {s.get('abort_reason')!r}")
        print(f"          hover_elapsed_s      : {s.get('hover_elapsed_s')}")
        print(f"          loop_iters           : {s.get('loop_iters')}")
        print(f"          tick_journal_writes  : {s.get('tick_journal_writes')}")
        print(f"          errors               : {s.get('errors')}")
    else:
        print(f"\n[verdict] NO SUMMARY — mission hung or crashed early")
        if REPL_LOG.is_file():
            print(f"--- repl.log tail ---")
            print(REPL_LOG.read_text()[-2000:])

    if JOURNAL.is_file():
        lines = JOURNAL.read_text().splitlines()
        print(f"\n[verdict] journal — {len(lines)} entries; last 12:")
        for ln in lines[-12:]:
            print(f"  {ln}")

    # ── SafetyTask C++-side journal (authoritative — what the
    # firmware actually saw) ────────────────────────────────────────
    sj_ptr = WORKDIR / "safety_journal_dir"
    if sj_ptr.is_file():
        sjdir = Path(sj_ptr.read_text().strip())
        if sjdir.is_dir():
            files = sorted(sjdir.glob("t*_n*_f*.pgm"))
            rx = re.compile(r't(\d+)_n(\d+)_f(\d+)\.pgm')
            n_counter = Counter()
            parsed = []
            for f in files:
                m = rx.match(f.name)
                if m:
                    ms   = int(m.group(1))
                    n    = int(m.group(2))
                    fseq = int(m.group(3))
                    parsed.append((ms, n, fseq))
                    n_counter[n] += 1
            print(f"\n[verdict] SafetyTask C++ journal: {sjdir}")
            print(f"          frames processed by sentai.safety: {len(files)}")
            for n in sorted(n_counter):
                pct = 100.0 * n_counter[n] / len(files) if files else 0
                print(f"          n_det={n}: {n_counter[n]} ({pct:.1f}%)")
            # Chronological snapshot of n_dets at 1s buckets.
            if parsed:
                buckets = {}    # second → list of n_dets
                for ms, n, _f in parsed:
                    sec = ms // 1000
                    buckets.setdefault(sec, []).append(n)
                print(f"          per-second n_det trace (max over each 1s):")
                for sec in sorted(buckets):
                    ns = buckets[sec]
                    print(f"            t={sec:3d}s  n={ns}  max={max(ns)}")
    # ── Optional host-side cv2 inspect (independent confirmation) ───
    inspect_ptr = WORKDIR / "inspect_dir"
    if inspect_ptr.is_file():
        inspect = Path(inspect_ptr.read_text().strip())
        if inspect.is_dir():
            files = list(inspect.glob("t*_n*_f*.ppm"))
            rx = re.compile(r't(\d+)_n(\d+)_f(\d+)\.ppm')
            n_dets_counter = Counter()
            for f in files:
                m = rx.match(f.name)
                if m: n_dets_counter[int(m.group(2))] += 1
            print(f"\n[verdict] host-side cv2 inspect: {inspect}")
            print(f"          frames: {len(files)}")
            for n in sorted(n_dets_counter):
                pct = 100.0 * n_dets_counter[n] / len(files) if files else 0
                print(f"          n_det={n}: {n_dets_counter[n]} ({pct:.1f}%)")

    gt = _load_jsonl(GT)
    if gt:
        t0 = gt[0]["t_wall"]
        z_peak = max(g["z"] for g in gt)
        t_peak = next(g["t_wall"]-t0 for g in gt if g["z"] == z_peak)
        last = gt[-1]
        print(f"\n[verdict] GT records: {len(gt)} over {gt[-1]['t_wall']-t0:.1f}s")
        print(f"          z peak       : {z_peak:.3f}m at t={t_peak:.1f}s")
        print(f"          final xyz    : ({last['x']:+.3f},{last['y']:+.3f},{last['z']:+.3f})")
        dist_origin = math.hypot(last["x"], last["y"])
        print(f"          final dist   : {dist_origin*100:.1f} cm")

    if s is None:
        print(f"\n[verdict] FAIL — mission produced no summary")
        return 1
    if s.get("status") != "OK":
        print(f"\n[verdict] FAIL — mission status={s.get('status')}")
        return 1
    if "landed" not in s.get("phases_done", []):
        print(f"\n[verdict] FAIL — drone never landed")
        return 1
    if s.get("aborted"):
        print(f"\n[verdict] PASS — SAFETY FIRED ({s['abort_reason']}); "
              f"mission aborted at {s['hover_elapsed_s']}s, landed cleanly")
    else:
        print(f"\n[verdict] PASS — hover {s.get('hover_elapsed_s')}s without "
              f"safety abort; landed cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
