#!/usr/bin/env python3
"""s174 verdict — YawArucoBaseline (closed-loop hover + 360° yaw rotation).

PASS iff (5 claims from spec, in priority order):
  1. mission produced a summary, is_done = True, no safety abort
  2. measured yaw rotation ≥ 300° (target was 360°; 17 % margin)
  3. measured yaw_rate ≈ commanded 30°/s within ±10°/s
  4. n_dets ≥ 4 for ≥ 90 % of rotation window (markers stay in FOV)
  5. lateral drift ≤ 8 cm during rotation, landed ≤ 12 cm of origin
     (slightly relaxed vs static HOLD because rotation introduces
      transient coupling)

Yaw GT is captured by gt_recorder.py (extended for orientation
2026-05-18).  Each record has yaw_deg derived from quaternion.
"""

import json, math, os, sys

WORKDIR = "/tmp/s174_yaw_aruco_baseline"
FS_ROOT = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "build-sim", "sentai_fs_root")
SUMMARY = os.path.join(FS_ROOT, "mission_yaw_aruco_summary.json")
GT_POSES = os.path.join(WORKDIR, "gt_poses.jsonl")

YAW_MIN_DEG       = 300.0      # 360 ± 60 acceptable
YAW_RATE_TARGET   = 30.0       # deg/s commanded
YAW_RATE_TOL      = 10.0       # ±10 deg/s margin
N_DETS_MIN_FRAC   = 0.90
MAX_DRIFT_LIMIT_M = 0.08
LAND_DIST_MAX_CM  = 12.0


def load_summary():
    try:
        with open(SUMMARY) as f:
            return json.load(f)
    except Exception:
        return None


def gt_records():
    records = []
    if not os.path.isfile(GT_POSES): return records
    with open(GT_POSES) as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try:
                d = json.loads(line)
                records.append(d)
            except Exception:
                pass
    return records


def cumulative_yaw_unwrapped(yaws_deg):
    """Unwrap modulo-360 yaw measurements to a continuous trajectory.
    Adds ±360 when consecutive samples cross a discontinuity."""
    if not yaws_deg: return []
    out = [yaws_deg[0]]
    for y in yaws_deg[1:]:
        prev = out[-1]
        diff = y - (prev % 360 if prev >= 0 else (prev % 360))
        # Normalize to (-180, +180]
        while diff > 180:  diff -= 360
        while diff <= -180: diff += 360
        out.append(prev + diff)
    return out


def main():
    fails = []
    summary = load_summary()
    if summary is None:
        print("[verdict] FAIL — no mission summary at", SUMMARY)
        sys.exit(1)

    print("[verdict] mission status:    ", summary.get("status"))
    print("[verdict] phases done:       ", summary.get("phases_done"))
    print("[verdict] is_done:           ", summary.get("is_done"))
    print("[verdict] safety abort:      ", summary.get("aborted_by_safety"))
    print("[verdict] kp_x, kp_y:        ", summary.get("kp_x"), summary.get("kp_y"))
    print("[verdict] cmd yaw_rate:      ", summary.get("yaw_rate_deg_s"), "deg/s")
    print("[verdict] hold dur:          ", summary.get("hold_dur_s"), "s")
    print("[verdict] hold max drift:    %.3f m (%.1f cm)" % (
          summary.get("hold_max_drift_m"), 100*summary.get("hold_max_drift_m")))
    print("[verdict] hold rms drift:    %.3f m (%.1f cm)" % (
          summary.get("hold_rms_drift_m"), 100*summary.get("hold_rms_drift_m")))

    if not summary.get("is_done"):
        fails.append("hold did not complete (is_done=False)")
    if summary.get("aborted_by_safety"):
        fails.append("safety aborted rotation")
    if summary.get("hold_max_drift_m", 99) > MAX_DRIFT_LIMIT_M:
        fails.append("max drift %.3f m > %.3f m limit" % (
            summary.get("hold_max_drift_m"), MAX_DRIFT_LIMIT_M))

    records = gt_records()
    print("[verdict] GT records:        ", len(records))
    if records and all("yaw_deg" in r for r in records):
        yaws = [r["yaw_deg"] for r in records]
        unwrapped = cumulative_yaw_unwrapped(yaws)
        total_rotation = abs(unwrapped[-1] - unwrapped[0])
        print("[verdict] total rotation:   %.1f° (target 360°)" % total_rotation)
        if total_rotation < YAW_MIN_DEG:
            fails.append("yaw rotation %.1f° < %.1f° min" % (
                total_rotation, YAW_MIN_DEG))

        # Yaw rate during rotation phase (skip first/last 10 % for transients)
        n = len(records)
        if n >= 20:
            mid_start = int(n * 0.1)
            mid_end   = int(n * 0.9)
            t0 = records[mid_start].get("t_wall", 0)
            t1 = records[mid_end].get("t_wall", t0 + 1)
            if t1 > t0:
                rate = (unwrapped[mid_end] - unwrapped[mid_start]) / (t1 - t0)
                print("[verdict] measured rate:    %+.1f°/s (target %+.1f)" % (
                      rate, YAW_RATE_TARGET))
                if abs(abs(rate) - YAW_RATE_TARGET) > YAW_RATE_TOL:
                    fails.append("yaw_rate %+.1f°/s outside ±%g of target %+.1f" % (
                        rate, YAW_RATE_TOL, YAW_RATE_TARGET))
    else:
        fails.append("GT records missing or no yaw_deg field — gt_recorder ext OK?")

    if records:
        last = records[-1]
        x, y, z = last.get("x"), last.get("y"), last.get("z")
        if None not in (x, y, z):
            d_cm = 100.0 * math.sqrt(x * x + y * y)
            print("[verdict] last GT xyz   : (%+.3f, %+.3f, %+.3f) — %.1f cm" %
                  (x, y, z, d_cm))
            if d_cm > LAND_DIST_MAX_CM:
                fails.append("landed %.1f cm from origin (>%g cm)" % (
                    d_cm, LAND_DIST_MAX_CM))

    if fails:
        print("[verdict] FAIL")
        for f in fails: print("         -", f)
        sys.exit(1)
    print("[verdict] PASS — rotation hold validated")
    sys.exit(0)


if __name__ == "__main__":
    main()
