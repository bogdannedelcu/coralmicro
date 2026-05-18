#!/usr/bin/env python3
"""s173 verdict — Flow loop closed-loop validation.

PASS iff:
  - mission produced a summary
  - is_done is True (HOLD ran for full duration)
  - HOLD max_drift < 0.10 m (10 cm tolerance for both-axes hold)
  - HOLD rms_drift < 0.05 m  (5 cm RMS — reasonable steady-state)
  - drone landed ≤ 10 cm of origin               [[sim-test-must-return-home]]
"""

import json, math, os, sys

WORKDIR = "/tmp/s173_flow_hold_validation"
FS_ROOT = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "build-sim", "sentai_fs_root")
SUMMARY = os.path.join(FS_ROOT, "mission_flow_hold_summary.json")
GT_POSES = os.path.join(WORKDIR, "gt_poses.jsonl")

MAX_DRIFT_LIMIT_M = 0.10
RMS_DRIFT_LIMIT_M = 0.05
LAND_DIST_MAX_CM  = 10.0


def load_summary():
    try:
        with open(SUMMARY) as f:
            return json.load(f)
    except Exception:
        return None


def last_gt_pose():
    if not os.path.isfile(GT_POSES): return None
    last = None
    with open(GT_POSES) as f:
        for line in f:
            line = line.strip()
            if not line: continue
            try: last = json.loads(line)
            except Exception: pass
    return last


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
    print("[verdict] kp_x, kp_y used:   ", summary.get("kp_x"), summary.get("kp_y"))
    print("[verdict] hold max drift:    %.3f m (%.1f cm)" % (
          summary.get("hold_max_drift_m"), 100*summary.get("hold_max_drift_m")))
    print("[verdict] hold rms drift:    %.3f m (%.1f cm)" % (
          summary.get("hold_rms_drift_m"), 100*summary.get("hold_rms_drift_m")))

    if not summary.get("is_done"):
        fails.append("hold did not complete (is_done=False)")
    if summary.get("aborted_by_safety"):
        fails.append("safety aborted hold")
    if summary.get("hold_max_drift_m", 99) > MAX_DRIFT_LIMIT_M:
        fails.append("max drift %.3f m > %.3f m limit" % (
            summary.get("hold_max_drift_m"), MAX_DRIFT_LIMIT_M))
    if summary.get("hold_rms_drift_m", 99) > RMS_DRIFT_LIMIT_M:
        fails.append("RMS drift %.3f m > %.3f m limit" % (
            summary.get("hold_rms_drift_m"), RMS_DRIFT_LIMIT_M))

    last = last_gt_pose()
    if last is not None:
        if "pose" in last and isinstance(last["pose"], dict):
            x, y, z = last["pose"].get("x"), last["pose"].get("y"), last["pose"].get("z")
        else:
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
    print("[verdict] PASS — closed-loop Flow held within bounds")
    sys.exit(0)


if __name__ == "__main__":
    main()
