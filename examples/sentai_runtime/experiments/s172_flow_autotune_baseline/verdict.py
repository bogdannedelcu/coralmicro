#!/usr/bin/env python3
"""s172 verdict — Flow autotune baseline.

PASS iff:
  - mission produced a summary
  - Kp_x ∈ [0.2, 8.0]   (sanity bounds; ZN for our drone should land here)
  - is_done is True (not timed out or safety-aborted)
  - drone landed ≤ 10 cm of origin                          [[sim-test-must-return-home]]
  - sentai.fr events.csv contains "autotune_done_ok"
"""

import json, math, os, sys

WORKDIR = "/tmp/s172_flow_autotune_baseline"
FS_ROOT = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "build-sim", "sentai_fs_root")
SUMMARY = os.path.join(FS_ROOT, "mission_flow_autotune_summary.json")
FR_EVENTS = os.path.join(WORKDIR, "fr_current", "events.csv")
GT_POSES  = os.path.join(WORKDIR, "gt_poses.jsonl")

KP_LO, KP_HI = 0.20, 8.0
LAND_DIST_MAX_CM = 10.0


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


def fr_has_event(name):
    if not os.path.isfile(FR_EVENTS): return False
    with open(FR_EVENTS) as f:
        for line in f:
            if line.startswith("#"): continue
            parts = line.strip().split(",", 2)
            if len(parts) >= 2 and parts[1].strip() == name:
                return True
    return False


def main():
    fails = []
    summary = load_summary()
    if summary is None:
        print("[verdict] FAIL — no mission summary at", SUMMARY)
        sys.exit(1)

    print("[verdict] mission status:", summary.get("status"))
    print("[verdict] phases done:   ", summary.get("phases_done"))
    print("[verdict] is_done:       ", summary.get("is_done"))
    print("[verdict] safety abort:  ", summary.get("aborted_by_safety"))
    print("[verdict] Kp_x:          ", summary.get("kp_x"))
    print("[verdict] Kp_y:          ", summary.get("kp_y"))
    print("[verdict] td_ms:         ", summary.get("td_ms"))

    if not summary.get("is_done"):
        fails.append("autotune did not converge (is_done=False)")
    if summary.get("aborted_by_safety"):
        fails.append("safety aborted autotune")

    kp_x = summary.get("kp_x", -1.0)
    if kp_x < KP_LO or kp_x > KP_HI:
        fails.append("Kp_x=%.3f outside sanity bounds [%g, %g]" % (
            kp_x, KP_LO, KP_HI))

    last = last_gt_pose()
    if last is not None:
        # GT pose schema from gt_recorder.py: dict with 'pose' key carrying
        # {x,y,z}; some recorders use top-level x/y/z.  Try both.
        if "pose" in last and isinstance(last["pose"], dict):
            x, y, z = last["pose"].get("x"), last["pose"].get("y"), last["pose"].get("z")
        else:
            x, y, z = last.get("x"), last.get("y"), last.get("z")
        if None not in (x, y, z):
            d_cm = 100.0 * math.sqrt(x * x + y * y)
            print("[verdict] last GT xyz   :  (%+.3f, %+.3f, %+.3f) — %.1f cm from origin" %
                  (x, y, z, d_cm))
            if d_cm > LAND_DIST_MAX_CM:
                fails.append("landed %.1f cm from origin (>%g cm)" % (
                    d_cm, LAND_DIST_MAX_CM))
        else:
            fails.append("GT pose schema unknown")
    else:
        fails.append("no GT poses captured")

    if not fr_has_event("autotune_done_ok"):
        fails.append('sentai.fr events.csv missing "autotune_done_ok"')

    if fails:
        print("[verdict] FAIL")
        for f in fails: print("         -", f)
        sys.exit(1)
    print("[verdict] PASS — Kp_x=%.3f, landed within %.1f cm" % (
        kp_x, 100.0 * math.sqrt((last.get('pose', last).get('x') or 0)**2
                                 + (last.get('pose', last).get('y') or 0)**2)))
    sys.exit(0)


if __name__ == "__main__":
    main()
