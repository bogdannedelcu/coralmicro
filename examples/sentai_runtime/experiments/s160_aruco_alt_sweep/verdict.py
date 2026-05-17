"""s160 verdict — sentai.aruco altitude-sweep detection.

Reads mission_s160_summary.json from sentai_fs_root and reports
per-altitude detection rate + unique-id count.

PASS criteria (informational, not a hard regression gate at this point —
this is a CALIBRATION-DATA experiment first; gates land later once we
know what the realistic numbers are):

  * Mission completed (status == "OK").
  * At least ONE altitude with detection_rate >= 0.5.
  * Closure-vs-origin within 15 cm (looser than s146 since the sweep
    flight is longer).
"""
from __future__ import annotations

import ast
import sys
from pathlib import Path


def main(summary_path: str) -> int:
    p = Path(summary_path)
    if not p.exists():
        print(f"FAIL — summary not found: {summary_path}")
        return 1
    raw = p.read_text()
    try:
        summary = ast.literal_eval(raw)
    except (ValueError, SyntaxError) as e:
        print(f"FAIL — summary unparseable: {e}\n--- raw ---\n{raw[:400]}")
        return 1

    print("=" * 60)
    print("s160 verdict — sentai.aruco altitude sweep")
    print("=" * 60)
    print(f"  status          : {summary.get('status')}")
    print(f"  origin          : {summary.get('origin')}")
    print(f"  closure_xy_m    : {summary.get('closure_xy')}")
    print(f"  errors          : {summary.get('errors')}")
    print()
    print(f"  {'z_nom':<8}{'z_act':<8}{'samples':<10}{'rate':<8}"
          f"{'ids':<14}{'tvec_z':<14}{'reproj':<8}")
    print(f"  {'-'*8}{'-'*8}{'-'*10}{'-'*8}{'-'*14}{'-'*14}{'-'*8}")
    rows = summary.get("by_altitude") or []
    best_rate = 0.0
    for r in rows:
        z_act = r.get('z_actual')
        z_act_str = f"{z_act:.3f}" if isinstance(z_act, (int, float)) else "?"
        rate = r.get('detect_rate', 0.0)
        if rate > best_rate:
            best_rate = rate
        ids = r.get('unique_ids') or []
        z_min = r.get('tvec_z_min')
        z_max = r.get('tvec_z_max')
        tvec_str = (f"[{z_min:.2f},{z_max:.2f}]"
                     if z_min is not None and z_max is not None else "<none>")
        reproj = r.get('reproj_mean')
        reproj_str = f"{reproj:.2f}" if reproj is not None else "<none>"
        print(f"  {r.get('z_nominal'):<8}{z_act_str:<8}"
              f"{r.get('n_samples'):<10}{rate:<8.2f}"
              f"{str(ids):<14}{tvec_str:<14}{reproj_str:<8}")

    ok = (summary.get("status") == "OK")
    closure = summary.get("closure_xy")
    ok = ok and (best_rate >= 0.5)
    ok = ok and isinstance(closure, (int, float)) and closure < 0.15
    print()
    print(f"VERDICT: {'PASS' if ok else 'FAIL'} "
          f"(best_rate={best_rate:.2f}, closure={closure})")
    return 0 if ok else 1


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: verdict.py <mission_s160_summary.json>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
