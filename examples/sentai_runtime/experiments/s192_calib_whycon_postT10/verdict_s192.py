#!/usr/bin/env python3
"""verdict_s192 — summarize s192 detection regression.

Usage:
    python3 verdict_s192.py <iter_dir>

Reads <iter_dir>/mission_s192_summary.json + journal and writes
<iter_dir>/verdict_s192.json + a human-readable verdict.log line.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path


def _load_journal(path: Path) -> list[dict]:
    out: list[dict] = []
    if not path.exists():
        return out
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        # journal lines look like:  "TIMESTAMP event {payload-json}"
        # split off first 2 tokens.
        parts = line.split(None, 2)
        if len(parts) < 3:
            continue
        event = parts[1]
        rest = parts[2]
        try:
            payload = json.loads(rest)
        except json.JSONDecodeError:
            payload = rest
        out.append({"event": event, "payload": payload})
    return out


def _load_gt(path: Path) -> list[dict]:
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(errors="replace").splitlines():
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    return rows


def main():
    if len(sys.argv) < 2:
        print("usage: verdict_s192.py <iter_dir>", file=sys.stderr)
        sys.exit(2)
    iter_dir = Path(sys.argv[1])

    summary_path = iter_dir / "mission_s192_summary.json"
    journal_path = iter_dir / "mission_s192_journal.txt"
    gt_path      = iter_dir / "cf2_gt.jsonl"

    if not summary_path.exists():
        print(f"FAIL: {summary_path} missing")
        sys.exit(1)

    summary = json.loads(summary_path.read_text())
    journal = _load_journal(journal_path)
    gt      = _load_gt(gt_path)

    detect = summary.get("detect_window") or {}
    status = summary.get("status", "?")
    passed = summary.get("pass", False)

    # Landing offset from GT (XY of final pose vs takeoff origin).
    land_err_m = None
    if gt:
        first = gt[0]
        last = gt[-1]
        x0 = first.get("x", 0.0)
        y0 = first.get("y", 0.0)
        x1 = last.get("x", 0.0)
        y1 = last.get("y", 0.0)
        land_err_m = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)

    verdict = {
        "status":               status,
        "phase_reached":        summary.get("phase_reached"),
        "pd_alt_stable":        summary.get("pd_alt_stable", False),
        "ramp_handoff_z_m":     (summary.get("ramp_handoff") or {}).get("pnp", [None]*4)[2]
                                  if summary.get("ramp_handoff") else None,
        "n_total_ticks":        detect.get("n_total"),
        "n_ticks_n_ge_4":       detect.get("n_ge4"),
        "n_ticks_n_ge_6":       detect.get("n_ge6"),
        "n_pnp_valid_ticks":    detect.get("n_pnp_valid"),
        "frac_n_ge_4":          detect.get("frac_ge4"),
        "frac_n_ge_6":          detect.get("frac_ge6"),
        "n_histogram":          detect.get("n_histogram"),
        "pnp_mean":             detect.get("pnp_mean"),
        "pnp_x_range":          detect.get("pnp_x_range"),
        "pnp_y_range":          detect.get("pnp_y_range"),
        "pnp_z_range":          detect.get("pnp_z_range"),
        "land_err_m":           land_err_m,
        "pass":                 passed,
    }

    out_path = iter_dir / "verdict_s192.json"
    out_path.write_text(json.dumps(verdict, indent=2))

    # Human-readable line.
    print("=" * 70)
    print(f"s192 verdict — {iter_dir.name}")
    print("=" * 70)
    print(f"status                : {status}  (phase_reached={summary.get('phase_reached')})")
    print(f"pd alt stable         : {summary.get('pd_alt_stable')}")
    if verdict["ramp_handoff_z_m"] is not None:
        print(f"ramp-handoff z        : {verdict['ramp_handoff_z_m']:.3f} m")
    if detect:
        print(f"detect window ticks   : {detect.get('n_total')}")
        print(f"  n>=4                : {detect.get('n_ge4')}  "
              f"({(detect.get('frac_ge4') or 0)*100:.1f}%)")
        print(f"  n>=6 (full pad)     : {detect.get('n_ge6')}  "
              f"({(detect.get('frac_ge6') or 0)*100:.1f}%)")
        print(f"  PnP-valid           : {detect.get('n_pnp_valid')}")
        print(f"  n histogram         : {detect.get('n_histogram')}")
        mean = detect.get("pnp_mean") or [None]*3
        if mean[0] is not None:
            print(f"  PnP mean xyz        : ({mean[0]:.3f}, {mean[1]:.3f}, {mean[2]:.3f}) m")
            print(f"  PnP x range         : {detect.get('pnp_x_range')}")
            print(f"  PnP y range         : {detect.get('pnp_y_range')}")
            print(f"  PnP z range         : {detect.get('pnp_z_range')}")
    if land_err_m is not None:
        print(f"land err (GT XY)      : {land_err_m*100:.1f} cm")
    print(f"PASS                  : {passed}")
    print("=" * 70)
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
