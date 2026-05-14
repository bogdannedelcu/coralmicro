"""s131 verdict — combines synth_result.json + replay_result.json.

synth is the AUTHORITATIVE math validation gate (single-cycle test of
the EKF on controlled synthetic data). Replay is supplementary: it
demonstrates the lifter operates end-to-end on real Gazebo frames
without diverging.

Exit code:
    0 — synth PASS and replay PASS (or SKIPPED)
    1 — synth FAIL (math broken; do NOT proceed to C++ port)
    2 — synth PASS but replay FAIL (math OK, real-frame integration
         issue; investigate before s132)
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

WORKDIR = Path("/tmp/s131_lifter_replay")


def main() -> int:
    synth_path = WORKDIR / "synth_result.json"
    replay_path = WORKDIR / "replay_result.json"

    if not synth_path.exists():
        print(f"[s131 verdict] FAIL — no synth_result.json at {synth_path}")
        Path(WORKDIR / "summary.json").write_text(json.dumps(
            {"pass": False, "reason": "no_synth_result"}, indent=2))
        return 1
    synth = json.loads(synth_path.read_text())

    if not replay_path.exists():
        replay = {"pass": True, "skipped": True,
                  "reason": "no_replay_result_file"}
    else:
        replay = json.loads(replay_path.read_text())

    synth_pass = bool(synth.get("pass", False))
    replay_pass = bool(replay.get("pass", False))
    replay_skipped = bool(replay.get("skipped", False))

    overall_pass = synth_pass and (replay_pass or replay_skipped)
    exit_code = 0 if overall_pass else (1 if not synth_pass else 2)

    summary = {
        "pass": overall_pass,
        "exit_code": exit_code,
        "synth": {
            "pass": synth_pass,
            "scenarios": [
                {"name": s["name"], "pass": s["pass"],
                 "final_err_m": s["final_err_m"],
                 "ready_at_init": s["ready_at_init"],
                 "ready_at_s": s["ready_at_s"]}
                for s in synth.get("scenarios", [])
            ],
        },
        "replay": {
            "pass": replay_pass, "skipped": replay_skipped,
            "reason": replay.get("reason"),
            "per_marker": [
                {"mk": m["marker_id"], "pass": m["pass"],
                 "n_obs": m["n_valid_obs"], "n_rej": m["n_rejected"],
                 "xy_err_m": m["final_xy_err_m"],
                 "z_err_m": m["final_z_err_m"]}
                for m in replay.get("per_marker", [])
            ] if not replay_skipped else [],
        },
    }
    (WORKDIR / "summary.json").write_text(json.dumps(summary, indent=2))

    label = "PASS" if overall_pass else (
        "FAIL_SYNTH" if not synth_pass else "FAIL_REPLAY"
    )
    print(f"[s131 verdict] {label}")
    print(f"  synth  : {'PASS' if synth_pass else 'FAIL'} "
          f"({len(summary['synth']['scenarios'])} scenarios)")
    for s in summary["synth"]["scenarios"]:
        print(f"    - {s['name']}: {'PASS' if s['pass'] else 'FAIL'} "
              f"final_err={s['final_err_m']:.4f}m")
    print(f"  replay : {'PASS' if replay_pass else ('SKIP' if replay_skipped else 'FAIL')}")
    for m in summary["replay"]["per_marker"]:
        print(f"    - mk{m['mk']}: {'PASS' if m['pass'] else 'FAIL'} "
              f"xy_err={m['xy_err_m']:.4f}m z_err={m['z_err_m']:.4f}m "
              f"(n_obs={m['n_obs']}, rej={m['n_rej']})")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
