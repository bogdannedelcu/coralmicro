"""s159 verdict — OP-S6-W3-T7 PASS gate for sentai.aruco synthetic smoke.

Mirrors s157_calib_smoke/verdict.py: PASS iff every `[s159] Tn …` line
is PASS AND a final `[s159] OVERALL PASS` is present.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path


def main(log_path: str) -> int:
    log = Path(log_path).read_text()
    print(log, end="" if log.endswith("\n") else "\n")

    # Lines look like: "[s159] T4a id=0 size=48 PASS n=1 ...".  The
    # T-tag is the first whitespace-delimited token after the tag;
    # status is the first PASS/FAIL token thereafter.
    pat = re.compile(r"\[s159\] (T\S+)\s+.*?(PASS|FAIL)", re.MULTILINE)
    matches = pat.findall(log)
    fails = [(t,) for t, status in matches if status == "FAIL"]
    overall_pass = "[s159] OVERALL PASS" in log
    overall_fail = "[s159] OVERALL FAIL" in log
    t_names = sorted({m[0] for m in matches})

    print("=" * 60)
    print("s159 verdict — OP-S6-W3-T7 sentai.aruco synthetic smoke")
    print("=" * 60)
    print(f"  T-lines seen      : {len(t_names)} ({t_names})")
    print(f"  T failures        : {fails or '<none>'}")
    print(f"  OVERALL PASS line : {overall_pass}")
    print(f"  OVERALL FAIL line : {overall_fail}")

    if fails or (not overall_pass) or overall_fail or len(t_names) < 10:
        print("VERDICT: FAIL")
        return 1
    print("VERDICT: PASS")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: verdict.py <sentai_sim.log>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
