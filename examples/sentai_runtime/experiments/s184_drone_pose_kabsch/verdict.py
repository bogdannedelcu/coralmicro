#!/usr/bin/env python3
"""s184 verdict -- OP-S10-W19-T6b drone-pose Kabsch + yaw-anchor smoke."""

import re
import sys
from pathlib import Path


def main():
    if len(sys.argv) != 2:
        print("usage: verdict.py <sentai_sim.log>", file=sys.stderr)
        sys.exit(2)
    log = Path(sys.argv[1]).read_text(errors="replace")

    expected = {"1", "2", "3", "4", "5", "6"}
    seen = set()
    failures = []
    for line in log.splitlines():
        m = re.match(r"^\[s184\] T(\d+) (PASS|FAIL)\b", line)
        if not m:
            continue
        n, status = m.group(1), m.group(2)
        seen.add(n)
        if status == "FAIL":
            failures.append(line.rstrip())

    overall_pass = bool(re.search(r"^\[s184\] OVERALL PASS\b", log, re.M))
    overall_fail = bool(re.search(r"^\[s184\] OVERALL FAIL\b", log, re.M))
    missing = expected - seen

    sep = "=" * 60
    print(sep)
    print("s184 verdict -- OP-S10-W19-T6b drone-pose Kabsch smoke")
    print(sep)
    print(f"  T-lines seen      : {sorted(seen)}")
    print(f"  T-lines missing   : {sorted(missing) if missing else '<none>'}")
    print(f"  T failures        : {failures if failures else '<none>'}")
    print(f"  OVERALL PASS line : {overall_pass}")
    print(f"  OVERALL FAIL line : {overall_fail}")

    if missing or failures or not overall_pass or overall_fail:
        print("VERDICT: FAIL")
        sys.exit(1)
    print("VERDICT: PASS")


if __name__ == "__main__":
    main()
