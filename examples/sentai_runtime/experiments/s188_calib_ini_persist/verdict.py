"""s188 verdict -- OP-S10-W21-T2 INI persist PASS gate."""
from __future__ import annotations

import re
import sys
from pathlib import Path


def main(log_path: str) -> int:
    log = Path(log_path).read_text()
    print(log, end="" if log.endswith("\n") else "\n")

    pat = re.compile(r"\[s188\] T(\d+) (PASS|FAIL) (.+)$", re.MULTILINE)
    matches = pat.findall(log)
    expected = {"1", "2", "3", "4", "5"}
    seen = {m[0] for m in matches}
    missing = expected - seen
    fails = [(t, msg) for t, status, msg in matches if status == "FAIL"]

    overall_pass = "[s188] OVERALL PASS" in log
    overall_fail = "[s188] OVERALL FAIL" in log

    print("=" * 60)
    print("s188 verdict -- OP-S10-W21-T2 INI persist smoke")
    print("=" * 60)
    print(f"  T-lines seen      : {sorted(seen) or '<none>'}")
    print(f"  T-lines missing   : {sorted(missing) or '<none>'}")
    print(f"  T failures        : {fails or '<none>'}")
    print(f"  OVERALL PASS line : {overall_pass}")
    print(f"  OVERALL FAIL line : {overall_fail}")

    if missing or fails or (not overall_pass) or overall_fail:
        print("VERDICT: FAIL")
        return 1
    print("VERDICT: PASS")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: verdict.py <sentai_sim.log>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
