#!/usr/bin/env python3
"""Compare ARM and SIM root `sentai.*` namespace exports.

This is intentionally source-based.  It catches drift before either runtime is
booted, and it stays small enough to be useful during namespace alignment work.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ARM = ROOT / "examples/sentai_runtime/modsentai.c"
DEFAULT_SIM = ROOT / "sim/modsentai_sim.c"

QSTR_RE = re.compile(r"MP_ROM_QSTR\s*\(\s*MP_QSTR_([A-Za-z0-9_]+)\s*\)")
TABLE_START_RE = re.compile(r"sentai(?:_module)?_globals_table\s*\[\s*\]")


def extract_root_exports(path: Path) -> list[str]:
    exports: list[str] = []
    in_table = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if not in_table:
            if TABLE_START_RE.search(line):
                in_table = True
            continue
        if line.strip().startswith("};"):
            break
        match = QSTR_RE.search(line)
        if not match:
            continue
        name = match.group(1)
        if name == "__name__":
            continue
        exports.append(name)
    return exports


def print_block(title: str, names: list[str]) -> None:
    print(title)
    if names:
        print("  " + ", ".join(names))
    else:
        print("  none")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare root sentai.* namespace exports between ARM and SIM"
    )
    parser.add_argument("--arm", type=Path, default=DEFAULT_ARM)
    parser.add_argument("--sim", type=Path, default=DEFAULT_SIM)
    parser.add_argument(
        "--allow-sim-only",
        action="append",
        default=["sim"],
        help="Name allowed to exist only in SIM. May be repeated.",
    )
    args = parser.parse_args()

    arm_exports = extract_root_exports(args.arm)
    sim_exports = extract_root_exports(args.sim)
    arm = set(arm_exports)
    sim = set(sim_exports)
    allowed_sim_only = set(args.allow_sim_only or [])

    arm_only = sorted(arm - sim)
    sim_only = sorted((sim - arm) - allowed_sim_only)
    allowed = sorted((sim - arm) & allowed_sim_only)
    shared = [name for name in arm_exports if name in sim]

    print(f"ARM source: {args.arm}")
    print(f"SIM source: {args.sim}")
    print(f"ARM exports: {len(arm_exports)}")
    print(f"SIM exports: {len(sim_exports)}")
    print(f"Shared exports: {len(shared)}")
    print_block("ARM-only exports:", arm_only)
    print_block("SIM-only exports:", sim_only)
    print_block("Allowed SIM-only exports:", allowed)

    if arm_only or sim_only:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
