#!/usr/bin/env python3
"""Compare ARM and SIM `sentai.*` namespace exports.

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
DEFAULT_BINDINGS = ROOT / "examples/sentai_runtime/bindings"
DEFAULT_SIM_DIR = ROOT / "sim"

QSTR_RE = re.compile(r"MP_ROM_QSTR\s*\(\s*MP_QSTR_([A-Za-z0-9_]+)\s*\)")
TABLE_START_RE = re.compile(r"sentai(?:_module)?_globals_table\s*\[\s*\]")
ENTRY_RE = re.compile(
    r"MP_ROM_QSTR\s*\(\s*MP_QSTR_([A-Za-z0-9_]+)\s*\).*?(MP_ROM_PTR|MP_ROM_INT|MP_ROM_QSTR)"
)


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


def _sleep_binding_name(name: str) -> str:
    return "sleep_ns" if name == "sleep" else name


def binding_source(namespace: str, *, sim: bool) -> Path | None:
    if sim:
        sim_path = DEFAULT_SIM_DIR / f"modsentai_sim_{namespace}.c"
        if sim_path.exists():
            return sim_path
    path = DEFAULT_BINDINGS / f"modsentai_{_sleep_binding_name(namespace)}.c"
    if path.exists():
        return path
    return None


def extract_module_members(path: Path, namespace: str, functions_only: bool) -> list[str]:
    table_name = f"sentai_{namespace}_globals_table"
    if namespace == "sleep":
        table_name = "sentai_sleep_globals_table"
    table_re = re.compile(r"\b" + re.escape(table_name) + r"\s*\[\s*\]")

    members: list[str] = []
    in_table = False
    for line in path.read_text(encoding="utf-8").splitlines():
        if not in_table:
            if table_re.search(line):
                in_table = True
            continue
        if line.strip().startswith("};"):
            break
        match = ENTRY_RE.search(line)
        if not match:
            continue
        name, kind = match.groups()
        if name == "__name__":
            continue
        if functions_only and kind != "MP_ROM_PTR":
            continue
        members.append(name)
    return members


def print_block(title: str, names: list[str]) -> None:
    print(title)
    if names:
        print("  " + ", ".join(names))
    else:
        print("  none")


def print_function_inventory(names: list[str]) -> int:
    failed = 0
    for name in names:
        arm_src = binding_source(name, sim=False)
        sim_src = binding_source(name, sim=True)
        if arm_src is None or sim_src is None:
            continue
        arm_members = extract_module_members(arm_src, name, functions_only=True)
        sim_members = extract_module_members(sim_src, name, functions_only=True)
        arm = set(arm_members)
        sim = set(sim_members)
        arm_only = sorted(arm - sim)
        sim_only = sorted(sim - arm)
        source_note = "shared" if arm_src == sim_src else "override"

        print(f"\n{name}: {source_note}")
        print(f"  ARM source: {arm_src.relative_to(ROOT)}")
        print(f"  SIM source: {sim_src.relative_to(ROOT)}")
        print(f"  ARM funcs: {len(arm_members)}")
        print(f"  SIM funcs: {len(sim_members)}")
        print_block("  ARM-only funcs:", arm_only)
        print_block("  SIM-only funcs:", sim_only)
        if arm_only or sim_only:
            failed = 1
    return failed


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
    parser.add_argument(
        "--functions",
        action="store_true",
        help="Also compare function-level exports for each shared namespace.",
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

    rc = 1 if (arm_only or sim_only) else 0
    if args.functions:
        rc |= print_function_inventory(shared)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
