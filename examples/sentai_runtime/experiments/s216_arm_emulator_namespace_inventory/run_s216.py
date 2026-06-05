#!/usr/bin/env python3
"""Run S216: inventory the current ARM-emulator sentai.* namespace surface."""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import shutil
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[4]
EXP = pathlib.Path(__file__).resolve().parent
RENODE = pathlib.Path("/home/bogdan/work/renode_portable/renode")
BUILD_EMU = ROOT / "build_emu"
TARGET = "sentai_emu_namespace_inventory"
RENODE_SCRIPT = ROOT / "emu/renode/sentai_emu_namespace_inventory.resc"
RENODE_UI_SCRIPT = ROOT / "emu/renode/sentai_emu_namespace_inventory_ui.resc"
UART_LOG = ROOT / "emu/output/sentai_emu_namespace_inventory.log"
ARM_ROOT = ROOT / "examples/sentai_runtime/modsentai.c"
BINDINGS = ROOT / "examples/sentai_runtime/bindings"

QSTR_RE = re.compile(r"MP_ROM_QSTR\s*\(\s*MP_QSTR_([A-Za-z0-9_]+)\s*\)")
TABLE_START_RE = re.compile(r"sentai(?:_module)?_globals_table\s*\[\s*\]")
ENTRY_RE = re.compile(
    r"MP_ROM_QSTR\s*\(\s*MP_QSTR_([A-Za-z0-9_]+)\s*\).*?"
    r"(MP_ROM_PTR|MP_ROM_INT|MP_ROM_QSTR)"
)


def next_iter_dir() -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9]*_*"))
    next_id = 1
    if existing:
        next_id = 1 + max(
            int(path.name.split("_", 1)[0].replace("iter", ""))
            for path in existing
        )
    out = EXP / f"iter{next_id:02d}_namespace_inventory"
    out.mkdir(parents=True)
    return out


def run_to_file(cmd: list[str], cwd: pathlib.Path, log_path: pathlib.Path,
                timeout_s: int | None = None) -> subprocess.CompletedProcess[str]:
    started = time.monotonic()
    with log_path.open("w+", encoding="utf-8") as log:
        log.write("$ " + " ".join(cmd) + "\n")
        proc = subprocess.Popen(
            cmd,
            cwd=str(cwd),
            text=True,
            stdout=log,
            stderr=subprocess.STDOUT,
        )
        timed_out = False
        try:
            rc = proc.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            timed_out = True
            proc.terminate()
            try:
                rc = proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                rc = proc.wait()
        elapsed = time.monotonic() - started
        if timed_out:
            log.write(f"\n[TIMEOUT] exceeded {timeout_s}s\n")
        log.write(f"\n[elapsed_s] {elapsed:.3f}\n")
        log.seek(0)
        out = log.read()
    return subprocess.CompletedProcess(cmd, rc, out)


def run_capture(cmd: list[str], cwd: pathlib.Path, timeout_s: int = 30) -> str:
    proc = subprocess.run(
        cmd, cwd=str(cwd), text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=timeout_s, check=False)
    return proc.stdout


def parse_hex(label: str, text: str) -> int | None:
    match = re.search(rf"{re.escape(label)}:\s*\r?\n\s*(0x[0-9A-Fa-f]+)",
                      text)
    if not match:
        return None
    return int(match.group(1), 16)


def extract_root_exports(path: pathlib.Path) -> list[str]:
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
        if name != "__name__":
            exports.append(name)
    return exports


def binding_source(namespace: str) -> pathlib.Path | None:
    source_name = "sleep_ns" if namespace == "sleep" else namespace
    path = BINDINGS / f"modsentai_{source_name}.c"
    return path if path.exists() else None


def extract_module_members(path: pathlib.Path, namespace: str) -> list[str]:
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
        name, _kind = match.groups()
        if name != "__name__":
            members.append(name)
    return members


def parse_inventory(text: str) -> dict[str, object]:
    root: list[str] = []
    namespaces: dict[str, dict[str, object]] = {}
    help_status: dict[str, str] = {}
    version = None
    fs_write_size = None
    saw_begin = False
    saw_end = False

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line == "INV_BEGIN":
            saw_begin = True
        elif line == "INV_END":
            saw_end = True
        elif line.startswith("VERSION|"):
            version = line.split("|", 1)[1]
        elif line.startswith("ROOT|"):
            payload = line.split("|", 1)[1]
            root = [part for part in payload.split(",") if part]
        elif line.startswith("NS|"):
            parts = line.split("|", 3)
            if len(parts) == 4:
                _, name, status, payload = parts
                namespaces[name] = {
                    "status": status,
                    "members": [part for part in payload.split(",") if part],
                }
        elif line.startswith("HELP|"):
            parts = line.split("|")
            if len(parts) >= 3:
                topic = parts[1]
                status = parts[2]
                if status != "begin":
                    help_status[topic] = status
        elif line.startswith("FS_WRITE|"):
            try:
                fs_write_size = int(line.split("|", 1)[1])
            except ValueError:
                fs_write_size = None

    return {
        "saw_begin": saw_begin,
        "saw_end": saw_end,
        "version": version,
        "root": root,
        "namespaces": namespaces,
        "help": help_status,
        "fs_write_size": fs_write_size,
    }


def compare_to_shared_root(inventory: dict[str, object]) -> dict[str, object]:
    arm_exports = extract_root_exports(ARM_ROOT)
    emu_root = inventory.get("root", [])
    emu = set(emu_root if isinstance(emu_root, list) else [])
    arm = set(arm_exports)
    namespaces = inventory.get("namespaces", {})

    member_compare: dict[str, object] = {}
    if isinstance(namespaces, dict):
        for name, info in namespaces.items():
            if not isinstance(info, dict) or info.get("status") != "present":
                continue
            source = binding_source(name)
            if source is None:
                continue
            shared_members = extract_module_members(source, name)
            emu_members = set(info.get("members", []))
            shared = set(shared_members)
            member_compare[name] = {
                "shared_source": str(source.relative_to(ROOT)),
                "shared_members": shared_members,
                "emu_members": info.get("members", []),
                "emu_only": sorted(emu_members - shared),
                "shared_only": sorted(shared - emu_members),
            }

    return {
        "shared_root_source": str(ARM_ROOT.relative_to(ROOT)),
        "shared_root_exports": arm_exports,
        "present_shared": [name for name in arm_exports if name in emu],
        "missing_shared": [name for name in arm_exports if name not in emu],
        "unexpected_emu": sorted(emu - arm),
        "member_compare": member_compare,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--renode-ui", action="store_true",
                        help=("run Renode with UI and UART analyzer enabled "
                              "instead of --plain/--disable-xwt"))
    args = parser.parse_args()

    iter_dir = next_iter_dir()
    (iter_dir / "renode").mkdir()
    renode_script = RENODE_UI_SCRIPT if args.renode_ui else RENODE_SCRIPT
    renode_cmd = [str(RENODE), "--console", str(renode_script.relative_to(ROOT))]
    if not args.renode_ui:
        renode_cmd = [str(RENODE), "--plain", "--console", "--disable-xwt",
                      str(renode_script.relative_to(ROOT))]

    commands = {
        "configure": [
            "cmake", "-S", ".", "-B", str(BUILD_EMU.relative_to(ROOT)),
            "-DSENTAI_ARM_EMU=ON", "-DSENTAI_SKIP_SDK_PATCHES=ON",
        ],
        "build": [
            "cmake", "--build", str(BUILD_EMU.relative_to(ROOT)),
            "--target", TARGET, f"-j{subprocess.os.cpu_count() or 1}",
        ],
        "renode": renode_cmd,
    }
    results: dict[str, object] = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "target": TARGET,
        "renode_script": str(renode_script.relative_to(ROOT)),
        "renode_ui": args.renode_ui,
        "commands": commands,
    }

    for name in ("configure", "build"):
        proc = run_to_file(commands[name], ROOT, iter_dir / f"{name}.log", 300)
        results[f"{name}_rc"] = proc.returncode
        if proc.returncode != 0:
            (iter_dir / "verdict_s216.json").write_text(
                json.dumps(results, indent=2) + "\n", encoding="utf-8")
            print(proc.stdout)
            return proc.returncode

    if UART_LOG.exists():
        UART_LOG.unlink()

    ss_before = run_capture(["ps", "-eo", "pid,ppid,stat,cmd"], ROOT)
    (iter_dir / "processes_before_renode.log").write_text(ss_before,
                                                          encoding="utf-8")

    renode_proc = run_to_file(commands["renode"], ROOT,
                              iter_dir / "renode.log", 120)
    results["renode_rc"] = renode_proc.returncode

    uart_text = ""
    if UART_LOG.exists():
        shutil.copy2(UART_LOG, iter_dir / "uart.log")
        uart_text = (iter_dir / "uart.log").read_text(
            encoding="utf-8", errors="replace")
    else:
        results["uart_log_missing"] = True

    shutil.copy2(renode_script, iter_dir / "renode" / renode_script.name)
    shutil.copy2(ROOT / "emu/renode/sentai_rt1176.repl",
                 iter_dir / "renode/sentai_rt1176.repl")

    symbols = {
        "boot_state": parse_hex("sentai_emu_namespace_inventory boot_state",
                                renode_proc.stdout),
        "heartbeat": parse_hex("sentai_emu_namespace_inventory heartbeat",
                               renode_proc.stdout),
        "repl_lines": parse_hex("sentai_emu_namespace_inventory repl_lines",
                                renode_proc.stdout),
        "last_tick": parse_hex("sentai_emu_namespace_inventory last_tick",
                               renode_proc.stdout),
    }
    inventory = parse_inventory(uart_text)
    comparison = compare_to_shared_root(inventory)
    results["symbols"] = symbols
    results["inventory"] = inventory
    results["comparison"] = comparison
    results["pass"] = (
        renode_proc.returncode == 0
        and symbols.get("boot_state") == 0x0500
        and inventory.get("saw_begin") is True
        and inventory.get("saw_end") is True
        and isinstance(inventory.get("fs_write_size"), int)
        and inventory["fs_write_size"] > 0
    )

    (iter_dir / "inventory_s216.txt").write_text(
        "\n".join(
            line.strip() for line in uart_text.splitlines()
            if line.startswith(("INV_", "VERSION|", "ROOT|", "NS|",
                                "HELP|", "FS_WRITE"))
        ) + "\n",
        encoding="utf-8",
    )
    (iter_dir / "comparison_s216.json").write_text(
        json.dumps(comparison, indent=2) + "\n", encoding="utf-8")
    (iter_dir / "verdict_s216.json").write_text(
        json.dumps(results, indent=2) + "\n", encoding="utf-8")

    ps_after = run_capture(["ps", "-eo", "pid,ppid,stat,cmd"], ROOT)
    (iter_dir / "processes_after_renode.log").write_text(ps_after,
                                                         encoding="utf-8")

    present = comparison.get("present_shared", [])
    missing = comparison.get("missing_shared", [])
    print(f"S216 iter: {iter_dir.relative_to(ROOT)}")
    print(f"pass={results['pass']} present_shared={len(present)} "
          f"missing_shared={len(missing)} "
          f"unexpected={len(comparison.get('unexpected_emu', []))}")
    return 0 if results["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
