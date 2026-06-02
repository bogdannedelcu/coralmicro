#!/usr/bin/env python3
"""Run B8.1 ARM emulator idle heartbeat experiment."""

from __future__ import annotations

import json
import argparse
import pathlib
import re
import shutil
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[4]
EXP = pathlib.Path(__file__).resolve().parent
RENODE = pathlib.Path("/home/bogdan/work/renode_portable/renode")
BUILD_DIR = ROOT / "build_emu"
TARGETS = {
    "idle": {
        "cmake_target": "sentai_emu_idle",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_idle.resc",
        "iter_suffix": "renode_idle_heartbeat",
        "uart_log": None,
    },
    "uart": {
        "cmake_target": "sentai_emu_uart",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_uart.resc",
        "iter_suffix": "renode_uart_heartbeat",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_uart.log",
    },
    "repl": {
        "cmake_target": "sentai_emu_repl",
        "renode_script": ROOT / "emu" / "renode" / "sentai_emu_repl.resc",
        "iter_suffix": "renode_repl_oneplusone",
        "uart_log": ROOT / "emu" / "output" / "sentai_emu_repl.log",
    },
}


def next_iter_dir(suffix: str) -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9][0-9]_*"))
    next_id = 1
    if existing:
        last = existing[-1].name.split("_", 1)[0]
        next_id = int(last.replace("iter", "")) + 1
    out = EXP / f"iter{next_id:02d}_{suffix}"
    out.mkdir(parents=True)
    return out


def run_cmd(cmd: list[str], cwd: pathlib.Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )


def parse_hex(label: str, text: str) -> int | None:
    pattern = rf"{re.escape(label)}:\s*\r?\n\s*(0x[0-9A-Fa-f]+)"
    match = re.search(pattern, text)
    if not match:
        return None
    return int(match.group(1), 16)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", choices=sorted(TARGETS), default="idle")
    args = parser.parse_args()
    cfg = TARGETS[args.target]

    iter_dir = next_iter_dir(cfg["iter_suffix"])
    snapshots = iter_dir / "renode"
    snapshots.mkdir()

    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    configure_cmd = [
        "cmake",
        "-S",
        ".",
        "-B",
        str(BUILD_DIR.relative_to(ROOT)),
        "-DSENTAI_ARM_EMU=ON",
        "-DSENTAI_SKIP_SDK_PATCHES=ON",
    ]
    build_cmd = [
        "cmake",
        "--build",
        str(BUILD_DIR.relative_to(ROOT)),
        "--target",
        cfg["cmake_target"],
        f"-j{subprocess.os.cpu_count() or 1}",
    ]
    renode_cmd = [
        str(RENODE),
        "--plain",
        "--console",
        "--disable-xwt",
        str(cfg["renode_script"].relative_to(ROOT)),
    ]

    logs: dict[str, subprocess.CompletedProcess[str]] = {}
    for name, cmd in (
        ("configure", configure_cmd),
        ("build", build_cmd),
        ("renode", renode_cmd),
    ):
        result = run_cmd(cmd, ROOT)
        logs[name] = result
        (iter_dir / f"{name}.log").write_text(result.stdout)
        if result.returncode != 0:
            break

    renode_log = logs.get("renode").stdout if "renode" in logs else ""
    renode_label = cfg["cmake_target"]
    boot_state = parse_hex(f"{renode_label} boot_state", renode_log)
    heartbeat = parse_hex(f"{renode_label} heartbeat", renode_log)
    last_tick = parse_hex(f"{renode_label} last_tick", renode_log)
    repl_lines = parse_hex(f"{renode_label} repl_lines", renode_log)

    if args.target == "repl":
        # REPL target reuses heartbeat as "completed mp_embed_exec_str count";
        # boot_state=0x500 marks the post-banner state.
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x500
            and repl_lines is not None
            and repl_lines >= 1
            and last_tick is not None
            and last_tick > 0
        )
    else:
        passed = (
            all(result.returncode == 0 for result in logs.values())
            and boot_state == 0x300
            and heartbeat is not None
            and heartbeat > 1
            and last_tick is not None
            and last_tick > 0
        )

    for src in (
        ROOT / "emu" / "renode" / "sentai_rt1176.repl",
        cfg["renode_script"],
    ):
        shutil.copy2(src, snapshots / src.name)

    uart_log_text = ""
    uart_log_bytes = b""
    if cfg["uart_log"] and cfg["uart_log"].exists():
        uart_log_bytes = cfg["uart_log"].read_bytes()
        uart_log_text = uart_log_bytes.decode("utf-8", errors="replace")
        # Preserve raw CRLF bytes — read_text/write_text would translate them
        # and turn the REPL answer assertion below into a false negative.
        (iter_dir / "uart.log").write_bytes(uart_log_bytes)

    if args.target == "uart":
        passed = passed and b"SentAI EMU UART boot" in uart_log_bytes
        passed = passed and b"SentAI EMU UART heartbeat" in uart_log_bytes

    if args.target == "repl":
        # Inputs injected by the resc are `1+1\r`; the REPL prints the prompt
        # before, echoes the typed bytes, then prints `2` followed by another
        # prompt. Assert the value rather than the prompt to catch silent
        # parsing failures.
        passed = passed and b"MicroPython embed ready" in uart_log_bytes
        passed = passed and b"\r\n2\r\n" in uart_log_bytes

    verdict = {
        "experiment": "s213_arm_emulator_idle",
        "iter": iter_dir.name,
        "target_kind": args.target,
        "started_at": started_at,
        "commands": {
            "configure": configure_cmd,
            "build": build_cmd,
            "renode": renode_cmd,
        },
        "target": str((BUILD_DIR / "emu" / cfg["cmake_target"]).relative_to(ROOT)),
        "renode_script": str(cfg["renode_script"].relative_to(ROOT)),
        "returncodes": {name: result.returncode for name, result in logs.items()},
        "boot_state": boot_state,
        "heartbeat": heartbeat,
        "last_tick": last_tick,
        "repl_lines": repl_lines,
        "uart_log_contains_boot": b"SentAI EMU UART boot" in uart_log_bytes,
        "uart_log_contains_heartbeat": b"SentAI EMU UART heartbeat" in uart_log_bytes,
        "uart_log_contains_repl_banner": b"MicroPython embed ready" in uart_log_bytes,
        "uart_log_contains_repl_answer": b"\r\n2\r\n" in uart_log_bytes,
        "pass": passed,
    }
    (iter_dir / "verdict_s213.json").write_text(json.dumps(verdict, indent=2) + "\n")

    print(json.dumps(verdict, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
