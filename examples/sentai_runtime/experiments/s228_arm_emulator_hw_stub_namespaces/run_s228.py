#!/usr/bin/env python3
"""Run S228: verify hardware/safety sentai.* stubs in ARM emulation."""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import shutil
import subprocess
import time


ROOT = pathlib.Path(__file__).resolve().parents[4]
EXP = pathlib.Path(__file__).resolve().parent
RENODE = pathlib.Path("/home/bogdan/work/renode_portable/renode")
BUILD_EMU = ROOT / "build_emu"
TARGET = "sentai_emu_hw_stub_namespace_smoke"
RENODE_SCRIPT = ROOT / "emu/renode/sentai_emu_hw_stub_namespace_smoke.resc"
RENODE_UI_SCRIPT = ROOT / "emu/renode/sentai_emu_hw_stub_namespace_smoke_ui.resc"
UART_LOG = ROOT / "emu/output/sentai_emu_hw_stub_namespace_smoke.log"


def next_iter_dir() -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9]*_*"))
    next_id = 1
    if existing:
        next_id = 1 + max(
            int(path.name.split("_", 1)[0].replace("iter", ""))
            for path in existing
        )
    out = EXP / f"iter{next_id:02d}_hw_stub_namespaces"
    out.mkdir(parents=True)
    return out


def run_to_file(cmd: list[str], cwd: pathlib.Path, log_path: pathlib.Path,
                timeout_s: int | None = None) -> subprocess.CompletedProcess[str]:
    started = time.monotonic()
    with log_path.open("w+", encoding="utf-8") as log:
        log.write("$ " + " ".join(cmd) + "\n")
        proc = subprocess.Popen(
            cmd, cwd=str(cwd), text=True,
            stdout=log, stderr=subprocess.STDOUT)
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


def parse_hw(text: str) -> dict[str, object]:
    checks: dict[str, dict[str, str]] = {}
    fails = None
    fs_write_size = None
    saw_begin = False
    saw_end = False
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line == "HW_BEGIN":
            saw_begin = True
        elif line == "HW_END":
            saw_end = True
        elif line.startswith("CHECK|"):
            parts = line.split("|", 3)
            if len(parts) == 4:
                _, name, status, detail = parts
                checks[name] = {"status": status, "detail": detail}
        elif line.startswith("HW_FAILS|"):
            try:
                fails = int(line.split("|", 1)[1])
            except ValueError:
                fails = None
        elif line.startswith("FS_WRITE|"):
            try:
                fs_write_size = int(line.split("|", 1)[1])
            except ValueError:
                fs_write_size = None
    return {
        "saw_begin": saw_begin,
        "saw_end": saw_end,
        "checks": checks,
        "fails": fails,
        "fs_write_size": fs_write_size,
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
            (iter_dir / "verdict_s228.json").write_text(
                json.dumps(results, indent=2) + "\n", encoding="utf-8")
            print(proc.stdout)
            return proc.returncode

    if UART_LOG.exists():
        UART_LOG.unlink()

    (iter_dir / "processes_before_renode.log").write_text(
        run_capture(["ps", "-eo", "pid,ppid,stat,cmd"], ROOT),
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
        "boot_state": parse_hex("sentai_emu_hw_stub_namespace_smoke boot_state",
                                renode_proc.stdout),
        "heartbeat": parse_hex("sentai_emu_hw_stub_namespace_smoke heartbeat",
                               renode_proc.stdout),
        "repl_lines": parse_hex("sentai_emu_hw_stub_namespace_smoke repl_lines",
                                renode_proc.stdout),
        "last_tick": parse_hex("sentai_emu_hw_stub_namespace_smoke last_tick",
                               renode_proc.stdout),
    }
    hw = parse_hw(uart_text)
    results["symbols"] = symbols
    results["hardware_stubs"] = hw
    results["pass"] = (
        renode_proc.returncode == 0
        and symbols.get("boot_state") == 0x0500
        and hw.get("saw_begin") is True
        and hw.get("saw_end") is True
        and hw.get("fails") == 0
        and isinstance(hw.get("fs_write_size"), int)
        and hw["fs_write_size"] > 0
    )

    (iter_dir / "hardware_s228.txt").write_text(
        "\n".join(
            line.strip() for line in uart_text.splitlines()
            if line.startswith(("HW_", "CHECK|", "FS_WRITE|"))
        ) + "\n",
        encoding="utf-8")
    (iter_dir / "verdict_s228.json").write_text(
        json.dumps(results, indent=2) + "\n", encoding="utf-8")
    (iter_dir / "processes_after_renode.log").write_text(
        run_capture(["ps", "-eo", "pid,ppid,stat,cmd"], ROOT),
        encoding="utf-8")

    print(f"S228 iter: {iter_dir.relative_to(ROOT)}")
    print(f"pass={results['pass']} checks={len(hw.get('checks', {}))} "
          f"fails={hw.get('fails')}")
    return 0 if results["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
