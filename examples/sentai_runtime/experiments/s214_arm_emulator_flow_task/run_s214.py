#!/usr/bin/env python3
"""Run S214: production FlowTask-only benchmark in ARM Renode."""

from __future__ import annotations

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
BUILD_DIR = ROOT / "build_emu"
TARGET = "sentai_emu_flow_task_runtime"
RENODE_SCRIPT = ROOT / "emu/renode/sentai_emu_flow_task_runtime.resc"
EMU_UART_LOG = ROOT / "emu/output/sentai_emu_flow_task_runtime.log"


def next_iter_dir() -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9]*_*"))
    next_id = 1
    if existing:
        next_id = 1 + max(
            int(path.name.split("_", 1)[0].replace("iter", ""))
            for path in existing
        )
    out = EXP / f"iter{next_id:02d}_renode_flow_task_runtime"
    out.mkdir(parents=True)
    return out


def run_to_file(cmd: list[str], cwd: pathlib.Path, log_path: pathlib.Path,
                timeout_s: int | None = None) -> subprocess.CompletedProcess[str]:
    with log_path.open("w+", encoding="utf-8") as log:
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
        if timed_out:
            log.write(f"\n[TIMEOUT] exceeded {timeout_s}s\n")
        log.seek(0)
        out = log.read()
    return subprocess.CompletedProcess(cmd, rc, out)


def parse_hex(label: str, text: str) -> int | None:
    match = re.search(rf"{re.escape(label)}:\s*\r?\n\s*(0x[0-9A-Fa-f]+)",
                      text)
    if not match:
        return None
    return int(match.group(1), 16)


def as_i32(value: int | None) -> int | None:
    if value is None:
        return None
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def parse_uart(text: str) -> dict:
    verdict: dict[str, object] = {
        "validate_pass": "FLOW_VALIDATE_PASS" in text,
        "done": "FLOW_DONE" in text,
    }
    fps_re = re.search(
        r"FLOW_FPS frames=(?P<frames>\d+) elapsed_ms=(?P<elapsed>\d+) "
        r"fps_x100=(?P<fps>\d+) last_dx_q1000=(?P<dx>-?\d+) "
        r"last_dy_q1000=(?P<dy>-?\d+) conf=(?P<conf>\d+)",
        text,
    )
    if fps_re:
        verdict.update({
            "frames": int(fps_re.group("frames")),
            "elapsed_ms": int(fps_re.group("elapsed")),
            "fps_x100": int(fps_re.group("fps")),
            "fps": int(fps_re.group("fps")) / 100.0,
            "last_dx_q1000": int(fps_re.group("dx")),
            "last_dy_q1000": int(fps_re.group("dy")),
            "last_conf": int(fps_re.group("conf")),
        })
    validations = []
    val_re = re.compile(
        r"FLOW_VALIDATE frame=(?P<frame>\d+) exp_dx=(?P<exp_dx>-?\d+) "
        r"exp_dy=(?P<exp_dy>-?\d+) dx_q1000=(?P<dx>-?\d+) "
        r"dy_q1000=(?P<dy>-?\d+) conf=(?P<conf>\d+)"
        r"(?: match=(?P<match>[01]))?"
    )
    for m in val_re.finditer(text):
        validations.append({
            "frame": int(m.group("frame")),
            "expected_dx_px": int(m.group("exp_dx")),
            "expected_dy_px": int(m.group("exp_dy")),
            "dx_q1000": int(m.group("dx")),
            "dy_q1000": int(m.group("dy")),
            "confidence": int(m.group("conf")),
            "match": (
                None if m.group("match") is None else bool(int(m.group("match")))
            ),
        })
    verdict["validations"] = validations
    return verdict


def main() -> int:
    iter_dir = next_iter_dir()
    (iter_dir / "assets").mkdir()
    (iter_dir / "renode").mkdir()

    started_at = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    prep_cmd = [
        sys.executable,
        str((EXP / "prepare_cat80.py").relative_to(ROOT)),
    ]
    configure_cmd = [
        "cmake",
        "-S", ".",
        "-B", str(BUILD_DIR.relative_to(ROOT)),
        "-DSENTAI_ARM_EMU=ON",
        "-DSENTAI_SKIP_SDK_PATCHES=ON",
    ]
    build_cmd = [
        "cmake",
        "--build", str(BUILD_DIR.relative_to(ROOT)),
        "--target", TARGET,
        f"-j{subprocess.os.cpu_count() or 1}",
    ]
    renode_cmd = [
        str(RENODE),
        "--plain",
        "--console",
        "--disable-xwt",
        str(RENODE_SCRIPT.relative_to(ROOT)),
    ]

    results: dict[str, object] = {
        "started_at": started_at,
        "target": TARGET,
        "commands": {
            "prepare": prep_cmd,
            "configure": configure_cmd,
            "build": build_cmd,
            "renode": renode_cmd,
        },
    }

    for name, cmd, timeout in (
        ("prepare", prep_cmd, 30),
        ("configure", configure_cmd, 120),
        ("build", build_cmd, 240),
    ):
        proc = run_to_file(cmd, ROOT, iter_dir / f"{name}.log", timeout)
        results[f"{name}_rc"] = proc.returncode
        if proc.returncode != 0:
            (iter_dir / "verdict_s214.json").write_text(
                json.dumps(results, indent=2) + "\n")
            print(proc.stdout)
            return proc.returncode

    assets_src = EXP / "assets"
    for name in ("cat80_base.bin", "cat80_base.png"):
        shutil.copy2(assets_src / name, iter_dir / "assets" / name)

    EMU_UART_LOG.parent.mkdir(parents=True, exist_ok=True)
    if EMU_UART_LOG.exists():
        EMU_UART_LOG.unlink()

    renode_proc = run_to_file(renode_cmd, ROOT, iter_dir / "renode.log", 300)
    results["renode_rc"] = renode_proc.returncode

    if EMU_UART_LOG.exists():
        shutil.copy2(EMU_UART_LOG, iter_dir / "uart.log")
        uart_text = (iter_dir / "uart.log").read_text(
            encoding="utf-8", errors="replace")
    else:
        uart_text = ""
        results["uart_log_missing"] = True

    shutil.copy2(RENODE_SCRIPT, iter_dir / "renode" / RENODE_SCRIPT.name)
    shutil.copy2(ROOT / "emu/renode/sentai_rt1176.repl",
                 iter_dir / "renode/sentai_rt1176.repl")

    labels = {
        "boot_state": "sentai_emu_flow_task_runtime boot_state",
        "heartbeat": "sentai_emu_flow_task_runtime heartbeat",
        "last_tick": "sentai_emu_flow_task_runtime last_tick",
        "flow_completed": "sentai_emu_flow_task_runtime flow_completed",
        "flow_detect_ok": "sentai_emu_flow_task_runtime flow_detect_ok",
        "flow_fail_code": "sentai_emu_flow_task_runtime flow_fail_code",
        "flow_elapsed_ms": "sentai_emu_flow_task_runtime flow_elapsed_ms",
        "flow_fps_x100": "sentai_emu_flow_task_runtime flow_fps_x100",
        "flow_last_dx_q1000": "sentai_emu_flow_task_runtime flow_last_dx_q1000",
        "flow_last_dy_q1000": "sentai_emu_flow_task_runtime flow_last_dy_q1000",
        "flow_last_conf": "sentai_emu_flow_task_runtime flow_last_conf",
    }
    symbols: dict[str, int | None] = {
        key: parse_hex(label, renode_proc.stdout)
        for key, label in labels.items()
    }
    symbols["flow_last_dx_q1000_signed"] = as_i32(symbols["flow_last_dx_q1000"])
    symbols["flow_last_dy_q1000_signed"] = as_i32(symbols["flow_last_dy_q1000"])
    results["symbols"] = symbols
    results["uart"] = parse_uart(uart_text)
    results["pass"] = (
        renode_proc.returncode == 0
        and symbols.get("flow_completed") == 24
        and symbols.get("flow_detect_ok") == 1
        and symbols.get("flow_fail_code") == 0
        and results["uart"].get("validate_pass") is True
        and results["uart"].get("done") is True
    )

    (iter_dir / "verdict_s214.json").write_text(
        json.dumps(results, indent=2) + "\n")

    fps = results["uart"].get("fps")
    print(f"S214 iter: {iter_dir.relative_to(ROOT)}")
    print(f"pass={results['pass']} fps={fps} "
          f"detect_ok={symbols.get('flow_detect_ok')} "
          f"fail_code={symbols.get('flow_fail_code')}")
    return 0 if results["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
