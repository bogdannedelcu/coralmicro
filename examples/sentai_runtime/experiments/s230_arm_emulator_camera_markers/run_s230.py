#!/usr/bin/env python3
"""Run S230: sentai.camera + sentai.markers in ARM emulation."""

from __future__ import annotations

import json
import os
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
TARGETS = [
    "sentai_emu_camera_markers_stage_assets",
    "sentai_emu_camera_markers_repl",
]
STAGE_SCRIPT = ROOT / "emu/renode/sentai_emu_camera_markers_stage_assets.resc"
RENODE_SCRIPT = ROOT / "emu/renode/sentai_emu_camera_markers.resc"
UART_STAGE_LOG = ROOT / "emu/output/sentai_emu_camera_markers_stage_assets.log"
UART_LOG = ROOT / "emu/output/sentai_emu_camera_markers.log"
OUTPUT_LOG_PATTERNS = [
    "sentai_emu_camera_markers.log*",
    "sentai_emu_camera_markers_stage_assets.log*",
]


def next_iter_dir() -> pathlib.Path:
    existing = sorted(EXP.glob("iter[0-9]*_*"))
    next_id = 1
    if existing:
        next_id = 1 + max(
            int(path.name.split("_", 1)[0].replace("iter", ""))
            for path in existing
        )
    out = EXP / f"iter{next_id:02d}_camera_markers"
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


def run_capture(cmd: list[str], cwd: pathlib.Path,
                timeout_s: int = 30) -> str:
    proc = subprocess.run(
        cmd, cwd=str(cwd), text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=timeout_s, check=False)
    return proc.stdout


def parse_hex(label: str, text: str) -> int | None:
    match = re.search(rf"{re.escape(label)}:\s*\r?\n\s*(0x[0-9A-Fa-f]+)",
                      text)
    return int(match.group(1), 16) if match else None


def parse_uart(text: str) -> dict[str, object]:
    out: dict[str, object] = {
        "done": "CAM_MARK_DONE" in text,
    }
    patterns = {
        "pgm_n": r"CAM_MARK_PGM_N\s+(-?\d+)",
        "cam_n": r"CAM_MARK_CAM_N\s+(-?\d+)",
        "loop": r"CAM_MARK_LOOP\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, text)
        if not match:
            continue
        if key == "loop":
            out["loop_frames"] = int(match.group(1))
            out["loop_ok"] = int(match.group(2))
            out["loop_dt_ms"] = int(match.group(3))
            out["loop_fps_x100"] = int(match.group(4))
        else:
            out[key] = int(match.group(1))
    out["camera_detection_line"] = next(
        (line.strip() for line in text.splitlines()
         if line.startswith("CAM_MARK_CAM_DET0")),
        "")
    out["observation_line"] = next(
        (line.strip() for line in text.splitlines()
         if line.startswith("CAM_MARK_OBS")),
        "")
    return out


def maybe_copy(src: pathlib.Path, dst: pathlib.Path) -> bool:
    if not src.exists():
        return False
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def cleanup_output_logs() -> None:
    output_dir = ROOT / "emu/output"
    for pattern in OUTPUT_LOG_PATTERNS:
        for path in output_dir.glob(pattern):
            if path.is_file():
                path.unlink()


def main() -> int:
    iter_dir = next_iter_dir()
    (iter_dir / "renode").mkdir()
    (iter_dir / "assets").mkdir()

    results: dict[str, object] = {
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "targets": TARGETS,
        "renode_script": str(RENODE_SCRIPT.relative_to(ROOT)),
        "stage_script": str(STAGE_SCRIPT.relative_to(ROOT)),
    }

    commands = {
        "prepare_assets": [
            sys.executable,
            str((EXP / "prepare_s230_assets.py").relative_to(ROOT)),
        ],
        "configure": [
            "cmake", "-S", ".", "-B", str(BUILD_EMU.relative_to(ROOT)),
            "-DSENTAI_ARM_EMU=ON", "-DSENTAI_SKIP_SDK_PATCHES=ON",
        ],
        "build": [
            "cmake", "--build", str(BUILD_EMU.relative_to(ROOT)),
            "--target", *TARGETS, f"-j{os.cpu_count() or 1}",
        ],
        "reset_nand": [
            sys.executable, "emu/scripts/init_emu_nand.py", "--reset",
        ],
        "renode_stage": [
            str(RENODE), "--plain", "--console", "--disable-xwt",
            str(STAGE_SCRIPT.relative_to(ROOT)),
        ],
        "renode": [
            str(RENODE), "--plain", "--console", "--disable-xwt",
            str(RENODE_SCRIPT.relative_to(ROOT)),
        ],
    }
    results["commands"] = commands

    for name in ("prepare_assets", "configure", "build", "reset_nand"):
        proc = run_to_file(commands[name], ROOT, iter_dir / f"{name}.log", 300)
        results[f"{name}_rc"] = proc.returncode
        if proc.returncode != 0:
            (iter_dir / "verdict_s230.json").write_text(
                json.dumps(results, indent=2) + "\n", encoding="utf-8")
            return proc.returncode

    for src in (EXP / "assets").glob("*"):
        if src.is_file():
            shutil.copy2(src, iter_dir / "assets" / src.name)

    cleanup_output_logs()

    (iter_dir / "processes_before_renode.log").write_text(
        run_capture(["ps", "-eo", "pid,ppid,stat,cmd"], ROOT),
        encoding="utf-8")

    stage_proc = run_to_file(commands["renode_stage"], ROOT,
                             iter_dir / "renode_stage.log", 60)
    results["renode_stage_rc"] = stage_proc.returncode
    maybe_copy(UART_STAGE_LOG, iter_dir / "uart_stage.log")

    renode_proc = run_to_file(commands["renode"], ROOT,
                              iter_dir / "renode.log", 90)
    results["renode_rc"] = renode_proc.returncode
    uart_text = ""
    if maybe_copy(UART_LOG, iter_dir / "uart.log"):
        uart_text = (iter_dir / "uart.log").read_text(
            encoding="utf-8", errors="replace")

    shutil.copy2(STAGE_SCRIPT, iter_dir / "renode" / STAGE_SCRIPT.name)
    shutil.copy2(RENODE_SCRIPT, iter_dir / "renode" / RENODE_SCRIPT.name)
    shutil.copy2(ROOT / "emu/renode/sentai_rt1176.repl",
                 iter_dir / "renode/sentai_rt1176.repl")

    symbols = {
        "stage_boot_state": parse_hex(
            "sentai_emu_camera_markers_stage_assets boot_state",
            stage_proc.stdout),
        "stage_files": parse_hex(
            "sentai_emu_camera_markers_stage_assets stage_files",
            stage_proc.stdout),
        "stage_errors": parse_hex(
            "sentai_emu_camera_markers_stage_assets stage_errors",
            stage_proc.stdout),
        "boot_state": parse_hex("sentai_emu_camera_markers boot_state",
                                renode_proc.stdout),
        "heartbeat": parse_hex("sentai_emu_camera_markers heartbeat",
                               renode_proc.stdout),
        "repl_lines": parse_hex("sentai_emu_camera_markers repl_lines",
                                renode_proc.stdout),
    }
    camera_markers = parse_uart(uart_text)
    results["symbols"] = symbols
    results["camera_markers"] = camera_markers
    results["pass"] = (
        stage_proc.returncode == 0
        and renode_proc.returncode == 0
        and symbols.get("stage_boot_state") == 0x0700
        and symbols.get("stage_files") == 2
        and symbols.get("stage_errors") == 0
        and camera_markers.get("done") is True
        and camera_markers.get("pgm_n", 0) >= 1
        and camera_markers.get("cam_n", 0) >= 1
        and camera_markers.get("loop_frames") == 10
        and camera_markers.get("loop_ok") == 10
    )

    summary = [
        f"pass={results['pass']}",
        f"pgm_n={camera_markers.get('pgm_n')}",
        f"cam_n={camera_markers.get('cam_n')}",
        f"loop_ok={camera_markers.get('loop_ok')}/"
        f"{camera_markers.get('loop_frames')}",
        f"loop_dt_ms={camera_markers.get('loop_dt_ms')}",
        f"loop_fps_x100={camera_markers.get('loop_fps_x100')}",
        camera_markers.get("camera_detection_line", ""),
        camera_markers.get("observation_line", ""),
    ]
    (iter_dir / "summary_s230.txt").write_text(
        "\n".join(str(x) for x in summary if x) + "\n",
        encoding="utf-8")
    (iter_dir / "verdict_s230.json").write_text(
        json.dumps(results, indent=2) + "\n", encoding="utf-8")
    (iter_dir / "processes_after_renode.log").write_text(
        run_capture(["ps", "-eo", "pid,ppid,stat,cmd"], ROOT),
        encoding="utf-8")
    cleanup_output_logs()

    print(f"S230 iter: {iter_dir.relative_to(ROOT)}")
    print((iter_dir / "summary_s230.txt").read_text(encoding="utf-8").strip())
    return 0 if results["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
