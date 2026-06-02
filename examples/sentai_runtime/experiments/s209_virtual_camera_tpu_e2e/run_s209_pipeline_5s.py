#!/usr/bin/env python3
"""s209 diagnostic: start virtual camera + pipeline, sleep 5s, stop."""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import shutil
import subprocess
import time
from pathlib import Path

from PIL import Image

from run_s209 import REPO, EXP, SIM, SRC_IMG, SRC_MODEL, cleanup_stale_runtime


SRC_MISSION = EXP / "mission_s209_pipeline_5s.py"


def next_iter_dir(label: str) -> Path:
    existing = []
    for p in EXP.iterdir():
        if p.is_dir():
            m = re.match(r"iter(\d+)_", p.name)
            if m:
                existing.append(int(m.group(1)))
    n = (max(existing) + 1) if existing else 1
    return EXP / f"iter{n:02d}_{label}"


def write_shifted(src: Image.Image, dst: Path, dx: int, dy: int) -> None:
    out = Image.new("RGB", (640, 480), (0, 0, 0))
    resampling = getattr(Image, "Resampling", Image).LANCZOS
    sprite = src.resize((300, 225), resampling)
    out.paste(sprite, (30 + dx, 120 + dy))
    out.save(dst)


def prepare_fs(run_dir: Path, step_px: int, frames: int) -> Path:
    fs_root = run_dir / "fs_root"
    frame_dir = fs_root / "images" / "shift"
    frame_dir.mkdir(parents=True, exist_ok=True)
    (fs_root / "models").mkdir(parents=True, exist_ok=True)
    shutil.copy2(SRC_MODEL, fs_root / "models" / SRC_MODEL.name)
    shutil.copy2(SRC_MISSION, fs_root / SRC_MISSION.name)
    img = Image.open(SRC_IMG).convert("RGB")
    for i in range(frames):
        write_shifted(img, frame_dir / f"frame_{i:03d}.bmp", i * step_px, 0)
    return fs_root


def cleanup_extra() -> None:
    cleanup_stale_runtime()
    proc = subprocess.run(
        ["pgrep", "-f", "sentai_tpu_pycoral_server.py"],
        text=True, capture_output=True, check=False)
    pids = [int(x) for x in proc.stdout.split() if x.isdigit()]
    for pid in pids:
        if pid not in (os.getpid(), os.getppid()):
            try:
                os.kill(pid, 15)
            except OSError:
                pass
    time.sleep(0.2)


def text_or_empty(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)


def run_sim(run_dir: Path, fs_root: Path, duration_ms: int,
            play_count: int, use_replay: bool, use_flow: bool,
            direct_tensor: int) -> dict:
    stdout_path = run_dir / "sim_stdout.log"
    host_log_path = run_dir / "host_driver.log"
    t0 = time.monotonic()

    def host_log(msg: str) -> None:
        with host_log_path.open("a", encoding="utf-8") as f:
            f.write(f"{time.monotonic() - t0:8.3f} {msg}\n")

    program = "\n".join((
        "import mission_s209_pipeline_5s as m",
        f"m.run({int(duration_ms)}, {int(play_count)}, "
        f"{bool(use_replay)!r}, {bool(use_flow)!r}, {int(direct_tensor)})",
        "exit",
        "",
    ))
    host_log("SEND import mission_s209_pipeline_5s as m")
    host_log(f"SEND m.run({int(duration_ms)}, {int(play_count)}, "
             f"{bool(use_replay)!r}, {bool(use_flow)!r}, {int(direct_tensor)})")
    host_log("SEND exit")
    host_log("WAIT")
    try:
        proc = subprocess.run(
            [str(SIM)],
            cwd=str(REPO),
            input=program,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env={**os.environ, "SENTAI_SIM_ROOT": str(fs_root)},
            timeout=max(180, duration_ms // 1000 + 60),
        )
        stdout_path.write_text(proc.stdout, encoding="utf-8")
        out = proc.stdout
    except subprocess.TimeoutExpired as exc:
        out = text_or_empty(exc.stdout) + text_or_empty(exc.stderr)
        stdout_path.write_text(out, encoding="utf-8")
        cleanup_extra()
        raise

    debug_log = fs_root / "fr" / "debug.log"
    if debug_log.exists():
        out += "\n" + debug_log.read_text(encoding="utf-8", errors="replace")
    (run_dir / "sim_output.txt").write_text(out, encoding="utf-8")
    if proc.returncode != 0:
        raise SystemExit(f"sentai_sim failed rc={proc.returncode}; see {run_dir / 'sim_output.txt'}")
    result_path = fs_root / "pipeline_5s_results.txt"
    if not result_path.exists():
        raise SystemExit(f"missing result; see {run_dir / 'sim_output.txt'}")
    return ast.literal_eval(result_path.read_text(encoding="utf-8"))


def validate(result: dict, min_delta: int) -> None:
    if int(result.get("delta_count", 0)) < min_delta:
        raise SystemExit(f"pipeline did not advance enough: {result}")
    if int(result.get("latest_seq", -1)) <= 0:
        raise SystemExit(f"missing latest detection: {result}")
    if not result.get("latest_dets"):
        raise SystemExit(f"no detections: {result}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration-ms", type=int, default=5000)
    ap.add_argument("--frames", type=int, default=80)
    ap.add_argument("--step-px", type=int, default=2)
    ap.add_argument("--min-delta", type=int, default=2)
    ap.add_argument("--replay", action="store_true",
                    help="Load one virtual frame once, then republish it from RAM.")
    ap.add_argument("--no-flow", action="store_true",
                    help="Do not start sentai.flow; isolate virtual camera + TPU pipeline.")
    ap.add_argument("--direct-tensor", type=int, choices=(0, 1), default=1,
                    help="Use direct PrepTask->TPU tensor buffers (1) or legacy copy path (0).")
    ap.add_argument("--label", default="pipeline_5s")
    ap.add_argument("--no-validate", action="store_true")
    args = ap.parse_args()

    missing = [p for p in (SIM, SRC_IMG, SRC_MODEL, SRC_MISSION) if not p.exists()]
    if missing:
        raise SystemExit("missing required inputs:\n" + "\n".join(str(p) for p in missing))

    cleanup_extra()
    run_dir = next_iter_dir(args.label)
    run_dir.mkdir(parents=True)
    fs_root = prepare_fs(run_dir, args.step_px, args.frames)
    result = run_sim(run_dir, fs_root, args.duration_ms, args.frames,
                     use_replay=args.replay, use_flow=not args.no_flow,
                     direct_tensor=args.direct_tensor)
    if not args.no_validate:
        validate(result, args.min_delta)
    (run_dir / "summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(f"OK s209-pipeline-5s {run_dir}")
    print(result)


if __name__ == "__main__":
    main()
