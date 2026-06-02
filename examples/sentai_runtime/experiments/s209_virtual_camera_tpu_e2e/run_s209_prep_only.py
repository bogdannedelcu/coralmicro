#!/usr/bin/env python3
"""s209 diagnostic: virtual camera publishes frames consumed by prep slots."""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import shutil
import subprocess
from pathlib import Path

from PIL import Image

from run_s209 import REPO, EXP, SIM, SRC_IMG, cleanup_stale_runtime


SRC_MISSION = EXP / "mission_s209_prep_only.py"


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
    shutil.copy2(SRC_MISSION, fs_root / SRC_MISSION.name)
    img = Image.open(SRC_IMG).convert("RGB")
    for i in range(frames):
        write_shifted(img, frame_dir / f"frame_{i:03d}.bmp", i * step_px, 0)
    return fs_root


def run_sim(run_dir: Path, fs_root: Path, duration_ms: int,
            play_count: int, use_replay: bool, use_flow: bool) -> dict:
    stdout_path = run_dir / "sim_stdout.log"
    program = "\n".join((
        "import mission_s209_prep_only as m",
        f"m.run({int(duration_ms)}, {int(play_count)}, "
        f"{bool(use_replay)!r}, {bool(use_flow)!r})",
        "exit",
        "",
    ))
    try:
        proc = subprocess.run(
            [str(SIM)],
            cwd=str(REPO),
            input=program,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env={**os.environ, "SENTAI_SIM_ROOT": str(fs_root)},
            timeout=max(30, duration_ms // 1000 + 20),
        )
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", "replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", "replace")
        stdout_path.write_text(
            stdout + stderr,
            encoding="utf-8")
        cleanup_stale_runtime()
        raise

    stdout_path.write_text(proc.stdout, encoding="utf-8")
    out = proc.stdout
    debug_log = fs_root / "fr" / "debug.log"
    if debug_log.exists():
        out += "\n" + debug_log.read_text(encoding="utf-8", errors="replace")
    (run_dir / "sim_output.txt").write_text(out, encoding="utf-8")
    if proc.returncode != 0:
        raise SystemExit(f"sentai_sim failed rc={proc.returncode}; see {run_dir / 'sim_output.txt'}")
    result_path = fs_root / "prep_only_results.txt"
    if not result_path.exists():
        raise SystemExit(f"missing result; see {run_dir / 'sim_output.txt'}")
    return ast.literal_eval(result_path.read_text(encoding="utf-8"))


def validate(result: dict, min_delta: int) -> None:
    if int(result.get("delta_frames", 0)) < min_delta:
        raise SystemExit(f"prep frames did not advance enough: {result}")
    if int(result.get("flow_slot_seq_delta", 0)) < min_delta:
        raise SystemExit(f"flow prep slot did not advance enough: {result}")
    if int(result.get("delta_aux", 0)) < min_delta:
        raise SystemExit(f"aux prep frames did not advance enough: {result}")
    if int(result.get("prep_stop_rc", 0)) != 0:
        raise SystemExit(f"prep task did not stop cleanly: {result}")
    if result.get("use_flow") and int(result.get("flow_pub_stats", {}).get("frames", 0)) < min_delta:
        raise SystemExit(f"flow did not publish enough frames: {result}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration-ms", type=int, default=3000)
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--step-px", type=int, default=2)
    ap.add_argument("--min-delta", type=int, default=5)
    ap.add_argument("--fs-play", action="store_true",
                    help="Use camera.play(dir) so every frame is read from FS.")
    ap.add_argument("--no-flow", action="store_true",
                    help="Do not start FlowTask; validate PrepTask slots only.")
    ap.add_argument("--label", default="prep_only")
    ap.add_argument("--no-validate", action="store_true")
    args = ap.parse_args()

    missing = [p for p in (SIM, SRC_IMG, SRC_MISSION) if not p.exists()]
    if missing:
        raise SystemExit("missing required inputs:\n" + "\n".join(str(p) for p in missing))

    cleanup_stale_runtime()
    run_dir = next_iter_dir(args.label)
    run_dir.mkdir(parents=True)
    fs_root = prepare_fs(run_dir, args.step_px, args.frames)
    result = run_sim(run_dir, fs_root, args.duration_ms, args.frames,
                     use_replay=not args.fs_play,
                     use_flow=not args.no_flow)
    if not args.no_validate:
        validate(result, args.min_delta)
    (run_dir / "summary.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(f"OK s209-prep-only {run_dir}")
    print(result)


if __name__ == "__main__":
    main()
