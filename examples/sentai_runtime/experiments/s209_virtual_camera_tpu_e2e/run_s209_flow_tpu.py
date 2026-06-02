#!/usr/bin/env python3
"""s209 combined smoke: virtual-camera flow active while TPU detects shifts."""

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

from run_s209 import (REPO, EXP, SIM, SRC_IMG, SRC_MODEL,
                      cleanup_stale_runtime, text_or_empty)


SRC_MISSION = EXP / "mission_s209_flow_tpu.py"
N_FRAMES = 16


def next_iter_dir(label: str) -> Path:
    existing = []
    for p in EXP.iterdir():
        if p.is_dir():
            m = re.match(r"iter(\d+)_", p.name)
            if m:
                existing.append(int(m.group(1)))
    n = (max(existing) + 1) if existing else 1
    return EXP / f"iter{n:02d}_{label}"


def ensure_inputs() -> None:
    missing = [p for p in (SIM, SRC_IMG, SRC_MODEL, SRC_MISSION) if not p.exists()]
    if missing:
        raise SystemExit("missing required inputs:\n" + "\n".join(str(p) for p in missing))


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
        write_shifted(img, frame_dir / f"frame_{i:03d}.bmp",
                      i * step_px, 0)
    return fs_root


def run_sim(run_dir: Path, fs_root: Path, frames: int) -> tuple[str, dict]:
    stdout_path = run_dir / "sim_stdout.log"
    host_log_path = run_dir / "host_driver.log"
    t0 = time.monotonic()
    def host_log(msg: str) -> None:
        with host_log_path.open("a", encoding="utf-8") as f:
            f.write(f"{time.monotonic() - t0:8.3f} {msg}\n")
            f.flush()
    def send(proc: subprocess.Popen[str], line: str) -> None:
        assert proc.stdin is not None
        host_log(f"SEND {line}")
        proc.stdin.write(line + "\n")
        proc.stdin.flush()

    try:
        with stdout_path.open("w", encoding="utf-8") as log:
            proc = subprocess.Popen(
                [str(SIM)],
                cwd=str(REPO),
                stdin=subprocess.PIPE,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                env={**os.environ, "SENTAI_SIM_ROOT": str(fs_root)},
            )
            send(proc, "import mission_s209_flow_tpu as m")
            send(proc, f"m.start({frames})")
            wait_s = max(4.0, frames / 10.0 + 3.0)
            host_log(f"HOST_SLEEP {wait_s:.3f}")
            time.sleep(wait_s)
            send(proc, "m.collect()")
            send(proc, "exit")
            host_log("WAIT")
            proc.wait(timeout=180)
    except subprocess.TimeoutExpired as exc:
        cleanup_stale_runtime()
        out = stdout_path.read_text(encoding="utf-8", errors="replace") if stdout_path.exists() else ""
        debug_log = fs_root / "fr" / "debug.log"
        if debug_log.exists():
            out += "\n" + debug_log.read_text(encoding="utf-8", errors="replace")
        (run_dir / "sim_output.txt").write_text(out, encoding="utf-8")
        raise
    out = stdout_path.read_text(encoding="utf-8", errors="replace")
    debug_log = fs_root / "fr" / "debug.log"
    if debug_log.exists():
        out += "\n" + debug_log.read_text(encoding="utf-8", errors="replace")
    (run_dir / "sim_output.txt").write_text(out, encoding="utf-8")
    if proc.returncode != 0:
        raise SystemExit(f"sentai_sim failed rc={proc.returncode}; see {run_dir / 'sim_output.txt'}")
    result_path = fs_root / "flow_tpu_results.txt"
    if not result_path.exists():
        raise SystemExit(f"missing combined result file; see {run_dir / 'sim_output.txt'}")
    result = ast.literal_eval(result_path.read_text(encoding="utf-8"))
    return out, result


def best_cat_x(dets):
    cats = [d for d in dets if int(d[5]) in (16, 17)]
    if not cats:
        return None
    best = max(cats, key=lambda d: int(d[4]))
    return int(best[0])


def validate(result: dict, frames: int) -> None:
    records = result.get("callback_records") or result["records"]
    if len(records) < frames:
        raise SystemExit(f"expected at least {frames} records, got {len(records)}")
    events = result.get("events", [])
    if len(events) < frames:
        raise SystemExit(f"expected at least {frames} event records, got {len(events)}")
    if int(result.get("final_count", 0)) < int(result.get("initial_count", 0)) + frames:
        raise SystemExit(f"detection event counter did not advance enough: {result}")

    flow_nonzero = 0
    xs = []
    prev_event_count = int(result.get("initial_count", 0))
    for rec in records:
        i, ev, flow, li, inv, pipe_seq, dets = rec
        if int(li) != 0 or int(inv) < 0 or int(pipe_seq) <= 0:
            raise SystemExit(f"bad frame status at {i}: {(li, inv, pipe_seq)}")
        if i > 0 and abs(int(flow[2])) > 20:
            flow_nonzero += 1
        x = best_cat_x(dets)
        if x is not None:
            xs.append(x)
    for ev in events:
        frame_i, after_count, after_seq, det_count, cam_id = ev
        if int(after_count) <= prev_event_count:
            raise SystemExit(f"event counter not monotonic: {events}")
        if int(after_seq) <= 0 or int(det_count) <= 0:
            raise SystemExit(f"bad detection event at {frame_i}: {ev}")
        prev_event_count = int(after_count)

    if flow_nonzero < frames - 3:
        raise SystemExit(f"flow did not detect enough offset steps: {records}")
    if len(xs) < 2:
        raise SystemExit(f"not enough cat detections: {records}")
    if max(xs) <= min(xs):
        raise SystemExit(f"cat detections did not move in x: {xs}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="virtual_camera_flow_tpu_shift")
    ap.add_argument("--step-px", type=int, default=2)
    ap.add_argument("--frames", type=int, default=N_FRAMES)
    ap.add_argument("--no-validate", action="store_true")
    args = ap.parse_args()

    ensure_inputs()
    cleanup_stale_runtime()
    run_dir = next_iter_dir(args.label)
    run_dir.mkdir(parents=True)
    fs_root = prepare_fs(run_dir, args.step_px, args.frames)
    _out, result = run_sim(run_dir, fs_root, args.frames)
    if not args.no_validate:
        validate(result, args.frames)
    (run_dir / "summary.json").write_text(
        json.dumps({"step_px": args.step_px, "result": result},
                   indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(f"OK s209-flow-tpu {run_dir}")
    print(result)


if __name__ == "__main__":
    main()
