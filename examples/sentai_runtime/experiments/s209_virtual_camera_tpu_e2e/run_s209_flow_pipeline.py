#!/usr/bin/env python3
"""s209 combined smoke: virtual-camera flow plus continuous TPU pipeline."""

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

from run_s209 import (REPO, EXP, SIM, SRC_IMG, SRC_MODEL,
                      cleanup_stale_runtime, start_helper, stop_helper,
                      text_or_empty)


SRC_MISSION = EXP / "mission_s209_flow_pipeline.py"
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
    out = Image.new("RGB", src.size, (0, 0, 0))
    out.paste(src, (dx, dy))
    out.save(dst)


def prepare_fs(run_dir: Path, step_px: int) -> Path:
    fs_root = run_dir / "fs_root"
    (fs_root / "images").mkdir(parents=True, exist_ok=True)
    (fs_root / "models").mkdir(parents=True, exist_ok=True)
    shutil.copy2(SRC_MODEL, fs_root / "models" / SRC_MODEL.name)
    shutil.copy2(SRC_MISSION, fs_root / SRC_MISSION.name)
    img = Image.open(SRC_IMG).convert("RGB")
    for i in range(N_FRAMES):
        write_shifted(img, fs_root / "images" / f"cat_shift_{i:02d}.bmp",
                      i * step_px, 0)
    return fs_root


def run_sim(run_dir: Path, fs_root: Path) -> tuple[str, dict]:
    program = """
import mission_s209_flow_pipeline
mission_s209_flow_pipeline.run()
""".lstrip()
    try:
        proc = subprocess.run(
            [str(SIM)],
            cwd=str(REPO),
            input=program,
            text=True,
            capture_output=True,
            timeout=60,
            env={**os.environ, "SENTAI_SIM_ROOT": str(fs_root)},
        )
    except subprocess.TimeoutExpired as exc:
        out = text_or_empty(exc.stdout) + text_or_empty(exc.stderr)
        debug_log = fs_root / "fr" / "debug.log"
        if debug_log.exists():
            out += "\n" + debug_log.read_text(encoding="utf-8", errors="replace")
        (run_dir / "sim_output.txt").write_text(out, encoding="utf-8")
        cleanup_stale_runtime()
        raise
    out = proc.stdout + proc.stderr
    (run_dir / "sim_output.txt").write_text(out, encoding="utf-8")
    if proc.returncode != 0:
        raise SystemExit(f"sentai_sim failed rc={proc.returncode}; see {run_dir / 'sim_output.txt'}")
    result_path = fs_root / "flow_pipeline_results.txt"
    if not result_path.exists():
        raise SystemExit(f"missing combined result file; see {run_dir / 'sim_output.txt'}")
    result = ast.literal_eval(result_path.read_text(encoding="utf-8"))
    return out, result


def best_cat_x(frame):
    if frame is None:
        return None
    dets = frame[0]
    cats = [d for d in dets if int(d[5]) in (16, 17)]
    if not cats:
        return None
    best = max(cats, key=lambda d: int(d[4]))
    return int(best[0])


def validate(result: dict) -> None:
    records = result["records"]
    if len(records) != N_FRAMES:
        raise SystemExit(f"expected {N_FRAMES} records, got {len(records)}")
    flow_nonzero = 0
    for i, rc, flow, _frame in records[1:]:
        if int(rc) != 0:
            raise SystemExit(f"camera select failed at {i}: {records}")
        if abs(int(flow[2])) > 20 or abs(int(flow[3])) > 20:
            flow_nonzero += 1
    if flow_nonzero < N_FRAMES - 3:
        raise SystemExit(f"flow did not detect enough offset steps: {records}")

    xs = [best_cat_x(frame) for _i, _rc, _flow, frame in records]
    xs = [x for x in xs if x is not None]
    if len(xs) < 2:
        raise SystemExit(f"not enough cat detections while pipeline+flow ran: {records}")
    if max(xs) <= min(xs):
        raise SystemExit(f"cat detections did not move in x: {xs}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="virtual_camera_flow_pipeline_shift")
    ap.add_argument("--step-px", type=int, default=2)
    ap.add_argument("--no-validate", action="store_true")
    args = ap.parse_args()

    ensure_inputs()
    cleanup_stale_runtime()
    run_dir = next_iter_dir(args.label)
    run_dir.mkdir(parents=True)
    fs_root = prepare_fs(run_dir, args.step_px)
    helper = start_helper(run_dir)
    try:
        _out, result = run_sim(run_dir, fs_root)
    finally:
        stop_helper(helper)
    if not args.no_validate:
        validate(result)
    (run_dir / "summary.json").write_text(
        json.dumps({"step_px": args.step_px, "result": result},
                   indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(f"OK s209-flow-pipeline {run_dir}")
    print(result)


if __name__ == "__main__":
    main()
