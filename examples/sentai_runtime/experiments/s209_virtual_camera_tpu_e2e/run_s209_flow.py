#!/usr/bin/env python3
"""s209 flow smoke: virtual camera static frame and 2px-offset BMP."""

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

from run_s209 import cleanup_stale_runtime, text_or_empty


REPO = Path(__file__).resolve().parents[4]
EXP = REPO / "examples" / "sentai_runtime" / "experiments" / "s209_virtual_camera_tpu_e2e"
SIM = REPO / "build-sim" / "sim" / "sentai_sim"
SRC_IMG = REPO / "test_data" / "cat_640x480.bmp"
SRC_MISSION = EXP / "mission_s209_flow.py"


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
    missing = [p for p in (SIM, SRC_IMG, SRC_MISSION) if not p.exists()]
    if missing:
        raise SystemExit("missing required inputs:\n" + "\n".join(str(p) for p in missing))


def write_shifted(src: Path, dst: Path, dx: int, dy: int) -> None:
    img = Image.open(src).convert("RGB")
    out = Image.new("RGB", img.size, (0, 0, 0))
    out.paste(img, (dx, dy))
    out.save(dst)


def prepare_fs(run_dir: Path, dx: int, dy: int) -> Path:
    fs_root = run_dir / "fs_root"
    (fs_root / "images").mkdir(parents=True, exist_ok=True)
    shutil.copy2(SRC_IMG, fs_root / "images" / SRC_IMG.name)
    write_shifted(SRC_IMG, fs_root / "images" / "cat_640x480_shift_x2.bmp", dx, dy)
    shutil.copy2(SRC_MISSION, fs_root / SRC_MISSION.name)
    return fs_root


def run_sim(run_dir: Path, fs_root: Path) -> tuple[str, dict]:
    program = """
import mission_s209_flow
mission_s209_flow.run()
""".lstrip()
    try:
        proc = subprocess.run(
            [str(SIM)],
            cwd=str(REPO),
            input=program,
            text=True,
            capture_output=True,
            timeout=30,
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
    result_path = fs_root / "flow_results.txt"
    if not result_path.exists():
        raise SystemExit(f"missing flow result file; see {run_dir / 'sim_output.txt'}")
    result = ast.literal_eval(result_path.read_text(encoding="utf-8"))
    return out, result


def flow_tuple(result: dict, key: str) -> tuple[int, tuple]:
    rc, reading = result[key]
    return int(rc), tuple(reading)


def validate(result: dict) -> None:
    static_rc, static_r = flow_tuple(result, "static_same")
    shift_rc, shift_r = flow_tuple(result, "shifted")
    if static_rc != 0 or shift_rc != 0:
        raise SystemExit(f"camera select failed: {result}")
    static_dx = int(static_r[2])
    static_dy = int(static_r[3])
    shift_dx = int(shift_r[2])
    shift_dy = int(shift_r[3])
    if abs(static_dx) > 1 or abs(static_dy) > 1:
        raise SystemExit(f"static virtual frame reports motion: {result}")
    if abs(shift_dx) <= abs(static_dx) and abs(shift_dy) <= abs(static_dy):
        raise SystemExit(f"2px shifted image did not increase flow signal: {result}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="virtual_camera_flow_static_shift")
    ap.add_argument("--dx", type=int, default=2)
    ap.add_argument("--dy", type=int, default=0)
    ap.add_argument("--no-validate", action="store_true")
    args = ap.parse_args()

    ensure_inputs()
    cleanup_stale_runtime()
    run_dir = next_iter_dir(args.label)
    run_dir.mkdir(parents=True)
    fs_root = prepare_fs(run_dir, args.dx, args.dy)
    _out, result = run_sim(run_dir, fs_root)
    if not args.no_validate:
        validate(result)
    (run_dir / "summary.json").write_text(
        json.dumps({"dx": args.dx, "dy": args.dy, "result": result},
                   indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(f"OK s209-flow {run_dir}")
    print(result)


if __name__ == "__main__":
    main()
