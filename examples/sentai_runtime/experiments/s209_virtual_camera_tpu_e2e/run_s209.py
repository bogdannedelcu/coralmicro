#!/usr/bin/env python3
"""s209: host PyCoral vs SIM virtual-camera TPU E2E parity."""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from PIL import Image, ImageDraw


REPO = Path(__file__).resolve().parents[4]
EXP = REPO / "examples" / "sentai_runtime" / "experiments" / "s209_virtual_camera_tpu_e2e"
SIM = REPO / "build-sim" / "sim" / "sentai_sim"
HELPER = REPO / "sim" / "scripts" / "sim_tpu_helper.py"
HELPER_PY = REPO / "venv-coral" / "bin" / "python3"
SRC_IMG = REPO / "test_data" / "cat_640x480.bmp"
SRC_MODEL = REPO / "models" / "tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
SRC_MISSION = EXP / "mission_s209.py"


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
    missing = [p for p in (SIM, HELPER, HELPER_PY, SRC_IMG, SRC_MODEL, SRC_MISSION)
               if not p.exists()]
    if missing:
        raise SystemExit("missing required inputs:\n" + "\n".join(str(p) for p in missing))


def prepare_fs(run_dir: Path) -> Path:
    fs_root = run_dir / "fs_root"
    (fs_root / "images").mkdir(parents=True, exist_ok=True)
    (fs_root / "models").mkdir(parents=True, exist_ok=True)
    shutil.copy2(SRC_IMG, fs_root / "images" / SRC_IMG.name)
    shutil.copy2(SRC_MODEL, fs_root / "models" / SRC_MODEL.name)
    shutil.copy2(SRC_MISSION, fs_root / SRC_MISSION.name)
    return fs_root


def start_helper(run_dir: Path) -> subprocess.Popen[str]:
    sock = Path("/tmp/sentai_tpu.sock")
    try:
        sock.unlink()
    except FileNotFoundError:
        pass
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    log = open(run_dir / "helper.log", "w", encoding="utf-8")
    proc = subprocess.Popen(
        [str(HELPER_PY), str(HELPER)],
        cwd=str(REPO),
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
        start_new_session=True,
    )
    proc._sentai_log = log  # type: ignore[attr-defined]
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if sock.exists():
            return proc
        if proc.poll() is not None:
            log.flush()
            raise SystemExit(f"TPU helper exited early; see {run_dir / 'helper.log'}")
        time.sleep(0.05)
    raise SystemExit("TPU helper did not create /tmp/sentai_tpu.sock")


def stop_helper(proc: subprocess.Popen[str]) -> None:
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=3)
    log = getattr(proc, "_sentai_log", None)
    if log:
        log.close()


def pycoral_baseline(run_dir: Path) -> list[tuple[int, int, int, int, int, int]]:
    code = r"""
from PIL import Image
from pycoral.utils.edgetpu import make_interpreter
import numpy as np
import sys

model = sys.argv[1]
image = sys.argv[2]
interpreter = make_interpreter(model)
interpreter.allocate_tensors()
inp = interpreter.get_input_details()[0]
h = int(inp['shape'][1])
w = int(inp['shape'][2])
src = np.asarray(Image.open(image).convert('RGB'), dtype=np.uint8)
sh, sw, _ = src.shape
dst = np.empty((h, w, 3), dtype=np.uint8)
for y in range(h):
    sy = (y * sh) // h
    if sy >= sh:
        sy = sh - 1
    for x in range(w):
        sx = (x * sw) // w
        if sx >= sw:
            sx = sw - 1
        dst[y, x] = src[sy, sx]
arr = dst.astype(inp['dtype'], copy=False).reshape(inp['shape'])
interpreter.set_tensor(inp['index'], arr)
interpreter.invoke()
outs = interpreter.get_output_details()
sc = interpreter.get_tensor(outs[0]['index']).reshape(-1)
bx = interpreter.get_tensor(outs[1]['index']).reshape(-1, 4)
nm = interpreter.get_tensor(outs[2]['index']).reshape(-1)
cl = interpreter.get_tensor(outs[3]['index']).reshape(-1)
n = min(int(nm[0]), 20)
dets = []
for i in range(n):
    permil = int(float(sc[i]) * 1000.0 + 0.5)
    if permil < 100:
        continue
    ymin, xmin, ymax, xmax = [float(v) for v in bx[i]]
    dets.append((
        int(xmin * w),
        int(ymin * h),
        int(xmax * w),
        int(ymax * h),
        permil,
        int(cl[i]),
    ))
print(repr(dets))
"""
    proc = subprocess.run(
        [str(HELPER_PY), "-c", code, str(SRC_MODEL), str(SRC_IMG)],
        cwd=str(REPO),
        text=True,
        capture_output=True,
        timeout=25,
    )
    (run_dir / "host_pycoral_stdout.txt").write_text(proc.stdout, encoding="utf-8")
    (run_dir / "host_pycoral_stderr.txt").write_text(proc.stderr, encoding="utf-8")
    if proc.returncode != 0:
        raise SystemExit(f"host PyCoral baseline failed; see {run_dir}")
    lines = [ln.strip() for ln in proc.stdout.splitlines() if ln.strip()]
    dets = list(ast.literal_eval(lines[-1]))
    (run_dir / "host_detections.txt").write_text(repr(dets) + "\n", encoding="utf-8")
    return dets


def run_sim(run_dir: Path, fs_root: Path) -> tuple[str, list[tuple[int, int, int, int, int, int]]]:
    program = """
import mission_s209
mission_s209.run()
""".lstrip()
    proc = subprocess.run(
        [str(SIM)],
        cwd=str(REPO),
        input=program,
        text=True,
        capture_output=True,
        timeout=45,
        env={**os.environ, "SENTAI_SIM_ROOT": str(fs_root)},
    )
    out = proc.stdout + proc.stderr
    (run_dir / "sim_output.txt").write_text(out, encoding="utf-8")
    if proc.returncode != 0:
        raise SystemExit(f"sentai_sim failed rc={proc.returncode}; see {run_dir / 'sim_output.txt'}")
    detections_path = fs_root / "detections.txt"
    if not detections_path.exists():
        raise SystemExit(f"missing detections file; see {run_dir / 'sim_output.txt'}")
    dets = list(ast.literal_eval(detections_path.read_text(encoding="utf-8").strip()))
    (run_dir / "sim_detections.txt").write_text(repr(dets) + "\n", encoding="utf-8")
    return out, dets


def validate(out: str,
             host_dets: list[tuple[int, int, int, int, int, int]],
             sim_dets: list[tuple[int, int, int, int, int, int]]) -> None:
    required = {
        "IMG_SIZE": r"IMG_SIZE\s+921654",
        "MODEL_SIZE": r"MODEL_SIZE\s+7077792",
        "CAM_SELECT": r"CAM_SELECT\s+0",
        "TPU_LOAD": r"TPU_LOAD\s+0",
        "TPU_READY": r"TPU_READY\s+True",
        "PIPE_STEP": r"PIPE_STEP\s+[0-9]+",
        "TPU_OUTPUTS": r"TPU_OUTPUTS\s+4",
    }
    missing = [name for name, pat in required.items() if not re.search(pat, out)]
    if missing:
        raise SystemExit(f"missing expected SIM markers: {missing}")
    if host_dets != sim_dets:
        raise SystemExit(f"detection mismatch\nHOST={host_dets}\nSIM ={sim_dets}")
    if not any(d[5] in (16, 17) for d in sim_dets):
        raise SystemExit(f"cat class missing from detections: {sim_dets}")


def draw_overlay(run_dir: Path,
                 dets: list[tuple[int, int, int, int, int, int]]) -> None:
    def draw_one(draw, img, det, color, label, width):
        x1, y1, x2, y2, _conf, _cls = det
        sx = img.width / 300.0
        sy = img.height / 300.0
        rx1 = int(x1 * sx)
        ry1 = int(y1 * sy)
        rx2 = int(x2 * sx)
        ry2 = int(y2 * sy)
        draw.rectangle((rx1, ry1, rx2, ry2), outline=color, width=width)
        tw = 7 * len(label) + 4
        draw.rectangle((rx1, max(0, ry1 - 14), rx1 + tw, max(12, ry1)),
                       fill=color)
        draw.text((rx1 + 2, max(0, ry1 - 13)), label, fill=(0, 0, 0))

    cat_dets = [d for d in dets if d[5] in (16, 17)]
    best_cat = max(cat_dets, key=lambda d: d[4]) if cat_dets else None

    img = Image.open(SRC_IMG).convert("RGB")
    draw = ImageDraw.Draw(img)
    if best_cat:
        draw_one(draw, img, best_cat, (255, 64, 64),
                 f"cat:{best_cat[4]}", 4)
    img.save(run_dir / "detected_overlay.png")

    img_all = Image.open(SRC_IMG).convert("RGB")
    draw_all = ImageDraw.Draw(img_all)
    for det in dets:
        x1, y1, x2, y2, conf, cls = det
        if conf < 200 and cls not in (16, 17):
            continue
        color = (255, 64, 64) if cls in (16, 17) else (64, 200, 255)
        label = f"{cls}:{conf}"
        draw_one(draw_all, img_all, det, color, label, 2)
    img_all.save(run_dir / "all_detections_overlay.png")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="virtual_camera_tpu_cat")
    args = ap.parse_args()

    ensure_inputs()
    run_dir = next_iter_dir(args.label)
    run_dir.mkdir(parents=True, exist_ok=False)
    fs_root = prepare_fs(run_dir)

    host_dets = pycoral_baseline(run_dir)
    helper = start_helper(run_dir)
    try:
        out, sim_dets = run_sim(run_dir, fs_root)
        validate(out, host_dets, sim_dets)
        draw_overlay(run_dir, sim_dets)
        summary = {
            "ok": True,
            "run_dir": str(run_dir),
            "fs_root": str(fs_root),
            "host_detections": host_dets,
            "sim_detections": sim_dets,
        }
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                               encoding="utf-8")
        print(f"OK s209 {run_dir}")
        return 0
    finally:
        stop_helper(helper)


if __name__ == "__main__":
    raise SystemExit(main())
