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
HELPER_PY = REPO / "venv-coral" / "bin" / "python3"
SRC_IMG = REPO / "test_data" / "cat_640x480.bmp"
SRC_MODEL = REPO / "models" / "tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite"
SRC_MISSION = EXP / "mission_s209.py"
CAMERA_SOCK = Path("/tmp/sentai_cam.sock")
TPU_SMOKE = REPO / "build-sim" / "sim" / "tpu_posix_smoke"


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
    missing = [p for p in (SIM, HELPER_PY, SRC_IMG, SRC_MODEL, SRC_MISSION)
               if not p.exists()]
    if missing:
        raise SystemExit("missing required inputs:\n" + "\n".join(str(p) for p in missing))


def cleanup_stale_runtime() -> None:
    exact_exes = {str(SIM), str(TPU_SMOKE)}
    script_names = {"gz_to_camera_bridge.py"}
    protected = {os.getpid(), os.getppid()}
    stale_pids: set[int] = set()
    proc = subprocess.run(["pgrep", "-f", "sentai_sim|tpu_posix_smoke|gz_to_camera_bridge.py"],
                          text=True,
                          capture_output=True,
                          check=False)
    for line in proc.stdout.splitlines():
        try:
            pid = int(line.strip())
        except ValueError:
            continue
        if pid in protected:
            continue
        try:
            raw = Path(f"/proc/{pid}/cmdline").read_bytes()
        except OSError:
            continue
        argv = [p.decode("utf-8", "replace") for p in raw.split(b"\0") if p]
        if not argv:
            continue
        exe = argv[0]
        if exe in exact_exes:
            stale_pids.add(pid)
            continue
        if any(Path(arg).name in script_names for arg in argv[1:]):
            stale_pids.add(pid)

    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in sorted(stale_pids):
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                pass
        if sig == signal.SIGTERM:
            time.sleep(0.2)
    try:
        CAMERA_SOCK.unlink()
    except FileNotFoundError:
        pass


def start_helper(run_dir: Path):
    (run_dir / "helper.log").write_text(
        "no helper: sentai_sim uses direct POSIX/libusb EdgeTPU backend\n",
        encoding="utf-8")
    return None


def stop_helper(helper) -> None:
    pass


def text_or_empty(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bytes):
        return value.decode("utf-8", "replace")
    return str(value)


def prepare_fs(run_dir: Path) -> Path:
    fs_root = run_dir / "fs_root"
    (fs_root / "images").mkdir(parents=True, exist_ok=True)
    (fs_root / "models").mkdir(parents=True, exist_ok=True)
    shutil.copy2(SRC_IMG, fs_root / "images" / SRC_IMG.name)
    shutil.copy2(SRC_MODEL, fs_root / "models" / SRC_MODEL.name)
    shutil.copy2(SRC_MISSION, fs_root / SRC_MISSION.name)
    return fs_root


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
src = np.asarray(Image.open(image).convert('RGB'), dtype=np.uint32)
sh, sw, _ = src.shape
dst = np.empty((h, w, 3), dtype=np.uint8)
for y in range(h):
    y0 = (y * sh) // h
    y1 = ((y + 1) * sh) // h
    if y1 <= y0:
        y1 = y0 + 1
    for x in range(w):
        x0 = (x * sw) // w
        x1 = ((x + 1) * sw) // w
        if x1 <= x0:
            x1 = x0 + 1
        block = src[y0:y1, x0:x1]
        n = block.shape[0] * block.shape[1]
        dst[y, x] = ((block.sum(axis=(0, 1)) + (n // 2)) // n).astype(np.uint8)
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
    try:
        proc = subprocess.run(
            [str(SIM)],
            cwd=str(REPO),
            input=program,
            text=True,
            capture_output=True,
            timeout=45,
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
    debug_log = fs_root / "fr" / "debug.log"
    if debug_log.exists():
        out += "\n" + debug_log.read_text(encoding="utf-8", errors="replace")
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
        "TPU_LOAD_IMAGE": r"TPU_LOAD_IMAGE\s+0",
        "TPU_INVOKE": r"TPU_INVOKE\s+-?\d+",
        "TPU_OUTPUTS": r"TPU_OUTPUTS\s+4",
        "OVERLAY_WRITTEN": r"OVERLAY_WRITTEN\s+0",
    }
    missing = [name for name, pat in required.items() if not re.search(pat, out)]
    if missing:
        raise SystemExit(f"missing expected SIM markers: {missing}")
    host_strong = [d for d in host_dets if d[4] >= 180]
    sim_strong = [d for d in sim_dets if d[4] >= 180]
    if host_strong != sim_strong:
        raise SystemExit(
            f"high-confidence detection mismatch\nHOST={host_strong}\nSIM ={sim_strong}")
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
    cleanup_stale_runtime()
    run_dir = next_iter_dir(args.label)
    run_dir.mkdir(parents=True, exist_ok=False)
    fs_root = prepare_fs(run_dir)

    host_dets = pycoral_baseline(run_dir)
    out, sim_dets = run_sim(run_dir, fs_root)
    validate(out, host_dets, sim_dets)
    draw_overlay(run_dir, sim_dets)
    summary = {
        "ok": True,
        "run_dir": str(run_dir),
        "fs_root": str(fs_root),
        "host_detections": host_dets,
        "sim_detections": sim_dets,
        "sim_tpu_backend": "direct_posix_libusb",
        "host_baseline": "pycoral",
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n",
                                           encoding="utf-8")
    print(f"OK s209 {run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
