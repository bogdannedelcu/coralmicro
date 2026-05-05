#!/usr/bin/env python3
# _host_flow_validate.py -- drive _t_flow_validate.py + render trajectory.
#
# Pipeline (per agent.md sec.5.1.5: thin host pipe, all params on board):
#   1. push diag/_t_flow_validate.py via _host_upload_repl
#   2. exec it; stream until "=== done ==="
#   3. discover newest /diags/sNNN_flow_validate via /api/ls
#   4. download trace.csv (or bulk_gray.bin via MSC if BULK mode) +
#      scene_*.jpg + gray_*.pgm via HTTP /api/raw
#   5. parse + summarize + (optional) render annotated PNG
#
# Output: /tmp/sNNN_flow_validate/ (auto-created from board's session id).

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import serial
from PIL import Image, ImageDraw, ImageFont


REMOTE_DIAG_DIR = "/diags"
REMOTE_DRIVER   = "/lib/diag/_t_flow_validate.py"
LOCAL_DRIVER    = Path(__file__).parent / "_t_flow_validate.py"
HTTP_BASE       = "http://10.0.0.1"


def open_repl():
    s = serial.Serial("/dev/ttyACM0", 115200, timeout=0.3,
                      rtscts=False, xonxoff=False, dsrdtr=False)
    time.sleep(0.3); s.reset_input_buffer()
    s.write(b"\x03\r\n"); time.sleep(0.3); s.read(4096)
    return s


def push_driver():
    print("push:", LOCAL_DRIVER, "->", REMOTE_DRIVER)
    cwd = Path(__file__).parent.parent
    subprocess.run(
        [sys.executable, "diag/_host_upload_repl.py", "--file",
         LOCAL_DRIVER.name],
        cwd=cwd, check=True)


def run_driver(s, max_wall_s=120):
    print("exec:", REMOTE_DRIVER, "(wall<=%ds)" % max_wall_s)
    s.write(('exec(sentai.fs.read_str("%s"), globals())\r\n'
             % REMOTE_DRIVER).encode())
    deadline = time.time() + max_wall_s
    while time.time() < deadline:
        c = s.read(8192)
        if c:
            sys.stdout.write(c.decode(errors="replace"))
            sys.stdout.flush()
            if b"=== done ===" in c:
                return
    raise RuntimeError("driver did not print '=== done ===' within %ds"
                       % max_wall_s)


def http_get(path, timeout=15):
    url = HTTP_BASE + path
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def newest_session(prefix):
    raw = http_get("/api/ls/diags").decode()
    data = json.loads(raw)
    items = data.get("items", data) if isinstance(data, dict) else data
    candidates = [it["name"] for it in items
                  if isinstance(it, dict) and prefix in it["name"]]
    if not candidates:
        raise RuntimeError("no session with prefix %r" % prefix)
    candidates.sort()
    return candidates[-1]


def pull(remote_path, local_path, optional=False, retries=3):
    local_path.parent.mkdir(parents=True, exist_ok=True)
    last_err = None
    for _ in range(retries):
        try:
            data = http_get("/api/raw" + remote_path, timeout=30)
            local_path.write_bytes(data)
            print("pull: %s (%d bytes)" % (local_path, len(data)))
            return data
        except Exception as e:
            last_err = e
            time.sleep(0.5)
    if optional:
        print("optional skip %s (%s)" % (remote_path, last_err))
        return None
    raise last_err


def parse_csv(csv_bytes):
    """Parse trace.csv.  dx_q1000/dy_q1000 carry milli-grid-pixels
    (1000 = 1 grid-px = 8 raw-px after PXP downscale)."""
    rows = []
    text = csv_bytes.decode(errors="replace")
    lines = text.strip().split("\n")
    if not lines: return rows
    header = lines[0].split(",")
    dx_key = "dx_q1000" if "dx_q1000" in header else "dx"
    dy_key = "dy_q1000" if "dy_q1000" in header else "dy"
    scale = 1000.0 if dx_key == "dx_q1000" else 1.0
    for ln in lines[1:]:
        parts = ln.split(",")
        if len(parts) != len(header): continue
        d = dict(zip(header, parts))
        try:
            rows.append({
                "t_ms":      int(d["t_ms"]),
                "frame_seq": int(d["frame_seq"]),
                "dx":        int(d[dx_key]) / scale,
                "dy":        int(d[dy_key]) / scale,
                "sad":       int(d["sad"]),
                "conf":      int(d["conf"]),
                "phase":     d["phase"],
                "side_idx":  int(d.get("side_idx", d.get("beat_idx", 0))),
            })
        except Exception:
            continue
    return rows


def summarize(rows):
    if not rows: return "no rows\n"
    n = len(rows)
    move = [r for r in rows if r["phase"] == "MOVE"]
    hold = [r for r in rows if r["phase"] == "HOLD"]
    def avg(xs, k): return (sum(r[k] for r in xs) / len(xs)) if xs else 0.0
    def stuck(xs):
        if not xs: return 0.0
        z = sum(1 for r in xs if abs(r["dx"]) < 0.05 and abs(r["dy"]) < 0.05)
        return 100.0 * z / len(xs)
    eff_hz = (n * 1000.0) / max(1, rows[-1]["t_ms"] - rows[0]["t_ms"])
    cum_x = sum(r["dx"] for r in rows)
    cum_y = sum(r["dy"] for r in rows)
    return (
        "rows=%d  effective_hz=%.2f\n"
        "MOVE:  n=%d  avg_conf=%.1f  stuck<0.05=%.1f%%\n"
        "HOLD:  n=%d  avg_conf=%.1f  stuck<0.05=%.1f%%\n"
        "cumsum: dx=%.2f gp  dy=%.2f gp  closure=%.2f gp\n"
        % (n, eff_hz,
           len(move), avg(move, "conf"), stuck(move),
           len(hold), avg(hold, "conf"), stuck(hold),
           cum_x, cum_y, (cum_x**2 + cum_y**2) ** 0.5))


def render(rows, scene_path, out_path):
    img = Image.open(scene_path).convert("RGB")
    W, H = img.size
    cx, cy = W / 2.0, H / 2.0
    sx = sy = 8  # 1 grid-px = 8 raw-px after PXP downscale
    pts = [(cx, cy)]
    confs = []
    cum_x = cum_y = 0.0
    for r in rows:
        cum_x += r["dx"]; cum_y += r["dy"]
        pts.append((cx + cum_x * sx, cy + cum_y * sy))
        confs.append(r["conf"])
    d = ImageDraw.Draw(img, "RGBA")
    d.ellipse((cx - 6, cy - 6, cx + 6, cy + 6),
              outline=(0, 200, 255, 255), width=2)
    for i in range(1, len(pts)):
        c = confs[i - 1]
        if c >= 200: col = (0, 220, 0, 220)
        elif c >= 100: col = (220, 220, 0, 220)
        else: col = (220, 50, 50, 200)
        d.line([pts[i - 1], pts[i]], fill=col, width=2)
    closure = ((pts[-1][0] - cx) ** 2 + (pts[-1][1] - cy) ** 2) ** 0.5
    img.save(out_path)
    print("render: %s (closure %.1f raw-px)" % (out_path, closure))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--max-wall-s", type=int, default=180)
    ap.add_argument("--no-push", action="store_true")
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()

    if not args.no_push:
        push_driver()

    s = open_repl()
    run_driver(s, max_wall_s=args.max_wall_s)

    sess = newest_session("flow_validate")
    print("session on board:", sess)
    out = Path(args.out_dir or ("/tmp/" + sess))
    out.mkdir(parents=True, exist_ok=True)
    base = "/diags/" + sess

    # Try pulling files: small ones via HTTP, bulk via reminder if too big.
    pull(base + "/params.txt",   out / "params.txt", optional=True)
    pull(base + "/summary.txt",  out / "summary.txt", optional=True)
    pull(base + "/scene_start.jpg", out / "scene_start.jpg", optional=True)
    pull(base + "/scene_end.jpg",   out / "scene_end.jpg",   optional=True)
    pull(base + "/gray_start.pgm",  out / "gray_start.pgm",  optional=True)
    pull(base + "/gray_end.pgm",    out / "gray_end.pgm",    optional=True)

    # Trace.csv if non-bulk; bulk_gray.bin needs MSC for big files.
    csv = pull(base + "/trace.csv", out / "trace.csv", optional=True)
    bulk = pull(base + "/bulk_gray.bin", out / "bulk_gray.bin", optional=True)

    if csv:
        rows = parse_csv(csv)
        summary = summarize(rows)
        (out / "summary_host.txt").write_text(summary)
        print(summary)
        scene = out / "scene_start.jpg"
        if scene.exists():
            render(rows, scene, out / "annotated.png")

    if bulk:
        print("bulk_gray.bin downloaded; use replay_full.py for offline SAD.")
    else:
        print("bulk_gray.bin too big for HTTP -- pull via USB MSC:")
        print("  sudo mount /dev/sda /mnt/sentai")
        print("  cp /mnt/sentai%s/bulk_gray.bin %s/" % (base, out))
        print("  sudo umount /mnt/sentai && printf 'q\\r\\n' > /dev/ttyACM0")

    print("done. artefacts in:", out)


if __name__ == "__main__":
    main()
