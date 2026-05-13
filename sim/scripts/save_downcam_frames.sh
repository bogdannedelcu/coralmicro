#!/bin/bash
# save_downcam_frames.sh — subscribe to /downward_cam/image and save each
# frame to <out_dir>/frame_NNN.png with a sidecar _stats.csv.
# Run INSIDE distrobox crazysim-garden (needs gz CLI + python3-pil + numpy).
#
# Usage: save_downcam_frames.sh <out_dir> [<seconds>]
set -e

OUT="${1:?usage: save_downcam_frames.sh <out_dir> [seconds]}"
SECS="${2:-60}"
mkdir -p "$OUT"

SAVER=$(mktemp /tmp/_save_frames_XXX.py)
cat > "$SAVER" <<'PYEOF'
import sys, os, json, base64
from PIL import Image
import numpy as np

out_dir = sys.argv[1]
stats = open(os.path.join(out_dir, "_stats.csv"), "w")
stats.write("idx,mean,std,grad_sum,unique_colors\n")
print(f"[saver] writing to {out_dir}", file=sys.stderr, flush=True)

# Streaming JSON object splitter — we accumulate bytes and yield
# complete top-level {...} objects as soon as we see the matching `}`.
# Quote-aware: skip braces inside string literals.
buf = bytearray()
depth = 0
in_str = False
esc = False
obj_start = -1
frame_idx = 0

while True:
    chunk = sys.stdin.buffer.read(65536)
    if not chunk:
        break
    pos = len(buf)
    buf.extend(chunk)
    # Scan the NEW bytes; previous bytes already accounted for in depth.
    for i in range(pos, len(buf)):
        ch = buf[i]
        if esc:
            esc = False
            continue
        if ch == 0x5C:   # '\\'
            esc = True
            continue
        if ch == 0x22:   # '"'
            in_str = not in_str
            continue
        if in_str:
            continue
        if ch == 0x7B:   # '{'
            if depth == 0:
                obj_start = i
            depth += 1
        elif ch == 0x7D: # '}'
            depth -= 1
            if depth == 0 and obj_start >= 0:
                try:
                    obj = json.loads(bytes(buf[obj_start:i+1]).decode('utf-8'))
                    w = int(obj['width']); h = int(obj['height'])
                    step = int(obj.get('step', w*3))
                    raw = base64.b64decode(obj['data'])
                    img = Image.frombytes('RGB', (w, h), raw[:h*step], 'raw', 'RGB', step, 1)
                    fn = os.path.join(out_dir, f"frame_{frame_idx:04d}.png")
                    img.save(fn)
                    arr = np.asarray(img.convert('L'))
                    grad = int(np.abs(np.diff(arr, axis=1)).sum() + np.abs(np.diff(arr, axis=0)).sum())
                    rgb = np.asarray(img).reshape(-1, 3)
                    unique = len(np.unique(rgb, axis=0))
                    stats.write(f"{frame_idx},{arr.mean():.2f},{arr.std():.2f},{grad},{unique}\n")
                    stats.flush()
                    if frame_idx % 10 == 0:
                        print(f"[saver] {frame_idx:4d}: mean={arr.mean():.1f} std={arr.std():.2f} grad={grad}",
                              file=sys.stderr, flush=True)
                    frame_idx += 1
                except Exception as e:
                    print(f"[saver] parse err: {e}", file=sys.stderr, flush=True)
                obj_start = -1
    # Free already-processed bytes (everything before the in-flight object,
    # or all of buf if no object is in flight).
    if obj_start >= 0:
        buf = buf[obj_start:]
        obj_start = 0   # because we just trimmed
    else:
        buf = bytearray()

stats.close()
print(f"[saver] done — {frame_idx} frames", file=sys.stderr, flush=True)
PYEOF

echo "[grab] sinking $SECS seconds of /downward_cam/image → $OUT" >&2
exec timeout "$SECS" gz topic -e --json-output -t /downward_cam/image | \
  python3 "$SAVER" "$OUT"
