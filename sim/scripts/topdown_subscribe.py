#!/usr/bin/env python3
"""Subscribe to gz camera Image topic via `gz topic -e --json-output`
and save the latest frame as PNG.

Run this INSIDE distrobox crazysim-garden where gz CLI is available.
Pipes `gz topic -e --json-output -t <topic>` into stdin parser; first
complete Image message extracted, decoded, saved.

Usage:
  gz topic -e --json-output -t /cam/topdown | python3 topdown_subscribe.py \
      --out /tmp/topdown_render/harmonic_topdown.png
"""
import argparse, base64, json, struct, sys
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="PNG output path")
    ap.add_argument("--max-messages", type=int, default=3,
                    help="number of frames to receive before exit (latest wins)")
    args = ap.parse_args()

    # gz topic -e --json-output streams JSON lines; each line is one msg
    # Image msg format: width, height, step, pixel_format_type, data (base64 in JSON)
    received = 0
    last = None
    buf = ""
    while True:
        chunk = sys.stdin.read(65536)
        if not chunk:
            break
        buf += chunk
        # Try to parse out complete JSON objects.  Iterative bracket counter.
        depth = 0
        start = 0
        in_str = False
        esc = False
        for i, ch in enumerate(buf):
            if esc:
                esc = False; continue
            if ch == '\\':
                esc = True; continue
            if ch == '"':
                in_str = not in_str; continue
            if in_str:
                continue
            if ch == '{':
                if depth == 0:
                    start = i
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0:
                    obj_text = buf[start:i+1]
                    try:
                        obj = json.loads(obj_text)
                        last = obj
                        received += 1
                        print(f"[sub] msg #{received}: width={obj.get('width')} height={obj.get('height')} step={obj.get('step')} fmt={obj.get('pixelFormatType')}", file=sys.stderr)
                        if received >= args.max_messages:
                            buf = ""
                            break
                    except json.JSONDecodeError as e:
                        print(f"[sub] parse err: {e}", file=sys.stderr)
        else:
            continue
        break

    if last is None:
        print("[sub] FAIL — no image messages received", file=sys.stderr)
        return 1

    # Decode base64 data
    data_b64 = last.get('data')
    if not data_b64:
        print("[sub] FAIL — no data field in message", file=sys.stderr)
        return 2
    raw = base64.b64decode(data_b64)
    w = int(last['width'])
    h = int(last['height'])

    # Save via PIL
    from PIL import Image as PILImage
    fmt = last.get('pixelFormatType', 'RGB_INT8')
    if 'RGB' in str(fmt).upper() or fmt == 3:
        mode = 'RGB'
    else:
        mode = 'RGB'
    img = PILImage.frombytes(mode, (w, h), raw)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)
    print(f"[sub] saved {w}x{h} → {out} ({out.stat().st_size} B)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
