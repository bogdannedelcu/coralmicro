#!/usr/bin/env python3
"""Capture one Gazebo image frame -> PNG via JSON (base64 data, lossless)."""
import subprocess, sys, json, base64
from PIL import Image

if len(sys.argv) < 3:
    print(f"usage: {sys.argv[0]} <topic> <output.png>")
    sys.exit(1)

topic, out = sys.argv[1], sys.argv[2]
print(f"capturing {topic} -> {out}", flush=True)
proc = subprocess.run(['gz', 'topic', '--json-output', '-e', '-t', topic, '-n', '1'],
                      capture_output=True, timeout=30)
if proc.returncode != 0:
    print(f"gz topic exit {proc.returncode}, stderr: {proc.stderr.decode()[:300]}")
    sys.exit(1)

j = json.loads(proc.stdout.decode())
w, h = j['width'], j['height']
pf = j.get('pixelFormatType', 'RGB_INT8')
data = base64.b64decode(j['data'])
print(f"  {w}x{h} {pf}, raw={len(data)} bytes (expect {w*h*3} for RGB)")

if pf in ('RGB_INT8',):
    img = Image.frombytes('RGB', (w, h), data[:w*h*3])
elif pf in ('R_FLOAT32',):
    import numpy as np
    arr = np.frombuffer(data, dtype='<f4').reshape(h, w)
    img = Image.fromarray((arr * 255 / arr.max()).astype('uint8'))
else:
    print(f"unsupported pixel format {pf}")
    sys.exit(1)
img.save(out)
print(f"saved {out}")
