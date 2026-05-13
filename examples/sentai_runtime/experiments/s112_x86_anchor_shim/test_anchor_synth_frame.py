#!/usr/bin/env python3
"""End-to-end test with a real cv2.aruco detection.

Unlike test_anchor_wire.py (which sends a hand-crafted struct.pack
packet), this:
  1. Synthesizes a 640×480 PPM containing a real cv2.aruco marker.
  2. Runs sim/scripts/aruco_pose_publisher.py in --frames-dir mode
     against that PPM, so it goes through the real detection codepath
     (cv2.aruco + solvePnP + estimate_drone_world_pose).
  3. Spawns sentai_sim and verifies sentai.flow.anchor_pose() reports
     detected=True (the world pose may be imprecise because the
     synthetic frame isn't a perfect projection — what matters is
     that the detector→UDS→shim→MicroPython chain is intact).

This validates everything except live gz transport, which is
independent code (subscribe_gz path in the publisher).
"""
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"
PUB_SCRIPT = ROOT / "sim/scripts/aruco_pose_publisher.py"
VENV_PY = ROOT / "venv/bin/python3"
RECV_UDS = "/tmp/sentai_aruco_pose_recv.sock"

assert SIM_BIN.exists(),    f"build SIM first: {SIM_BIN}"
assert PUB_SCRIPT.exists(), f"publisher missing: {PUB_SCRIPT}"
assert VENV_PY.exists(),    f"venv missing: {VENV_PY}  (cv2 not available without it)"

import cv2
assert hasattr(cv2, "aruco"), "cv2.aruco missing — install opencv-contrib-python"

# ----------------------------------------------------------------- synth frame
print("[test] synthesising 640x480 RGB frame with 4 ArUco markers …")
W, H = 640, 480
img = np.full((H, W, 3), 220, dtype=np.uint8)        # light background
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
# Marker world positions on the floor (from KNOWN_POSITIONS_M); we just
# need image-space placements that the detector will pick up.  Quad of
# four corners around the centre.
markers_image_xy = [
    (0, 120, 120),   # id 0, top-left
    (1, 420, 120),   # id 1, top-right
    (2, 120, 320),   # id 2, bottom-left
    (3, 420, 320),   # id 3, bottom-right
]
M_SIZE = 100  # marker pixel size in image
for mid, mx, my in markers_image_xy:
    marker_px = cv2.aruco.generateImageMarker(d, mid, M_SIZE)
    marker_rgb = cv2.cvtColor(marker_px, cv2.COLOR_GRAY2RGB)
    img[my:my + M_SIZE, mx:mx + M_SIZE] = marker_rgb

# Save as PPM (publisher --frames-dir reads PPMs via PIL).
tmpdir = Path(tempfile.mkdtemp(prefix="s112_synth_"))
ppm = tmpdir / "synth_aruco_001.ppm"
# PIL expects RGB; save via cv2 by converting BGR-first.
cv2.imwrite(str(ppm.with_suffix(".png")), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
# Convert to PPM by reading the PNG and saving as P6.
from PIL import Image as PILImage
PILImage.fromarray(img, "RGB").save(ppm)
print(f"[test] saved {ppm} ({ppm.stat().st_size} B)")

# ----------------------------------------------------------------- run pipeline
# Clean stale UDS.
try: os.unlink(RECV_UDS)
except FileNotFoundError: pass

print(f"[test] spawning SIM: {SIM_BIN}")
sim = subprocess.Popen([str(SIM_BIN)], stdin=subprocess.PIPE,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, bufsize=1)

def repl(line):
    sim.stdin.write(line + "\n"); sim.stdin.flush()

repl("import sentai")
repl('print("=BOOT_OK")')
repl('print(sentai.flow.mode("anchor"))')

# Wait for SIM to bind UDS.
for _ in range(60):
    if Path(RECV_UDS).exists(): break
    time.sleep(0.05)
else:
    print(f"[test] FAIL: SIM never bound {RECV_UDS}", file=sys.stderr)
    sim.kill(); sys.exit(1)
print(f"[test] SIM bound {RECV_UDS}")

# Start publisher in replay mode.
print(f"[test] launching publisher in --frames-dir mode")
pub = subprocess.Popen([str(VENV_PY), str(PUB_SCRIPT),
                        "--frames-dir", str(tmpdir),
                        "--recv-uds", RECV_UDS,
                        "-v"],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, bufsize=1)
# Publisher loops PPMs at ~30 fps; wait ~1.5 s for several detections.
time.sleep(1.5)

# Query SIM for the latest pose.
repl('p=sentai.flow.anchor_pose(); print("=POSE", p["detected"], p["num_markers"], p["frame_seq"], p["x"], p["y"], p["z"])')
repl("exit")

# Drain.
sim_out, _ = sim.communicate(timeout=5)
pub.terminate()
pub_out, _ = pub.communicate(timeout=3)

# Cleanup.
shutil.rmtree(tmpdir, ignore_errors=True)

print("=== SIM stdout (last lines) ===")
for ln in sim_out.splitlines()[-15:]:
    print(ln)
print("=== publisher stdout (first/last) ===")
pub_lines = pub_out.splitlines()
for ln in pub_lines[:4]: print(ln)
print("...")
for ln in pub_lines[-4:]: print(ln)

# Verify.
for ln in sim_out.splitlines():
    if ln.startswith("=POSE"):
        parts = ln.split()
        det = parts[1] == "True"
        n   = int(parts[2])
        seq = int(parts[3])
        x, y, z = float(parts[4]), float(parts[5]), float(parts[6])
        print()
        print(f"[test] anchor_pose -> detected={det} n={n} seq={seq} "
              f"pose=({x:+.3f},{y:+.3f},{z:+.3f})m")
        if det and n >= 1 and seq > 0:
            print("[test] PASS — real cv2.aruco detection flowed through "
                  "the entire publisher→UDS→shim→MicroPython chain")
            sys.exit(0)
        print(f"[test] FAIL: expected detected=True num_markers>=1 seq>0")
        sys.exit(1)

print("[test] FAIL — no =POSE line in SIM output", file=sys.stderr)
sys.exit(2)
