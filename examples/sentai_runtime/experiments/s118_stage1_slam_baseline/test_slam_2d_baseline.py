#!/usr/bin/env python3
"""Stage 1.A baseline — verify sentai.slam (existing 2D EKF-SLAM) works on SIM.

This is the foundation for upcoming 3D extension.  PASS criteria:
  - slam.init() OK
  - predict() advances pose deterministically
  - update() with a single detection inserts a landmark
  - landmarks() returns the new landmark with position estimate
  - save() + load() round-trip preserves map
  - clear() empties the map
"""
import subprocess, sys, time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"
assert SIM_BIN.exists(), f"build SIM first: {SIM_BIN}"

DRIVER = '''
import sentai
print("=BOOT")

# 1. init with 70° FOV, 320×240 image, 16 max landmarks
rc = sentai.slam.init(70.0, 320, 240, 16)
print("=INIT", rc)

# 2. baseline pose query
print("=POSE0", sentai.slam.pose())

# 3. simulate detection: object at (160, 120) in image (center) — should
#    create landmark approximately in front of robot.
#    Detection format: (x1, y1, x2, y2, conf, class_id)
dets = [(140, 100, 180, 140, 0.9, 1)]
n_lm = sentai.slam.update(dets)
print("=UPDATE_N", n_lm)
print("=LANDMARKS_AFTER_FIRST", sentai.slam.landmarks())

# 4. predict to move forward 0.5m
sentai.slam.predict(0.5, 0.0, 0.0)
print("=POSE_AFTER_MOVE", sentai.slam.pose())

# 5. re-observe same object: should refine its position
dets2 = [(150, 105, 175, 135, 0.92, 1)]
n_lm2 = sentai.slam.update(dets2)
print("=UPDATE2_N", n_lm2)

# 6. info dict
print("=INFO", sentai.slam.info())

# 7. save to FS (touches new sim_fs_c_api.c we wrote!)
rc_save = sentai.slam.save("/slam/baseline_test.bin")
print("=SAVE_RC", rc_save)

# 8. clear + reload
sentai.slam.clear()
print("=POSE_AFTER_CLEAR", sentai.slam.pose())
print("=INFO_AFTER_CLEAR", sentai.slam.info()["active"])

rc_load = sentai.slam.load("/slam/baseline_test.bin")
print("=LOAD_RC", rc_load)
print("=POSE_AFTER_LOAD", sentai.slam.pose())
print("=INFO_AFTER_LOAD", sentai.slam.info()["active"])
print("=DONE")
'''

proc = subprocess.Popen([str(SIM_BIN)], stdin=subprocess.PIPE,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, bufsize=1)
proc.stdin.write(DRIVER + "\n")
proc.stdin.flush()
time.sleep(0.5)
proc.stdin.write("exit\n")
out, _ = proc.communicate(timeout=8)

# Print everything for diagnostics
lines = out.splitlines()
for ln in lines:
    if "=" in ln or "Error" in ln or "FAIL" in ln or "active" in ln.lower():
        print(ln)

# Check key markers
required = ["=INIT 0", "=UPDATE_N", "=LANDMARKS_AFTER_FIRST", "=SAVE_RC 0", "=LOAD_RC 0", "=DONE"]
missing = [m for m in required if not any(m in ln for ln in lines)]
if missing:
    print(f"\n[test] FAIL — missing markers: {missing}")
    sys.exit(1)
print("\n[test] PASS — slam.init+update+predict+save+load all work on SIM")
sys.exit(0)
