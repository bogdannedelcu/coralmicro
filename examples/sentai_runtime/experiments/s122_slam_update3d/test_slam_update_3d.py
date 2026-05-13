#!/usr/bin/env python3
"""Stage 1.C — verify sentai.slam.update_3d() uses class-size prior for depth.

Driver:
  1. init slam @ fov=60°, 320×240 image
  2. set_class_prior(0, 1.7) — class 0 is "person" → 1.7 m tall
  3. set_class_prior(11, 0.6) — class 11 is "stop_sign" → 0.6 m
  4. Feed two detections of class 0:
       - bbox h=120 px (person fills ~half frame) → small depth
       - bbox h=30 px  (person far away)          → ~4× larger depth
  5. After update_3d, query landmarks() — both should be created
  6. The far one's range must be ≥ 3× the near one
  7. update_3d with class that has no prior set (class 99) → still creates
     a landmark but uses SLAM_DEFAULT_RANGE (2.0 m)
  8. Degenerate bbox (y2==y1) — skipped, no landmark added
"""
import subprocess, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
SIM_BIN = ROOT / "build-sim/sim/sentai_sim"

DRIVER = '''
import sentai

# init: fov=60°, 320×240 → focal = 320 / (2 tan(30°)) = 277.13 px
sentai.slam.init(60.0, 320, 240)
sentai.slam.set_class_prior(0, 1.7)
sentai.slam.set_class_prior(11, 0.6)

# Near person: cx=160, cy=120, w=80, h=120
# range = 1.7 × 277.13 / 120 = 3.93 m
det_near = (120, 60, 200, 180, 0.9, 0)

# Far person: cx=160, cy=120, w=20, h=30
# range = 1.7 × 277.13 / 30 = 15.7 m
det_far = (150, 105, 170, 135, 0.8, 0)

n = sentai.slam.update_3d([det_near, det_far])
print("=N_UPDATED_PAIR", n)

lms = sentai.slam.landmarks()
print("=LMS_COUNT", len(lms))

# Each landmark dict has x/y/class_id.  Compute Euclidean distance from origin.
# Both detections share the bearing of the center pixel, so landmarks lie on
# the same ray; |xy| equals the EKF-converged range.
d0 = (lms[0]["x"]**2 + lms[0]["y"]**2) ** 0.5
d1 = (lms[1]["x"]**2 + lms[1]["y"]**2) ** 0.5
near_d, far_d = (d0, d1) if d0 < d1 else (d1, d0)
print("=NEAR_RANGE_OK", 3.5 < near_d < 4.5)        # expected 3.93
print("=FAR_RANGE_OK",  14.0 < far_d  < 17.0)      # expected 15.71
print("=RATIO_OK", (far_d / near_d) >= 3.0)

# Reset and feed class without prior — should still create landmark
# but use SLAM_DEFAULT_RANGE.  Class 99 has no prior set.
sentai.slam.clear()
det_unk = (140, 100, 180, 140, 0.7, 33)
n2 = sentai.slam.update_3d([det_unk])
print("=UNK_N", n2)
unk_lms = sentai.slam.landmarks()
print("=UNK_LMS", len(unk_lms))
# range was SLAM_DEFAULT_RANGE = 2.0 m; landmark dist ~2.0
unk_dist = (unk_lms[0]["x"]**2 + unk_lms[0]["y"]**2) ** 0.5
print("=UNK_DIST_NEAR_2", abs(unk_dist - 2.0) < 0.5)

# Degenerate bbox: y2 == y1, height 0 — must be skipped
sentai.slam.clear()
det_bad = (100, 50, 200, 50, 0.9, 0)
n3 = sentai.slam.update_3d([det_bad])
print("=DEGEN_N", n3)
print("=DEGEN_LMS", len(sentai.slam.landmarks()))

print("=DONE")
'''

proc = subprocess.Popen([str(SIM_BIN)], stdin=subprocess.PIPE,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, bufsize=1)
proc.stdin.write(DRIVER + "\nexit\n")
proc.stdin.flush()
out, _ = proc.communicate(timeout=10)

def parse(prefix):
    for ln in out.splitlines():
        i = ln.find(prefix)
        if i >= 0:
            return ln[i + len(prefix):].strip()
    return None

for ln in out.splitlines():
    if "=" in ln:
        print(ln)

ok = True
def check(name, prefix, expected):
    global ok
    got = parse(prefix)
    if got != expected:
        print(f"FAIL: {name}: expected {expected!r}, got {got!r}")
        ok = False
    else:
        print(f"  OK: {name}")

print("\n--- Verification ---")
check("update_3d returned 2 landmarks",   "=N_UPDATED_PAIR",    "2")
check("two landmarks in map",             "=LMS_COUNT",         "2")
check("near landmark depth ~3.9 m",       "=NEAR_RANGE_OK",     "True")
check("far landmark depth ~15.7 m",       "=FAR_RANGE_OK",      "True")
check("far/near ratio ≥ 3×",              "=RATIO_OK",          "True")
check("no-prior class still creates lm",  "=UNK_N",             "1")
check("no-prior produces 1 landmark",     "=UNK_LMS",           "1")
check("no-prior range ≈ default 2.0 m",   "=UNK_DIST_NEAR_2",   "True")
check("degenerate bbox skipped (n=0)",    "=DEGEN_N",           "0")
check("degenerate bbox produces 0 lms",   "=DEGEN_LMS",         "0")

if ok:
    print("\n[test] PASS — slam.update_3d uses class prior for pseudo-depth")
    sys.exit(0)
print("\n[test] FAIL")
sys.exit(1)
