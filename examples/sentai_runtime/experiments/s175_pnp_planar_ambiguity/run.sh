#!/bin/bash
# s175 — sentai_aruco PnP rotation invariance test.
#
# Hypothesis (operator 2026-05-18): sentai_aruco's IPPE-style PnP
# implementation only computes ONE of the two analytical solutions
# for a planar marker (see sentai_aruco.cc line 521 comment vs
# actual code at 635-665).  At certain in-plane image rotations,
# the chosen solution may flip → tvec_cam[2] jumps → cf2 EKF gets
# inconsistent z → drone climbs uncontrolled during T16 yaw test.
#
# This test feeds the SAME PHYSICAL SCENE (n=4 markers, drone
# hovering at z≈1.05 m above the marker grid) as a PGM to
# sentai_aruco_detect, with the IMAGE ROTATED by 0° and by 35°.
# If PnP is rotation-invariant, per-marker tvec_cam[2] should be
# identical between the two runs (markers are at fixed 3D
# positions; in-plane image rotation = drone yaw rotation around
# camera optical Z axis, which doesn't change marker depth).
#
# If tvec_cam[2] differs between rotations → ambiguity confirmed.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
VENV_PY="$REPO_ROOT/venv/bin/python3"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"

INPUT="$SCRIPT_DIR/frame_original.pgm"
ROT35="$SCRIPT_DIR/frame_rot35.pgm"

# Step 1: produce rotated PGM via cv2.warpAffine.
echo "[s175] generating frame_rot35.pgm (in-plane rotation 35°)"
"$VENV_PY" "$SCRIPT_DIR/rotate_pgm.py" \
    --input  "$INPUT" \
    --angle  35.0 \
    --output "$ROT35"

# Step 2: stage the PGMs where sentai_sim can read them.
cp "$INPUT" "$FS_ROOT/frame_original.pgm"
cp "$ROT35" "$FS_ROOT/frame_rot35.pgm"

# Step 3: run sentai_sim REPL — call sentai.aruco._test_pgm() twice
# and let the C-side print PGM_RESULT lines to stderr.
echo "[s175] running sentai_aruco detect on both frames via sentai_sim"
LOG="$SCRIPT_DIR/run.log"
echo "
import sentai
sentai.aruco.init()
print('=== frame_original.pgm ===')
n0 = sentai.aruco._test_pgm('/home/bogdan/work/coralmicro/build-sim/sentai_fs_root/frame_original.pgm')
print('n_dets original =', n0)
print('=== frame_rot35.pgm ===')
n1 = sentai.aruco._test_pgm('/home/bogdan/work/coralmicro/build-sim/sentai_fs_root/frame_rot35.pgm')
print('n_dets rot35 =', n1)
" | timeout 10 "$SIM_BIN" > "$LOG" 2>&1 || true

echo ""
echo "=== RESULTS ==="
grep -E "PGM_RESULT|n_dets" "$LOG" || cat "$LOG"

# Step 4: compute Z deltas via Python verdict.
"$VENV_PY" "$SCRIPT_DIR/verdict.py" "$LOG"
