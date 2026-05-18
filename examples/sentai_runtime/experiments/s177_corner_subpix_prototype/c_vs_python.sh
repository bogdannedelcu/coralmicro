#!/bin/bash
# Compare C subpix output (via sentai_sim _test_pgm) vs Python ref
# on the two real captured frames.  C should produce per-marker z that
# matches Python (cv2-equivalent) to mm-precision.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
SIM_BIN="$REPO_ROOT/build-sim/sim/sentai_sim"
FS_ROOT="$REPO_ROOT/build-sim/sentai_fs_root"

cp "$SCRIPT_DIR/frame_horiz_n4.pgm" "$FS_ROOT/sub_horiz.pgm"
cp "$SCRIPT_DIR/frame_rot45_n1.pgm" "$FS_ROOT/sub_rot45.pgm"

LOG="$SCRIPT_DIR/c_vs_python.log"
echo "
import sentai
sentai.aruco.init()
print('=== HORIZONTAL ===')
sentai.aruco._test_pgm('$FS_ROOT/sub_horiz.pgm')
print('=== ROTATED45 ===')
sentai.aruco._test_pgm('$FS_ROOT/sub_rot45.pgm')
" | timeout 10 "$SIM_BIN" > "$LOG" 2>&1 || true

echo "=== C side (sentai_sim with T18-C subpix) ==="
grep -E "PGM_RESULT|HORIZONTAL|ROTATED" "$LOG" || cat "$LOG"

echo ""
echo "=== Python ref (cv2 IPPE_SQUARE + cv2.cornerSubPix) ==="
cd "$SCRIPT_DIR" && /home/bogdan/work/coralmicro/venv/bin/python3 -c "
import cv2, numpy as np
FX,FY,CX,CY = 240.,240.,160.,120.
K = np.array([[FX,0,CX],[0,FY,CY],[0,0,1]], dtype=np.float64)
DIST = np.zeros((5,), dtype=np.float64)
MARKER_SIZE = 0.094
L2 = MARKER_SIZE * 0.5
OBJ = np.array([[-L2,+L2,0],[+L2,+L2,0],[+L2,-L2,0],[-L2,-L2,0]], dtype=np.float64)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
params.cornerRefinementWinSize = 5
params.cornerRefinementMaxIterations = 30
params.cornerRefinementMinAccuracy = 0.01
det = cv2.aruco.ArucoDetector(aruco_dict, params)
for name, fn in [('HORIZONTAL','frame_horiz_n4.pgm'), ('ROTATED45','frame_rot45_n1.pgm')]:
    print(f'--- {name} ---')
    img = cv2.imread(fn, cv2.IMREAD_GRAYSCALE)
    corners, ids, _ = det.detectMarkers(img)
    if ids is None: print('  no markers'); continue
    for k in range(len(ids)):
        mid = int(ids[k,0])
        c = corners[k].reshape(4,2).astype(np.float64)
        ok, rvec, tvec, reproj = cv2.solvePnPGeneric(OBJ, c, K, DIST,
                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
        z = float(tvec[0].reshape(3)[2]) if ok else float('nan')
        rp = float(reproj[0,0]) if ok else float('nan')
        print(f'  id={mid}  z={z:+.4f}  reproj={rp:.3f}')
"
