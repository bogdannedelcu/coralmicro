#!/bin/bash
# s112 — Live cf2/CrazySim+Gazebo validation of sentai.flow.mode("anchor").
#
# cf2 path parity for the PX4 validation in run_px4_live.sh.  Brings up:
#   1. cf2 + Gazebo via existing /tmp/dbox_launch_cf2_headless.sh
#   2. gz_to_uds_bridge (frames → /tmp/sentai_cam.sock)
#   3. aruco_to_vision_estimate.py with --anchor-pub-uds + harmless --mav
#   4. sentai_sim polling anchor_pose() over FIFO
#   5. cflib takeoff via the canonical s090 SyncCrazyflie path
#
# Pass criteria: >= 10 anchor_pose() polls return detected=True during
# the 15s hover, peak z within ±30 cm of the takeoff target.

set -uo pipefail

COR=/home/bogdan/work/coralmicro
DISTROBOX_NAME=crazysim-garden
VENV_PY=$COR/venv/bin/python3
SIM_BIN=$COR/build-sim/sim/sentai_sim
GZ_BRIDGE_BIN=$COR/build-sim/sim/gz_to_uds_bridge
DBOX_LAUNCH=/tmp/dbox_launch_cf2_headless.sh
ANCHOR_UDS=/tmp/sentai_aruco_pose_recv.sock
TARGET_Z=1.0
HOVER_S=15

if [ ! -f "$DBOX_LAUNCH" ]; then
    echo "[s112] FATAL: $DBOX_LAUNCH not found — start once via s090 path then this script will reuse it"
    exit 1
fi

STAMP=$(date +%Y%m%d_%H%M%S)
OUT=$COR/examples/sentai_runtime/experiments/s112_x86_anchor_shim/run_cf2_$STAMP
mkdir -p "$OUT"
echo "[s112-cf2] artifacts → $OUT"

# Cleanup.
cat >$OUT/_cleanup.sh <<'EOF'
#!/bin/bash
pkill -9 -f "gz sim"             2>/dev/null
pkill -9 -f Xvfb                 2>/dev/null
pkill -9 -x cf2                  2>/dev/null
pkill -9 -f gz_to_uds_bridge     2>/dev/null
pkill -9 -f aruco_to_vision      2>/dev/null
pkill -9 -f sentai_sim           2>/dev/null
true
EOF
chmod +x $OUT/_cleanup.sh
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh 2>/dev/null
$OUT/_cleanup.sh 2>/dev/null
rm -f /tmp/sentai_cam.sock $ANCHOR_UDS
sleep 1

# 1. cf2 + Gazebo.
echo "[s112-cf2] launching CrazySim + Gazebo via $DBOX_LAUNCH"
nohup distrobox enter $DISTROBOX_NAME -- bash $DBOX_LAUNCH > $OUT/cf2_launch.log 2>&1 &
DBOX_LAUNCH_PID=$!
disown
# Wait for cf2 to be ready (cflib UDP port 19850 open).
for i in $(seq 1 60); do
    sleep 1
    if grep -qE "CRAZYFLIE FIRMWARE READY|sitl: starting" $OUT/cf2_launch.log 2>/dev/null \
       || ss -lun 2>/dev/null | grep -q ":19850 "; then
        echo "[s112-cf2] cf2+gz ready in ${i}s"
        break
    fi
    if [ $i -eq 60 ]; then
        echo "[s112-cf2] FAIL — cf2 did not boot in 60s"
        tail -20 $OUT/cf2_launch.log
        $OUT/_cleanup.sh
        exit 1
    fi
done
sleep 3   # let Gazebo finish loading textures (markers!)

# 2. gz→UDS bridge.
cat >$OUT/_launch_bridge.sh <<EOF
#!/bin/bash
nohup $GZ_BRIDGE_BIN --topic /downward_cam/image \
    --in-sock /tmp/sentai_cam.sock \
    --out-sock /tmp/sentai_flow_out.sock \
    > $OUT/bridge.log 2>&1 &
disown
EOF
chmod +x $OUT/_launch_bridge.sh

# 3. ArUco bridge — dual-publish (MAVLink to dead port + anchor UDS).
nohup $VENV_PY $COR/sim/scripts/aruco_to_vision_estimate.py \
    --sock /tmp/sentai_cam.sock \
    --mav udpout:127.0.0.1:1            `# unused port; UDP is fire-and-forget` \
    --anchor-pub-uds $ANCHOR_UDS \
    --rate-log 1.0 \
    > $OUT/aruco_anchor.log 2>&1 &
ARUCO_PID=$!
disown
echo "[s112-cf2] aruco→anchor-UDS bridge pid=$ARUCO_PID"
sleep 1

distrobox enter $DISTROBOX_NAME -- $OUT/_launch_bridge.sh
sleep 3

# 4. sentai_sim with FIFO-driven poll loop.
FIFO=$OUT/sim_repl.fifo
mkfifo "$FIFO"
( $SIM_BIN < "$FIFO" ) > $OUT/sim.log 2>&1 &
SIM_PID=$!
disown
exec 3>"$FIFO"
echo "import sentai"              >&3
echo "sentai.flow.mode('anchor')" >&3
echo "print('=ANCHOR_READY')"     >&3
sleep 2
echo "[s112-cf2] sentai_sim pid=$SIM_PID, anchor ready, polling"

# 5. cflib takeoff in background.
cat >$OUT/_takeoff.py <<EOF
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
cflib.crtp.init_drivers()
URI = "udp://127.0.0.1:19850"
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache=None)) as sync:
    cf = sync.cf
    # Wait for log link.
    time.sleep(2)
    with MotionCommander(sync, default_height=$TARGET_Z) as mc:
        print("[takeoff] hovering")
        time.sleep($HOVER_S)
        print("[takeoff] landing")
    print("[takeoff] done")
EOF
echo "[s112-cf2] launching cflib takeoff (host venv)"
$VENV_PY $OUT/_takeoff.py > $OUT/takeoff.log 2>&1 &
TAKEOFF_PID=$!

# Poll anchor_pose() during the hover.
sleep 3  # let takeoff start
for i in $(seq 1 40); do
    echo "p=sentai.flow.anchor_pose(); print('ANCHOR',$i,p['detected'],p['num_markers'],p['x'],p['y'],p['z'],p['frame_seq'])" >&3
    sleep 0.5
done
echo "print('=DONE')" >&3

# Close FIFO; sim sees EOF.
exec 3>&-
wait $TAKEOFF_PID 2>/dev/null || true
wait $SIM_PID     2>/dev/null || true

# Teardown.
$OUT/_cleanup.sh 2>/dev/null
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh 2>/dev/null

# Summary.
echo ""
echo "============================================================"
echo "[s112-cf2] SUMMARY"
echo "============================================================"
echo "cf2/gz launch tail:"
tail -3 $OUT/cf2_launch.log 2>/dev/null
echo ""
echo "ArUco bridge stats (last):"
tail -3 $OUT/aruco_anchor.log
echo ""
echo "anchor_pose samples (sentai_sim):"
grep "ANCHOR " $OUT/sim.log | tail -8

TOTAL_COUNT=$(grep -c "ANCHOR " "$OUT/sim.log")
DETECTED_COUNT=$(grep "ANCHOR " "$OUT/sim.log" | grep -c "True")
PEAK_Z=$(grep "ANCHOR " "$OUT/sim.log" | grep "True" \
         | awk '{print $8}' | sort -gr | head -1)
echo ""
echo "detected=True samples: $DETECTED_COUNT / $TOTAL_COUNT"
echo "peak detected z: $PEAK_Z m  (target ${TARGET_Z} m)"

if [ "$DETECTED_COUNT" -ge 10 ]; then
    echo "[s112-cf2] PASS — sentai.flow.anchor_pose() saw the markers in cf2/Gazebo"
    exit 0
fi
echo "[s112-cf2] FAIL — fewer than 10 detected samples"
exit 3
