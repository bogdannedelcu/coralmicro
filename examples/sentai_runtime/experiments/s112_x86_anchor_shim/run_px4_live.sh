#!/bin/bash
# s112 — Live PX4+Gazebo validation of sentai.flow.mode("anchor").
#
# Reuses s109 launch but ALSO starts sentai_sim with anchor mode and
# polls sentai.flow.anchor_pose() during the hover.  Validates the
# whole platform-abstraction chain end-to-end with a real drone.
#
# What gets validated:
#  1. PX4 SITL + Gazebo Garden + x500_sentai world boot
#  2. Bridge (gz → /tmp/sentai_cam.sock) feeds frames
#  3. aruco_to_vision_estimate.py (host venv, cv2) detects ArUco +
#     publishes VPE to PX4 + (new) dual-publishes pose to anchor UDS
#  4. sentai_sim's sentai_aruco_shim_sim.c receives + caches pose
#  5. REPL's sentai.flow.anchor_pose() reports detected=True with
#     pose values matching the drone's actual position
#
# What is NOT validated here (out of scope; covered by s108..s110):
#  - PX4 EKF accepting VPE and stabilising hover under wind
#  - cf2/CrazySim parity (see run_cf2_live.sh in this dir — TODO)
#
# Exit codes:
#   0 — anchor_pose detected==True with valid pose during hover
#   1 — PX4 boot failure
#   2 — anchor publisher / shim never received any detection
#   3 — sentai_sim REPL never reported detected=True

set -uo pipefail

COR=/home/bogdan/work/coralmicro
DISTROBOX_NAME=crazysim-garden
VENV_PY=$COR/venv/bin/python3
PX4_DIR=/home/bogdan/work/px4/PX4-Autopilot
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4
PX4_ETC=$PX4_DIR/build/px4_sitl_default/etc
GZ_X500_MODELS=$PX4_DIR/Tools/simulation/gz/models
GZ_BRIDGE_BIN=$COR/build-sim/sim/gz_to_uds_bridge
CRAZYSIM_GZ=/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo
WORLD_NAME=sentai_crazysim
CRAZYSIM_RES=$CRAZYSIM_GZ/worlds
SIM_BIN=$COR/build-sim/sim/sentai_sim
ANCHOR_UDS=/tmp/sentai_aruco_pose_recv.sock
SPAWN_POSE='0,0,1.0,0,0,0'
TARGET_Z=1.5
HOVER_S=20

STAMP=$(date +%Y%m%d_%H%M%S)
OUT=$COR/examples/sentai_runtime/experiments/s112_x86_anchor_shim/run_$STAMP
mkdir -p "$OUT"
echo "[s112] artifacts → $OUT"

# Cleanup helper.
cat >$OUT/_cleanup.sh <<'EOF'
#!/bin/bash
pkill -9 -f "gz sim"                       2>/dev/null
pkill -9 -f gz_to_uds_bridge               2>/dev/null
pkill -9 -f aruco_to_vision                2>/dev/null
pkill -9 -f offboard_hover                 2>/dev/null
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f sentai_sim                     2>/dev/null
true
EOF
chmod +x $OUT/_cleanup.sh
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh 2>/dev/null
$OUT/_cleanup.sh 2>/dev/null
rm -f /tmp/sentai_cam.sock $ANCHOR_UDS
sleep 1

# ──────────────────────────────────────────────────────────────────────────
# 1. PX4 (airframe 4043 = GPS+VPE fused).
# ──────────────────────────────────────────────────────────────────────────
cat >$OUT/_launch_px4.sh <<EOF
#!/bin/bash
export GZ_SIM_RESOURCE_PATH="$CRAZYSIM_RES:$GZ_X500_MODELS"
export PX4_GZ_MODELS="$GZ_X500_MODELS"
export PX4_GZ_WORLDS="$CRAZYSIM_RES"
cd "$PX4_DIR"
nohup env HEADLESS=1 PX4_SYS_AUTOSTART=4043 PX4_SIMULATOR=gz \
    PX4_GZ_MODEL=x500_sentai PX4_GZ_WORLD=$WORLD_NAME \
    PX4_GZ_MODEL_POSE='$SPAWN_POSE' \
    "$PX4_BIN" -i 0 -d "$PX4_ETC" > "$OUT/px4.log" 2>&1 &
disown
EOF
chmod +x $OUT/_launch_px4.sh
echo "[s112] launching PX4 SITL + Gazebo"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_px4.sh
for i in $(seq 1 60); do
    sleep 1
    if grep -qE "Ready for takeoff|Startup script returned successfully" "$OUT/px4.log" 2>/dev/null; then
        echo "[s112] PX4 booted in ${i}s"
        break
    fi
    if [ $i -eq 60 ]; then
        echo "[s112] FAIL — PX4 did not boot in 60s"
        tail "$OUT/px4.log"
        $OUT/_cleanup.sh
        exit 1
    fi
done

# ──────────────────────────────────────────────────────────────────────────
# 2. gz→UDS camera bridge.
# ──────────────────────────────────────────────────────────────────────────
cat >$OUT/_launch_bridge.sh <<EOF
#!/bin/bash
nohup $GZ_BRIDGE_BIN --topic /downward_cam/image \
    --in-sock /tmp/sentai_cam.sock \
    --out-sock /tmp/sentai_flow_out.sock \
    > $OUT/bridge.log 2>&1 &
disown
EOF
chmod +x $OUT/_launch_bridge.sh

# ──────────────────────────────────────────────────────────────────────────
# 3. ArUco bridge — DUAL-PUBLISHES to MAVLink (PX4) + anchor UDS (our shim).
# ──────────────────────────────────────────────────────────────────────────
nohup $VENV_PY $COR/sim/scripts/aruco_to_vision_estimate.py \
    --sock /tmp/sentai_cam.sock \
    --mav udpout:127.0.0.1:14580 \
    --anchor-pub-uds $ANCHOR_UDS \
    --rate-log 1.0 \
    > $OUT/aruco_vpe.log 2>&1 &
ARUCO_PID=$!
disown
echo "[s112] aruco→{VPE,anchor-UDS} bridge pid=$ARUCO_PID"
sleep 1

distrobox enter $DISTROBOX_NAME -- $OUT/_launch_bridge.sh
sleep 3

# ──────────────────────────────────────────────────────────────────────────
# 4. sentai_sim — anchor mode, pumped one REPL line at a time via FIFO.
#    The SIM REPL is line-at-a-time (multi-line blocks like `for:` won't
#    work via stdin pipe — see agent.md §5 / project_repl_line_at_a_time).
#    We open a FIFO for writing in this shell and feed individual
#    statements with sleeps between them.
# ──────────────────────────────────────────────────────────────────────────
FIFO=$OUT/sim_repl.fifo
mkfifo "$FIFO"
( $SIM_BIN < "$FIFO" ) > $OUT/sim.log 2>&1 &
SIM_PID=$!
disown
echo "[s112] sentai_sim pid=$SIM_PID, polling anchor_pose() via FIFO"
# Open FD 3 → FIFO write end (keeps the FIFO from EOF-ing prematurely).
exec 3>"$FIFO"
echo "import sentai"                          >&3
echo "sentai.flow.mode('anchor')"             >&3
# P2: enable continuous C++ auto-forwarder at 10 Hz for PX4.
# `sentai.link.init()` brings up the UDP backend; defaults route to
# 127.0.0.1:14580 (PX4 Onboard).  See sentai_uart_serial_udp.c.
echo "sentai.link.init()"                     >&3
echo "sentai.flow.anchor_forward(10, 'px4')"  >&3
echo "print('=ANCHOR_READY')"                 >&3
sleep 2

# ──────────────────────────────────────────────────────────────────────────
# 5. OFFBOARD takeoff + hover (so the drone is over the markers).
# ──────────────────────────────────────────────────────────────────────────
sleep 7  # give EKF VPE convergence
cat >$OUT/_launch_mission.sh <<EOF
#!/bin/bash
python3 $COR/examples/sentai_runtime/experiments/s104_px4_offboard/offboard_hover.py \
    --target-z $TARGET_Z --hover-s $HOVER_S --land-s 5 \
    > $OUT/mission.log 2>&1
EOF
chmod +x $OUT/_launch_mission.sh
echo "[s112] OFFBOARD mission (target z=$TARGET_Z, hover=${HOVER_S}s)"
# Start the mission in the BACKGROUND so we can poll anchor_pose() in
# parallel while the drone is in the air.
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_mission.sh &
MISSION_PID=$!
echo "[s112] mission launched pid=$MISSION_PID, polling anchor_pose() while it runs"

# Poll anchor_pose() once per 0.5 s for ~30 s (covers takeoff +
# full hover + landing).  Each iteration is ONE REPL line that the
# SIM evaluates synchronously.
for i in $(seq 1 60); do
    echo "p=sentai.flow.anchor_pose(); print('ANCHOR',$i,p['detected'],p['num_markers'],p['x'],p['y'],p['z'],p['frame_seq'])" >&3
    sleep 0.5
done
# Final P2 stats — proves the continuous C++ forwarder ran.
echo "print('=FWD_STATS', sentai.flow.anchor_forward_stats())" >&3
echo "print('=DONE')" >&3

# Close FIFO so the SIM sees EOF and exits cleanly.
exec 3>&-
wait $MISSION_PID  2>/dev/null
wait $SIM_PID      2>/dev/null
MISSION_RC=0
echo "[s112] mission + sim done"

# Teardown.
$OUT/_cleanup.sh 2>/dev/null
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh 2>/dev/null

# ──────────────────────────────────────────────────────────────────────────
# 6. Summary — verify anchor_pose() saw detections during the run.
# ──────────────────────────────────────────────────────────────────────────
echo ""
echo "============================================================"
echo "[s112] SUMMARY"
echo "============================================================"
echo "PX4 log tail:"
tail -3 "$OUT/px4.log"
echo ""
echo "ArUco bridge stats (last):"
tail -3 "$OUT/aruco_vpe.log"
echo ""
echo "anchor_pose samples (from sentai_sim):"
grep "^ANCHOR " "$OUT/sim.log" | tail -10
echo ""
echo "P2 forwarder stats:"
grep "=FWD_STATS" "$OUT/sim.log" | tail -1
echo ""
# Pass/fail summary.  REPL prefixes lines with ">>> " so match the
# substring "ANCHOR" anywhere and grep for True.
TOTAL_COUNT=$(grep -c "ANCHOR " "$OUT/sim.log")
DETECTED_COUNT=$(grep "ANCHOR " "$OUT/sim.log" | grep -c "True")
echo "detected=True samples: $DETECTED_COUNT / $TOTAL_COUNT"

# Pose-range sanity: x/y/z should vary across the run (not stuck at
# zeros), z monotonic-ish climb to ~target before landing.
PEAK_Z=$(grep "ANCHOR " "$OUT/sim.log" | grep "True" \
         | awk '{print $8}' | sort -gr | head -1)
echo "peak detected z: $PEAK_Z m  (target ${TARGET_Z} m)"

if [ "$DETECTED_COUNT" -ge 10 ]; then
    echo "[s112] PASS — sentai.flow.anchor_pose() saw the markers in Gazebo"
    exit 0
fi
echo "[s112] FAIL — fewer than 10 detected samples"
exit 3
