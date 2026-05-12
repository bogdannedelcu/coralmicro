#!/bin/bash
# s102 — PX4 takeoff/hover/land via VISION_POSITION_ESTIMATE (mock
# source = gz ground-truth pose).  Validates B-path PX4 EKF fusion.

set -e

DISTROBOX_NAME=crazysim-garden
COR=/home/bogdan/work/coralmicro
PX4_DIR=/home/bogdan/work/px4/PX4-Autopilot
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4
PX4_ETC=$PX4_DIR/build/px4_sitl_default/etc
GZ_X500_MODELS=$PX4_DIR/Tools/simulation/gz/models

CRAZYSIM_GZ=/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo
WORLD_FILE=$CRAZYSIM_GZ/worlds/sentai_crazysim.sdf
WORLD_NAME=sentai_crazysim
CRAZYSIM_RES=$CRAZYSIM_GZ/worlds

GUI_CONFIG=$COR/sim/gazebo/sentai_gui.config
SENTAI_BIN=$COR/build-sim/sim/sentai_sim

SPAWN_POSE='0,0,1.0,0,0,0'
TARGET_Z=1.5
TAKEOFF_WAIT_S=10
HOVER_S=15
LAND_WAIT_S=10

STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/tmp/sentai_s102_$STAMP
mkdir -p "$OUT"
echo "[s102] artifacts → $OUT"

# 0. Cleanup
cat >$OUT/_cleanup.sh <<'EOF'
#!/bin/bash
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f gz_pose_logger_text 2>/dev/null
pkill -9 -f gz_pose_to_vision_estimate 2>/dev/null
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f sentai_sim 2>/dev/null
true
EOF
chmod +x $OUT/_cleanup.sh
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -9 -f sentai_sim 2>/dev/null || true
rm -f /tmp/sim_repl_fifo
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true
sleep 1

# 1. PX4 + gz (HEADLESS + airframe 4042 vision).
cat >$OUT/_launch_px4.sh <<EOF
#!/bin/bash
export GZ_SIM_RESOURCE_PATH="$CRAZYSIM_RES:$GZ_X500_MODELS"
export PX4_GZ_MODELS="$GZ_X500_MODELS"
export PX4_GZ_WORLDS="$CRAZYSIM_RES"
cd "$PX4_DIR"
nohup env HEADLESS=1 \
          PX4_SYS_AUTOSTART=4042 \
          PX4_SIMULATOR=gz \
          PX4_GZ_MODEL=x500_sentai \
          PX4_GZ_WORLD=$WORLD_NAME \
          PX4_GZ_MODEL_POSE='$SPAWN_POSE' \
          "$PX4_BIN" -i 0 -d "$PX4_ETC" > "$OUT/px4.log" 2>&1 &
disown
EOF
chmod +x $OUT/_launch_px4.sh
echo "[s102] launching PX4 (airframe 4042 = external vision, no GPS, no flow)"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_px4.sh
for i in $(seq 1 60); do
    sleep 1
    if grep -qE "Ready for takeoff|Startup script returned successfully" $OUT/px4.log 2>/dev/null; then
        echo "[s102] PX4 boot OK (${i}s)"; break
    fi
    [ $i -eq 60 ] && { echo "[s102] PX4 TIMEOUT"; tail -30 $OUT/px4.log; exit 2; }
done

# 1b. GUI.
cat >$OUT/_launch_gui.sh <<EOF
#!/bin/bash
nohup gz sim --gui-config "$GUI_CONFIG" -g > "$OUT/gz_gui.log" 2>&1 &
disown
EOF
chmod +x $OUT/_launch_gui.sh
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_gui.sh
sleep 3

# 2. Vision bridge — pipe gz dynamic_pose to PX4 VISION_POSITION_ESTIMATE.
cat >$OUT/_launch_vision.sh <<EOF
#!/bin/bash
nohup bash -c "gz topic -e -t /world/$WORLD_NAME/dynamic_pose/info 2>$OUT/vision.gzerr | \
    python3 $COR/sim/scripts/gz_pose_to_vision_estimate.py \
        --model x500_sentai_0 \
        --mav udpout:127.0.0.1:18570 \
        --rate 30 \
        --log $OUT/vision_pose.csv" \
    > $OUT/vision.log 2>&1 &
disown
EOF
chmod +x $OUT/_launch_vision.sh
echo "[s102] launching vision bridge (gz pose → MAVLink VISION_POSITION_ESTIMATE)"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_vision.sh
sleep 3

# 3. Pose logger (independent ground truth).
cat >$OUT/_launch_pose.sh <<EOF
#!/bin/bash
nohup bash -c "gz topic -e -t /world/$WORLD_NAME/dynamic_pose/info 2>$OUT/pose.gzerr | \
    python3 $COR/sim/scripts/gz_pose_logger_text.py --model x500_sentai_0 --out $OUT/pose.csv" \
    > $OUT/pose.log 2>&1 &
disown
EOF
chmod +x $OUT/_launch_pose.sh
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_pose.sh
sleep 2

# 4. sentai_sim + REPL for arm/takeoff/land.
mkfifo /tmp/sim_repl_fifo
nohup $SENTAI_BIN < /tmp/sim_repl_fifo > $OUT/sentai.log 2>&1 &
SIM_PID=$!
disown
( tail -f /dev/null > /tmp/sim_repl_fifo ) &
FIFO_KEEPER=$!
disown
echo "[s102] sentai_sim pid=$SIM_PID"
sleep 2

inject() {
    echo "[repl] $1"
    echo "$1" > /tmp/sim_repl_fifo
    sleep "${2:-0.4}"
}

inject "import sentai" 0.5
inject "sentai.link.debug(1)" 0.3
inject "sentai.link.init()" 1.0
# init() starts a 1 Hz heartbeat task in C — no manual HB needed.

# 4b. Set fake GPS origin + home (no GPS sim → PX4 has no home → won't
#     arm).  Compares s101 px4.log (has "tone_alarm: home set") vs
#     s102 (missing that line).
cat >$OUT/_set_origin.sh <<EOF
#!/bin/bash
python3 /tmp/_set_origin.py > $OUT/origin.log 2>&1
EOF
chmod +x $OUT/_set_origin.sh
echo "[s102] setting fake GPS origin + home"
distrobox enter $DISTROBOX_NAME -- $OUT/_set_origin.sh
sleep 1

# 5. Wait for EKF vision convergence (5 s).
echo "[s102] waiting 5s for EKF vision convergence..."
sleep 5

# 6. Probe EKF state via pymavlink BEFORE arm.
cat >$OUT/_probe.sh <<EOF
#!/bin/bash
python3 - <<PY
import time
from pymavlink import mavutil
m = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=254)
m.wait_heartbeat(timeout=5)
m.mav.request_data_stream_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 4, 1)
m.mav.request_data_stream_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 4, 1)
t0 = time.time()
got_est = False; got_local = False
while time.time() - t0 < 3 and not (got_est and got_local):
    msg = m.recv_match(timeout=0.5)
    if not msg: continue
    if msg.get_type() == 'ESTIMATOR_STATUS' and not got_est:
        print(f"ESTIMATOR_STATUS pos_horiz_ratio={msg.pos_horiz_ratio} vel_ratio={msg.vel_ratio} pos_horiz_accuracy={msg.pos_horiz_accuracy:.3f}")
        got_est = True
    if msg.get_type() == 'LOCAL_POSITION_NED' and not got_local:
        print(f"LOCAL_POSITION_NED x={msg.x:+.3f} y={msg.y:+.3f} z={msg.z:+.3f}")
        got_local = True
PY
EOF
chmod +x $OUT/_probe.sh
echo "[s102] probing EKF state..."
distrobox enter $DISTROBOX_NAME -- $OUT/_probe.sh > $OUT/probe.log 2>&1
cat $OUT/probe.log

# 7. Arm + takeoff + hover + land.  Heartbeats run automatically in C.
inject "print('--- arm + takeoff to ${TARGET_Z}m (external vision, no GPS) ---')" 0.2
inject "sentai.link.arm(1)" 1.5
inject "sentai.link.takeoff(${TARGET_Z})" "$TAKEOFF_WAIT_S"
inject "print('--- HOVER ${HOVER_S}s ---')" 0.2
inject "sentai.rtos.sleep_ms($((HOVER_S * 1000)))" $((HOVER_S + 1))
inject "print('--- land ---')" 0.2
inject "sentai.link.land()" "$LAND_WAIT_S"
inject "sentai.link.arm(0)" 0.5

# 8. Teardown.
sleep 2
kill $FIFO_KEEPER 2>/dev/null || true
kill $SIM_PID 2>/dev/null || true
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true
rm -f /tmp/sim_repl_fifo

# 9. Summary.
python3 - <<PYEOF >$OUT/summary.txt 2>&1
import csv, math, os, sys

CSV = "$OUT/pose.csv"
if not os.path.exists(CSV):
    print("FAIL: pose.csv missing"); sys.exit(1)
rows = []
with open(CSV) as f:
    for r in csv.DictReader(f):
        try: rows.append((float(r["t_s"]), float(r["x_m"]), float(r["y_m"]), float(r["z_m"])))
        except: continue
if not rows:
    print("FAIL: empty pose"); sys.exit(1)

ts=[r[0] for r in rows]; xs=[r[1] for r in rows]; ys=[r[2] for r in rows]; zs=[r[3] for r in rows]
target=${TARGET_Z}
print(f"=== s102 (B-mock: gz pose → VISION_POSITION_ESTIMATE) ===")
print(f"pose samples: {len(rows)}")
print(f"z range: [{min(zs):+.3f}, {max(zs):+.3f}] m  target {target:.2f}")

hover_lo = target - 0.30
i_start = next((i for i,z in enumerate(zs) if z >= hover_lo), None)
if i_start is None:
    print(f"\nFAIL: never reached z >= {hover_lo:.2f}m (peak z={max(zs):+.3f}m)")
    sys.exit(0)
i_end = next((i for i in range(i_start, len(ts)) if ts[i]-ts[i_start] >= $HOVER_S), len(ts)-1)
x_h=xs[i_start:i_end+1]; y_h=ys[i_start:i_end+1]; z_h=zs[i_start:i_end+1]
x0,y0=x_h[0],y_h[0]
drifts=[math.hypot(x-x0,y-y0) for x,y in zip(x_h,y_h)]
print(f"\n--- hover window ({ts[i_end]-ts[i_start]:.1f}s) ---")
print(f"z mean: {sum(z_h)/len(z_h):+.3f}m")
print(f"xy drift mean={sum(drifts)/len(drifts)*100:.1f}cm max={max(drifts)*100:.1f}cm")
print(f"final xy: ({x_h[-1]-x0:+.3f}, {y_h[-1]-y0:+.3f}) m")
print(f"\nFLEW: YES (peak z={max(zs):+.3f}m)")
PYEOF

echo ""
echo "=== s102 SUMMARY ==="
cat $OUT/summary.txt
echo ""
echo "--- PX4 events ---"
grep -E "Takeoff|Land|Arm|disarm|Preflight|Compass|EKF|origin" $OUT/px4.log 2>/dev/null | head -15
echo ""
echo "--- vision bridge tail ---"
tail -5 $OUT/vision.log
echo ""
echo "[s102] artifacts in $OUT"
