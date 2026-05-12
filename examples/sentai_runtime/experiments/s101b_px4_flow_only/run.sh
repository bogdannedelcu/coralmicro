#!/bin/bash
# s101b — PX4 flow-only nav, no GPS, no wind.

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
GZ_BRIDGE_BIN=$COR/build-sim/sim/gz_to_uds_bridge

SPAWN_POSE='0,0,1.0,0,0,0'
TARGET_Z=1.5
TAKEOFF_WAIT_S=10
HOVER_S=15
LAND_WAIT_S=10

STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/tmp/sentai_s101b_$STAMP
mkdir -p "$OUT"
echo "[s101b] artifacts → $OUT"

cat >$OUT/_cleanup.sh <<'EOF'
#!/bin/bash
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz_pose_logger" 2>/dev/null || true
pkill -9 -f "gz_to_uds_bridge" 2>/dev/null || true
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -9 -f sentai_sim 2>/dev/null || true
true
EOF
chmod +x $OUT/_cleanup.sh
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -9 -f sentai_sim 2>/dev/null || true
rm -f /tmp/sim_repl_fifo /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true
sleep 1

# 1. PX4 launches gz server (HEADLESS=1 — single GUI).  Airframe 4041
#    = flow-only nav (no GPS).
cat >$OUT/_launch_px4.sh <<EOF
#!/bin/bash
export GZ_SIM_RESOURCE_PATH="$CRAZYSIM_RES:$GZ_X500_MODELS"
export PX4_GZ_MODELS="$GZ_X500_MODELS"
export PX4_GZ_WORLDS="$CRAZYSIM_RES"
cd "$PX4_DIR"
nohup env HEADLESS=1 \
          PX4_SYS_AUTOSTART=4041 \
          PX4_SIMULATOR=gz \
          PX4_GZ_MODEL=x500_sentai \
          PX4_GZ_WORLD=$WORLD_NAME \
          PX4_GZ_MODEL_POSE='$SPAWN_POSE' \
          "$PX4_BIN" -i 0 -d "$PX4_ETC" > "$OUT/px4.log" 2>&1 &
disown
EOF
chmod +x $OUT/_launch_px4.sh
echo "[s101b] launching PX4 (airframe 4041, NO GPS, HEADLESS)"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_px4.sh
for i in $(seq 1 90); do
    sleep 1
    if grep -qE "Ready for takeoff|Startup script returned successfully" $OUT/px4.log 2>/dev/null; then
        echo "[s101b] PX4 boot OK (${i}s)"; break
    fi
    [ $i -eq 90 ] && { echo "[s101b] PX4 TIMEOUT"; tail -30 $OUT/px4.log; exit 2; }
done

# 1b. Single GUI with PiP.
cat >$OUT/_launch_gui.sh <<EOF
#!/bin/bash
nohup gz sim --gui-config "$GUI_CONFIG" -g > "$OUT/gz_gui.log" 2>&1 &
disown
EOF
chmod +x $OUT/_launch_gui.sh
echo "[s101b] launching gz GUI"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_gui.sh
sleep 3

# 2. Pose logger (ground truth via gz topic -e parser).
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

# 3. sentai_sim + REPL fifo.
mkfifo /tmp/sim_repl_fifo
nohup $SENTAI_BIN < /tmp/sim_repl_fifo > $OUT/sentai.log 2>&1 &
SIM_PID=$!
disown
( tail -f /dev/null > /tmp/sim_repl_fifo ) &
FIFO_KEEPER=$!
disown
echo "[s101b] sentai_sim pid=$SIM_PID"
sleep 2

inject() {
    echo "[repl] $1"
    echo "$1" > /tmp/sim_repl_fifo
    sleep "${2:-0.4}"
}

# 4. Bring up sentai.link + flow forwarder.
inject "import sentai" 0.5
inject "sentai.link.debug(1)" 0.3
inject "sentai.link.init()" 1.0
inject "sentai.link.heartbeat()" 0.3
inject "sentai.link.heartbeat()" 0.5
inject "sentai.link.flow(1, 1.0)" 1.0
inject "print('PRE_BRIDGE_STATS:', sentai.link.stats())" 0.3

# 5. Start gz→UDS camera bridge (C++, no Python gz bindings).
cat >$OUT/_launch_bridge.sh <<EOF
#!/bin/bash
nohup $GZ_BRIDGE_BIN \
    --topic /downward_cam/image \
    --in-sock /tmp/sentai_cam.sock \
    --out-sock /tmp/sentai_flow_out.sock \
    > $OUT/bridge.log 2>&1 &
disown
EOF
chmod +x $OUT/_launch_bridge.sh
echo "[s101b] launching gz→UDS camera bridge"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_bridge.sh
sleep 2

# 5b. Set fake GPS origin + home position via MAVLink.  Without GPS,
#     PX4 navigator refuses takeoff ("no home").  This is the canonical
#     workaround for flow-only nav in SITL.
cat >$OUT/_set_origin.sh <<EOF
#!/bin/bash
python3 /tmp/_set_origin.py > $OUT/origin.log 2>&1
EOF
chmod +x $OUT/_set_origin.sh
echo "[s101b] setting fake GPS origin + home"
distrobox enter $DISTROBOX_NAME -- $OUT/_set_origin.sh
sleep 1

# 6. EKF convergence — wait until flow forwarder publishes ≥ 50 frames
#    (sentai stats[8]) before arming.  This is the "pre-arm flow data"
#    that lets EKF2 build a position estimate without GPS.
echo "[s101b] waiting for ≥50 OPTICAL_FLOW_RAD frames (EKF convergence)..."
for i in $(seq 1 30); do
    inject "print('CONV:', sentai.link.stats())" 0.3
    sleep 0.5
    # Parse the last CONV line from sentai.log.
    tx_flow=$(grep "^CONV:" $OUT/sentai.log 2>/dev/null | tail -1 | sed -E 's/.*, ([0-9]+)\)$/\1/' | head -c 8)
    if [ -n "$tx_flow" ] && [ "$tx_flow" -ge 50 ] 2>/dev/null; then
        echo "[s101b] flow frames seen: $tx_flow (after ${i}s)"
        break
    fi
    if [ $i -eq 30 ]; then
        echo "[s101b] WARN: only $tx_flow flow frames before arm — EKF may not converge"
    fi
done
sleep 2

# 7. Arm + takeoff.
inject "print('--- arm + takeoff to ${TARGET_Z}m (no GPS, flow only) ---')" 0.2
inject "sentai.link.arm(1)" 2.0
inject "sentai.link.takeoff(${TARGET_Z})" "$TAKEOFF_WAIT_S"
inject "print('--- HOVER ${HOVER_S}s ---')" 0.2
inject "sentai.rtos.sleep_ms($((HOVER_S * 1000)))" $((HOVER_S + 1))
inject "print('--- land ---')" 0.2
inject "sentai.link.land()" "$LAND_WAIT_S"
inject "sentai.link.arm(0)" 0.5
inject "sentai.link.flow(0)" 0.3
inject "print('END:', sentai.link.stats())" 0.5

# 8. Teardown.
sleep 2
kill $FIFO_KEEPER 2>/dev/null || true
kill $SIM_PID 2>/dev/null || true
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true
rm -f /tmp/sim_repl_fifo

# 9. Post-process pose.
python3 - <<PYEOF >$OUT/summary.txt 2>&1
import csv, math, os, sys

CSV = "$OUT/pose.csv"
if not os.path.exists(CSV):
    print("FAIL: pose.csv missing"); sys.exit(1)

rows = []
with open(CSV) as f:
    rd = csv.DictReader(f)
    for r in rd:
        try:
            rows.append((float(r["t_s"]), float(r["x_m"]), float(r["y_m"]), float(r["z_m"])))
        except (ValueError, KeyError):
            continue

if not rows:
    print("FAIL: no parseable pose rows"); sys.exit(1)

ts = [r[0] for r in rows]; xs = [r[1] for r in rows]
ys = [r[2] for r in rows]; zs = [r[3] for r in rows]
target = ${TARGET_Z}
print(f"=== s101b ===")
print(f"pose samples: {len(rows)}")
print(f"time span:    {ts[0]:.2f}s -> {ts[-1]:.2f}s ({ts[-1]-ts[0]:.1f}s wall)")
print(f"z range:      [{min(zs):+.3f}, {max(zs):+.3f}] m  (target hover {target:.2f})")
print(f"xy range:     x=[{min(xs):+.3f}, {max(xs):+.3f}]  y=[{min(ys):+.3f}, {max(ys):+.3f}]")

hover_lo = target - 0.30
i_start = next((i for i,z in enumerate(zs) if z >= hover_lo), None)
if i_start is None:
    print(f"\nFAIL: drone never reached z >= {hover_lo:.2f}m (peak z={max(zs):+.3f}m)")
    sys.exit(0)
i_end = next((i for i in range(i_start, len(ts)) if ts[i]-ts[i_start] >= $HOVER_S), len(ts)-1)
x_h = xs[i_start:i_end+1]; y_h = ys[i_start:i_end+1]; z_h = zs[i_start:i_end+1]
x0, y0 = x_h[0], y_h[0]
drifts = [math.hypot(x-x0, y-y0) for x,y in zip(x_h, y_h)]

print(f"\n--- hover window ({ts[i_end]-ts[i_start]:.1f}s) ---")
print(f"z mean:    {sum(z_h)/len(z_h):+.3f}m")
print(f"z range:   [{min(z_h):+.3f}, {max(z_h):+.3f}]m")
print(f"xy drift:  mean={sum(drifts)/len(drifts)*100:.1f} cm  max={max(drifts)*100:.1f} cm")
print(f"final xy:  ({x_h[-1]-x0:+.3f}, {y_h[-1]-y0:+.3f}) m")
print(f"\nFLEW: YES (peak z={max(zs):+.3f}m)")
PYEOF

echo ""
echo "=== s101b SUMMARY ==="
cat $OUT/summary.txt
echo ""
echo "--- PX4 events ---"
grep -E "Takeoff|Land|land|Arm|disarm|Preflight|Compass|EKF" $OUT/px4.log 2>/dev/null | head -15
echo ""
echo "--- bridge stats ---"
tail -5 $OUT/bridge.log
echo ""
echo "--- sentai END stats ---"
grep -E "^END:|^CONV:" $OUT/sentai.log | tail -3
echo ""
echo "[s101b] artifacts in $OUT"
