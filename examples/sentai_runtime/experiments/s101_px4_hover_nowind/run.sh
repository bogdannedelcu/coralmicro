#!/bin/bash
# s101 step 1: Validate basic PX4 + gz + airframe 4040 (GPS-enabled).
# No flow yet — just confirm x500_sentai spawns, arms, takes off,
# hovers, lands.  Once this works, s101b disables GPS and connects
# the flow forwarder.

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

SPAWN_POSE='0,0,1.0,0,0,0'   # clear of ArUco posts (top z=0.20) — user-spotted bug
TARGET_Z=1.5
TAKEOFF_WAIT_S=10
HOVER_S=10
LAND_WAIT_S=10

STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/tmp/sentai_s101_$STAMP
mkdir -p "$OUT"
echo "[s101] artifacts → $OUT"

cat >$OUT/_cleanup.sh <<'EOF'
#!/bin/bash
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gz_pose_logger" 2>/dev/null || true
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -9 -f sentai_sim 2>/dev/null || true
true
EOF
chmod +x $OUT/_cleanup.sh
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -9 -f sentai_sim 2>/dev/null || true
rm -f /tmp/sim_repl_fifo
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true
sleep 1

# 1. PX4 launches gz server (HEADLESS=1 -> no bare GUI; lockstep stays
#    engaged because PX4 owns the gz server lifetime).  We launch our
#    own GUI with --gui-config separately below.  This eliminates the
#    double-GUI race where PX4's bare GUI + our PiP GUI fought for the
#    same X session and one would hang.
cat >$OUT/_launch_px4.sh <<EOF
#!/bin/bash
export GZ_SIM_RESOURCE_PATH="$CRAZYSIM_RES:$GZ_X500_MODELS"
export PX4_GZ_MODELS="$GZ_X500_MODELS"
export PX4_GZ_WORLDS="$CRAZYSIM_RES"
cd "$PX4_DIR"
nohup env HEADLESS=1 \
          PX4_SYS_AUTOSTART=4040 \
          PX4_SIMULATOR=gz \
          PX4_GZ_MODEL=x500_sentai \
          PX4_GZ_WORLD=$WORLD_NAME \
          PX4_GZ_MODEL_POSE='$SPAWN_POSE' \
          "$PX4_BIN" -i 0 -d "$PX4_ETC" > "$OUT/px4.log" 2>&1 &
disown
EOF
chmod +x $OUT/_launch_px4.sh
echo "[s101] launching PX4 (HEADLESS=1, lockstep mode, no bare GUI)"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_px4.sh
for i in $(seq 1 90); do
    sleep 1
    if grep -qE "Ready for takeoff|Startup script returned successfully" $OUT/px4.log 2>/dev/null; then
        echo "[s101] PX4 boot OK (${i}s)"; break
    fi
    [ $i -eq 90 ] && { echo "[s101] PX4 TIMEOUT"; tail -30 $OUT/px4.log; exit 2; }
done

# 1b. Single GUI with sentai_gui.config (PiP camera per Sim.md §10c rule 1).
cat >$OUT/_launch_gui.sh <<EOF
#!/bin/bash
nohup gz sim --gui-config "$GUI_CONFIG" -g > "$OUT/gz_gui.log" 2>&1 &
disown
EOF
chmod +x $OUT/_launch_gui.sh
echo "[s101] launching gz GUI with PiP config"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_gui.sh
sleep 3   # let GUI attach + EKF settle

# 3. Pose logger — pipes `gz topic -e` text output into a Python parser.
#    Avoids needing gz Python bindings (not available in distrobox).
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

# 4. sentai_sim + REPL fifo (for arm/takeoff/land via sentai.link.*).
mkfifo /tmp/sim_repl_fifo
nohup $SENTAI_BIN < /tmp/sim_repl_fifo > $OUT/sentai.log 2>&1 &
SIM_PID=$!
disown
( tail -f /dev/null > /tmp/sim_repl_fifo ) &
FIFO_KEEPER=$!
disown
echo "[s101] sentai_sim pid=$SIM_PID"
sleep 2

inject() {
    echo "[repl] $1"
    echo "$1" > /tmp/sim_repl_fifo
    sleep "${2:-0.4}"
}

# 5. Mission.
inject "import sentai" 0.5
inject "sentai.link.debug(1)" 0.3
inject "sentai.link.init()" 1.0
inject "sentai.link.heartbeat()" 0.3
inject "sentai.link.heartbeat()" 1.0
inject "print('PRE_HOME:', sentai.link.stats())" 0.3

inject "print('--- arm + takeoff to ${TARGET_Z}m ---')" 0.2
inject "sentai.link.arm(1)" 1.0
inject "sentai.link.takeoff(${TARGET_Z})" "$TAKEOFF_WAIT_S"
HOVER_START=$(wc -l < $OUT/pose.csv)
inject "print('--- HOVER ${HOVER_S}s ---')" 0.2
inject "sentai.rtos.sleep_ms($((HOVER_S * 1000)))" $((HOVER_S + 1))
HOVER_END=$(wc -l < $OUT/pose.csv)
inject "print('--- land ---')" 0.2
inject "sentai.link.land()" "$LAND_WAIT_S"
inject "sentai.link.arm(0)" 0.5
inject "print('END:', sentai.link.stats())" 0.5

# 6. Teardown.
sleep 2
kill $FIFO_KEEPER 2>/dev/null || true
kill $SIM_PID 2>/dev/null || true
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true
rm -f /tmp/sim_repl_fifo

# 7. Post-process pose.
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

ts = [r[0] for r in rows]
xs = [r[1] for r in rows]
ys = [r[2] for r in rows]
zs = [r[3] for r in rows]
target = ${TARGET_Z}
z_max = max(zs)
z_min = min(zs)
print(f"=== s101 ===")
print(f"pose samples: {len(rows)}")
print(f"time span:    {ts[0]:.2f}s -> {ts[-1]:.2f}s ({ts[-1]-ts[0]:.1f}s wall)")
print(f"z range:      [{z_min:+.3f}, {z_max:+.3f}] m   (target hover {target:.2f})")
print(f"xy range:     x=[{min(xs):+.3f}, {max(xs):+.3f}]  y=[{min(ys):+.3f}, {max(ys):+.3f}]")

# Hover-window heuristic: z>=target-0.3 sustained.
hover_lo = target - 0.30
i_start = next((i for i,z in enumerate(zs) if z >= hover_lo), None)
if i_start is None:
    print(f"\nFAIL: drone never reached z >= {hover_lo:.2f}m (peak z={z_max:+.3f}m)")
    sys.exit(0)
# End of hover = i_start + HOVER_S in pose time.
i_end = next((i for i in range(i_start, len(ts)) if ts[i]-ts[i_start] >= $HOVER_S), len(ts)-1)
x_h = xs[i_start:i_end+1]; y_h = ys[i_start:i_end+1]; z_h = zs[i_start:i_end+1]
x0, y0 = x_h[0], y_h[0]
drifts = [math.hypot(x-x0, y-y0) for x,y in zip(x_h, y_h)]
z_mean = sum(z_h)/len(z_h)

print(f"\n--- hover window [{i_start}..{i_end}] ({ts[i_end]-ts[i_start]:.1f}s) ---")
print(f"z mean:    {z_mean:+.3f}m  (target {target:.2f})")
print(f"z range:   [{min(z_h):+.3f}, {max(z_h):+.3f}]m")
print(f"xy drift:  mean={sum(drifts)/len(drifts)*100:.1f} cm  max={max(drifts)*100:.1f} cm")
print(f"final xy:  ({x_h[-1]-x0:+.3f}, {y_h[-1]-y0:+.3f}) m")
print()
print(f"FLEW: YES (peak z={z_max:+.3f}m, hover z_mean={z_mean:+.3f}m)")
PYEOF

echo ""
echo "=== s101 SUMMARY ==="
cat $OUT/summary.txt
echo ""
echo "--- PX4 events ---"
grep -E "Takeoff|Land|land|Arm|disarm|Preflight|EKF|TAKEOFF|LAND" $OUT/px4.log 2>/dev/null | head -20
echo ""
echo "--- sentai REPL tail ---"
tail -10 $OUT/sentai.log
echo ""
echo "[s101] artifacts in $OUT"
echo "[s101] gz GUI left running — close manually"
