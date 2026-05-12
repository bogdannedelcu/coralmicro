#!/bin/bash
# s104 — PX4 OFFBOARD hover via SET_POSITION_TARGET_LOCAL_NED.
# Uses airframe 4040 (GPS-enabled, known good).  Validates that
# OFFBOARD mode works on our PX4 + gz + x500_sentai stack.

set -e

DISTROBOX_NAME=crazysim-garden
COR=/home/bogdan/work/coralmicro
PX4_DIR=/home/bogdan/work/px4/PX4-Autopilot
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4
PX4_ETC=$PX4_DIR/build/px4_sitl_default/etc
GZ_X500_MODELS=$PX4_DIR/Tools/simulation/gz/models

CRAZYSIM_GZ=/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo
WORLD_NAME=sentai_crazysim
CRAZYSIM_RES=$CRAZYSIM_GZ/worlds

GUI_CONFIG=$COR/sim/gazebo/sentai_gui.config

SPAWN_POSE='0,0,1.0,0,0,0'
TARGET_Z=1.5
HOVER_S=15

STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/tmp/sentai_s104_$STAMP
mkdir -p "$OUT"
echo "[s104] artifacts → $OUT"

# 0. Cleanup.
cat >$OUT/_cleanup.sh <<'EOF'
#!/bin/bash
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f gz_pose_logger 2>/dev/null
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f offboard_hover 2>/dev/null
true
EOF
chmod +x $OUT/_cleanup.sh
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true
sleep 1

# 1. PX4 + gz (HEADLESS + airframe 4040 GPS-enabled).
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
echo "[s104] launching PX4 (airframe 4040, GPS baseline)"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_px4.sh
for i in $(seq 1 60); do
    sleep 1
    if grep -qE "Ready for takeoff|Startup script returned successfully" $OUT/px4.log 2>/dev/null; then
        echo "[s104] PX4 boot OK (${i}s)"; break
    fi
    [ $i -eq 60 ] && { echo "[s104] PX4 TIMEOUT"; tail -30 $OUT/px4.log; exit 2; }
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

# 2. Pose logger (ground truth).
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

# 3. OFFBOARD hover (Python in distrobox).
cat >$OUT/_launch_offboard.sh <<EOF
#!/bin/bash
python3 $COR/examples/sentai_runtime/experiments/s104_px4_offboard/offboard_hover.py \
    --target-z $TARGET_Z --hover-s $HOVER_S --land-s 8 \
    > $OUT/offboard.log 2>&1
EOF
chmod +x $OUT/_launch_offboard.sh
echo "[s104] running OFFBOARD mission..."
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_offboard.sh

# 4. Teardown.
sleep 2
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true

# 5. Summary.
python3 - <<PYEOF >$OUT/summary.txt 2>&1
import csv, math, os, sys

CSV="$OUT/pose.csv"
if not os.path.exists(CSV):
    print("FAIL: pose.csv missing"); sys.exit(1)
rows=[]
with open(CSV) as f:
    for r in csv.DictReader(f):
        try: rows.append((float(r["t_s"]),float(r["x_m"]),float(r["y_m"]),float(r["z_m"])))
        except: continue
if not rows: print("FAIL: empty"); sys.exit(1)

ts=[r[0] for r in rows]; xs=[r[1] for r in rows]; ys=[r[2] for r in rows]; zs=[r[3] for r in rows]
target=$TARGET_Z
print(f"=== s104 (OFFBOARD setpoint hover, GPS baseline) ===")
print(f"pose samples: {len(rows)}")
print(f"z range: [{min(zs):+.3f}, {max(zs):+.3f}] m  target {target:.2f}")

hover_lo=target-0.30
i_start=next((i for i,z in enumerate(zs) if z>=hover_lo), None)
if i_start is None:
    print(f"\nFAIL: never reached z>={hover_lo:.2f}m (peak={max(zs):+.3f}m)")
    sys.exit(0)
i_end=next((i for i in range(i_start,len(ts)) if ts[i]-ts[i_start]>=$HOVER_S), len(ts)-1)
x_h=xs[i_start:i_end+1]; y_h=ys[i_start:i_end+1]; z_h=zs[i_start:i_end+1]
x0,y0=x_h[0],y_h[0]
drifts=[math.hypot(x-x0,y-y0) for x,y in zip(x_h,y_h)]
print(f"\n--- hover window ({ts[i_end]-ts[i_start]:.1f}s) ---")
print(f"z mean: {sum(z_h)/len(z_h):+.3f}m  range [{min(z_h):+.3f},{max(z_h):+.3f}]")
print(f"xy drift mean={sum(drifts)/len(drifts)*100:.1f}cm max={max(drifts)*100:.1f}cm")
print(f"final xy: ({x_h[-1]-x0:+.3f}, {y_h[-1]-y0:+.3f}) m")
print(f"\nFLEW: YES (peak z={max(zs):+.3f}m)")
PYEOF

echo ""
echo "=== s104 SUMMARY ==="
cat $OUT/summary.txt
echo ""
echo "--- offboard log tail ---"
tail -15 $OUT/offboard.log
echo ""
echo "--- PX4 events ---"
grep -E "Takeoff|Land|land|Arm|Disarm|disarm|OFFBOARD|Offboard|offboard|Preflight|Compass|EKF|origin|home" $OUT/px4.log 2>/dev/null | head -10
echo ""
echo "[s104] artifacts in $OUT"
