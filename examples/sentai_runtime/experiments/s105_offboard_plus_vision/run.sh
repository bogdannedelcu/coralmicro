#!/bin/bash
# s105 — Hybrid path A + B-mock:
#   - OFFBOARD setpoints (s104 path) bypass PX4 arm-gate
#   - VISION_POSITION_ESTIMATE from gz ground truth (s102 bridge)
#     gives EKF a position anchor so attitude stays valid during hover
#   - No GPS, no flow (yet)
#
# Validates end-to-end "fly without GPS" using both A + B together.
# Real flow (s104b) can replace the vision-pose mock once the lockstep
# vs phase-corr cadence issue is resolved.

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
OUT=/tmp/sentai_s105_$STAMP
mkdir -p "$OUT"
echo "[s105] artifacts → $OUT"

cat >$OUT/_cleanup.sh <<'EOF'
#!/bin/bash
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f gz_pose 2>/dev/null
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null
pkill -9 -f offboard_hover 2>/dev/null
true
EOF
chmod +x $OUT/_cleanup.sh
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true
sleep 1

# 1. PX4 (airframe 4042 — external vision pos, no GPS).
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
echo "[s105] launching PX4 (airframe 4042 = external vision, no GPS)"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_px4.sh
for i in $(seq 1 60); do
    sleep 1
    if grep -qE "Ready for takeoff|Startup script returned successfully" $OUT/px4.log 2>/dev/null; then
        echo "[s105] PX4 boot OK (${i}s)"; break
    fi
    [ $i -eq 60 ] && { echo "[s105] PX4 TIMEOUT"; tail -30 $OUT/px4.log; exit 2; }
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

# 2. Pose logger (ground truth recording).
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

# 3. Vision bridge (gz pose → VISION_POSITION_ESTIMATE).
cat >$OUT/_launch_vision.sh <<EOF
#!/bin/bash
nohup bash -c "gz topic -e -t /world/$WORLD_NAME/dynamic_pose/info 2>$OUT/vision.gzerr | \
    python3 $COR/sim/scripts/gz_pose_to_vision_estimate.py \
        --model x500_sentai_0 \
        --mav udpout:127.0.0.1:18570 \
        --rate 30" \
    > $OUT/vision.log 2>&1 &
disown
EOF
chmod +x $OUT/_launch_vision.sh
echo "[s105] launching vision-pose bridge"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_vision.sh
sleep 3

# 4. Set fake GPS origin (helps PX4 establish home).
cat >$OUT/_set_origin.sh <<EOF
#!/bin/bash
python3 /tmp/_set_origin.py > $OUT/origin.log 2>&1
EOF
chmod +x $OUT/_set_origin.sh
distrobox enter $DISTROBOX_NAME -- $OUT/_set_origin.sh
sleep 1

# 5. EKF convergence window (vision flowing 5s).
echo "[s105] waiting 5s for EKF vision convergence..."
sleep 5

# 6. OFFBOARD hover (Python in distrobox, port 14550 to avoid clash).
cat >$OUT/_launch_offboard.sh <<EOF
#!/bin/bash
python3 $COR/examples/sentai_runtime/experiments/s104_px4_offboard/offboard_hover.py \
    --target-z $TARGET_Z --hover-s $HOVER_S --land-s 8 \
    > $OUT/offboard.log 2>&1
EOF
chmod +x $OUT/_launch_offboard.sh
echo "[s105] running OFFBOARD mission..."
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_offboard.sh

# 7. Teardown.
sleep 2
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true

# 8. Summary.
python3 - <<PYEOF >$OUT/summary.txt 2>&1
import csv, math, os, sys
CSV="$OUT/pose.csv"
if not os.path.exists(CSV): print("FAIL: pose.csv missing"); sys.exit(1)
rows=[]
with open(CSV) as f:
    for r in csv.DictReader(f):
        try: rows.append((float(r["t_s"]),float(r["x_m"]),float(r["y_m"]),float(r["z_m"])))
        except: continue
if not rows: print("FAIL: empty"); sys.exit(1)
ts=[r[0] for r in rows]; xs=[r[1] for r in rows]; ys=[r[2] for r in rows]; zs=[r[3] for r in rows]
target=$TARGET_Z
print(f"=== s105 (OFFBOARD + mock vision, no GPS) ===")
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
echo "=== s105 SUMMARY ==="
cat $OUT/summary.txt
echo ""
echo "--- offboard log tail ---"
tail -10 $OUT/offboard.log
echo ""
echo "--- vision tx count ---"
grep "tx=" $OUT/vision.log | tail -2
echo ""
echo "--- PX4 events ---"
grep -E "Takeoff|Land|Arm|Disarm|OFFBOARD|Preflight|EKF|home" $OUT/px4.log 2>/dev/null | head -10
echo ""
echo "[s105] artifacts in $OUT"
