#!/bin/bash
# s110 — Wind sweep on PX4 SITL to find max wind drone can hover under.
# Uses s109 setup (airframe 4043 GPS+VPE fused, stress factors active).
# Iterates wind 0.2 → 3.0 m/s, records drift per run.
#
# Wind value is edited inline in the world SDF before each run.
# (cf2 s091 baseline at 0.2 m/s: 32cm drift; here we want to find
#  where PX4 x500_sentai breaks.)

set -e

DISTROBOX_NAME=crazysim-garden
COR=/home/bogdan/work/coralmicro
WORLD_PATH=/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_crazysim.sdf

WINDS="${WINDS:-0.2 0.5 1.0 2.0 3.0}"

STAMP=$(date +%Y%m%d_%H%M%S)
SWEEP_DIR=/tmp/sentai_s110_$STAMP
mkdir -p "$SWEEP_DIR"
echo "[s110] sweep dir → $SWEEP_DIR"
echo "wind_mps,drift_mean_cm,drift_max_cm,peak_z,hover_z_mean,flew" > $SWEEP_DIR/sweep.csv

for W in $WINDS; do
    echo ""
    echo "=========================================="
    echo "[s110] wind = $W m/s"
    echo "=========================================="

    # Edit wind in world SDF (replace any <linear_velocity>X 0 0</linear_velocity>)
    sed -i -E "s|<linear_velocity>[0-9.]+ 0 0</linear_velocity>|<linear_velocity>$W 0 0</linear_velocity>|" "$WORLD_PATH"
    grep linear_velocity "$WORLD_PATH" | head -1

    # Run s109 once.
    bash /home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/s109_px4_aruco_wind/run.sh > $SWEEP_DIR/run_w${W}.log 2>&1

    # Extract drift from the run output.
    drift_mean=$(grep "drift mean=" $SWEEP_DIR/run_w${W}.log | sed -E 's/.*drift mean=([0-9.]+)cm.*/\1/' | head -1)
    drift_max=$(grep "drift max=" $SWEEP_DIR/run_w${W}.log | sed -E 's/.*max=([0-9.]+)cm.*/\1/' | head -1)
    peak_z=$(grep "peak z=" $SWEEP_DIR/run_w${W}.log | sed -E 's/.*peak z=([0-9.+-]+)m.*/\1/' | head -1)
    hover_z=$(grep "z mean:" $SWEEP_DIR/run_w${W}.log | sed -E 's/.*z mean:[[:space:]]*([0-9.+-]+)m.*/\1/' | head -1)
    flew=$(grep "FLEW:" $SWEEP_DIR/run_w${W}.log | head -1 | grep -oE "YES|NO" || echo "NO")
    drift_mean=${drift_mean:-N/A}
    drift_max=${drift_max:-N/A}
    peak_z=${peak_z:-N/A}
    hover_z=${hover_z:-N/A}
    echo "$W,$drift_mean,$drift_max,$peak_z,$hover_z,$flew" >> $SWEEP_DIR/sweep.csv
    echo "[s110] wind=$W drift_mean=${drift_mean}cm drift_max=${drift_max}cm flew=$flew"

    # Cleanup processes between runs.
    distrobox enter $DISTROBOX_NAME -- /tmp/_full_clean.sh 2>/dev/null || true
    pkill -9 -f aruco_to_vision 2>/dev/null || true
    sleep 3
done

# Reset wind to 0 after sweep.
sed -i -E "s|<linear_velocity>[0-9.]+ 0 0</linear_velocity>|<linear_velocity>0 0 0</linear_velocity>|" "$WORLD_PATH"

echo ""
echo "=========================================="
echo "[s110] SWEEP COMPLETE"
echo "=========================================="
cat $SWEEP_DIR/sweep.csv | column -t -s,
echo ""
echo "[s110] artifacts in $SWEEP_DIR"
