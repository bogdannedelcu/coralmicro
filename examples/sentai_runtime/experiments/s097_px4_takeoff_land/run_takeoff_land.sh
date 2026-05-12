#!/bin/bash
# s097 — Decolare/aterizare PX4 SITL driven from the sentai_sim REPL.
#
# Reproduces the working pattern from s091 (MotionCommander → cflib)
# but on PX4 SITL via sentai.link.* MAVLink wrappers.  No GPS — PX4
# sihsim simulator provides altitude (baro) + attitude (IMU), drone
# hovers at requested altitude but drifts in X/Y (no flow yet — that
# comes in s098).

set -e

DISTROBOX_NAME=crazysim-garden
PX4_DIR=/home/bogdan/work/px4/PX4-Autopilot
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4
PX4_ETC=$PX4_DIR/build/px4_sitl_default/etc
SENTAI_BIN=/home/bogdan/work/coralmicro/build-sim/sim/sentai_sim

STAMP=$(date +%Y%m%d_%H%M%S)
OUT_DIR=/tmp/sentai_takeoffland_$STAMP
mkdir -p "$OUT_DIR"
echo "[s097] output → $OUT_DIR"

# 0. Cleanup
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -9 -f sentai_sim 2>/dev/null || true
distrobox enter $DISTROBOX_NAME -- pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
rm -f /tmp/sim_repl_fifo
sleep 1

# 1. Start PX4 SITL with sihsim_quadx airframe (no Gazebo, no GPS).
#    PX4 EKF2 in SITL still consumes barometer + accelerometer + gyro
#    so altitude hold works.  Position estimate WILL drift (no GPS, no
#    optical flow yet).
echo "[s097] starting PX4 SITL inside $DISTROBOX_NAME"
distrobox enter $DISTROBOX_NAME -- bash -c \
    "cd $PX4_DIR && nohup env PX4_SYS_AUTOSTART=10040 PX4_SIMULATOR=sihsim \
     PX4_SIM_MODEL=quadx $PX4_BIN -i 0 -d $PX4_ETC > $OUT_DIR/px4.log 2>&1 & disown"
PX4_BOOT_OK=0
for i in $(seq 1 30); do
    if grep -q "Ready for takeoff\|All preflight checks passed\|Startup script returned successfully" \
            $OUT_DIR/px4.log 2>/dev/null; then
        echo "[s097] PX4 boot OK after ${i}s"
        PX4_BOOT_OK=1
        break
    fi
    sleep 1
done
if [ $PX4_BOOT_OK -eq 0 ]; then
    echo "[s097] PX4 boot timeout — log tail:"
    tail -10 $OUT_DIR/px4.log
    exit 2
fi
sleep 2

# 2. Start sentai_sim with a fifo for live REPL injection.
mkfifo /tmp/sim_repl_fifo
nohup $SENTAI_BIN < /tmp/sim_repl_fifo > $OUT_DIR/sentai.log 2>&1 &
SIM_PID=$!
disown
( tail -f /dev/null > /tmp/sim_repl_fifo ) &
TAIL_PID=$!
disown
echo "[s097] sentai_sim pid=$SIM_PID (fifo keeper pid=$TAIL_PID)"
sleep 2

# 3. Drive the experiment via the fifo — pseudo-REPL session.
inject() {
    echo "[s097][repl] $1"
    echo "$1" > /tmp/sim_repl_fifo
    sleep "${2:-0.4}"
}

inject "import sentai"  0.3
inject "sentai.link.debug(1)" 0.3
inject "sentai.link.init()"   1.0
inject "sentai.link.heartbeat()" 0.3
inject "sentai.link.heartbeat()" 1.0   # let PX4 see two heartbeats first

# Arm + takeoff to 1 m.
inject "print('--- arming + takeoff to 1.0m ---')" 0.2
inject "sentai.link.arm(1)" 0.5
inject "sentai.link.takeoff(1.0)" 5.0   # let drone climb

# Hover 5 s (no flow → drone holds altitude via baro, drifts XY).
inject "print('--- hover ---')" 0.2
inject "sentai.rtos.sleep_ms(5000)" 5.5

# Land.
inject "print('--- landing ---')" 0.2
inject "sentai.link.land()" 5.0

# Disarm.
inject "sentai.link.arm(0)" 0.5
inject "print('--- stats ---')" 0.2
inject "print(sentai.link.stats())" 0.5

# 4. Tear down.
sleep 2
kill $TAIL_PID 2>/dev/null || true
kill $SIM_PID 2>/dev/null || true
distrobox enter $DISTROBOX_NAME -- pkill -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
rm -f /tmp/sim_repl_fifo

# 5. Report — pull altitude trace from PX4 log via vehicle_local_position
#    (PX4 already logs ulog; for MVP just grep "Land detected").
echo ""
echo "=== sentai REPL transcript ==="
tail -40 $OUT_DIR/sentai.log
echo ""
echo "=== PX4 mavlink hits ==="
grep -E "Takeoff|takeoff|Land|land|Arm|arm" $OUT_DIR/px4.log 2>/dev/null | head -15
echo ""
echo "[s097] done — artifacts in $OUT_DIR"
