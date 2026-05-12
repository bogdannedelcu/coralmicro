#!/bin/bash
# s095 — sentai ↔ PX4 MAVLink ping smoke test.
#
# Spawns PX4 SITL inside crazysim-garden distrobox + sentai_sim on host,
# drives `sentai.link` through a few rounds of heartbeat exchange, and
# reports stats.
#
# Exit code:
#   0 — heartbeat round-trip OK (sentai sent + received from PX4)
#   1 — only TX worked (PX4 didn't respond)
#   2 — TX failed entirely
#
# Ports (Sim.md §10m):
#   14540 → sentai bind
#   14580 → PX4 offboard listen

set -e

DISTROBOX_NAME=crazysim-garden
PX4_DIR=/home/bogdan/work/px4/PX4-Autopilot
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4
PX4_ETC=$PX4_DIR/build/px4_sitl_default/etc
SENTAI_BIN=/home/bogdan/work/coralmicro/build-sim/sim/sentai_sim

# Stamp output dir under /tmp + symlink for the bench's CSV.
STAMP=$(date +%Y%m%d_%H%M%S)
OUT_DIR=/tmp/sentai_px4_ping_$STAMP
mkdir -p "$OUT_DIR"
echo "[s095] output → $OUT_DIR"

if [ ! -x "$PX4_BIN" ]; then
    echo "ERROR: PX4 binary not found at $PX4_BIN"
    echo "       Run: bash sim/scripts/install_px4_sitl.sh + the build cmd"
    exit 2
fi
if [ ! -x "$SENTAI_BIN" ]; then
    echo "ERROR: sentai_sim not built — run cmake --build build-sim --target sentai_sim"
    exit 2
fi

# Clean ports — kill stale PX4 / sentai
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
pkill -9 -f sentai_sim 2>/dev/null || true
sleep 1

# 1. Start PX4 SITL with the "sihsim" simulator (lightweight, no Gazebo)
#    so we don't need a world.  PX4 still spins up mavlink on 14580/14540.
#    PX4_SYS_AUTOSTART=10040 = sihsim_quadx (see ROMFS/.../airframes/).
echo "[s095] starting PX4 SITL (sihsim_quadx, no gazebo) inside $DISTROBOX_NAME"
distrobox enter $DISTROBOX_NAME -- bash -c \
    "cd $PX4_DIR && PX4_SYS_AUTOSTART=10040 PX4_SIMULATOR=sihsim PX4_SIM_MODEL=quadx \
     $PX4_BIN -i 0 -d $PX4_ETC >$OUT_DIR/px4.log 2>&1 &"
PX4_WAIT=0
while [ $PX4_WAIT -lt 30 ]; do
    if grep -q "INFO  \[mavlink\] partner IP" "$OUT_DIR/px4.log" 2>/dev/null \
       || grep -q "Ready for takeoff" "$OUT_DIR/px4.log" 2>/dev/null \
       || grep -q "mavlink#" "$OUT_DIR/px4.log" 2>/dev/null; then
        echo "[s095] PX4 boot seen after ${PX4_WAIT}s"
        break
    fi
    sleep 1
    PX4_WAIT=$((PX4_WAIT+1))
done
sleep 2

# 2. Drive sentai_sim through the REPL non-interactively.
echo "[s095] driving sentai_sim REPL → ping test"
timeout 25 $SENTAI_BIN >$OUT_DIR/sentai.log 2>&1 <<'EOF'
import sentai
sentai.link.debug(1)
print("INIT", sentai.link.init())
sentai.link.heartbeat(); sentai.rtos.sleep_ms(500); sentai.link.heartbeat(); sentai.rtos.sleep_ms(1500)
sentai.link.heartbeat(); sentai.rtos.sleep_ms(1500); sentai.link.heartbeat(); sentai.rtos.sleep_ms(1500)
sentai.link.heartbeat(); sentai.rtos.sleep_ms(3000)
s = sentai.link.stats()
print("RESULT tx_hb=%d rx_hb=%d peer_sys=%d" % (s[0], s[3], s[6]))
sentai.link.stop()
EOF

# 3. Cleanup PX4 — fire and forget.
distrobox enter $DISTROBOX_NAME -- pkill -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true

# 4. Report
echo ""
echo "=== sentai output (last 20 lines) ==="
tail -20 $OUT_DIR/sentai.log
echo ""
echo "=== PX4 mavlink hits (grep) ==="
grep -E "mavlink|partner" $OUT_DIR/px4.log 2>/dev/null | head -5

# 5. Exit code based on RESULT line
RESULT_LINE=$(grep "RESULT" $OUT_DIR/sentai.log | tail -1)
TX=$(echo "$RESULT_LINE" | grep -oE 'tx_hb=[0-9]+' | cut -d= -f2)
RX=$(echo "$RESULT_LINE" | grep -oE 'rx_hb=[0-9]+' | cut -d= -f2)
TX=${TX:-0}; RX=${RX:-0}
echo ""
echo "Summary: TX=$TX  RX=$RX"
if [ "$RX" -gt 0 ]; then
    echo "[s095] PASS — round-trip heartbeat with PX4"
    exit 0
elif [ "$TX" -gt 0 ]; then
    echo "[s095] PARTIAL — sentai sent OK but no PX4 reply (check ports)"
    exit 1
else
    echo "[s095] FAIL — sentai sent nothing"
    exit 2
fi
