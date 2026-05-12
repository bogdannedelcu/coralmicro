#!/bin/bash
# s100 — Phase 6d step 0: x500_sentai spawns into the *canonical*
# sentai_crazysim.sdf world (the one used by s091 wind tests, vendored
# in CrazySim) and its downward cam publishes frames.  GUI + PiP per
# Sim.md §10c.  Apples-to-apples reuse of the cf2 world so we can
# later compare PX4 hover-stability numbers vs s091.

set -e

DISTROBOX_NAME=crazysim-garden
PX4_DIR=/home/bogdan/work/px4/PX4-Autopilot
PX4_BIN=$PX4_DIR/build/px4_sitl_default/bin/px4
PX4_ETC=$PX4_DIR/build/px4_sitl_default/etc
GZ_X500_MODELS=$PX4_DIR/Tools/simulation/gz/models

# Canonical s091 world — lives in CrazySim vendor tree (has 4 ArUco
# markers id0..3 + 6 color cubes + cat + checkerboard + wind plugin).
CRAZYSIM_GZ=/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo
WORLD_FILE=$CRAZYSIM_GZ/worlds/sentai_crazysim.sdf
WORLD_NAME=sentai_crazysim   # MUST match <world name="..."> inside SDF
CRAZYSIM_RES=$CRAZYSIM_GZ/worlds   # materials/textures live here

GUI_CONFIG=/home/bogdan/work/coralmicro/sim/gazebo/sentai_gui.config

STAMP=$(date +%Y%m%d_%H%M%S)
OUT=/tmp/sentai_s100_$STAMP
mkdir -p "$OUT"
echo "[s100] artifacts → $OUT"
echo "[s100] world: $WORLD_FILE"

# 0. Cleanup
cat >$OUT/_cleanup.sh <<'EOF'
#!/bin/bash
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
true
EOF
chmod +x $OUT/_cleanup.sh
pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null || true
distrobox enter $DISTROBOX_NAME -- $OUT/_cleanup.sh || true
sleep 1

# 1. Launch gz with GUI (Garden 7.9 inside distrobox).
cat >$OUT/_launch_gz.sh <<EOF
#!/bin/bash
export GZ_SIM_RESOURCE_PATH="$CRAZYSIM_RES:$GZ_X500_MODELS"
cd "$CRAZYSIM_RES"
# -r = run on start, --gui-config = PiP layout, no -s (GUI stays).
nohup gz sim --verbose=1 -r --gui-config "$GUI_CONFIG" "$WORLD_FILE" \
    > "$OUT/gz.log" 2>&1 &
disown
echo \$! > "$OUT/gz.pid"
EOF
chmod +x $OUT/_launch_gz.sh
echo "[s100] launching gz GUI"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_gz.sh

# Wait for gz world clock topic.
GZ_OK=0
for i in $(seq 1 60); do
    sleep 1
    if distrobox enter $DISTROBOX_NAME -- bash -c "gz topic -l 2>/dev/null | grep -q /world/$WORLD_NAME/clock"; then
        echo "[s100] gz boot OK after ${i}s"
        GZ_OK=1
        break
    fi
done
if [ $GZ_OK -eq 0 ]; then
    echo "[s100] gz boot TIMEOUT — tail of gz.log:"
    tail -30 $OUT/gz.log
    exit 2
fi

# 2. PX4 SITL — spawn x500_sentai into the running gz world.
cat >$OUT/_launch_px4.sh <<EOF
#!/bin/bash
export GZ_SIM_RESOURCE_PATH="$CRAZYSIM_RES:$GZ_X500_MODELS"
export PX4_GZ_MODELS="$GZ_X500_MODELS"
cd "$PX4_DIR"
nohup env PX4_SYS_AUTOSTART=4001 \
          PX4_SIMULATOR=gz \
          PX4_GZ_MODEL=x500_sentai \
          PX4_GZ_WORLD=$WORLD_NAME \
          PX4_GZ_MODEL_POSE='0,0,0.2,0,0,0' \
          "$PX4_BIN" -i 0 -d "$PX4_ETC" > "$OUT/px4.log" 2>&1 &
disown
echo \$! > "$OUT/px4.pid"
EOF
chmod +x $OUT/_launch_px4.sh
echo "[s100] launching PX4 + x500_sentai (gz already running with $WORLD_NAME)"
distrobox enter $DISTROBOX_NAME -- $OUT/_launch_px4.sh

PX4_OK=0
for i in $(seq 1 60); do
    sleep 1
    if grep -qE "Ready for takeoff|Startup script returned successfully|All preflight checks passed" $OUT/px4.log 2>/dev/null; then
        echo "[s100] PX4 boot OK after ${i}s"
        PX4_OK=1
        break
    fi
done
if [ $PX4_OK -eq 0 ]; then
    echo "[s100] PX4 boot TIMEOUT — tail:"
    tail -40 $OUT/px4.log
fi

# 3. Inspect topics + camera.
cat >$OUT/_check.sh <<EOF
#!/bin/bash
gz topic -l 2>/dev/null | sort > "$OUT/topics.txt"
EOF
chmod +x $OUT/_check.sh
distrobox enter $DISTROBOX_NAME -- $OUT/_check.sh
echo ""
echo "[s100] gz topics (camera/world/x500 related):"
grep -E "/world/|cam|image|x500|aruco" $OUT/topics.txt | head -30

TOPIC_HIT=$(grep -c "/downward_cam/image" $OUT/topics.txt || true)
echo ""
if [ "$TOPIC_HIT" -gt 0 ]; then
    echo "[s100] /downward_cam/image present — sampling 3 frames..."
    cat >$OUT/_sample.sh <<EOF
#!/bin/bash
timeout 6 gz topic -e -t /downward_cam/image -n 3 > "$OUT/frame_dump.txt" 2>&1 || true
EOF
    chmod +x $OUT/_sample.sh
    distrobox enter $DISTROBOX_NAME -- $OUT/_sample.sh
    BYTES=$(wc -c < $OUT/frame_dump.txt 2>/dev/null || echo 0)
    echo "[s100] frame_dump.txt: $BYTES B"
else
    echo "[s100] /downward_cam/image NOT in topic list."
    echo "  candidates:"
    grep -iE "cam|image|x500" $OUT/topics.txt | head
fi

# 4. Report
echo ""
echo "=== s100 RESULTS ==="
PASS=1
[ $GZ_OK -eq 1 ] && echo "[OK]  gz running ($WORLD_NAME)" || { echo "[FAIL] gz boot"; PASS=0; }
[ $PX4_OK -eq 1 ] && echo "[OK]  PX4 booted" || { echo "[FAIL] PX4 boot"; PASS=0; }
if [ "$TOPIC_HIT" -gt 0 ]; then echo "[OK]  /downward_cam/image present"; else echo "[FAIL] camera topic missing"; PASS=0; fi
BYTES=$(wc -c < $OUT/frame_dump.txt 2>/dev/null || echo 0)
if [ "$BYTES" -gt 1000 ]; then echo "[OK]  camera publishes ($BYTES B)"; else echo "[FAIL] no camera bytes"; PASS=0; fi
# Bonus: verify ArUco models present in topics (means world loaded fully).
ARUCO_HIT=$(grep -c "aruco_id" $OUT/topics.txt || true)
if [ "$ARUCO_HIT" -gt 0 ]; then echo "[OK]  ArUco markers loaded ($ARUCO_HIT topics)"; else echo "[WARN] ArUco markers not visible in topics"; fi
echo ""
[ $PASS -eq 1 ] && echo "PASS" || echo "FAIL"
echo "[s100] artifacts in $OUT  (gz GUI left running — close window or run cleanup)"
