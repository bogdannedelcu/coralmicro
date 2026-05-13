#!/bin/bash
# s127 FlowBaseline — tear down the SITL stack.
# Order: bridge → sentai_sim + fifo keeper → gz GUI → gz server → cf2 → Xvfb.
WORKDIR=/tmp/s127_flowbaseline

# distrobox-enter has shell-escape quirks; one pkill per call is safe.
distrobox enter crazysim-garden -- pkill -9 -f gz_to_uds_bridge 2>/dev/null || true
distrobox enter crazysim-garden -- pkill -9 -f 'gz sim'        2>/dev/null || true
distrobox enter crazysim-garden -- pkill -9 -f sitl_make       2>/dev/null || true
distrobox enter crazysim-garden -- pkill -9 Xvfb               2>/dev/null || true
echo "[stop] dbox cleanup done"

[ -f "$WORKDIR/sim.pid" ] && kill -9 "$(cat "$WORKDIR/sim.pid")" 2>/dev/null
pkill -9 -f "sleep infinity" 2>/dev/null
pkill -9 -f "tail -f /dev/null" 2>/dev/null

rm -f /tmp/sentai_cam.sock /tmp/sentai_flow_out.sock /tmp/sentai_sim_stdin.fifo
echo "[stop] host cleanup done"
