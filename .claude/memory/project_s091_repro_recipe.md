---
name: s091 reproduction recipe (proven 2026-05-13)
description: Stack + world config that reproduces s091 #14 cf2 ArUco hover (sub-10cm drift, 100% all-4 marker visibility) at commit 180bbb5f
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
s091 #14 sentai.flow hover-over-ArUco reproduced 2026-05-13 on branch
`restore/pre-aruco-stack-cf2-sim` (commit 180bbb5f, 2026-05-12 19:22).
Drone hovers perfectly centered over the 4 ArUco markers at z=1m,
dist_mean 4.8-9.7 cm, all-4 visible 100%, flow_hz 19-26.

**Why:** baseline known-good stack for cf2 + sentai.flow demos and as
the reference point any future regression must beat.

**How to apply:**

State requirements (load-bearing):
- coralmicro: branch `restore/pre-aruco-stack-cf2-sim` at 180bbb5f
- CrazySim `crazyflie-simulation` submodule: world+model checked out from
  commit `aeb7ee6` ("realistic disturbance — 1% motor + IMU noise + moderate wind")
- CrazySim cf2 firmware: branch `sentai-flow-sim-support` at `e4374251`
  (17-byte SENSOR_FLOW_SIM packet with stdDev field)
- World wind: HALVED from aeb7ee6 default (0.2→0.1 m/s base, σ horiz
  0.15→0.075, σ dir 5°→2.5°, σ vert 0.08→0.04). Full wind = 32cm drift,
  half wind = 7.6cm drift baseline.
- Cat photo: z=0.005 (right on ground; aeb7ee6 default z=0.05 is fine too).

Pipeline launch order (load-bearing):
1. distrobox crazysim-garden: Xvfb :99 → gz sim -s on DISPLAY=:99 →
   spawn cf2 → gz sim -g with `sim/gazebo/sentai_gui.config` on real DISPLAY.
   **Without Xvfb the server-side camera sensor produces bit-identical
   frames (Sim.md §10d render-thread starvation).**
2. host: sentai_sim with **fifo stdin** (/dev/null EOFs REPL → exit) +
   env `SENTAI_DUMP_FRAMES_DIR` (must be on the exec, not on a pipe head).
3. distrobox: gz_to_uds_bridge connects to /tmp/sentai_cam.sock + creates
   /tmp/sentai_flow_out.sock.
4. host: aruco_hover.py with SENTAI_DUMP_FRAMES_DIR env set (script
   asserts dir exists; venv must have cflib + cv2 — works with
   /home/bogdan/work/coralmicro/venv/bin/python3, NOT venv-coral).

Launcher script: /tmp/s091_repro/launch_hybrid.sh (Xvfb + cf2 + GUI;
runs inside distrobox). Set DISPLAY=:99 ONLY for `gz sim -s`, keep real
DISPLAY for `gz sim -g`.

Gotchas:
- `gz sim -g` blocks the launcher; closing the GUI window triggers
  cleanup() which kills cf2 + Xvfb + gz server. Don't close GUI mid-test.
- Stale `[bridge] gz_to_uds_bridge` wrappers under distrobox accumulate
  across restarts; kill them by exact PID, not via `pkill` (distrobox
  shell escapes break pkill matching).
- aruco_hover.py reads `SENTAI_DUMP_FRAMES_DIR` env; if missing it
  KeyErrors out before MotionCommander takeoff. Always pass via `env`
  on the python invocation.
- cf2 firmware param `posCtlPid.xyKd` not in TOC (Sim.md §10l mentions
  it but `sentai-flow-sim-support` branch doesn't expose it). Other
  DAMP_PARAMS apply OK; ignore the WARN.
