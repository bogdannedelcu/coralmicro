---
name: No Gazebo Harmonic — use Garden 7.9 only
description: User dropped Harmonic for SentAI SIM Phase 4; Garden in distrobox is the only Gazebo target. Don't propose Harmonic again.
type: feedback
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
For Phase 4 (camera socket → Gazebo image bridge → cf2 SITL), the user has
chosen Gazebo Garden 7.9 (running inside the `crazysim-garden` distrobox)
as the SOLE Gazebo target. Do not suggest, install, or fall back to Gazebo
Harmonic 8.x.

**Why:** Harmonic broke CrazySim cf2 sensor calibration (Issue #17 on
gtfactslab/CrazySim — `STAB: Wait for sensor calibration...` infinite
loop) which we hit in this session. Garden 7.9 + the local socketlink
ASSERT fix is the verified-working stack. Switching back to Harmonic
re-introduces a known dead-end.

**How to apply:**
- Use `gz sim` from inside `distrobox enter crazysim-garden`, NEVER from
  the host (host has Harmonic).
- For Python bindings, use `gz.transport12` / `gz.msgs9` (Garden), not
  `gz.transport13` / `gz.msgs10` (Harmonic).
- The host-side `gz_to_camera_bridge.py` script must adapt to Garden:
  edit the import to `from gz.transport12 import Node` and
  `from gz.msgs9.image_pb2 import Image` once the Garden Python bindings
  are installed (host or via PYTHONPATH into distrobox).
- The CrazySim plugin auto-detects Garden vs Harmonic at build time;
  rebuild inside the distrobox so it links against `gz-sim7`.
- `sim/scripts/install_gazebo_harmonic.sh` should NOT be run on a fresh
  machine for Phase 4 work; treat it as legacy/aside and prefer the
  distrobox path documented in `sim/scripts/install_crazysim.sh`.

This rule supersedes any earlier note that said "Harmonic also works" or
"plugin auto-detects gz-sim8". Garden only, full stop.
