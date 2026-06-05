---
name: kill-all-gz-before-launch
description: "HARD RULE — every new Gazebo SIM experiment MUST start by killing any existing gz sim / cf2 / camera-bridge process AND closing every leftover GUI window. Operator-reaffirmed 2026-05-20 after the s182 first launch reused a stale GUI that was still rendering the previous (broken) texture state."
metadata:
  node_type: memory
  type: feedback
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

## The rule

Before launching any Gazebo SIM experiment, KILL:

  - every host `gz sim` process (`pkill -9 -f "gz sim"`)
  - every distrobox `gz sim` / `gz topic` process
    (`distrobox enter crazysim-garden -- pkill -9 -f "gz sim|gz topic"`)
  - every `cf2` SITL binary (`pkill -9 -f "cf2 19950"`)
  - every camera-bridge process (`pkill -9 -f "gz_to_camera_bridge"`)
  - every Xvfb on `:99` (`pkill -9 -f "Xvfb :99"`)
  - every existing GUI WINDOW the operator might have open from a
    prior session — even if the wrapper script has exited, the GUI
    window keeps rendering the LAST snapshot it received from the
    server.  Asking "what do you see?" on a stale window gives
    misleading feedback.

Then sleep ~2 s for the FAT/UDS sockets to close before relaunching.

## Why

Operator caught this 2026-05-20 during s182 launch:

  - First launch had a broken texture path → GUI rendered Krajník
    markers as white squares (fallback when texture file missing).
  - I fixed the texture path AND restarted the launch, but the
    OLD GUI window was still open with the broken-state snapshot.
  - Operator looked at the old window and reported "tot pătrate
    albe" — even though the fresh server WAS rendering circles
    correctly.

The 30 s of waste came from interpreting that visual feedback as
"the fix didn't work" rather than "the operator is looking at the
wrong window."

## How to apply

Every `run.sh` for a SIM experiment must start with a `cleanup_all()`
function that kills the full process list above, PLUS the readme/
comment must remind the operator to manually close any GUI window
they have lingering from prior sessions.

For interactive launches initiated by the assistant: announce the
kill list explicitly so the operator knows to dismiss their old GUI
window before looking at the new one.

## Counter-examples to avoid

  - "GUI is still up from before, I'll just reload the scene" —
    no.  Stale shaders + stale resource caches.  Always restart.
  - "I killed gz sim on the host but not in the distrobox" — the
    distrobox `gz sim -s` server is a separate PID; without
    `distrobox -- pkill gz sim` you leave a half-up stack.

## Cross-refs

  - `[[experiments-start-from-origin]]` — every experiment cf2
    respawned at origin (canonical companion rule).
  - `[[sim-test-must-return-home]]` — closure verdict gate.
  - `[[gazebo-gui-required]]` — GUI mandatory (the visual part of
    this rule's enforcement).
  - Sim.md §3 prep — distrobox setup.
