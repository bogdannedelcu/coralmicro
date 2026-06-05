---
name: gazebo-gui-required
description: Operator wants to watch every Gazebo SIM experiment visually — headless runs are invalid even when the verdict script returns PASS. Reaffirmed 2026-05-14 during L3 wrap-up.
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ef599a0d-1126-4214-964d-31fac8efaa48
---

For ANY Gazebo SIM experiment (Phase 4 closed-loop, FlowBaseline gate,
upcoming s128 ArUco->world, future SLAM/explore validation), launch
`gz sim -g --gui-config sim/gazebo/sentai_gui.config` alongside the
headless server and confirm the GUI window is visible BEFORE running
the test script.

**Why:** the operator wants to see pose drift, attitude oscillation,
camera framing, marker visibility, ground-plane hits, etc.  Numeric
log values from `stateEstimate.x/y/z` hide whether the world rendered
correctly, whether the camera plugin survived, whether physics didn't
silently crash.  A "PASS" verdict from a headless run is unverifiable
and treated as invalid.

**How to apply:** every script that touches Gazebo Garden + cf2 SITL +
the flow bridge MUST start the GUI; do not proceed past "stack ready"
until the window is visible.  When automating from CI later, this rule
relaxes — but in interactive operator-driven sessions, GUI is mandatory.

Reference: Sim.md §10c Rule 1 (added 2026-05-10, reaffirmed 2026-05-14).
The `sentai_gui.config` ships an ImageDisplay widget on
`/downward_cam/image` so the SentAI camera frame is visible PiP-style.
