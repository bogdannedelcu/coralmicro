---
name: flowbaseline2-4markers-abort
description: "HARD RULE for FlowBaseline2 (s167 iter #5 onward, operator-stated 2026-05-18).  If for 30 CONSECUTIVE frames the camera does not see ALL 4 ArUco markers (n_dets < 4 per frame), the mission MUST abort and the drone MUST land.  Strict threshold — partial detection (1-3 markers) counts as 'lost' because VPE-anchored calibration requires the full set for clean PnP. Streak resets only at n_dets==4 (not at any detection)."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

## The rule

**Hard rule (operator 2026-05-18)**: in the FlowBaseline2 family of
missions (`s167_flowbaseline_calibrated` and any successor), if the
camera fails to detect ALL 4 ArUco markers for 30 consecutive frames,
the mission MUST trigger an abort and the drone MUST land.

- Trigger: `n_dets < 4`  (NOT `n_dets == 0` — anything less than 4
  counts as "lost the 4-marker set").
- Streak length: 30 consecutive failing frames (~4.5 s at 6.6 Hz frame
  dump cadence).
- Streak resets ONLY when `n_dets == 4`.  Partial detections (1-3
  markers) do not reset the counter — they still represent degraded
  PnP, not a healthy state.
- Action: SafetyMonitor raises SafetyAbort → main() catches → emergency
  land via `mc.land(velocity=LANDING_VEL_MPS)`.

## Why

VPE-anchored calibration (the whole point of FlowBaseline2) requires
clean PnP pose from all 4 markers.  With only 1-3 visible, PnP is
ambiguous (under-constrained) or biased (asymmetric coverage of the
known marker pad).  Continuing to fly without full FOV produces:

- Wrong VPE injections into cf2 EKF → EKF drifts.
- Wrong PnP ground-truth in F2-iter LSQ fit → BX calibration garbage.
- Drone flying blind → silent crash with EKF showing nonsense state.

s167 iter #4 (2026-05-18) had 14/16 F3 samples with `n_det=0` and 2/16
with `n_det=2`.  Drone never saw all 4 markers in F3.  Mission still
"passed" because hover_setpoint(0,0,0,z) damps via flow regardless of
VPE — but the calibration intent was vacuous.  Operator: "in
experimentul asta nu am vazut ca s-ar opri misiunea daca nu mai vede
4 markeri."  Rule shipped as response.

## Why streak (not instant abort)

Brief occlusions (1-2 frames) during transients are expected and
recoverable.  30 frames ≈ 4.5 s sustained loss = drone is genuinely
out of marker-pad FOV, not transient.

## Why `< 4` not `<= 2`

Operator's "pierdem cei 4 markeri" is explicit: the FOUR markers are
the calibration anchor as a set.  PnP with 3 markers is mathematically
under-determined for full 6-DoF pose (needs 4+ for unambiguous
solvePnP under planar layout).  Don't compromise.

## Implementation site

`mission_flowbaseline2.py::SafetyMonitor.update_pnp` — single
integration point hooked from `PnPCollector.poll`.  Event log inside
SafetyMonitor.events captures every state change for forensic review
in `phases.json::safety_state`.

## Cross-references

- `[[op-s8-w1-mission-safety-triggers]]` — broader safety trigger spec.
- `[[op-s8-w1-cf2-sim-honest]]` — parent WP.
- `[[test-must-be-relevant-to-claim]]` — directly related; a "PASS"
  with no markers visible is not relevant to "vision-anchored hover".
