---
name: op-s10-w21-t4-session2-open
description: "Carry-over from 2026-05-21 night session — open T5..T9 for s187 calib bringup, sim-side patches uncommitted, blocking decision pending."
metadata: 
  node_type: memory
  type: project
  originSessionId: 8125b1d3-641a-4811-82a9-49cb58fa0d39
---

OP-S10-W21-T4 phase-2 NOT closed — Session 2 (2026-05-21 night, iter 59→89)
ground to halt on cf2 SITL no-baro hover instability.

**Tasks open (carry into next session)**:
- T5: PMW3901 optical flow emulation in `crazysim_plugin.cpp`
  (~50 LoC) OR re-enable full ExtPose forward in OdomCallback
  (5 min; needs operator anti-cheat override).  Either fixes cf2
  XY drift during SAMPLE.
- T6: Reduce cf2 takeoff overshoot 0.95m vs 0.6m target (PID_POS_Z
  tuning or `takeoff_with_velocity`).
- T7: Consolidate MP-side extPos sender into orchestrator C side.
- T8: Reconcile n>=4 (climb_seen) vs n>=6 (bringup SAMPLE) gates.
- T9: Cam_offset Kabsch accuracy gap (best 5.4cm vs 5mm gate).
  Root cause not isolated.

**Uncommitted sim-side patches** (left in WT):
1. `crazysim_plugin.cpp` OdomCallback → forwards z as SENSOR_TOF_SIM.
2. `model.sdf.jinja` re-enabled `gz-sim-odometry-publisher-system`.
3. `estimator_kalman.c` `KALMAN_USE_BARO_UPDATE` commented out.
4. `sentai_whycon_small.sdf` — new 0.5× scaled pad.
5. `mission_s187.py` — full takeoff rewrite (TOF + climb_seen + extPos).

**Operator decision pending**:
- (A) re-enable full ExtPose injection (overrides 2026-05-17 anti-cheat),
- (B) implement PMW3901 emulation realistic,
- (C) accept SIM cf2 hover limits, validate calib algorithm-only.

**Why:** 12h sunk Session 2.  cf2 SITL fundamentally needs a
positioning system at flight time (real HW has flow_deck);
SentAI nav stays visual-only.  Blocker is plugin emulation
completeness, not SentAI code.

**How to apply:** next session start, ask operator A/B/C decision
first.  Then resume by `cd build && ./apply_changes_resume.sh`
(or manually unstage the 5 uncommitted patches above).  Best
end-to-end result so far: iter-87 — SAMPLE phase ran 3.5s before
SAFETY abort due to X drift 0.46m.
