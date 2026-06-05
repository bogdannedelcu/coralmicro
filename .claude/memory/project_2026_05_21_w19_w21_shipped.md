---
name: 2026-05-21-w19-w21-shipped
description: "2026-05-21 day arc — closed W19-T6b + W20 refactor; opened W21 calib bringup, shipped T1..T4+T6, s187 phase-2 reproducible PASS on original pad."
metadata: 
  node_type: memory
  type: project
  originSessionId: bb5c42a5-74ed-4dd6-ae23-edcb11ac03e3
---

Big day, 11 commits.  Closes OP-S10-W19 host-side + opens + largely
ships OP-S10-W21 production calib bringup.  End-to-end pipeline
reproducibly green in SIM on the original WhyCon pad.

**Shipped**:

- W19-T6b: drone-pose Kabsch + yaw-anchor picker in
  `sentai_markers.{h,cc}` + MP bindings.  s184 smoke 6/6 PASS — covers
  all 3 SVD-ambiguity sources (permutation, Z-reflection, yaw mirror).
  Commit `631db542`.
- W20: Kabsch/SVD/Jacobi extracted to shared `sentai_svd3.{h,cc}`.
  s157 bit-identical pre/post.  `f1bc9850`.
- W21-T1..T4 + T6:
  - T1 markers refactor in calib_task — `13b6d7be`
  - T2 INI persistence schema v2 (replaces JSON) — `57a1716e`
  - T3 kp_x/y/yaw persist in calib.ini — `deb20f7b`
  - T4 `sentai_calib_run_bringup()` orchestrator, ~350 LoC, 7-phase FSM
    (SAMPLE → KABSCH → AUTOTUNE_X → AUTOTUNE_Y → HOLD → SAVE → DONE).
    `29a9f92f`.
  - T6 `SUBSYS_CALIB` + `assert_calibrated()` takeoff guard.  `8454b713`.
- s187 phase-2 Session 1: Kabsch reproducibly PASSES both gates on
  original pad after THRESH_C=30 + autotune VPE/canal fixes + `clear()`
  at mission setup.  Iter-29: R drift 0.33°, cam_offset 1.9 mm.
  Iter-33: first end-to-end PASS with soft-fallback Kp=0.39 when relay
  no-converge.  Commits `12d28a90`, `8c40bcc5`, `572442e5`, close-out.

**Other**: skills `sim-arm-parity-check` + `sim-mission-flight` filed.
MEMORY.md curated 37→21 KB.  WBS `OP-S10-W10-T1` (ROS2 narrative)
filed for thesis Evaluation chapter.

**Why:** the production-calib unification (W21) is what makes
`sentai.calib` deployable on every drone — see
[[sentai-calib-is-production-bringup]].  Closing W19-T6b unblocked
the multi-marker yaw recovery needed for s185 (deferred).

**How to apply:** when resuming W21, the foundation (orchestrator, INI,
guard, picker, primitives) is in place; remaining work is Gazebo
end-to-end on the smaller 0.5× pad which is blocked on cf2 SITL
positioning — see [[op-s10-w21-t4-session2-open]] for that carry-over.
On the original pad, s187 is already reproducibly PASS.

Related: [[yaw-anchor-mirror-picker]],
[[sentai-calib-is-production-bringup]], [[no-flow-deck-camera-imu-only]],
[[kill-all-gz-before-launch]], [[op-s10-w21-t4-session2-open]].
