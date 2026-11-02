---
name: project_pilot_follow_me_w26
description: "OP-S10-W26 pilot follow-me (walking person, safe distance, 2-D) — research done 2026-11-02, design decisions and open tasks"
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-02T20:41:25.175Z
---

New work package OP-S10-W26 opened 2026-11-02 (operator): the drone follows the walking pilot in the horizontal plane at a safe distance. Research (T2) done; design in `ideas/objects_plan/23_pilot_follow_tracking_research.md`, literature memos `23a` (vision) and `23b` (fusion). Operator is training the oblique-view person detector (T1) themselves.

Decisions / findings to keep:
- **One TPU model only** (the detector). No lightweight Siamese/transformer single-object tracker compiles end-to-end for the Edge TPU (template-as-kernel xcorr and attention are dynamic-weight ops; no int8 port exists); a second resident model costs a USB weight reload (~30 ms/MB) plus the W24-T9 switch-wedge risk. Bridge, lock, re-ID and estimator all go on the M7 around the existing SentAI-SORT (`sentai_tracker.cc`).
- **Operator concern (confirmed by the numbers)**: the ground-plane projection of the pilot is useless during pitch transients (h = 3 m, θ = 30°: 21 cm per 1°, 2.5 m for a 10° braking swing). Design answer: track in the ground frame, attitude interpolated to the frame exposure time, box-height range (pitch-insensitive) and bearing as separate measurements with attitude-rate-dependent noise, skip vision at high angular rate, coast on the CV pedestrian model, FOV budget (max pitch ~10°, cruise-pitch mount, CBF visibility constraint, two-tilt camera alternation).
- **Controller**: KF estimator + LQR + disturbance observer + CBF (d_min and FOV) instead of PID; residual RL MLP (AIfES) as a thesis-contribution candidate (no MCU/UAV precedent in the literature); MPC only if LQR lags at turns.
- **Anti-cheat**: a simulated phone GNSS in Gazebo must be a host-side emulator (actor pose + latency + noise + decimation) over the CRTP path; derived from gz state, so it needs explicit operator approval and a `sim/ANTI_CHEAT.md` entry before T7 runs.

**Why:** person-following is a new thesis thread beside ObjectsPlan; these decisions came from a two-memo literature survey plus the operator's field concern and should not be re-derived.
**How to apply:** when any W26 task starts, read chapter 23 first; do not propose a second TPU model or a pixel-space PID; measure CRTP pose latency (T3) before touching the estimator. Related: [[project_s265_camera_color_calibration_w24_t18]] (display rig for replay), [[feedback_sentai_sim_air_gapped_from_truth]].
