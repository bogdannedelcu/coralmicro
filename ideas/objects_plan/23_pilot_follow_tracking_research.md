# Chapter 23 — Pilot follow-me: continuous tracking of a walking person from the drone (research)

**WBS**: OP-S10-W26 (new work package, 2026-11-02) — task T2 (research) is this document; T1 (oblique-view person detector) is being done by the operator.
**Status**: research complete; design recommended; nothing implemented yet.
**Companion memos** (full literature survey with citations, English): `23a_pilot_follow_visual_tracking_memo.md` (vision: detector + bridge + re-ID + re-acquisition) and `23b_pilot_follow_target_fusion_memo.md` (estimator: box + external pilot coordinate + drone pose, latency, dropout).

## 1. Problem statement (operator, 2026-11-02)

A sub-250 g drone follows the pilot, who stands on the ground and walks, in the horizontal plane at a safe distance. Inputs available on the drone: a one-class person detector on the Edge TPU (20–30 Hz), the drone's own pose from the flight-controller EKF over CRTP (10–50 Hz), and the pilot's coordinate from a phone GNSS or UWB tag over the radio link (1–10 Hz, 0.1–1 s late, 1–5 m error for GNSS, 0.1–0.3 m for UWB). The camera is fixed to the airframe (no gimbal), so every drone pitch/roll shows up in the image.

Operator's two concerns, both confirmed by the survey:

1. A PID on the raw detection is not enough because the drone's motion is not deterministic (wind, latency, actuator saturation) — see §5.
2. The pilot's ground position is an *estimate* that is very sensitive to camera orientation and drone-position errors: the drone pitches forward when it moves and swings back when it stops, so the projected position jumps and the pilot sometimes leaves the field of view — see §3.

## 2. What already exists on the board (do not rebuild)

| Block | Where | What it gives | Gap for follow-me |
|---|---|---|---|
| Person detector (headless YOLOv5 p3p4, int8) | TPU via `sentai.pipeline` | boxes at 20–23 ms VGA / est. 8–12 ms at 320×240; firmware decoder fixed in build 1769 (EXP-s269) | trained on top-down synthetic; needs an oblique-view person model (W26-T1) |
| SentAI-SORT | `sentai_tracker.cc` (≈1.1 kLoC, 0.9 ms/frame) | ByteTrack two-stage association, 8-state constant-velocity KF `[cx,cy,a,h,v…]`, HSV histogram cost, IMU camera-motion compensation, TENTATIVE/CONFIRMED/LOST, events to MicroPython | multi-object; no single-target lock, no re-acquisition policy, IMU compensation uses the *latest* pitch/roll, not the pose at the frame's exposure time |
| Ground-plane projection | `sentai_tracker_set_pose()` → `gx_cm/gy_cm/dist_cm/lat/lon` per track, `get_footprint()` | ray–ground intersection with altitude + heading + IMU pitch/roll | same latency problem; no uncertainty output; ground-ref centroid is wrong for oblique views (needs box bottom) |
| Optical flow | `sentai.flow` on M7 (phase correlation + block SAD) | ego-translation, body-frame baseline | reusable as the hover fallback and as an NCC/MOSSE patch-tracker kernel |
| Drone pose | `sentai.crazy.pose_*` (CRTP LOG) | position, attitude at 10–50 Hz | latency not measured; no time-stamp alignment with camera frames |
| Action layer | `sentai.servo` (L4) | intent vocabulary, armed FSM, trace ring | no velocity-setpoint backend wired for follow |
| Safety | `sentai.safety` | aborts, triggers | needs LOST-target → hover/search/land policy |

## 3. The attitude-error and field-of-view budget (operator concern 2)

Camera at height h sees the pilot's feet under depression angle θ (below the horizon). Ground range r = h / tan θ, so dr/dθ = (r² + h²)/h. For h = 3 m:

| θ | r | error per 1° | error for a 10° pitch swing (braking) |
|---|---|---|---|
| 45° | 3.0 m | 0.10 m | 1.2 m |
| 30° | 5.2 m | 0.21 m | 2.5 m |
| 15° | 11.2 m | 0.78 m | > 10 m |

Conclusions that drive the design:

- **The raw ground-plane projection is unusable during transients and poor beyond ~6 m.** It must never be fed to the controller directly.
- **Box-height range (Z = f·H/h_px) is almost pitch-insensitive** (it varies with the cosine of the viewing angle) but has a ~15–20 % prior error from the person's unknown height and posture (crouching inflates it ~1.6×; box *width* and the ground-plane cue are the posture-independent fallbacks — memo 23a §4). Bearing from the pitch-compensated pixel is well conditioned (σ_px/f plus yaw error). So **range and bearing are separate measurements with separate, attitude-dependent noise** (memo 23b §1, §8).
- **Attitude must be the one at the frame's exposure time**, interpolated from a time-stamped CRTP pose ring, not the last sample received. The Crazyflie EKF attitude is good to ~1°, which is fine once aligned in time.
- **Noise inflation by angular rate**: when |pitch rate| or |roll rate| is high, the ground-plane range noise is inflated by (r²+h²)/h·σ_pitch and the measurement is skipped above a threshold (motion blur makes the box unreliable anyway). The filter coasts on the pedestrian motion model, which is credible for 1–2 s (walking speed ≤ 1.5 m/s, acceleration ≈ 0.7 m/s², memo 23b §2).
- **FOV budget**: vertical FOV 43°; a ±10° pitch swing consumes half of it. Levers, cheapest first: (a) limit the flight controller's maximum pitch to ~10° (≈ 2 m/s, enough for walking); (b) mount the camera at the *cruise* pitch so the pilot sits mid-frame while following, not while hovering; (c) a visibility constraint in the controller: when the box approaches the frame edge the drone trades speed for keeping the target in view (a second barrier function next to the minimum-distance one, §5); (d) the second OV5640 with a different tilt — camera alternation already works (W24 scheduler), so two tilts nearly double the vertical coverage without a gimbal.
- **Re-acquisition after a FOV exit**: the estimator predicts where the pilot re-enters the image once the drone levels; the detector output is gated there first (growing search window, memo 23a §3). The external pilot coordinate keeps the filter anchored during the gap.

## 4. Recommended tracking architecture

Keep exactly **one TPU model** (the detector) and build everything else on the M7. No lightweight Siamese or transformer single-object tracker compiles end-to-end for the Edge TPU (template-as-kernel cross-correlation and attention are dynamic-weight ops; no int8 Edge TPU port exists), and a second resident TPU model costs a weight reload over USB (~30 ms/MB) plus the W24-T9 model-switch wedge risk (memo 23a §2).

Layers, in the order the data flows:

1. **Detector** (W26-T1): one-class person YOLOv5n, oblique view, 320×240 every frame (or VGA every second frame). Candidate training data: COCO person + VisDrone + own recordings; EDAPA (top-down) is not the right domain.
2. **Single-target lock inside SentAI-SORT** (M7): a `PILOT` slot bound to one track ID, selected at lock time as the track whose ground position is nearest the pilot's external coordinate (or by REPL command). Per frame the pilot box is the matched CONFIRMED track. Add OC-SORT's observation-centric re-update on reappearance (fixes post-occlusion KF drift) and Deep-OC-SORT-style confidence-weighted appearance (upper/lower-body HSV part histograms, built on the histogram code we already have). ID switches are vetoed by the ground-plane distance prior using box height *and* width.
3. **Bridge during detector misses** (M7, sub-millisecond): MOSSE or NCC patch tracker on a 64×64 grey patch seeded from the last box, gated by its peak-to-sidelobe ratio, supplies the box for ≤ 0.3 s while the KF coasts with camera-motion compensation. Built from the existing FFT / block-SAD kernels. No Cortex-M figures exist in the literature; task T4 measures them.
4. **Re-acquisition FSM**: LOST beyond the grace window → `REACQUIRE`: full-frame detections gated against the predicted position with a per-frame growing radius (DaSiamRPN local-to-global), then against appearance; after N empty frames the controller yaws toward the predicted or bias-corrected GNSS bearing (staged escalation); after a timeout → `SEARCH`/hover per `sentai.safety`.
5. **Target estimator** (`sentai_target`, C on M7, memo 23b §8): state [p_n, p_e, v_n, v_e, b_n, b_e] in the drone-EKF local frame; constant-velocity process with σ_a ≈ 1 m/s²; b = random-walk bias between the pilot-GNSS frame and the vision frame (PX4 Vision-Target-Estimator pattern). Measurements: vision bearing (2-D, 20–30 Hz), vision range (box height or ground plane, whichever has the smaller σ at the current geometry), pilot coordinate z = p + b with the phone-reported accuracy. Drone pose is an *input* interpolated to the image time-stamp. Out-of-sequence measurements handled exactly by a 64-entry (t, x, P) ring with rewind and replay (Bar-Shalom 2002); Larsen 1998 extrapolation as fallback. Association with several people: chi-square gate (2-DOF, 9.21); two candidates inside the gate → coast and flag `ambiguous` (a wrong update is worse than none). Compute: typical < 5 µs, worst-case replay 60–80 µs, ~11 kB DTCM.
6. **Standoff reference**: p_ref = p̂ − d_cmd·u, with u = walking direction when |v̂| > 0.4 m/s (behind the pilot, as in the only published GNSS+vision follow-me system, Do & Ahn 2018), else line of sight; blended by a logistic weight to avoid flipping; d_cmd = d_standoff + k·√trace(P_pp), so uncertainty widens the distance. Both modes selectable from MicroPython.
7. **Controller** (§5), fed by the estimate, never by the raw box.

Everything from 2 to 6 is per-frame scalar work in C; MicroPython sees the pilot state, covariance and FSM state as scalars (no heavy data through MP).

## 5. Controller (operator concern 1)

Recommended stack, in build order; each layer alone is an improvement over PID on the raw detection:

| Layer | Role | Cost on M7 |
|---|---|---|
| Kalman estimator (§4.5) with pilot velocity as feed-forward | removes measurement noise and latency from the loop | µs |
| LQR on the 2-D double integrator (cf2 inner loops handle attitude) + disturbance observer (acceleration-bias state) | replaces the PID integral with an estimated, predictive disturbance term: no wind-up, no oscillation after gusts | µs |
| Control-barrier-function safety filter | hard minimum distance d_min and the FOV-retention constraint (§3) as inequalities, not gains; closed form for one or two linear constraints | µs |
| Residual policy (small MLP, AIfES on M7) trained by RL in Gazebo with domain randomisation | learns what the model does not capture (non-linear response, variable delay, ground effect); bounded to ±30 % of the LQR command; CBF stays on top | tens of µs |
| MPC (horizon 10, small QP) | only if LQR + CBF lag at sharp pilot turns | < 1 ms |

Literature check: the constant-velocity model beats neural predictors for pedestrians over 1–3 s (Schöller et al. 2020); no publication reports a learned Kalman gain (KalmanNet) or a residual-MLP estimator on an MCU or a UAV follow-me task, so the residual policy is a thesis contribution candidate, not a reproduction.

## 6. Evaluation plan

- **Operational metrics** (not AUC): lock-hold fraction, time-to-reacquire, ID switches / pilot swaps (must be zero), distance-hold RMS, d_min violations (must be zero), FOV-retention fraction, yaw-rate demand.
- **Public proxies**: UAV123 `person*`/`group*` subsets and DTB70 for camera-motion robustness; D-PTUAC (138 drone sequences of look-alike people in uniforms) for the pilot-among-look-alikes case; LaSOT `person` for long out-of-view gaps. Run in the x86 `sentai_sim` build first (SIL), then through the replay rig (virtual camera or the EXP-s269 display path) on the real firmware.
- **Gazebo**: a walking `<actor>` on scripted paths (straight, turn, stop-go, two actors crossing at 1.5 / 3 / 6 m), brake manoeuvres that produce the pitch swing of §3, with `gt_recorder` used only for post-mortem scoring. **Anti-cheat**: a simulated phone GNSS must be a host-side emulator (actor pose + 0.1–1 s latency + 1–5 m noise + 1–10 Hz decimation) delivered over the same CRTP path a real phone would use. It is an external sensor, not ground truth, but it is derived from Gazebo state, so it needs explicit operator approval and an entry in `sim/ANTI_CHEAT.md` before any run.
- **Hardware**: outdoor walk with a phone GNSS logger and the board; post-mortem truth from a hand-held RTK rover on the pilot.

## 7. Work breakdown proposed for OP-S10-W26

| Task | Content | Depends on |
|---|---|---|
| T1 | Oblique-view one-class person detector (COCO person + VisDrone + own frames), 320×240 export, miss profile vs person height in px | — (operator) |
| T2 | This research (DONE 2026-11-02) | — |
| T3 | Camera–pose chain audit: measure CRTP pose latency and camera time offset (LED flash / step test), time-stamped pose ring, verify R_CB against the flow body-frame convention; record in `camera_modules/` | — |
| T4 | M7 bridge trackers: MOSSE (CMSIS-DSP rFFT, 64×64) and NCC/block-SAD, `.ramfunc`, ms/frame and PSR distribution on recorded sequences | — |
| T5 | `PILOT` lock FSM in SentAI-SORT: OC-SORT re-update, growing re-acquisition gate, part histograms, height+width distance veto; SIL metrics on UAV123/DTB70/D-PTUAC | T1 (model), T4 |
| T6 | `sentai_target` estimator in C: 6-state CV + bias, OOSM ring, NIS gating, host unit tests on synthetic trajectories with injected latency (NEES/NIS consistency) | T3 |
| T7 | Gazebo world with walking actor(s) + host phone-GNSS emulator (anti-cheat approval first) + post-mortem scorer | — |
| T8 | Controller: LQR + disturbance observer + CBF (d_min, FOV) on cf2 SITL; standoff A/B (heading vs line of sight); brake-manoeuvre test of the §3 budget | T6, T7 |
| T9 | Residual RL policy in SIM, AIfES deployment on M7, bounded correction | T8 |
| T10 | Two-tilt camera alternation for FOV coverage (uses the W24 scheduler); max-pitch limit on cf2 | T8 |
| T11 | Outdoor hardware walk with phone GNSS and RTK truth | T5, T6, T8 |

## 8. Open gaps found in the literature (candidate thesis contributions)

- No covariance-threshold rule for when a follower should switch from coasting to search (proposed in memo 23b §8 as a design choice).
- No closed-form error budget for box-height range with H = 1.7 m, and no quantification of crouching / partial occlusion on it.
- No constant-turn vs constant-velocity evidence for walking people seen from a *moving* UAV.
- No KCF/MOSSE/NCC timings on a Cortex-M-class core; no MCU-class re-ID result; no int8 Edge TPU port of any single-object tracker.
- The only GNSS+vision follow-me system published (Do & Ahn 2018) is a mode switch, not a fused estimator; nobody compares heading-aligned vs line-of-sight standoff.

Unverified items the memos flag (re-check before quoting in the thesis): the UAV123 person-sequence count, the Yeom & Cho 2019 RMSE table, PX4 VTE timeout defaults, the GAP9 Frontnet figure.
