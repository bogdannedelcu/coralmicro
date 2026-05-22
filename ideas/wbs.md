# WBS — ObjectsPlan Work Breakdown Structure (PMP-style)

**Status**: canonical numbering scheme, frozen 2026-05-17. Operator-approved.

**Why**: prior to this doc the project mixed Stage 1-10, A1-A8, W1-W4,
L1-L7, Track A/B, §N.M, sNNN and ad-hoc letters (Stage 4.A, Stage 1.A/B/C)
across `objects_plan.md`, `next_steps_*.md`, memory files, and commit
messages.  No single hierarchy.  This doc replaces that with PMP-aligned
codes.

**Hard rule**: every plan artifact (doc edit, commit subject line,
experiment folder name, memory entry, task tracker entry) MUST reference
its canonical WBS code from this doc.  New planning items append the
next code; never reorder existing codes (they are stable identifiers
like ticket IDs).  Codes are append-only; renames break cross-references.

## 1. The five canonical axes

### 1.1 Primary axis — WBS (deliverable hierarchy)

| Code | Level | PMP equivalent | Granularity |
|---|---|---|---|
| `OP` | Project | Project | The whole thesis. There is one. |
| `OP-S{N}` | Stage | Phase | Deliverable-bounded chunk. 1-10 mapped to §3 roadmap. FROZEN — adding a new stage requires explicit operator approval. |
| `OP-S{N}-W{M}` | Work Package | Work Package | Deliverable-level unit *inside* a stage.  ~1 week of owner-hours. |
| `OP-S{N}-W{M}-T{K}` | Task | Activity | Atomic action ≤ 1 day owner-hours.  Trackable in TaskCreate. |
| `OP-M{N}` | Milestone | Milestone | Cross-stage gate (e.g., "thesis defense ready"). |

Examples:
- `OP-S6-W1` — Stage 6 (loop-closure + calib), Work Package 1 (`sentai.calib` MP binding).
- `OP-S6-W1-T4` — Task: ship `EXP-s157` smoke (perturb ±5°, recover < 0.5°).
- `OP-M3` — Indoor demo L7, 20-run validation campaign.

### 1.2 Orthogonal axes (not WBS — different dimensions)

| Code | Meaning | Frozen? |
|---|---|---|
| `ARCH-L{n}` | Architecture layer (code module hierarchy). L1 ego-motion → L7 demo. | Yes — 7 layers fixed. |
| `EXP-s{NNN}` | Experiment / validation run. Sequential, append-only.  Latest at time of writing: s156. | Append-only; never renumber. |
| `FW-{NN}` | FutureWork item (deferred from ObjectsPlan).  Already in use as FW1..FW17 in `FutureWork.md`. | Append-only. |
| `RISK-{NN}` | Risk register entry (was §4 in objects_plan.md). | Append-only. |
| `F-AC-{NN}` | Feature/Anti-Cheat audit ID (already in use). | Append-only. |
| `§{N.M}` | objects_plan.md doc-section reference.  Stable markdown anchor across the split into chapter files. | Stable globally even after chapter split. |

### 1.3 Time axis (separate from WBS — scheduling layer)

- `W{N}` for calendar weeks in short-term plans is OK to keep — but **only as a scheduling tag, not a deliverable code**.  A "W1" cell in a Gantt-style table must always be paired with one or more WBS codes.

## 2. Canonical WBS for ObjectsPlan (2026-05-17 baseline)

The 10-stage roadmap from `objects_plan.md` §3 is the primary axis.
Sub-Work-Packages are listed where currently planned.

```
OP — ObjectsPlan thesis
│   Goal: Autonomous Object-Driven Drone Navigation on RT1176
│
├── OP-S1 — Data layer (sentai.objects.*)                       ✅ SHIPPED
│
├── OP-S2 — TPU model task-specific (DNN red_cube)              ⏸ DEFERRED to §24 chapter
│
├── OP-S3 — Mission FSM (sentai.explore / sentai.mission)       ✅ SHIPPED
│   └── L6 skeleton + EXP-s133..s138 SIM PASS
│
├── OP-S4 — Action layer (sentai.servo.*)                       ✅ SHIPPED
│   └── Stage 4.A backend dispatch (CF2 + PX4) shipped 2026-05-17
│
├── OP-S4.5 — Multi-marker constellation localization           ✅ SHIPPED
│   └── EXP-s130 image-only nav PASS
│
├── OP-S5 — Object lifter (inverse-depth EKF)                   ✅ SHIPPED
│   └── EXP-s131 math + EXP-s132 Gazebo PASS
│
├── OP-S6 — Loop closure + camera-mount calibration             🟡 IN PROGRESS
│   ├── OP-S6-W1 — sentai.calib MP binding                        🟡 active (this week)
│   │   ├── T1 — Port Kabsch from _shared/camera_calibration.py → C++ (sentai_calib.{cc,h})
│   │   ├── T2 — bindings/modsentai_calib.c (run_takeoff_pad, get_R_cam_to_body, is_calibrated)
│   │   ├── T3 — Persist via FxUser to /system/cam_calib.json
│   │   ├── T4 — EXP-s157 smoke (perturb ±5°, recover < 0.5°)
│   │   ├── T5 — EXP-s158 mission_s153 integration
│   │   └── T6 — ARM build + size delta < 8 KB
│   └── OP-S6-W2 — Multi-object Yaw-Wahba (§12.3)                  ⏸ deferred (FW candidate)
│
├── OP-S7 — Full SM3 (COAST/ALIGN track health)                 ⏸ DEFERRED → FW-12
│
├── OP-S8 — End-to-end SITL integration (cf2 + PX4)             🟡 partial
│   ├── cf2 SITL working (EXP-s153..s156)
│   ├── PX4 SITL pending OP-S9 ARM bring-up
│   │
│   └── OP-S8-W1 — cf2 SITL anti-cheat + honest vision-anchored validation
│       │   🚨 CRISIS-OPENED 2026-05-17.  Discovery: every cf2-side drift
│       │   number ever published in this project (s127 FlowBaseline 7.4 cm,
│       │   s130 1.2 cm, s142 1.8 cm closure, s147+ closures, etc.) was
│       │   masked by the `gz-sim-odometry-publisher-system` Gazebo plugin
│       │   injecting GT pose directly into cf2 EKF as `CrtpExtPose`.  With
│       │   the plugin disabled, cf2 has NO honest absolute reference until
│       │   we wire ArUco PnP → cf2 via VPE (`cf.extpos.send_extpos`).  See
│       │   [[cf2-sitl-cheat-odom-gt]] + [[op-s8-w1-cf2-sim-honest]] for
│       │   the full forensic log.
│       │
│       │   Until this WP is GREEN, NO new cf2-side flight test produces a
│       │   trustworthy drift number.  Hard freeze on quoting cf2 drift in
│       │   thesis text until OP-S8-W1 closes.
│       │
│       ├── T1 — Disable `gz-sim-odometry-publisher-system` in
│       │       `model.sdf.jinja`                                  ✅ SHIPPED 2026-05-17
│       ├── T2 — Update `aruco_detector.py` KNOWN_POSITIONS_M +
│       │       MARKER_SIZE_M to match current world SDF            ✅ SHIPPED 2026-05-17
│       ├── T3 — Add VPE forwarder in `aruco_hover.py`
│       │       (`cf.extpos.send_extpos` from PnP, gated)           ✅ SHIPPED 2026-05-17
│       ├── T4 — Switch FlowBaseline hover from velocity setpoint
│       │       to POSITION setpoint @ origin                       ✅ SHIPPED 2026-05-17
│       ├── T5 — Retune cf2 position-PID (Kp 3→1, VelMax 2.5→0.5)
│       │       + VPE rate 20→5 Hz to stop noise-chasing            ✅ SHIPPED 2026-05-17
│       ├── T6 — Enforce reproducible cf2 spawn at origin BEFORE every
│       │       trial (rule [[experiments-start-from-origin]] —
│       │       current run.sh respawn is racy)                     ⬜ TODO (tomorrow)
│       ├── T7 — Add GT recorder + verdict gate ON cf2-side
│       │       FlowBaseline (current verdict trusts EKF belief
│       │       which lies without anchor)                          ⬜ TODO (tomorrow)
│       ├── T8 — Run new FlowBaseline canonical (3 trials, σ),
│       │       publish as `[[flowbaseline-canonical-2-no-cheat]]`  ⬜ TODO (tomorrow)
│       ├── T9 — Document discovery in Sim.md + agent.md +
│       │       objects_plan.md risk register update                ⬜ TODO (tomorrow)
│       ├── T10 — Audit ALL prior cf2 drift claims; re-run any
│       │       experiment whose conclusions depended on a drift
│       │       number (s127, s130, s142, s147..s150)                ⬜ TODO (later)
│       └── T11 — Revisit OP-S10-W11-T5.A (1m square) with the new
│                 honest baseline — open question whether to keep
│                 EXP-s164 as a documented invalid trial or rerun    ⬜ TODO (later)
│
├── OP-S9 — ARM bring-up + DWT timing budget                    ⬜ TODO
│   ├── OP-S9-W1 — Generic LOG-channel RX FIFO in sentai_crazy.cc
│   ├── OP-S9-W2 — Radio bridge LOG forwarding verify
│   ├── OP-S9-W3 — ARM flash + radio smoke
│   ├── OP-S9-W4 — DWT cycle counter instrumentation per pipeline stage
│   └── OP-S9-W5 — 60s pipeline capture for thesis eval chapter
│
├── OP-S10 — Hardening / docs / paper-grade evidence            ⬜ TODO
│   (was: track A places + L1 tracker per next_steps.md A3+A2;
│    folded into Stage 10 since these are deliverables shipping
│    before paper writeup)
│   ├── OP-S10-W1 — PHOG descriptor                              ✅ SHIPPED (EXP-s139)
│   ├── OP-S10-W2 — GIST-lite descriptor                         ✅ SHIPPED (EXP-s140)
│   ├── OP-S10-W3 — DescriptorBaseline anti-regression           ✅ SHIPPED (EXP-s141)
│   ├── OP-S10-W4 — HSV descriptor                               ✅ SHIPPED (EXP-s161, 2026-05-17)
│   ├── OP-S10-W5 — FFT-mag log-polar descriptor                 ⬜ TODO (calendar W3)
│   ├── OP-S10-W6 — Opposite-direction recall ≥ 80% (§14)        ⬜ TODO (calendar W3)
│   ├── OP-S10-W7 — L1 tracker minimal w/ ArUco shim             ⬜ TODO (calendar W4)
│   ├── OP-S10-W8 — L7 indoor integrated demo (§23.1 north star) ⬜ TODO
│   ├── OP-S10-W9 — Outdoor PX4 experiments (2-3 runs)           ⬜ TODO
│   ├── OP-S10-W10 — Evaluation chapter writeup                  ⬜ TODO
│   │   ├── OP-S10-W10-T1 — Positioning vs ROS2                  ⬜ TODO (defense narrative, 2026-05-21)
│   │   │   Frame the thesis engineering against ROS2 (rclcpp / DDS /
│   │   │   nav2 / message_filters / TF2 / launch).  Defense committee
│   │   │   will ask "why not just ROS2?" — we must answer crisply.
│   │   │   Angles:
│   │   │     (a) MCU-class target (RT1176, 800 MHz, 32 MB SDRAM, no
│   │   │         MMU, no Linux) — ROS2 client libs need a POSIX OS
│   │   │         with shared memory + DDS discovery; closest fit is
│   │   │         micro-ROS, which still pulls in 200 KB+ rmw / XRCE-
│   │   │         DDS and gives no compute primitives (PXP, CMSIS-DSP,
│   │   │         CSI ISR routing, etc).  We're an order of magnitude
│   │   │         below micro-ROS's bring-up footprint.
│   │   │     (b) Air-gap discipline ([[sentai-sim-air-gapped-from-truth]])
│   │   │         — ROS2 conflates "topic" + "service" + "GT replay"
│   │   │         on the same bus; our SIM and ARM share zero IPC, so
│   │   │         GT poisoning is structurally impossible.  Defensible
│   │   │         claim against a ROS-trained committee.
│   │   │     (c) Hot-path discipline (.ramfunc + ITCM placement, SIMD
│   │   │         inner loops, cycle-counted SLOT pipeline) — ROS2's
│   │   │         executor / callback queue / DDS heap allocation
│   │   │         pattern is incompatible with hard-real-time CSI ISR
│   │   │         hooks.  Numbers: flow SAD 1 ms M7 SIMD vs ROS2 pub-
│   │   │         sub round-trip 100 µs+ on Linux x86.
│   │   │     (d) Where we DID adopt ROS conventions: REP-103 frame
│   │   │         IDs (body / camera_link / world), Rodrigues / quat
│   │   │         conventions, sentai.markers tvec_body semantics.
│   │   │         Defensive — committee can map our work onto their
│   │   │         mental model without translation tax.
│   │   │   Output: 1-2 page subsection in evaluation chapter +
│   │   │   single-slide table for defense.  Cite micro-ROS papers (Belsare
│   │   │   et al. 2023, Casini et al. 2022 executor analysis).
│   │   │   Drives discussion of FW item "ROS2 bridge over CRTP" if any.
│   ├── OP-S10-W11 — sentai_prep frame slot pipeline              🟡 IN PROGRESS (2026-05-17)
│   │                Spec: ideas/objects_plan/OP-S10-W11_sentai_prep.md
│   │                (cadence, refcount, ARM/SIM differences, design
│   │                invariants, full task contract).
│   │   │   Cross-cutting frame producer (PrepTask fan-out) + SlamTask
│   │   │   InferTask-style consumer.  Foundation for places.compute_*
│   │   │   _from_camera variants per [[no-heavy-data-through-mp]].
│   │   ├── OP-S10-W11-T1 — sentai_prep.{h,cc} foundation        ✅ SHIPPED (commit b104d77e — retroactively-labeled "Phase 1a")
│   │   ├── OP-S10-W11-T2 — PrepTask SLOT_GRAY_NATIVE + grab_gray refactor  ✅ SHIPPED (commit 90b0523b — retroactively-labeled "Phase 1b")
│   │   ├── OP-S10-W11-T3 — slam_task.cc (perception loop)        ⬜ TODO (Phase 1c)
│   │   ├── OP-S10-W11-T4 — SIM mirror in camera_bridge_recv.c    ⬜ TODO (Phase 1d)
│   │   │                   Design decision 2026-05-19 (operator-
│   │   │                   approved): SLOT_GRAY_64 is DERIVED from
│   │   │                   SLOT_RGB_64 via RGB→Y luma cast (~7 µs
│   │   │                   NEON on 4096 px), NOT produced by a
│   │   │                   second independent PXP pass.  Rationale:
│   │   │                     (a) pixel-perfect consistency — both
│   │   │                         64×64 slots refer to the SAME
│   │   │                         source frame (no PXP race window);
│   │   │                     (b) one PXP setup per frame instead of
│   │   │                         two — saves SEMC bandwidth;
│   │   │                     (c) simpler producer code on both ARM
│   │   │                         (PrepTask) and SIM (camera_bridge_
│   │   │                         recv).
│   │   │                   Implementation note: when consumer enables
│   │   │                   SLOT_GRAY_64, producer first writes
│   │   │                   SLOT_RGB_64 (PXP HW), then luma-casts
│   │   │                   into SLOT_GRAY_64 (scalar / NEON), then
│   │   │                   commits both.  Document the invariant in
│   │   │                   sentai_prep.h header: "GRAY_64 corresponds
│   │   │                   to the same source frame as RGB_64 (luma
│   │   │                   downcast, not an independent capture)".
│   │   │                   Re-evaluate IF a future consumer needs
│   │   │                   gray-only AND wants to skip the RGB pass
│   │   │                   entirely — then a separate slot is the
│   │   │                   right answer.  Not the case for the
│   │   │                   thesis-MVP consumer set (HSV on RGB_64 +
│   │   │                   FFT-log-polar on GRAY_64 via W5).
│   │   └── OP-S10-W11-T5 — EXP-s162 live scene-discrimination    ⬜ TODO (Phase 1e)
│   │
│   ├── OP-S10-W12 — sentai.safety (firmware-side mission safety) 🟡 IN PROGRESS (opened 2026-05-18)
│   │   │   Continuous in-firmware mission-safety monitor per CLAUDE.md
│   │   │   compute-in-C principle + [[no-safety-logic-in-explore]].
│   │   │   Mission MP arms checks (ArUco first) and polls `aborted()`;
│   │   │   SafetyTask worker drives detection at camera FPS using the
│   │   │   EXISTING sentai.aruco + sentai.camera (no pipeline duplication).
│   │   │   Replaces interim host-side SafetyMonitor from s167.  See
│   │   │   Safety.md for full architecture.
│   │   ├── OP-S10-W12-T1 — sentai_safety.h API contract           ✅ SHIPPED
│   │   ├── OP-S10-W12-T2 — sentai_safety.cc state machine        ✅ SHIPPED
│   │   ├── OP-S10-W12-T3 — sentai_safety_task.cc worker          ✅ SHIPPED
│   │   ├── OP-S10-W12-T4 — bindings/modsentai_safety.c MP API    ✅ SHIPPED (minimal: 8 fns, no dicts)
│   │   ├── OP-S10-W12-T5 — SIM CMake + dispatch + QSTR regen     ✅ SHIPPED
│   │   ├── OP-S10-W12-T6 — EXP-s170 SecurityArucoBaseline smoke  🟡 IN PROGRESS (operator-named)
│   │   ├── OP-S10-W12-T7 — Migrate FlowBaseline2 mission to MP   ⬜ TODO (depends on FlightRecorder)
│   │   ├── OP-S10-W12-T8 — agent.md + Safety.md complete         ⬜ TODO
│   │   ├── OP-S10-W12-T9 — ARM build + ITCM budget + s127 gate   ⬜ TODO
│   │   ├── OP-S10-W12-T10 — Migrate in-place PGM journal from
│   │   │                     SafetyTask to sentai.fr (depends W13)  ⬜ TODO
│   │   └── OP-S10-W12-T11 — sentai_aruco frame_seq memoisation   ⬜ TODO (T-future)
│   │
│   └── OP-S10-W13 — sentai.fr (Flight Recorder subsystem)        🟡 IN PROGRESS (opened 2026-05-18)
│       │   Independent NASA/JPL-style flight data recorder per
│       │   embeded.md §3.1 (strict layer separation) + §7.2
│       │   (structured event log).  Multi-channel (frames / events /
│       │   scalars / kernel), bounded-queue producers, single drain
│       │   task → disk.  Producers (SafetyTask, mission, flow_task,
│       │   future health) push items O(1) non-blocking; recorder
│       │   thread does the I/O.  SIM-only initially; ARM port (FxUser
│       │   sinks) deferred.  Operator analogy: avionics FDR/CVR.
│       │   Format minimal per operator 2026-05-18 ("nu vreau mai
│       │   complicat de atat, vezi cum face PX4"): per-channel CSV
│       │   text, no JSON in MP path.
│       ├── OP-S10-W13-T1 — sentai_fr.h API contract              ✅ SHIPPED
│       ├── OP-S10-W13-T2 — sentai_fr.cc state + recorder task    ✅ SHIPPED
│       ├── OP-S10-W13-T3 — bindings/modsentai_fr.c MP API +
│       │                    `sentai.fs.record_image(source)` helper
│       │                    (caller picks rgb/gray/resized)         ⬜ TODO
│       ├── OP-S10-W13-T4 — SIM CMake + dispatch + QSTR regen      ⬜ TODO
│       ├── OP-S10-W13-T5 — Migrate sentai_safety_task PGM dump
│       │                    to sentai_fr_push_frame                ⬜ TODO
│       ├── OP-S10-W13-T6 — Migrate sentai_dmesg → fr "kernel" ch  ⬜ TODO (T-future)
│       ├── OP-S10-W13-T7 — EXP-s171 FlightRecorder smoke
│       │                    (writes + drops + queue depths)        ⬜ TODO
│       ├── OP-S10-W13-T8 — ARM port (FxUser sinks)                 ⬜ TODO (T-future)
│       └── OP-S10-W13-T9 — Migrate sim/modsentai_sim_journal.c
│                            (sentai.sim.journal_*) → sentai.fr
│                            events channel                          ⬜ TODO (T-future)
│
│   └── OP-S10-W14 — sentai.calib autotune (Flow loop Kp via relay) 🟢 ALGORITHM DEMONSTRATED (opened + algo proven 2026-05-18)
│       │   IDENTIFICATION RESULT: Kp_flow_x = 0.39 ± 0.04 (4 trials,
│       │   σ=0.043, 100% convergence success rate).  T_u ~9.5 s,
│       │   a_y ~100 mm.  Method: Åström-Hägglund relay + Ziegler-
│       │   Nichols P-only + 20 mm hysteresis + VPE forwarder
│       │   (PnP→cf2 ExtPos @ 30 Hz).  ~30 s flight per trial.
│       │   In-flight auto-calibration of the perception → control
│       │   loop, NOT drone-physics sysID.  Operator-narrowed
│       │   2026-05-18: cf2's inner attitude PID is fine; what we need
│       │   is the gain that converts drift (PnP) → velocity setpoint
│       │   smoothly (no oscillation, no lag) post-cheat removal.
│       │   Algorithm: Åström-Hägglund relay + Ziegler-Nichols.
│       │   Velocity-only excitation via sentai_crazy_hover (no
│       │   world-frame go_to — operator: "nu avem inca un sistem
│       │   reliable de coordonate").  sentai.calib promoted from
│       │   one-shot library to long-running C++ task (mirrors
│       │   sentai.safety + sentai_safety_task split).
│       ├── OP-S10-W14-T1 — Design doc + math + API
│       │                    (ideas/objects_plan/16_*.md)             ✅ SHIPPED
│       ├── OP-S10-W14-T2 — wbs.md OP-S10-W14 row (this section)      ✅ SHIPPED
│       ├── OP-S10-W14-T3 — sentai_calib_autotune.{h,cc} + task
│       │                    split (sentai_calib_task.{h,cc})         ✅ SHIPPED
│       ├── OP-S10-W14-T4 — bindings/modsentai_calib.c extension
│       │                    (set_context + task_start/stop + is_done
│       │                    + get_kp + get_td_ms; minimal MP)         ✅ SHIPPED
│       ├── OP-S10-W14-T5 — SIM CMake wiring + QSTR regen             ✅ SHIPPED
│       ├── OP-S10-W14-T6 — EXP-s172 FlowAutotuneBaseline smoke      ✅ SHIPPED
│       │                    4-trial X-axis: Kp_x=0.39 ± 0.04 (σ=0.043,
│       │                    100% convergence).  Lateral drift at land
│       │                    17-27 cm — incompatible with [[sim-test-
│       │                    must-return-home]] ≤10 cm rule at current
│       │                    parameters; algorithm itself succeeded.
│       ├── OP-S10-W14-T7 — flow_gains.txt (key=value) persist via   ⬜ TODO
│       │                    FxUser.  Operator-mandated 2026-05-18:
│       │                    "nu prea vreau sa tinem configurari in
│       │                    json, un format mai simplu cheie-valoare
│       │                    e mai potrivit pentru MCU".  One line
│       │                    per setting, ~20-line parser, no JSON
│       │                    library on ARM.
│       ├── OP-S10-W14-T8 — Y-axis autotune (mirror X)                ✅ SHIPPED
│       │                    3 trials Kp_y=0.39 ± 0.10 (median 0.336,
│       │                    σ=0.10).  Confirms cf2 X/Y symmetry.
│       │                    Higher variance than X (one outlier
│       │                    0.509 trial); 6 trials total would tighten
│       │                    the band but not change the answer.
│       ├── OP-S10-W14-T9 — Online td estimation via gyro × PnP
│       │                    cross-correlation (CMSIS-DSP)              ⬜ TODO (phase 2)
│       ├── OP-S10-W14-T10 — Active PnP-based altitude hold during
│       │                    autotune.  VPE forwarder (iter #13-15)
│       │                    achieves passive altitude stabilization
│       │                    via cf2 EKF (z stable ±5 cm).  Active
│       │                    PID closing on z_pnp deferred — only
│       │                    needed if baro drift exceeds VPE
│       │                    correction.                                ⬜ TODO (phase 2)
│       ├── OP-S10-W14-T17 — Abstract hover() / yaw_rate through      ⬜ TODO (FW)
│       │   `sentai.servo` instead of `sentai.crazy` directly.
│       │   Operator-noted 2026-05-18 ("daca are si PX4 le bagam in
│       │   sentai.servo... si folosim de acolo").  cf2 has hover.
│       │   yaw_rate via Generic Setpoint type 5; PX4 has it via
│       │   MAVLink SET_POSITION_TARGET_LOCAL_NED (type_mask) or
│       │   ATTITUDE_TARGET.body_yaw_rate.  Sentai.servo.hover(vx,
│       │   vy, yaw_rate, z) should dispatch to either backend.
│       │   Refactor target: sentai_calib_task.cc + any other caller.
│       │   Effort: ~1 h after sentai.servo has PX4 backend mapped.
│       ├── OP-S10-W14-T16 — YawArucoBaseline experiment.            ⬜ TODO (phase 2)
│       │   Operator-proposed 2026-05-18.  Drone hovers at z_hold,
│       │   rotates 360° at ~30°/s while holding (x,y), markers stay
│       │   in FOV throughout.  Profile: pre-hold 3 s → rotation
│       │   12 s → post-hold 3 s → land.
│       │
│       │   Validates 5 independent claims:
│       │     1. Yaw control: hover.yaw_rate actually rotates cf2
│       │     2. Position hold under rotation: x,y drift < 5 cm
│       │        even as body axes rotate beneath the relay
│       │     3. PnP rotation invariance: n_dets ≥ 4 for ≥ 95 % of
│       │        rotation window (markers move in image, still detect)
│       │     4. VPE quaternion correctness: cf2 yaw (via CRTP LOG)
│       │        matches GT yaw within 10°
│       │     5. Smooth motion: measured yaw_rate = ~30°/s ±5°/s
│       │
│       │   Prereq: replace T13's identity quaternion with PnP-derived
│       │   quaternion (rvec_cam aggregate → R_drone_world → yaw → qz,qw).
│       │   ~50 LoC extension to sentai_calib_task VPE forwarder.
│       │
│       │   Effort: ~3 h (PnP-quat ~1 h, HOLD yaw_rate arg ~0.5 h,
│       │   s174 experiment ~0.5 h, trials + analysis ~1 h).
│       ├── OP-S10-W14-T19 — VPE positive-feedback investigation    ✅ SHIPPED option A (2026-05-18)
│       │   After T18 detection jumped to 90 %, s174 yaw mission
│       │   showed positive-feedback loop between ExtPose quaternion
│       │   (open-loop integrated commanded yaw rate) and cf2's
│       │   yaw setpoint controller.  Commanded −3°/s → drone rotated
│       │   559° in 14 s (peak 103°/s).  Tested 3 options:
│       │     A. VPE position-only (ExtPos canal 0)  → STABLE,
│       │        14° rotation, 7 cm land drift, no abort.
│       │     B. PnP-derived REAL yaw quaternion     → CRASH
│       │        (resonance, 91 cm drift, abort + visual crash).
│       │     C. Identity quat always                 → similar to B.
│       │   Shipped option A.  Commit c226571d.
│       │
│       ├── OP-S10-W14-T20 — yaw rate authority + 360° rotation     ⬜ DEFERRED (research done, T21 is path)
│       │
│       │   Problem: with T19 option A, cf2 SITL only tracks ~17 %
│       │   of commanded yaw_rate (−3 cmd → 0.5°/s actual; −90 cmd →
│       │   18.5°/s actual but mission aborts via position drift).
│       │   No setting of cmd magnitude alone gives a clean 360° in
│       │   the 24 s mission window.
│       │
│       │   Trials run (all FAIL):
│       │     YAW_RATE | rotation | land drift | safety abort
│       │       −3°/s  |   14°    |    7 cm    | none ✓
│       │       −45°/s |   88°    |   173 cm   | yes
│       │       −90°/s |  432°    |   153 cm   | yes
│       │       −3 + auto-stop@360 (re-enable quat): 61°, 51 cm drift, abort
│       │
│       │   Research summary (cf2 firmware + literature):
│       │     • cf2 cascade: yaw POSITION PID → yaw RATE PID → motor diff
│       │     • Hover packet (Generic Setpoint type 5) sets
│       │       attitudeRate.yaw = -arg (cf2 negates).  modeVelocity
│       │       internally integrates this to maintain a yaw POSITION
│       │       setpoint.  Stop = send yaw_rate=0 → cf2 decelerates.
│       │     • ExtPose quat fused into EKF as a yaw MEASUREMENT.
│       │       If measurement disagrees with internal setpoint, the
│       │       yaw POSITION controller fights → motor output spikes.
│       │     • Our open-loop integrated quat lies behind the real
│       │       drone yaw (model = cmd × t, reality = cmd × t × 7.8
│       │       for our SIM).  Controller sees ESTIMATE < SETPOINT,
│       │       drives motors harder → drone over-rotates.
│       │     • Mocap-grade vision (200 Hz, <1° error) doesn't show
│       │       this — our 30 Hz PnP with 1-2° noise resonates.
│       │
│       │   SOTA approaches for vision-guided yaw rotation:
│       │     1. cf2 HL go_to(yaw=target) — minimum-jerk trajectory
│       │        between current and target yaw.  Smooth, stops
│       │        automatically.  But HL mode locks out hover() so
│       │        we'd need to combine position-hold into the go_to
│       │        call (x=0, y=0, z=z_hold, yaw=target).
│       │     2. Trapezoidal velocity profile generator (extern) —
│       │        ramp up rate, plateau, ramp down to reach target.
│       │     3. Step-wise (operator-proposed 2026-05-18) — small
│       │        target increments (22.5° × 16 steps = 360°) with
│       │        dwell between.  Robust, slow.  See T21.
│       │     4. PID on yaw ERROR with low-pass-filtered PnP yaw —
│       │        target yaw_rate = Kp × (target_yaw − measured_yaw).
│       │        Filter cuts the resonance frequency.
│       │
│       │   T20 conclusion: yaw rate tracking via hover() + ExtPose
│       │   is fundamentally NOT robust in our setup.  Time-tuning
│       │   the cmd magnitude doesn't converge to a clean 360 + safe
│       │   position hold.  Path forward = T21 step-wise.
│       │
│       ├── OP-S10-W14-T21 — step-wise yaw rotation (the path)      ⬜ TODO (next session priority)
│       │
│       │   Operator-proposed 2026-05-18: rotate in N steps of
│       │   360°/N (e.g., 22.5° × 16 = 360°).  Each step: command
│       │   target → wait for cf2 to settle (within ε of target) →
│       │   check markers in FOV → next step.  Total time ~30-60 s.
│       │
│       │   Two implementation paths:
│       │     (a) cf2 HL go_to(x=0, y=0, z=z_hold, yaw=cur+22.5°)
│       │         — uses cf2's trajectory generator.  Smooth motion,
│       │         stops automatically.  Requires exiting our
│       │         hover()-based HOLD mode for the rotation.
│       │     (b) hover(yaw_rate=±k) for short burst then yaw_rate=0
│       │         + 1 s settle, repeat.  Stays in HOLD mode but is
│       │         essentially open-loop bang-bang.
│       │
│       │   Recommendation: try (a) first.  cf2 HL handles position
│       │   hold + yaw target simultaneously; minimum-jerk profile
│       │   is the SOTA standard.  Effort ~3-4 h.
│       │
│       ├── OP-S10-W14-T18 — ArUco detector cv2 1:1 port           🟢 ALGORITHM PORTED (2026-05-18)
│       │
│       │   Operator-driven 2026-05-18: align sentai_aruco with
│       │   cv2.aruco at every pipeline stage, byte-for-byte where
│       │   feasible.  Starting state: 0-40 % 4/4 detection rate on
│       │   real s174 yaw-mission frame set (cv2: 92 %).  After T18:
│       │   90 % 4/4 — effective cv2 parity within 2 pp.
│       │
│       │   ├── T18-A — IPPE 2-solution disambiguation              ❌ REVERTED (no-op in our scene)
│       │   ├── T18-B — VPE median+outlier reject                   ⬜ ARM-build-blocked
│       │   ├── T18-C — Förstner subpix refinement                  ❌ DROPPED at ablation
│       │   │   (validated 0.0000 px vs cv2.cornerSubPix but
│       │   │   regressed real-flight detection by 1 pp; code
│       │   │   retained as static function for future use with
│       │   │   findContours port)
│       │   ├── T18-D/E — Moore-Neighbor + Douglas-Peucker          ✅ KEPT (essential)
│       │   ├── T18-F — Perspective warp + Otsu + cell decode       ✅ KEPT (essential)
│       │   ├── T18-G — Multi-scale threshold loop                   ❌ DROPPED at ablation
│       │   │   (6× scales [3,13,23,51,101,201]: only 1 pp gain vs
│       │   │   single block=201; massive CPU cost not worth it)
│       │   ├── T18-H — 8-connectivity flood fill                    ✅ KEPT (critical: fixed rot45 0→3 of 4)
│       │   ├── T18-I/J — Investigation + side-by-side diff           ✅ Done
│       │   ├── T18-K — Explicit CW corner winding                    ✅ KEPT (defensive)
│       │   ├── T18-M — IPPE_SQUARE PnP (~250 LoC port from cv2)     ✅ KEPT (essential, +10 pp on its own)
│       │   ├── T18-O — cv2 filter pipeline (borderErrors, convex,
│       │   │   minCornerDist, minDistToBorder, perim gates,
│       │   │   DP init_iters=3)                                    ✅ KEPT (init_iters=3 was worth +18 pp)
│       │   ├── T18-Q — Ablation + early-exit on n_ids==4            ✅ DONE (~25 % wall-time saved)
│       │   └── T18-P — findContours Suzuki-Abe port                 ⬜ TODO (final 2 pp gap)
│       │
│       │   Ablation results (s174 frame set, 356 frames):
│       │     Full T18 stack                                    90 % 4/4
│       │     − multi-scale (single block=201)                  89 %  (−1 pp)
│       │     − subpix (T18-C)                                  90 %  (+0 pp, +1 frame)
│       │     − IPPE_SQUARE (revert to DLT)                     80 %  (−10 pp)
│       │     + early-exit on all 4 ids found                   90 %  (+0 pp)
│       │   cv2.aruco oracle baseline                           92 % 4/4
│       │
│       │   Cumulative gains by component:
│       │     init_iters=3 (cv2 DP seed)             +18 pp  (5 lines of code)
│       │     IPPE_SQUARE PnP                        +10 pp
│       │     8-conn flood fill                       +3 pp  (critical for rot45)
│       │     cv2 multi-scale (now dropped)           +1 pp
│       │     border/convex/dist/perim gates          +0-2 pp combined
│       │     Förstner subpix (now dropped)           -1 pp  (regression!)
│       │
│       │   Compute saved by dropping unhelpful components:
│       │     − multi-scale → −83 % threshold + flood-fill work
│       │     − subpix       → −110 µs/frame (Förstner saddle, 30 iter)
│       │     + early-exit   → −25 % wall-time when 4 markers found early
│       │
│       │   Final ARM-side footprint (cf [[arm-hw-primitives-first]]):
│       │     - 8-conn flood fill (4-conn was the rot45 bug)
│       │     - Moore-Neighbor + DP with cv2 init_iters=3 seed selection
│       │     - Perspective warp + Otsu + cell-majority bit decoder
│       │     - _getBorderErrors + isContourConvex + perim gates
│       │     - IPPE_SQUARE PnP (~250 LoC math, ~50 µs/marker)
│       │     - Early-exit short-circuit on n_known_ids == 4
│       │
│       │   Commits: 6d22b295 (T18-C/D/E/F/G/H), ed4ff640 (T18-M),
│       │   0fb5bcb7 (T18-O steps 1-6), a89245c1 (T18-O step 7
│       │   init_iters=3), 5abe1df5 (T18-Q ablation).
│       │
│       │   Lesson learned: empirical ablation > intuition.  Most
│       │   compute-heavy "obvious wins" (multi-scale, subpix)
│       │   actually regressed or made no difference on real frames.
│       │   The 5-line DP seed loop fix delivered the biggest single
│       │   gain (+18 pp).  Always measure before keeping complexity.
│       │
│       │   Reference marker frames (raw PGM from real flight):
│       │     - examples/sentai_runtime/experiments/s175_pnp_planar_ambiguity/
│       │         frame_original.pgm     — axis-aligned 4 markers
│       │         frame_rot35.pgm        — same scene bilinear-rotated
│       │                                  35° (synthetic artefact;
│       │                                  not used for real measure)
│       │     - examples/sentai_runtime/experiments/s177_corner_subpix_prototype/
│       │         frame_horiz_n4.pgm     — real capture, axis-aligned,
│       │                                  4 markers, easy case
│       │         frame_rot45_n1.pgm     — real capture, ~45° in-plane
│       │                                  rotation, the WORST case
│       │                                  (pre-T18: 0 of 4 detected;
│       │                                  post-T18: 4 of 4)
│       │     - examples/sentai_runtime/experiments/s178_cv2_vs_ours_dump/
│       │         frame_133.pgm          — the SMOKING GUN frame from
│       │                                  s174 yaw mission: cv2 sees
│       │                                  ids 0,1,2,3 at z≈1.10 m,
│       │                                  pre-T18-M our pipeline saw
│       │                                  only id=3.  Used to isolate
│       │                                  the DLT-PnP-instability bug
│       │                                  → IPPE_SQUARE port.
│       │
│       └── OP-S10-W14-T11 — STEP RESPONSE identification alternative
│                            to ZN-relay (operator-noted 2026-05-18:
│                            relay produces ±6-10 cm lateral oscillation
│                            inherently, plus trial-to-trial variance
│                            high — iter #15 converged Kp=1.58, iter
│                            #17 same params landed 36 cm out + safety
│                            abort).  Step response: ONE 5 cm position
│                            step + 5 s record + 1st-order fit
│                            y(t)=K·(1-e^(-t/τ)).  Single trial, no
│                            oscillation, deterministic.  Implement
│                            via sentai_crazy_go_to(rel=0) absolute
│                            position step using PnP anchor.            ⬜ TODO (phase 2)
│
├── OP-S10-W15 — ARM memory allocation + task-priority budget       ⬜ TODO (open ToDo,
│                operator-requested 2026-05-19, not scheduled yet)
│   │
│   │  Motivation:
│   │
│   │  The ARM build broke silently across W12+W13+W14 because every
│   │  new subsystem (sentai.safety, sentai.fr pools, calib autotune
│   │  task) added static buffers to the default DTCM .bss and new
│   │  cold-path code to ITCM, without any global view of what the
│   │  budget actually IS.  T22 (commit 3a89466f, 2026-05-18) restored
│   │  the build reactively: shrunk HEAP_SIZE 16→14 MB, routed new
│   │  pools to .sdram_bss, parked cold-path code in .sentai_slow.
│   │  That fix is one-shot — the next subsystem (S6-W2 / S6-W3 / W11
│   │  prep pipeline expansion / DNN if ever) WILL repeat the same
│   │  break unless the project owns a documented allocation policy.
│   │
│   │  Scope:
│   │
│   │  Produce a Memory Allocation + Task Priority design document
│   │  (paper/arm_memory_budget.md) that fixes, for the SentAI
│   │  firmware on RT1176-M7:
│   │
│   │    1. The MEMORY MAP intent: which static buffer / pool goes
│   │       into which region (ITCM .ramfunc / DTCM m_data / OCRAM
│   │       m_ocram / SDRAM m_sdram / m_ncamera / m_heap), with
│   │       explicit rationale per category (DMA reach, cache
│   │       coherence, latency budget, contention class).
│   │
│   │    2. The TASK PRIORITY table: every FreeRTOS task in the
│   │       firmware (CameraTask, ArucoDetectTask, FlowTask, FR drain,
│   │       SafetyTask, CalibTask, MicroPython REPL, HTTPServer, ...)
│   │       with its (priority, stack size, stack location, periodic
│   │       deadline, justification).
│   │
│   │    3. The "WHERE DOES THIS GO?" decision flowchart for new
│   │       subsystems — 5-step checklist (Is it ISR? Is it
│   │       DMA-touched? Is it >1 KB? Is it per-frame? etc.) that
│   │       resolves to a region + section attribute name + linker
│   │       snippet.
│   │
│   │    4. A budget table with current (post-T22) consumption per
│   │       region + remaining headroom + soft alarm thresholds (e.g.,
│   │       "m_data ≥ 80 % full → file a W-future for offload").
│   │
│   │    5. Rules of engagement: when a new feature is allowed to
│   │       expand the budget (and how) vs when it must reuse / shrink
│   │       existing allocation.
│   │    6. CACHE-LINE ALIGNMENT POLICY (operator-emphasised 2026-05-19):
│   │       hot / frequently-used buffers (camera ring, TPU staging,
│   │       SLOT_GRAY_NATIVE, SLOT_RGB_64, SLOT_GRAY_64, flow
│   │       phase-corr buffers, aruco preprocessing, FR pool slots)
│   │       MUST be aligned to 32 bytes on M7 (one D-cache line).
│   │       Standardise a SENTAI_HOT_BUF_ALIGN macro expanding to
│   │       __attribute__((aligned(32))) and apply to every new
│   │       hot-path static buffer; T5 retrofits the existing scattered
│   │       __attribute__((aligned(...))) uses.  Audit script flags any
│   │       .sdram_bss / .ocram / m_data buffer ≥ 256 B not on a 32 B
│   │       boundary.
│   │
│   │  Why "not scheduled yet":
│   │
│   │  T22 bought us margin (m_sdram has ~3 MB free, m_data ~30 KB
│   │  free, m_text ~7 KB free post-routing).  Not urgent before the
│   │  next big subsystem lands.  But CRITICAL before the thesis
│   │  defense — committee will ask "how does this scale on a real
│   │  embedded target?" and the right answer is a documented memory
│   │  + scheduling policy, not "we route things to .sentai_slow when
│   │  it breaks."
│   │
│   │  Trigger to promote to scheduled status:
│   │
│   │  Either (a) the next subsystem (any one of S6-W2, S10-W5-W7,
│   │  DNN-on-TPU revisit, ARM port of W12/W13/W14) breaks the build
│   │  again, OR (b) thesis writing reaches the embedded-engineering
│   │  chapter — whichever comes first.
│   │
│   │  Tentative task breakdown (when promoted):
│   │
│   │    ├── OP-S10-W15-T1 — Memory map audit script: parse the .map
│   │    │                   output + dump per-region top consumers,
│   │    │                   per-region utilisation, post-link
│   │    │                   summary.  Auto-run in CI for both ARM
│   │    │                   and SIM builds.
│   │    │
│   │    ├── OP-S10-W15-T2 — Task priority + stack audit script:
│   │    │                   parse xTaskCreate/xTaskCreateStatic call
│   │    │                   sites, emit a table; compare against
│   │    │                   FreeRTOSConfig priorities + reservation.
│   │    │
│   │    ├── OP-S10-W15-T3 — paper/arm_memory_budget.md design doc
│   │    │                   (the 5 scope items above).
│   │    │
│   │    ├── OP-S10-W15-T4 — Per-region "soft alarm" assertions in
│   │    │                   CMake post-build: warn at 80 % / 90 %,
│   │    │                   error at 95 % to fail fast in CI.
│   │    │
│   │    └── OP-S10-W15-T5 — Retrofit existing subsystems to declare
│   │                        their memory class in a single header
│   │                        (e.g., SENTAI_MEM_COLD_PATH macro that
│   │                        expands to the right section attribute).
│   │                        Removes copy-paste of the
│   │                        __attribute__((section(...))) idiom
│   │                        scattered across ~15 files today.
│   │
│   │  Anti-scope:  This is a DESIGN + AUDIT WP, not a refactor.  No
│   │  functional changes; if any allocation moves, document the
│   │  before/after.  Lifetime: 1-2 days when actually pulled in.
│   │
├── OP-S10-W16 — Multi-core architecture (M7 + M4) clarification + replay  ⬜ TODO
│   │
│   │  Why: RT1176 has Cortex-M7 (800 MHz) + Cortex-M4 (400 MHz, with
│   │  single-precision FPU per NXP datasheet).  Build #1130+ removed
│   │  the M4 build target because flow_task_m4 had "unreliable SysTick
│   │  + freeze under load" (CMakeLists comment, 2026-05-05).  That
│   │  investigation predated the J-Link / SWD debugger now wired to
│   │  the board, so root-cause was guess-driven.  With the debugger
│   │  attached we can finally answer:
│   │    - Is M4 SysTick genuinely buggy on RT1176, or was it a
│   │      configuration mistake (clock root, ISR priority, FreeRTOS
│   │      tick handler placement)?
│   │    - What's the actual freeze mode (deadlock on RPMSG, WDOG
│   │      reset, SEMC bus contention, cache coherency)?
│   │    - Can ArUco run on M4 cleanly if SysTick is fixed?  ArUco
│   │      uses FPU only inside the PnP path (small).
│   │    - Same question for any future heavy task (TPU pre-process,
│   │      flow phase-corr).
│   │
│   │  Today's effective allocation (per code review 2026-05-19):
│   │     M7: camera ISR, PrepTask (PXP DMA), InferTask (TPU
│   │          orchestration via USB), Flow SAD (flow_task.cc),
│   │          ArUco (sentai_aruco_detect via SafetyTask),
│   │          Safety state machine, FR drain (FxUser writes),
│   │          Crazy radio bridge (UART2), REPL + MicroPython,
│   │          USB CDC-ACM + CDC-NCM stack, lwIP HTTP,
│   │          health/dmesg/calib/places/servo helpers.
│   │     M4: idle — flow_task_m4.cc kept as historical reference,
│   │          NOT compiled.
│   │
│   │    ├── OP-S10-W16-T1 — Document current core allocation in
│   │    │                   paper/multi_core_arch.md: per-task
│   │    │                   placement, period, priority, stack size,
│   │    │                   memory class.  Source: code review +
│   │    │                   FreeRTOS task list dump from on-board
│   │    │                   diag (uxTaskGetSystemState).
│   │    │
│   │    ├── OP-S10-W16-T2 — Wire J-Link/SWD investigation harness:
│   │    │                   tiny M4 "hello world" with FreeRTOS
│   │    │                   scheduler, vTaskDelay(100), GPIO toggle.
│   │    │                   Trace via OpenOCD/PyOCD; identify whether
│   │    │                   SysTick fires reliably under no load,
│   │    │                   under M7-heavy load, under SEMC pressure.
│   │    │
│   │    ├── OP-S10-W16-T3 — Reproduce the build-#1130 freeze: revive
│   │    │                   flow_task_m4.cc (or simpler kernel),
│   │    │                   capture exactly the failure mode + JTAG
│   │    │                   memory dump at freeze.  Compare to
│   │    │                   FreeRTOS internal state (xTickCount,
│   │    │                   pxReadyTasksLists, current task TCB).
│   │    │
│   │    ├── OP-S10-W16-T4 — Decision matrix: with root-cause in hand,
│   │    │                   document which workloads CAN move to M4
│   │    │                   safely.  Candidates ranked by M7 budget
│   │    │                   pressure: ArUco detect (22 ms × 30 Hz =
│   │    │                   66% M7), TPU pre-process if grow, future
│   │    │                   descriptor compute.
│   │    │
│   │    └── OP-S10-W16-T5 — Architecture freeze: paper/multi_core_
│   │                        final.md with the validated split (or
│   │                        "M4 stays unused, here's why" if M4 root-
│   │                        cause is fundamental NXP silicon bug).
│   │                        Tasks > T5 (actual offload work, if
│   │                        chosen) get their own per-subsystem WPs.
│   │
│   │  Anti-scope: This is INVESTIGATE + DECIDE.  Actual code moves
│   │  (e.g., ArUco on M4) get their own WP based on T4's verdict.
│   │  Promotion trigger: M7 budget squeeze that the W15 audit or a
│   │  s179-style HW bench shows is real (current SafetyTask 30 Hz
│   │  load is the immediate motivator).  Lifetime: ~1 week
│   │  investigation + 1 week design doc when actually pulled in.
│   │
│   ├── OP-S10-W17 — WhyCon-lite circular-marker detection         ✅ SHIPPED
│   │    │ (2026-05-19/20) M7 6.00 ms / 4-marker frame, 5.3× faster
│   │    │ than ArUco.  Spec: ideas/objects_plan/OP-S10-W17_whycon.md
│   │    │
│   │    ├── OP-S10-W17-T1 — WhyCon-lite implementation + M7 bench  ✅ SHIPPED
│   │    ├── OP-S10-W17-T2 — OCRAM + SIMD + inline 2nd moments       ✅ SHIPPED
│   │    ├── OP-S10-W17-T2.1 — Per-stage timing breakdown             ✅ SHIPPED
│   │    ├── OP-S10-W17-T4 — Flow SAD M7 vs M4 ablation               ✅ SHIPPED
│   │    ├── OP-S10-W17-T5 — Phase W3 concentric inner-disc check    ✅ SHIPPED (2026-05-20)
│   │    │   Auto-enabled on `sentai.markers.init('whycon')`.  Was the
│   │    │   root cause of "9 detections / frame" instead of expected 6 —
│   │    │   without W3, the centre dark dot and the outer dark annulus
│   │    │   both pass W1 fill-ratio, giving duplicate detections.
│   │    ├── OP-S10-W17-T6 — ArUco per-stage timing instrumentation  ✅ SHIPPED
│   │    ├── OP-S10-W17-T7 — Apples-to-apples bench + §4 fair table  ✅ SHIPPED
│   │    ├── OP-S10-W17-T8 — Fix W3 sample geometry + W1 selection   ✅ SHIPPED
│   │    └── OP-S10-W17-T9 — BUG #74 (synth detect returns 0 post-T18) ⬜ TODO
│   │
│   ├── OP-S10-W18 — Diamond search Flow (M7 + M4) + WhyCon M4 bench ✅ SHIPPED
│   │    │ (2026-05-19/20)
│   │    │
│   │    ├── OP-S10-W18-T1 — Diamond search Flow option              ✅ SHIPPED
│   │    └── OP-S10-W18-T2 — Define-ify + M4 WhyCon FULL bench       ✅ SHIPPED
│   │
│   ├── OP-S10-W19 — sentai.markers unified namespace + Gazebo eval  🟡 T1-T6 mostly SHIPPED
│   │    │ (2026-05-20).  Spec: ideas/objects_plan/OP-S10-W19_markers_unified.md
│   │    │ Final s183 numbers: X MAE 2.0 mm, Y MAE 2.1 mm, Z MAE 6.6 mm
│   │    │ (WhyCon beats cf2 EKF on all axes by 4-10×).
│   │    │
│   │    ├── OP-S10-W19-T1 — sentai.markers hard rename + struct ABI ✅ SHIPPED
│   │    │   Commit b36280b0.  No dicts; struct-based caller-allocated
│   │    │   bytearray; sentai_aruco/sentai_whycon namespaces deleted.
│   │    ├── OP-S10-W19-T2 — WhyCon closed-form PnP-z                 ✅ SHIPPED
│   │    │   Commit 89061cef.  tz = fx·D/(2·a)·ANNULUS_FACTOR;
│   │    │   ANNULUS_FACTOR=1.166 = √(1+0.6²) per analytic Krajník.
│   │    ├── OP-S10-W19-T3 — Multi-marker constellation pose          ⬜ TODO
│   │    │   (next session candidate).  Conic-section pose recovery
│   │    │   (Faugeras-Toscani 1986) or full PnP4P with marker IDs.
│   │    │   Yaw recovery from constellation.
│   │    ├── OP-S10-W19-T4 — SIM eval with thesis-quality X/Y/Z plots ✅ SHIPPED
│   │    │   s182 + s183 experiments.  Anti-cheat compliant
│   │    │   (sentai_sim consumes only camera + CRTP telemetry).
│   │    ├── OP-S10-W19-T5 — Cam extrinsics in sentai_markers         ✅ SHIPPED
│   │    │   Commit e52883e8.  set_cam_extrinsics(tx, ty, tz, roll,
│   │    │   pitch, yaw) → tvec auto-transformed to body frame.
│   │    │   Operator architectural directive 2026-05-20.
│   │    ├── OP-S10-W19-T6a — Yaw-anchor mirror picker (host)        ✅ SHIPPED (eod)
│   │    │   verdict_sota.py disambiguates the square-pad 4-fold
│   │    │   Kabsch ambiguity by checking sign of R[0,0]/R[1,1] vs
│   │    │   cos(cf2_yaw).  X/Y MAE went from ~4 cm → 2 mm.
│   │    └── OP-S10-W19-T6b — Yaw-anchor picker runtime port (C)     ✅ SHIPPED (2026-05-21 eve)
│   │        C-side `sentai_markers_get_drone_pose` on top of
│   │        [[op-s10-w20]]'s sentai_kabsch_align.  Handles 3 SVD
│   │        ambiguities on coplanar + 180-Z-symmetric pads:
│   │          (1) permutation assignment search over P(N,K),
│   │          (2) Z-plane reflection (drone-above-pad assumption),
│   │          (3) yaw-anchored X/Y mirror flip via sign(R diagonal)
│   │              vs cos(cf2_yaw), with flip-aware atan2 yaw extract.
│   │        ARM build #1421 + SIM build clean.  s184 smoke 6/6 PASS
│   │        (mirror-deployment T6 = 6-marker symmetric + scramble).
│   │        Dead code `cache_from_aruco_` removed (NASA discipline).
│   │        Memory entry [[yaw-anchor-mirror-picker]] already captures
│   │        the algorithm; runtime port references it.
│   │
│   ├── OP-S10-W20 — SVD/Kabsch shared module refactor               ✅ SHIPPED (2026-05-21)
│   │    │ Commit f1bc9850.  jacobi_sym3 + svd3 + reflection-safe
│   │    │ Kabsch composition extracted from sentai_calib.cc into
│   │    │ sentai_svd3.{h,cc} + new sentai_kabsch_align (full SE(3)
│   │    │ 3D-3D Procrustes).  sentai_calib.cc shrank 677->451 LoC,
│   │    │ public API unchanged.  s157 calib smoke 6/6 PASS pre- and
│   │    │ post-commit (Kabsch numerics bit-identical).  Unblocks
│   │    │ W19-T6b and W19-T3.
│   │
│   └── OP-S10-W21 — Unified calib bringup (extrinsics+PID+INI)      ⬜ DESIGN (2026-05-21)
│        │ Spec: ideas/objects_plan/OP-S10-W21_calib_unified_bringup.md
│        │ Operator request 2026-05-21: sentai.calib is the PRODUCTION
│        │ BRINGUP method for every shipped drone — auto-discovers
│        │ R_cam_to_body + cam_offset + Kp_x/y/yaw (+ future intrinsics)
│        │ and persists to /system/calib.ini.  One C-side orchestrator
│        │ `sentai_calib_run_bringup(ctx)` callable from REPL ONE-LINER;
│        │ never autoruns at boot (anti-brick).  Validated in Gazebo
│        │ as digital twin of bench bringup (s187).  See memory entry
│        │ [[sentai-calib-is-production-bringup]] for framing.
│        ├── OP-S10-W21-T1 — sentai.markers refactor in calib_task    ✅ SHIPPED (2026-05-21, commit 13b6d7be)
│        │   Discovery: code was already using sentai_markers_*;
│        │   only doc comments were stale.  4-comment refactor.
│        ├── OP-S10-W21-T2 — INI persistence (replaces cam_calib.json) ✅ SHIPPED (2026-05-21 eve)
│        │   Schema v1 JSON -> v2 INI.  Path /system/cam_calib.json
│        │   -> /system/calib.ini.  format_json/parse_json swapped
│        │   for format_ini/parse_ini.  Forward-compat parser
│        │   (unknown keys silently ignored).  s157 + s186 regress
│        │   green; new s188 5/5 PASS (on-disk format, schema reject,
│        │   forward-compat, corruption handling, comment tolerance).
│        │   ARM build #1424 + SIM build clean.
│        ├── OP-S10-W21-T3 — Schema v2 + Kp persist in calib.ini      ✅ SHIPPED (2026-05-21 eve)
│        │   SENTAI_CALIB_AXIS_YAW=2 added; s_kp_persisted[3] state;
│        │   commit_kp/get_persisted_kp API; format_ini/parse_ini
│        │   include kp_x, kp_y, kp_yaw lines.  MP bindings:
│        │   sentai.calib.commit_kp(axis, kp), get_persisted_kp(axis).
│        │   Axis parser accepts "x", "y", "yaw", or int 0/1/2.
│        │   Smokes 4/4: s157, s186, s188 regress green + new s189
│        │   5/5 PASS (round-trip, defaults, invalid reject, sentinel
│        │   accept, on-disk format).  ARM #1425 clean.
│        ├── OP-S10-W21-T4 — `sentai_calib_run_bringup()` orchestrator ✅ SHIPPED (2026-05-21, 29a9f92f)
│        │   ~350 LoC FreeRTOS task.  Phase FSM SAMPLE → KABSCH →
│        │   AUTOTUNE_X → AUTOTUNE_Y → HOLD → SAVE.  T4a-d phases
│        │   chased Gazebo end-to-end on original WhyCon pad —
│        │   reached reproducible PASS via clear() + THRESH_C=30 +
│        │   autotune VPE/canal fixes + soft-fallback Kp.  Phase-2
│        │   on smaller 0.5× pad blocked by cf2 SITL no-baro
│        │   positioning → superseded by T7 (clean rewrite).
│        ├── OP-S10-W21-T5 — Camera intrinsics auto-cal (DEFERRED)    ⬜ FW
│        ├── OP-S10-W21-T6 — SUBSYS_CALIB state + REPL-only trigger   ✅ SHIPPED (2026-05-21, 8454b713)
│        │   sentai.calib.assert_calibrated() takeoff-refusal guard.
│        ├── OP-S10-W21-T7 — Calib bringup via RPYT→HL handoff       ⚠️ SUPERSEDED-BY-T12 (2026-05-22)
│        │   Clean rewrite of T4 phase-2 path.  Operator-stated:
│        │   we DO NOT inject GT into cf2 SITL.  Instead:
│        │     1. pre-airborne + climb via Classic Commander RPYT
│        │        (CRTP port 3 ch 0, IMU-stabilized attitude+thrust,
│        │        no positioning needed) — same as XBox controller
│        │        flight per UART_RPYT_AGENT_PROMPT.md
│        │     2. ramp thrust gradually until markers visible (n≥4
│        │        + valid PnP stable 5 frames)
│        │     3. switch to ExtPos (port 6 ch 0) via
│        │        crtp_localization_service with PnP-derived (x,y,z)
│        │        — HW-parity, vision-only, anti-cheat clean
│        │     4. relax commander priority via meta-cmd
│        │        `notifySetpointsStop` (port 7 ch 1, byte 0)
│        │     5. transition to HL `go_to` (priority=HIGHLEVEL=1)
│        │     6. run sentai_calib_run_bringup() as normal
│        │   Pre-work: backout iter-85+ SENSOR_TOF_SIM cheat patches
│        │   (done); FR + journal auto-start at sentai_sim boot
│        │   (done as T10); gt_recorder auto-start via
│        │   sim/scripts/launch_sim.sh (done as T11).
│        │   Iter sweep: T_BASE / T_MAX / RAMP_S tuning per
│        │   drone mass — future FW-C: auto-learn ramp.
│        ├── OP-S10-W21-T8 — coplanar multi-marker PnP (vision Z)    ⬜ FW
│        │   Replace single-marker Krajník depth-from-ring (perspective
│        │   biased) with DLT/homography-on-plane PnP using all 6
│        │   pixel positions + marker_world.  Unblocks: unbiased Z →
│        │   ExtPos doesn't poison cf2 EKF (iter-13 positive-feedback
│        │   crash).  Sized at ~100-200 LoC C++.
│        ├── OP-S10-W21-T9 — sentai.flow Z-from-vision rewrite       ⬜ FW
│        │   Current sentai.flow assumes cf2 EKF provides altitude
│        │   for metric scaling — circular dependency.  Rewrite to
│        │   estimate Z from image scale (object size change) or
│        │   monocular SLAM-lite.  Required if we want flow as the
│        │   real-HW XY anchor independently of vision PnP.
│        ├── OP-S10-W21-T10 — sentai.fr + journal auto-start at boot ✅ SHIPPED (2026-05-22)
│        │   sim/main_sim.c: events.csv + scalars.csv auto-opened at
│        │   $SENTAI_FR_DIR or $SENTAI_SIM_ROOT/fr/.
│        │   Journal_open via mp_embed_exec_str("...").
│        │   Missions can still re-open with per-experiment paths.
│        ├── OP-S10-W21-T11 — gt_recorder auto-start in launch       ✅ SHIPPED (2026-05-22)
│        │   sim/scripts/launch_sim.sh — host-side wrapper that calls
│        │   launch_hybrid_cf2.sh (distrobox) + camera bridge +
│        │   gt_recorder.  Companion launch_sim_cleanup.sh.
│        └── OP-S10-W21-T12 — Calib RPYT-only cascaded PD            ⬜ IN-PROGRESS (2026-05-22, s191)
│            Replaces T7.  T7's RPYT→HL handoff bug was solved
│            (commit e0d5a67b) but bringup orchestrator's
│            hover()/HL commands still cause HELICAL DRIFT (operator
│            visual: 'incepe un drift ciudat elicoidal') because cf2
│            without mag/lighthouse has ~1°/s yaw drift — over 7s
│            bringup that's ~7° spiral.
│            New approach: STAY ON CLASSIC COMMANDER (CRTP 3/0) for
│            the entire calibration.  Mission-side cascaded PD on
│            X/Y/Z/Yaw using PnP feedback.  YAW LOCK eliminates
│            helical drift.  Implementation strategy: MP-FIRST then
│            promote to C++ in sentai.calib namespace once tuned.
│            Pre-condition: T7's PD takeoff (iter-21 GT peak 0.91m
│            stable) is REUSED — only the post-handoff path changes.
│            Reuses existing primitives: sentai.calib.commit_R,
│            commit_kp, save.  Adds: X-PD, Y-PD, Yaw-PD in mission.
│
└── Milestones
    ├── OP-M1 — Thesis MVP (SIM): 4 descriptors + L1 + calib working end-to-end
    ├── OP-M2 — ARM bring-up done: same source compiles + runs on RT1176 over radio
    ├── OP-M3 — Indoor L7 demo: 20-run validation, drift < 15 cm, ≥ 95% success
    ├── OP-M4 — Outdoor demo: 2 PX4 flights w/ GPS ground truth
    └── OP-M5 — Defense ready: evaluation chapter + writeup complete
```

## 3. Legacy → canonical mapping

For every legacy code still appearing in repo/memory, the canonical
WBS replacement:

| Legacy | Canonical | Where it appeared |
|---|---|---|
| `Stage 1` | `OP-S1` | objects_plan.md §3 |
| `Stage 1.A`, `1.B`, `1.C` | `OP-S1-W1` (smoke harness) etc. | objects_plan.md §10 |
| `Stage 4.A` | `OP-S4-W2` (backend dispatch) | objects_plan.md, commit 58f47bd4 |
| `Stage 4.5` | `OP-S4.5` | objects_plan.md §3 |
| `Stage 6` (calib + loop closure) | `OP-S6` | objects_plan.md §3 + §23.2 |
| `Stage 9` (ARM bring-up) | `OP-S9` | objects_plan.md §3 |
| `Stage 10` (hardening) | `OP-S10` | objects_plan.md §3 |
| `Stage 11` (places, §13.6 + §15.x) | folded into `OP-S10-W{1..6}` | research-addendum naming; no separate stage |
| `A1` (servo paradigm migration) | `OP-S4-W3-T{1..4}` (4 missions migrated) | next_steps_2026_05_17.md |
| `A2` (L1 tracker) | `OP-S10-W7` | next_steps_2026_05_17.md |
| `A3` (Track A descriptors HSV+FFT) | `OP-S10-W4`, `OP-S10-W5` | next_steps_2026_05_17.md |
| `A4` (calib) | `OP-S6-W1` | next_steps_2026_05_17.md |
| `A5` (ARM bring-up) | `OP-S9` | next_steps_2026_05_17.md |
| `A6` (L7 indoor demo) | `OP-S10-W8` (also `OP-M3` milestone) | next_steps_2026_05_17.md |
| `A7` (outdoor PX4) | `OP-S10-W9` (also `OP-M4`) | next_steps_2026_05_17.md |
| `A8` (evaluation chapter) | `OP-S10-W10` (also `OP-M5`) | next_steps_2026_05_17.md |
| `W1` (calendar) | tag, paired with `OP-S6-W1` | short_term_plan |
| `W2` (calendar) | tag, paired with `OP-S10-W4` | short_term_plan |
| `W3` (calendar) | tag, paired with `OP-S10-W5` + `OP-S10-W6` | short_term_plan |
| `W4` (calendar) | tag, paired with `OP-S10-W7` | short_term_plan |
| `L1 ego-motion` (code layer) | `ARCH-L1` | gap-analysis tables |
| `L2 detector` | `ARCH-L2` | gap-analysis tables |
| `L3 tracker` | `ARCH-L3` | gap-analysis tables |
| `L4 map EKF` | `ARCH-L4` | gap-analysis tables |
| `L5 state machine` | `ARCH-L5` | gap-analysis tables |
| `L6 controller` | `ARCH-L6` | gap-analysis tables |
| `L7 demo` | `ARCH-L7` (whole-system integration) | objects_plan.md §23.1 |
| `Track A` (no-DNN places) | tag (descriptor family); WPs go under `OP-S10-W{1..6}` | objects_plan.md §22 |
| `Track B` (DNN places) | tag; maps to `FW-1` per §23.3 | objects_plan.md §22 + FutureWork.md |
| `F-AC-1`, `F-AC-2` | keep `F-AC-{NN}` shape | sim/ANTI_CHEAT.md |
| `FW1..FW17` | keep `FW-{NN}` shape | FutureWork.md |
| `s100..s163` | keep as `EXP-s{NNN}` shape (folder name unchanged) | experiments/ |
| `Phase 1a/1b` (sentai_prep) | retroactively `OP-S10-W11-T1` / `T2` | commits `b104d77e` / `90b0523b` lacked the WBS prefix — flagged 2026-05-17 audit, going forward Phase 1c+ use OP-S10-W11-T{K} prefixes |

## 4. Hard rule (write into CLAUDE.md)

> Every plan artifact in this repo (markdown docs in `ideas/`,
> commit subject lines, experiment folder READMEs, memory entries,
> task tracker entries, PR descriptions) MUST reference its canonical
> WBS code from `ideas/wbs.md`.
>
> **Append-only**: never renumber, rename, or reorder existing WBS
> codes — they are stable identifiers like JIRA tickets.  New planning
> items append the next available code.  Adding a new Stage
> (`OP-S11+`) requires explicit operator approval since the 10-stage
> roadmap is frozen per `objects_plan.md` §3.
>
> **No ad-hoc letters**: do not invent `Stage X.A`, `A9`, `W5` etc.
> If you need finer granularity than what `wbs.md` provides today,
> append the next `OP-S{N}-W{M}-T{K}` code.
>
> **Experiments**: keep the `sNNN_<name>/` folder convention.  Their
> README headers must list which `OP-S{N}-W{M}` work package they
> validate.  Memory entries that capture experiment outcomes get the
> WBS code in their description: front-load it.
>
> **Commit subject convention**:
> `OP-S6-W1-T4: EXP-s157 calib smoke — perturb ±5° → recover 0.3°`
> i.e. `<WBS-code>: <one-line summary>`.  Multi-WBS commits list
> primary first, then `+ OP-S?-W?` etc.

## 5. Cross-references

- `objects_plan.md` §3 — the 10-stage roadmap (anchor for OP-S{N})
- `objects_plan.md` §23 — thesis MVP scope (anchor for what's in/out)
- `objects_plan.md` §4 — risk register (anchor for RISK-{NN})
- `FutureWork.md` — FW-{NN} entries
- `sim/ANTI_CHEAT.md` — F-AC-{NN} features
- `experiments/README.md` — EXP-s{NNN} convention
- `CLAUDE.md` — hard rule lives in top of file
- Memory: [[wbs-pmp-2026-05-17]]
