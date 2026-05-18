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
│   ├── OP-S10-W11 — sentai_prep frame slot pipeline              🟡 IN PROGRESS (2026-05-17)
│   │   │   Cross-cutting frame producer (PrepTask fan-out) + SlamTask
│   │   │   InferTask-style consumer.  Foundation for places.compute_*
│   │   │   _from_camera variants per [[no-heavy-data-through-mp]].
│   │   ├── OP-S10-W11-T1 — sentai_prep.{h,cc} foundation        ✅ SHIPPED (commit b104d77e — retroactively-labeled "Phase 1a")
│   │   ├── OP-S10-W11-T2 — PrepTask SLOT_GRAY_NATIVE + grab_gray refactor  ✅ SHIPPED (commit 90b0523b — retroactively-labeled "Phase 1b")
│   │   ├── OP-S10-W11-T3 — slam_task.cc (perception loop)        ⬜ TODO (Phase 1c)
│   │   ├── OP-S10-W11-T4 — SIM mirror in camera_bridge_recv.c    ⬜ TODO (Phase 1d)
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
│       ├── OP-S10-W14-T16 — YawArucoBaseline experiment (operator
│       │                    2026-05-18: "drona se invarte 360° in
│       │                    cerc in timp ce hover-uieste la markeri").
│       │                    Validates: yaw control via hover.yaw_rate,
│       │                    PnP robustness under camera rotation,
│       │                    necessity of PnP-DERIVED quaternion in
│       │                    ExtPose (T13's identity quat would fight
│       │                    the commanded rotation).  Prereq: extend
│       │                    sentai_calib_task to compute true yaw
│       │                    from aruco rvec_cam and send via ExtPose.
│       │                                                              ⬜ TODO (phase 2)
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
