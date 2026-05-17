# Implementation Plan — Autonomous Object-Driven Drone Navigation on RT1176

> Stage-by-stage plan to close the gap between
> `examples/sentai_runtime/` (the existing `sentai.*` namespace) and the
> architectural vision in `ideas/objects.md` (6 layers + 3 hierarchical
> state machines; target scenario: red-cube detect & land in a 20×20 m
> indoor space).
>
> Design discipline: NASA/JPL principles from
> `examples/sentai_runtime/agent/embeded.md` — bounded, deterministic,
> observable, recoverable.  Every stage has a fault model, a resource
> budget, measurable PASS criteria, and a SIM↔ARM port.

> ⚠ **Scope management (2026-05-15)**: the plan is frozen at
> **thesis-MVP scope** for PhD defense.  See **§23** below for the
> exact in-scope / out-of-scope list with the demo target as north
> star.  Deferred content (moved sub-sections) lives in
> [`FutureWork.md`](FutureWork.md) with documented promotion criteria.
> Add new ideas to `FutureWork.md`, not here, to keep this execution
> plan uncluttered.

> 🗂 **Structure (2026-05-17)**: this file is the **MASTER INDEX**.
> §0 executive summary and §23 thesis-MVP scope are kept here (hoisted
> for quick reference).  All other long-form chapters were split out
> into `objects_plan/*.md` to keep this file scannable.
>
> Section anchors `§N.M` are **stable globally** across the split —
> external references such as `objects_plan.md §12.3` still resolve via
> `grep -rn "§12.3" ideas/objects_plan/`.  The 10-stage roadmap
> (§3 → chapter 01) is the primary anchor for `OP-S{N}` codes per
> [`wbs.md`](wbs.md).
>
> WBS hard rule (per `wbs.md` + `CLAUDE.md`): every new plan artifact
> MUST reference its canonical WBS code.  Codes are append-only; no
> ad-hoc letters such as `Stage X.A` / `A9` / `W5`.

> 🌐 **Language**: all project documentation, code, and comments are
> in English.  Romanian is reserved for chat with the operator.

---

## 0. Executive summary

**What already exists** (~75% of the low-level infrastructure):

| Layer concept | In `sentai.*` today | Status |
|---|---|---|
| L1 Ego-motion EKF | `sentai.imu.*`, `sentai.flow.*` (PXP + phase-corr), `sentai.flow.anchor_forward` | Observations (flow, anchor) are fed into the PX4 / cf2 EKF.  **No EKF of our own runs on the M7.** |
| L2 Detector NN | `sentai.tpu.*` (EdgeTPU + multi-class YOLO), `sentai.camera.*` | Full pipeline at 41 fps; a task-specific (cube) model is missing. |
| L3 2D Tracker | `sentai_tracker.cc` (BoT-SORT-lite + ByteTrack + IMU CMC + histograms) | **Almost complete** — TENTATIVE/CONFIRMED/LOST states exist; IMU CMC behind a flag. |
| L4 Map EKF 3D | `sentai_aruco_shim` (anchored on KNOWN markers) | **Big gap**: no inverse-depth EKF for UNKNOWN objects. |
| L5 State machines | `sentai.pipeline.start/stop`, manual REPL/Python | **Big gap**: SM2 mission + SM3 health in C++ on the M7 are missing. |
| L6 Controller PBVS/IBVS | Velocity setpoints constructed in a Python sidecar, sent through `sentai.link` / `sentai.crazy` | **Big gap**: action layer + PBVS/IBVS in firmware are missing. |
| Anti-brick + watchdog + dmesg + ITCM budget | All production-grade | ✅ |
| Platform abstraction SIM ↔ ARM | `sentai_pxp_shim`, `sentai_fft_shim`, `sentai_aruco_shim` | Pattern proven. |

**The four big pieces to build**:
1. **Object map** (`sentai.objects.*`) — static 32-slot array + inverse-depth EKF
2. **Mission SM** (`sentai.mission.*`) — hierarchical SM1 + SM2 + SM3, intent emitter
3. **Action layer** (`sentai.servo.*`) — intent → velocity setpoint + PBVS/IBVS
4. **Domain-specific TPU model** — YOLOv8n trained for the red-cube + distractors scenario

**MCU/ARM feasibility verdict** (detailed in §11):
- ✅ **All four new modules fit in < 3 % M7 CPU** (compute is not the bottleneck — the existing pipeline already uses 48 % for TPU + tracker).
- ✅ **0 KB of ITCM** consumed by the new code (everything lives in `.sdram_text`; ITCM headroom stays at ~32 KB).
- ✅ **~30 KB SDRAM** total for code + data (16 MB available).
- ✅ **Hard single-precision FPU + CMSIS-DSP `arm_mat_*`** covers the EKF / quaternion / Kalman math.
- ⚠️ **The single real risk**: numerical stability of the inverse-depth EKF (Stage 5) — mitigated via Joseph form + ρ_min clamp + filter divergence guard.
- 🚫 **L1 ego-motion EKF on the M7 ourselves**: feasible (~1.5 % CPU) BUT redundant with the PX4 EKF2 / cf2 KF → out of scope, defer.

**Critical path** (~3–4 weeks of focused development):
```
Stage 1 → Stage 2 → Stage 3 → Stage 4 → Stage 5 → Stage 6 → Stage 7 → Stage 8
data layer  TPU model   SM2 minimal  action layer  inverse-depth  loop closure  full SM3  ARM perf
                          (cf2 SITL)                              (yaw correction)
```

---

## 23. Thesis-MVP scope (frozen 2026-05-15)

Companion to `FutureWork.md`.  Locks the in-scope vs. out-of-scope
decision for the PhD thesis to prevent scope creep.  Items move
bidirectionally between this plan and `FutureWork.md` as priorities
shift; both files date such moves.

### 23.1 Demo target (north star)

> **Indoor**: the drone takes off in a ~5×5 m room containing 4–6
> physical objects (ArUco markers + objects recognized by a custom
> on-board DNN).  Using only on-board perception (no MoCap, no ground
> station), it autonomously explores the environment, builds a 3D map
> indexed hexagonally, visits each object on operator command (over
> the radio REPL: "go to object X", "return to origin"), and lands at
> the takeoff point with accumulated drift < 15 cm.  Total flight time
> ~90 seconds.  Power log ≤ 1.2 W for the SentAI stack.  Repeatable
> ≥ 95 % across 20 consecutive runs.
>
> **Outdoor**: 2–3 missions on PX4 + a custom Crazyflie/drone with
> GPS ground truth.  The drone flies a waypoint pattern with ArUco
> markers at known positions.  Validates: (a) the same stack runs on
> PX4 (generality), (b) drift bounded to ~50 cm over 60–90 outdoor
> seconds, (c) the custom DNN recognizes ≥ 3 outdoor categories.

All priorities flow from this target.

### 23.2 In-scope (must ship for thesis)

| Element | Status | Note |
|---|---|---|
| **Foundation infra** (camera, TPU, flow, radio, USB, FS, SIM) | ✅ shipped | |
| **L2** `sentai.objects` | ✅ shipped | frozen API |
| **L3** `sentai.places` (H3-indexed) | ✅ shipped | frozen API |
| **L4** `sentai.servo` | ✅ shipped | frozen API |
| **L4.5** image-only nav (s130) | ✅ shipped | |
| **L5** `sentai.object_lifter` | ✅ shipped | s132 validated under Gazebo |
| **L1 tracker minimal** | TODO | required: stable tracklet_id for L5 (may be ArUco-id-as-tracklet for thesis MVP) |
| **L6** `sentai.explore` mission FSM | ✅ shipped (skeleton, s133) | 10 states, SIM smoke 100/100 PASS; Gazebo integration follows as s134 (PASS gate ≥ 80 % / 10 runs) |
| **Stage 6** `sentai.calib` on-board MP binding | TODO | mandatory before HW indoor flight (§21) |
| **Track A places** minimal (PHOG + GIST + HSV + FFT) | TODO | s133 → s134 → s135 → s136 → s137 |
| **Stage 9** ARM bring-up + DWT timing | TODO | validates "runs on the real MCU" — the essence of the thesis |
| **L7** integrated indoor demo | TODO | live demo, scenario §23.1 |
| **Outdoor PX4 experiments** (2–3 runs) | TODO | generality validation, GPS ground truth |
| **DNN models on-board** | TODO (parallel) | separate track |
| **Quantitative evaluation chapter** | TODO | drift/min, success rate, latency, power; numbers for defense |

### 23.3 Out-of-scope (moved to `FutureWork.md`)

| Item | FW # | Originally |
|---|---|---|
| DNN places encoder (Track B) | FW1 | §22.3 |
| SeqSLAM temporal | FW2 | §14.4 |
| ORB + VLAD (Strategy C) | FW3 | §14.3 |
| Rotation-equivariant CNN (D) | FW4 | §14.3 |
| Adaptive H3 subdivision | FW5 | §15.4 |
| Multi-altitude descriptor storage | FW6 | Stage 11.D, §15.5 |
| Multi-drone mesh gallery | FW7 | §15.12.3 |
| Multi-level honeycomb advanced | FW8 | §16 |
| Cross-scale composition | FW9 | §17 |
| Persistent gallery across boots | FW10 | §15.7 |
| Outdoor lat/lng / OSM | FW11 | §15.7 |
| Full SM3 COAST/ALIGN | FW12 | Stage 7 (partial — minimal stays) |
| Stage 8 standalone | FW13 | absorbed into L7 |
| Macro-category detection | FW14 | §13.3 |
| Auto-merge similar children | FW15 | §15.12.1 |
| 4D hex (time bucket) | FW16 | §15.12.2 |

**Note**: the original sections still live inside the chapter files
under `objects_plan/` for their literature-review value.
`FutureWork.md` lists only the implementation outline and deferral
rationale.  Bibliography stays here.

### 23.4 Thesis-specific risk register

| Risk | Severity | Mitigation |
|---|---|---|
| Drift accumulated in the live indoor demo fails | HIGH | (a) ArUco loop closure as primary (s130 proven 5 cm); (b) Track A places as secondary novelty; (c) rehearse 20× beforehand, measure the 95th percentile |
| DNN training + ObjectsPlan diverge | HIGH | define the DNN↔lifter interface EARLY; DNN can fall back to ArUco if late |
| Outdoor texture-less / variable lighting | MEDIUM | (a) choose an outdoor location with mechanical landmarks / test polygon; (b) large known ArUco anchors; (c) cap at 2–3 outdoor missions — not a robust generic solution |
| Scope creep (plan grows, implementation lags) | HIGH | **NOW**: freeze §23.2, refuse §24+; capture new ideas in `FutureWork.md` |
| Defense reviewers demand missing numbers | MEDIUM | maintain a `drift/altitude/object-count` benchmark table from the START, measured at every commit |
| HW Stage 9 reveals issues not caught in SIM | reduced (HW ready) | early bring-up with telemetry instrumentation |
| Single point of failure (single developer) | persistent | document everything; write code such that someone else can resume it |

### 23.5 Action plan post-s132 (ordered)

1. **L6 `sentai.explore` minimal**: FSM EXPLORE → APPROACH → INSPECT → RETURN → LAND
   - Radio-commandable via REPL: `sentai.explore.goto(object_id)` /
     `sentai.explore.return_home()` etc.
   - Uses the existing L5 objects and L3 places.
   - PASS gate: SIM indoor mission success ≥ 80 % over 10 runs.
2. **s133–s137 Track A places** (per §22.5 design): minimal version
   GIST + HSV + FFT-mag → 64 B → H3 → loop closure.
3. **Stage 6 `sentai.calib`** on-board MP binding (port "Pas 2" from
   `_shared/camera_calibration.py` to `sentai_runtime/modsentai_calib.c`).
4. **Stage 9 ARM bring-up**: ARM build, flash, smoke test, DWT timing
   measurement per pipeline component.
5. **L7 indoor integrated demo** — proof of the §23.1 scenario.
6. **DNN integration** (parallel tracks: you on DNN training, me on
   the `sentai_runtime` integration hook in `detection_task`).
7. **Outdoor PX4 experiments** — 2–3 runs with telemetry log.
8. **Evaluation chapter** — all the numbers in one place.

### 23.6 Cross-references

- `FutureWork.md` — deferred items + promotion criteria
- `[[s132-lifter-gazebo-shipped]]` — last completed milestone
- `[[places-two-track-decision]]` — §22 Track A/B split
- `[[camera-mount-calibration]]` — §21 Stage 6 motivation
- `[[gate-every-layer-no-exceptions]]` — discipline rule
- `[[experiments-start-from-origin]]` — reproducibility rule

---

*Frozen 2026-05-15.  Next review: post-L6 shipping (estimated 2026-05-22).
Movement between §23 in-scope and `FutureWork.md` is allowed at any
review point; both files date the move.*

---

## Table of contents — chapters

Long-form content lives in `objects_plan/*.md`.  Each chapter file
preserves the original `§N.M` section numbering as stable anchors.

| Chapter | Sections | Title | WBS / purpose |
|---|---|---|---|
| [00 Methodology](objects_plan/00_methodology.md) | §1, §2, §4–§10, §7.5 | NASA/JPL principles + budgets + risks + gates + dep graph + arch map + validation + kickoff | Cross-cutting discipline; `RISK-{NN}` live here |
| [01 Roadmap](objects_plan/01_roadmap.md) | §3 | 10-stage roadmap (Stage 1..10 + Stage 4.5) | **Primary anchor for `OP-S{N}`** |
| [02 MCU feasibility](objects_plan/02_mcu_feasibility.md) | §11 | Detailed cycle/memory budgets per `ARCH-L{1..7}` | Feasibility validation |
| [03 Research: constellation pose](objects_plan/03_research_constellation.md) | §12 | Multi-object Yaw-Wahba + EKF stack SOTA | `OP-S6-W2` candidate background |
| [04 Research: topo nav](objects_plan/04_research_toponav.md) | §13 | Pigeon-style topological navigation + global scene fingerprint | `sentai.places` background |
| [05 Research: places fingerprint](objects_plan/05_research_places.md) | §14 | Rotation invariance + opposite-direction recall | `OP-S10-W{4..6}` background |
| [06 Research: H3 hex indexing](objects_plan/06_research_hex.md) | §15 | Uber H3 hexagonal spatial indexing | L3 `sentai.places` backbone |
| [07 Research: honeycomb](objects_plan/07_research_honeycomb.md) | §16–§17 | Multi-level honeycomb + cross-scale composition (MOVED to FutureWork) | Stub only; see `FW-8` / `FW-9` |
| [08 Mission `sentai.explore`](objects_plan/08_explore_mission.md) | §18 | Two-flight exploration FSM SM2 | `OP-S3` / `ARCH-L5` |
| [09 Namespace audit](objects_plan/09_namespace_audit.md) | §19 | `sentai.*` policy decisions, noun-vs-verb mapping | Cross-cutting |
| [10 Bibliography](objects_plan/10_bibliography.md) | §20 | SOTA references | Literature citations |
| [11 Camera calib](objects_plan/11_camera_calib.md) | §21 | Camera-to-body extrinsic auto-calibration at takeoff | `OP-S6-W1` spec |
| [12 Places — two tracks](objects_plan/12_places_two_track.md) | §22 | Track A no-DNN / Track B DNN decision | `OP-S10-W{1..6}` = Track A; Track B → `FW-1` |
| [13 DNN dynamic objects](objects_plan/13_dnn_dyn_objects.md) | §24 | DNN dynamic-object detection — separate concern from flow/EKF | Parallel track |

## Cross-references

- WBS spec + hard rule: [`wbs.md`](wbs.md)
- FutureWork parking lot: [`FutureWork.md`](FutureWork.md)
- Short-term plan (current): [`next_steps_2026_05_17.md`](next_steps_2026_05_17.md)
- Research sanity check: [`research_review_2026_05_16.md`](research_review_2026_05_16.md)
- Original conceptual doc: [`objects.md`](objects.md)
- AirREPL mini-paper plan: [`AirREPL_paper.md`](AirREPL_paper.md)

## Change log

- **2026-05-17**: split into chapter files under `objects_plan/`; §0 + §23
  hoisted into this index.  Doc language switched fully to English.
  Pre-split version preserved in git at HEAD~1.
- **2026-05-15**: frozen §23 thesis-MVP scope; `FutureWork.md` spun off.
- See `git log` for older revisions.
