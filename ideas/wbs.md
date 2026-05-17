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
│   └── PX4 SITL pending OP-S9 ARM bring-up
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
│   └── OP-S10-W11 — sentai_prep frame slot pipeline              🟡 IN PROGRESS (2026-05-17)
│       │   Cross-cutting frame producer (PrepTask fan-out) + SlamTask
│       │   InferTask-style consumer.  Foundation for places.compute_*
│       │   _from_camera variants per [[no-heavy-data-through-mp]].
│       ├── OP-S10-W11-T1 — sentai_prep.{h,cc} foundation        ✅ SHIPPED (commit b104d77e — retroactively-labeled "Phase 1a")
│       ├── OP-S10-W11-T2 — PrepTask SLOT_GRAY_NATIVE + grab_gray refactor  ✅ SHIPPED (commit 90b0523b — retroactively-labeled "Phase 1b")
│       ├── OP-S10-W11-T3 — slam_task.cc (perception loop)        ⬜ TODO (Phase 1c)
│       ├── OP-S10-W11-T4 — SIM mirror in camera_bridge_recv.c    ⬜ TODO (Phase 1d)
│       └── OP-S10-W11-T5 — EXP-s162 live scene-discrimination    ⬜ TODO (Phase 1e)
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
