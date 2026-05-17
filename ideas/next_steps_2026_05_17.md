# Next steps — ObjectsPlan in-scope remainder (2026-05-17)

Plan strict scoped to `objects_plan.md` §23.2 thesis-MVP items
that are still TODO.  Research directions from
`research_review_2026_05_16.md` are sanity-check / FutureWork material;
they do NOT belong in this plan.

WBS codes per `ideas/wbs.md`.  Calendar weeks are tags only.

## Status table (post Stage 4.A landing, commit 58f47bd4, 2026-05-17)

| §23.2 element | WBS code | Status | Notes |
|---|---|---|---|
| Foundation infra | (pre-OP) | ✅ shipped | |
| L2 sentai.objects | `OP-S1` / `ARCH-L2` | ✅ shipped | |
| L3 sentai.places (H3) | folded into `OP-S10-W{1..6}` | ✅ partial | PHOG+GIST+baseline+HexPatrol+loop closure |
| L4 sentai.servo | `OP-S4` / `ARCH-L6` | ✅ shipped + Stage 4.A backend dispatch (CF2 + PX4) | |
| L4.5 image-only nav | `OP-S4.5` | ✅ shipped (EXP-s130) | |
| L5 sentai.object_lifter | `OP-S5` / `ARCH-L4` | ✅ shipped (EXP-s132 Gazebo PASS) | |
| L6 sentai.explore | `OP-S3` / `ARCH-L5` | ✅ skeleton + EXP-s135..s138 SIM PASS | |
| **L1 tracker minimal** | `OP-S10-W7` / `ARCH-L3` | ⬜ TODO | §23.2 item |
| **Stage 6 sentai.calib** | `OP-S6-W1` | 🟡 NEXT | §23.2 — mandatory before HW indoor |
| **Track A places (PHOG+GIST+HSV+FFT)** | `OP-S10-W{1..6}` | 🟡 partial: 3/4 (PHOG s139, GIST s140, baseline s141) ship; HSV+FFT+opposite-dir TODO | |
| **Stage 9 ARM bring-up + DWT timing** | `OP-S9` | ⬜ TODO | §23.2 — "rulează pe MCU real" |
| **L7 integrated indoor demo** | `OP-S10-W8` (= `OP-M3`) | ⬜ TODO | §23.1 north star |
| **Outdoor PX4 experiments** | `OP-S10-W9` (= `OP-M4`) | ⬜ TODO | 2-3 runs |
| **DNN models on-board** | `FW-1` track + §24 chapter | ⬜ TODO (parallel track) | |
| **Quantitative evaluation chapter** | `OP-S10-W10` (= `OP-M5`) | ⬜ TODO | |

---

## Critical path (linear-sequenced per `objects_plan.md` §23.5)

### `OP-S4-W3` — Migrate s147/s148/s150/s151 → sentai.servo paradigm ✅ SHIPPED 2026-05-17

**Status**: 4/4 PASS in SIM with respawn-at-origin + closure-vs-world
(anti-cheat enforced):
- `EXP-s153_explore_long_servo` — closure 5.62 cm (gate 10 cm), total_path 97 cm, 2 WP
- `EXP-s154_lost_recovery_servo` — closure 3.22 cm, Δz_apex 59.6 cm (gate 15 cm)
- `EXP-s155_loop_closure_servo` — closure 2.53 cm (gate 12 cm), 3/3 round-trip matches
- `EXP-s156_realframe_loop_closure_servo` — closure 3.38 cm, 3/3 REAL-frame matches (camera bridge live)

**Bonus delivered same session**:
- `sim/scripts/respawn_sitl.sh` shared helper (enforces [[experiments-start-from-origin]])
- `F-AC-1`/`F-AC-2` anti-cheat rule sealed: `sentai_sim` AIR-GAPPED from gz ground truth, audit script + bridge allowlist
- `objects_plan.md` §24 — DNN dynamic-object detection chapter (separate concern)
- `scripts/ai_distill.sh` — local LLM distillation tool (qwen3.5:35b-a3b on hpc.lan)

### `OP-S10-W7` — L1 tracker minimal (§23.2 TODO)
**Why**: L5 lifter needs stable tracklet_id across frames.  Currently the EXP-s132 path uses ArUco-id-as-tracklet for thesis MVP per §23.2 note.  For the indoor demo (`OP-S10-W8`) with DNN-detected objects, we need a real 2D tracker.

- Choice already made in `sentai_tracker.cc`: BoT-SORT-lite + ByteTrack + IMU CMC + histograms.  TENTATIVE/CONFIRMED/LOST states already exist.
- TODO: wire `sentai_tracker.update()` output (track_id) into `sentai_object_lifter.observe(track_id, bearing)` end-to-end on a Gazebo scene with multiple non-ArUco objects.
- Validation: SIM smoke with 2-3 cubes of different colors, drone orbits, lifter accumulates 3 landmarks with stable ids ≥ 5 s each.
- Estimated: 1 week (mostly wiring; algorithm already exists).

### `OP-S10-W4` + `OP-S10-W5` + `OP-S10-W6` — Track A places: HSV + FFT-mag + opposite-direction
**Why**: §22.5 design ships PHOG + GIST + HSV + FFT-mag (4 components).  PHOG + GIST already in (EXP-s139/s140 = `OP-S10-W1`/`W2`).  HSV + FFT-mag remain.

- `OP-S10-W4` HSV: 8×8×8 = 512-bin histogram on hue/sat/val, project to 64 B via top-K bins.  Cold-path C++, mirror `sentai_phog.cc` shape.
- `OP-S10-W5` FFT-mag: log-polar FFT magnitude (rotation invariance per §14).  32-bin coarse grid → 64 B.  CMSIS-DSP `arm_rfft_fast_f32` on ARM, FFTW3 on SIM (existing shim).
- Each lands in `sentai_places.cc` as a new compute method + 64-byte slot variant.
- Validation: `EXP-s159_hsv_baseline` + `EXP-s160_fft_baseline` anti-regression gates (mirror s141 PHOG/GIST goldens).
- `OP-S10-W6` Open: opposite-direction recall ≥ 80% test per §14 — requires drone to fly through gallery in BOTH directions and match.
- Estimated: 2 weeks (1 week descriptor coding + 1 week validation + opposite-dir test).

### `OP-S6-W1` — Stage 6 sentai.calib on-board MP binding
**Why**: §23.2 mandatory before HW indoor demo.  Per [[camera-mount-calibration]] real hardware has ±2-5° mount tolerance per unit.

- Port `_shared/camera_calibration.py` Kabsch 3D Procrustes logic to C (`sentai_calib.cc`)
- Run at boot if calibrated ArUco marker visible; persist to `/system/cam_calib.json` via FxUser
- MP binding: `sentai.calib.run_takeoff_pad()`, `sentai.calib.get_R_cam_to_body()`, `sentai.calib.is_calibrated()`
- Validation: SIM round-trip — perturb known-good extrinsics in Gazebo, calibrate, recover within 0.5°
- Estimated: 1 week.

### `OP-S9` — ARM bring-up + DWT timing
**Why**: §23.2 thesis-essence.  "Rulează pe MCU real" — the central claim.

Prerequisites (decompose first):
- `OP-S9-W1` — Add generic LOG-channel RX FIFO in `sentai_crazy.cc` so `sentai_crazy_recv_pop` on ARM actually drains (today: weak stub returning 0; `sentai.crazy.pose_subscribe()` returns -2 on ARM).  Single 32-entry SPSC ring; rest of `crazy_rx_task` per-block routing stays unchanged.
- `OP-S9-W2` — Verify radio bridge per [[crazyflie-radio-bridge]] (board #1224 + drone 53d72897) carries CRTP LOG frames over the UART CPX channel.  Drone-side bridge already forwards CRTP host→radio; need to confirm reverse direction.

Main bring-up:
- `OP-S9-W3` — Flash sentai_runtime onto Coral Dev Board Micro, verify `sentai.version()` reports correct build #
- Smoke `sentai.servo.init(CF2)` + `pose_subscribe` + `pose()` over radio
- `OP-S9-W4` — Instrument each pipeline stage with DWT cycle counter:
  - Camera → PXP → flow phase-corr
  - TPU invoke + detection
  - Tracker update
  - Lifter EKF step
  - Servo dispatch + transport TX
- `OP-S9-W5` — Capture full pipeline 1 Hz for 60 s; produce table for thesis eval chapter
- Estimated: 2 weeks (1 week ARM LOG ring + radio verify; 1 week timing harness + capture)

### `OP-S10-W8` (= `OP-M3`) — L7 integrated indoor demo (§23.1 north star)
**Why**: thesis defense demo.  Must repeat 95% / 20 runs.

Composition: `OP-S4-W3` (servo paradigm) + `OP-S10-W7` (tracker) + `OP-S10-W{1..6}` (full Track A places) + `OP-S6-W1` (calib) + `OP-S9` (ARM bring-up) → drone autonomously explores 5×5m room, visits objects on radio REPL command, returns to origin with drift < 15 cm.

- New mission `mission_l7_indoor_demo.py`: combines `sentai.explore` FSM + `sentai.places.match_*` recall + radio REPL command parser
- Validation: 20 consecutive runs, drift histogram, success rate, success := all phases fired + closure < 15 cm + ≥ N/3 places recovered
- Estimated: 3 weeks (1 week composition + 1 week tuning under real lighting + 1 week 20-run validation campaign)

### `OP-S10-W9` (= `OP-M4`) — Outdoor PX4 experiments (2 runs minimum)
**Why**: §23.2 generality validation.  Same source compiles for PX4 backend (already wired in #47).

- ArUco landmarks at known poses, GPS ground truth log
- Mission: takeoff → 3-waypoint pattern → return, on PX4 SITL first then real PX4 frame
- Validation: same closure gate adapted (50 cm outdoor budget per §23.1)
- Estimated: 2 weeks (1 week PX4 frame setup + 1 week flight ops + log capture)

### `OP-S10-W10` (= `OP-M5`) — Quantitative evaluation chapter
**Why**: §23.2 numbers reviewers will demand.

- Drift/min vs altitude, object count, mission duration
- Success rate (mission completion + closure)
- Per-pipeline-stage latency (from `OP-S9-W4` DWT instrumentation)
- Power (board-level, USB inline ammeter)
- Compile into thesis evaluation chapter
- Estimated: 2 weeks pure writing.

---

## Sequencing (12-week horizon to defense-ready)

| Cal-week | WBS | Focus |
|---|---|---|
| W1 | `OP-S6-W1` | sentai.calib MP binding |
| W2-3 | `OP-S10-W4` + `OP-S10-W5` + `OP-S10-W6` | HSV + FFT-mag descriptors + opposite-dir recall |
| W4 | `OP-S10-W7` | L1 tracker wiring to lifter |
| W5-6 | `OP-S9-W{1..5}` | ARM bring-up (LOG ring + radio + DWT timing) |
| W7-9 | `OP-S10-W8` (= `OP-M3`) | L7 indoor demo + 20-run validation |
| W10 | `OP-S10-W9` (= `OP-M4`) | outdoor PX4 (2 runs) |
| W11-12 | `OP-S10-W10` (= `OP-M5`) | evaluation chapter writeup |

Slack: ~10% buffer absorbed into `OP-S10-W8` (lighting tuning is unpredictable).

---

## Anti-list (do NOT pull in from research review)

The 5 improvement directions in `ideas/research_review_2026_05_16.md`
(Schmidt-EKF, SeqSLAM, SQ-SLAM, binary-VPR short-circuit, thesis writeup
artifact) are **FutureWork material** (`FW-{NN}`), not in-scope per §23.2.

Per [[objectsplan-vs-futurework]]: research review = sanity check that
our choices stay aligned with literature; promotion criteria for any
direction to move from FutureWork into ObjectsPlan must be documented
(e.g., "Schmidt-EKF promotes iff inverse-depth lifter shows divergence
> 5% on real-world ARM bring-up").  None of the 5 currently meet a
promotion criterion.

---

## Cross-references

- WBS spec: `ideas/wbs.md`
- Scope freeze: `objects_plan.md` §23 (frozen 2026-05-15)
- FutureWork parking lot: `ideas/FutureWork.md`
- Sanity-check research review: `ideas/research_review_2026_05_16.md`
- Canonical mission pattern: `examples/sentai_runtime/experiments/s152_hex_patrol_servo/`
- SIL execution model: `Sim.md` §10z
- Memory: [[wbs-pmp-2026-05-17]], [[short-term-plan-2026-05-17]],
  [[missions-run-in-sentai-only]], [[sim-test-must-return-home]],
  [[test-must-be-relevant-to-claim]], [[gate-every-layer-no-exceptions]]
