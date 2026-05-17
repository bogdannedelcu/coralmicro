# Next steps — ObjectsPlan in-scope remainder (2026-05-17)

Plan strict scoped to `ideas/objects_plan.md` §23.2 thesis-MVP items
that are still TODO. Research directions from `research_review_2026_05_16.md`
are sanity-check / FutureWork material; they do NOT belong in this plan.

Updated tally of §23.2 status post sentai.servo Stage 4.A landing
(commit 58f47bd4, 2026-05-17):

| §23.2 element | Status | Notes |
|---|---|---|
| Foundation infra | ✅ shipped | |
| L2 sentai.objects | ✅ shipped | |
| L3 sentai.places (H3) | ✅ shipped | |
| L4 sentai.servo | ✅ shipped — **now with Stage 4.A backend dispatch (CF2 + PX4)** | |
| L4.5 image-only nav (s130) | ✅ shipped | |
| L5 sentai.object_lifter | ✅ shipped (s132 Gazebo PASS) | |
| L6 sentai.explore | ✅ skeleton + s135-s138 SIM PASS | Stage A1 below extends |
| **L1 tracker minimal** | TODO | item §23.2 |
| **Stage 6 sentai.calib (on-board MP binding)** | TODO | item §23.2 — mandatory before HW indoor |
| **Track A places minimal (PHOG+GIST+HSV+FFT)** | partial: PHOG (s139) + GIST (s140) + baseline (s141) + HexPatrol (s142) + loop closure (s143-s144) all SIM PASS | HSV + FFT-mag descriptors still TODO; cross-track validation against opposite-direction recall ≥ 80% TODO |
| **Stage 9 ARM bring-up + DWT timing** | TODO | item §23.2 — "rulează pe MCU real" |
| **L7 integrated indoor demo (§23.1 north star)** | TODO | item §23.2 |
| **Outdoor PX4 experiments (2-3 runs)** | TODO | item §23.2 |
| **DNN models on-board** | TODO (parallel track, not on this plan) | |
| **Quantitative evaluation chapter** | TODO | item §23.2 |

---

## Critical path (linear-sequenced per §23.5)

### A1. Migrate s147/s148/s150/s151 → sentai.servo paradigm
**Why first**: closes [[s147-s151-migrations-shipped]] loop and validates Stage 4.A across 5 mission shapes before anything new ships on top of it. s152 already PASS as the template.

- Drop `import crtp_log`, replace `sentai.crazy.*` with `sentai.servo.*`
- Apply the `_converge_to(settle_ms / poll_ms)` fix from s152 mission (settle_ms < poll_ms invariant)
- Each migrated mission must pass its existing closure gate (10 cm single-lap, 12 cm 2-lap per [[sim-test-must-return-home]])
- Estimated: 3-4h per mission × 4 = ~2 days
- Output: `s153_s149_servo`, `s154_s147_servo`, `s155_s148_servo`, `s156_s150_servo`, `s157_s151_servo` (or rename in-place if cleaner)

### A2. L1 tracker minimal (§23.2 TODO)
**Why**: L5 lifter needs stable tracklet_id across frames. Currently the s132 path uses ArUco-id-as-tracklet for thesis MVP per §23.2 note. For the indoor demo (L7) with DNN-detected objects, we need a real 2D tracker.

- Choice already made in `sentai_tracker.cc`: BoT-SORT-lite + ByteTrack + IMU CMC + histograms. TENTATIVE/CONFIRMED/LOST states already exist.
- TODO: wire `sentai_tracker.update()` output (track_id) into `sentai_object_lifter.observe(track_id, bearing)` end-to-end on a Gazebo scene with multiple non-ArUco objects.
- Validation: SIM smoke with 2-3 cubes of different colors, drone orbits, lifter accumulates 3 landmarks with stable ids ≥ 5 s each.
- Estimated: 1 week (mostly wiring; algorithm already exists).

### A3. Track A places: complete HSV + FFT-mag descriptors
**Why**: §22.5 design ships PHOG + GIST + HSV + FFT-mag (4 components). PHOG + GIST already in (s139/s140). HSV + FFT-mag remain.

- HSV: 8×8×8 = 512-bin histogram on hue/sat/val, project to 64 B via top-K bins. Cold-path C++, mirror `sentai_phog.cc` shape.
- FFT-mag: log-polar FFT magnitude (rotation invariance per §14). 32-bin coarse grid → 64 B. CMSIS-DSP `arm_rfft_fast_f32` on ARM, FFTW3 on SIM (existing shim).
- Each lands in `sentai_places.cc` as a new compute method + 64-byte slot variant.
- Validation: s158_hsv_baseline + s159_fft_baseline anti-regression gates (mirror s141 PHOG/GIST goldens).
- Open: opposite-direction recall ≥ 80% test per §14 — requires drone to fly through gallery in BOTH directions and match.
- Estimated: 2 weeks (1 week descriptor coding + 1 week validation + opposite-dir test).

### A4. Stage 6 sentai.calib on-board MP binding
**Why**: §23.2 mandatory before HW indoor demo. Per [[camera-mount-calibration]] real hardware has ±2-5° mount tolerance per unit.

- Port `_shared/camera_calibration.py` Kabsch 3D Procrustes logic to C (`sentai_calib.cc`)
- Run at boot if calibrated ArUco marker visible; persist to `/system/cam_calib.json` via FxUser
- MP binding: `sentai.calib.run_takeoff_pad()`, `sentai.calib.get_R_cam_to_body()`, `sentai.calib.is_calibrated()`
- Validation: SIM round-trip — perturb known-good extrinsics in Gazebo, calibrate, recover within 0.5°
- Estimated: 1 week.

### A5. Stage 9 ARM bring-up + DWT timing
**Why**: §23.2 thesis-essence. "Rulează pe MCU real" — the central claim.

Prerequisites (decompose first):
- Add generic LOG-channel RX FIFO in `sentai_crazy.cc` so `sentai_crazy_recv_pop` on ARM actually drains (today: weak stub returning 0; `sentai.crazy.pose_subscribe()` returns -2 on ARM). Single 32-entry SPSC ring; rest of `crazy_rx_task` per-block routing stays unchanged.
- Verify radio bridge per [[crazyflie-radio-bridge]] (board #1224 + drone 53d72897) carries CRTP LOG frames over the UART CPX channel. Drone-side bridge already forwards CRTP host→radio; need to confirm reverse direction.

Main bring-up:
- Flash sentai_runtime onto Coral Dev Board Micro, verify `sentai.version()` reports correct build #
- Smoke `sentai.servo.init(CF2)` + `pose_subscribe` + `pose()` over radio
- Instrument each pipeline stage with DWT cycle counter:
  - Camera → PXP → flow phase-corr
  - TPU invoke + detection
  - Tracker update
  - Lifter EKF step
  - Servo dispatch + transport TX
- Capture full pipeline 1 Hz for 60 s; produce table for thesis eval chapter
- Estimated: 2 weeks (1 week ARM LOG ring + radio verify; 1 week timing harness + capture)

### A6. L7 integrated indoor demo (§23.1 north star)
**Why**: thesis defense demo. Must repeat 95% / 20 runs.

Composition: A1 (servo paradigm) + A2 (tracker) + A3 (full Track A places) + A4 (calib) + A5 (ARM bring-up) → drone autonomously explores 5×5m room, visits objects on radio REPL command, returns to origin with drift < 15 cm.

- New mission `s160_l7_indoor_demo.py`: combines `sentai.explore` FSM + `sentai.places.match_*` recall + radio REPL command parser
- Validation: 20 consecutive runs, drift histogram, success rate, success := all phases fired + closure < 15 cm + ≥ N/3 places recovered
- Estimated: 3 weeks (1 week composition + 1 week tuning under real lighting + 1 week 20-run validation campaign)

### A7. Outdoor PX4 experiments (2 runs minimum)
**Why**: §23.2 generality validation. Same source compiles for PX4 backend (already wired in #47).

- ArUco landmarks at known poses, GPS ground truth log
- Mission: takeoff → 3-waypoint pattern → return, on PX4 SITL first then real PX4 frame
- Validation: same closure gate adapted (50 cm outdoor budget per §23.1)
- Estimated: 2 weeks (1 week PX4 frame setup + 1 week flight ops + log capture)

### A8. Quantitative evaluation chapter
**Why**: §23.2 numbers reviewers will demand.

- Drift/min vs altitude, object count, mission duration
- Success rate (mission completion + closure)
- Per-pipeline-stage latency (from A5 DWT instrumentation)
- Power (board-level, USB inline ammeter)
- Compile into thesis evaluation chapter
- Estimated: 2 weeks pure writing.

---

## Sequencing (12-week horizon to defense-ready)

| Week | Focus |
|---|---|
| 1 | A1 migrate s147/s148/s150/s151 (paradigm closure) |
| 2-3 | A3 HSV + FFT-mag descriptors + anti-regression gates |
| 4 | A2 L1 tracker wiring to lifter |
| 5 | A4 Stage 6 calib MP binding |
| 6-7 | A5 ARM bring-up (LOG ring + radio + DWT timing) |
| 8-10 | A6 L7 indoor demo + 20-run validation |
| 11 | A7 outdoor PX4 (2 runs) |
| 12 | A8 evaluation chapter writeup |

Slack: ~10% buffer absorbed into A6 (lighting tuning is unpredictable).

---

## Anti-list (do NOT pull in from research review)

The 5 improvement directions in `ideas/research_review_2026_05_16.md`
(Schmidt-EKF, SeqSLAM, SQ-SLAM, binary-VPR short-circuit, thesis writeup
artifact) are **FutureWork material**, not in-scope per §23.2.

Per [[objectsplan-vs-futurework]]: research review = sanity check that
our choices stay aligned with literature; promotion criteria for any
direction to move from FutureWork into ObjectsPlan must be documented
(e.g., "Schmidt-EKF promotes iff inverse-depth lifter shows divergence
> 5% on real-world ARM bring-up"). None of the 5 currently meet a
promotion criterion.

---

## Cross-references

- Scope freeze: `ideas/objects_plan.md` §23 (frozen 2026-05-15)
- FutureWork parking lot: `ideas/FutureWork.md`
- Sanity-check research review: `ideas/research_review_2026_05_16.md`
- Canonical mission pattern: `examples/sentai_runtime/experiments/s152_hex_patrol_servo/`
- SIL execution model: `Sim.md` §10z
- Memory rules: [[missions-run-in-sentai-only]], [[sim-test-must-return-home]], [[test-must-be-relevant-to-claim]], [[gate-every-layer-no-exceptions]]
