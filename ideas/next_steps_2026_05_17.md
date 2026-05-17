# Next steps — post sentai.servo Stage 4.A (2026-05-17)

Plan derivat din memoriile cross-session + research review 2026-05-16.
Ordonat critical-path → improvements → hardening → future work.

---

## A. Critical path (thesis-defense blockers)

### A1. Migrate remaining missions s147/s148/s150/s151 → sentai.servo paradigm
- **State**: s152_hex_patrol_servo PASS (closure 2.31 cm) demonstrates the canonical pattern. s149-equivalent. s147/s148/s150/s151 still on old `sentai.crazy.*` + `crtp_log.py` shape.
- **Effort**: ~3-4h per mission, mostly mechanical. Drop `import crtp_log`, replace `sentai.crazy.X` with `sentai.servo.X`, fix `_converge_to` settle_ms bug if present.
- **Gate**: each migrated mission must pass its existing verdict closure gate (10 cm single-lap, 12 cm 2-lap per [[sim-test-must-return-home]]).
- **Why now**: closes [[s147-s151-migrations-shipped]] memory loop, validates servo paradigm across 5 mission shapes, kills the `crtp_log.py` host-side dependency completely.

### A2. Stage 9 prep — ARM port of sentai_crazy_recv_pop + LOG routing
- **State**: ARM links via weak stub of `sentai_crazy_recv_pop`. On real hardware, `sentai.crazy.pose_subscribe()` returns -2 (TOC timeout). Per CLAUDE.md hard rule "ARM build must never regress" + [[missions-run-in-sentai-only]].
- **Effort**: ~1 week. Add a generic LOG-channel RX FIFO in `sentai_crazy.cc` (`crazy_rx_task` already parses LOG packets per-block; widen the routing to a 32-entry SPSC ring for non-altitude blocks). Drop the weak stub.
- **Gate**: smoke pose_subscribe + pose() on the live Crazyflie radio bridge ([[crazyflie-radio-bridge]] board #1224 + drone 53d72897).

### A3. Camera-mount R_cam_to_body auto-calibration on ARM
- **State**: SIM has 3D Procrustes Kabsch auto-calib at takeoff via ArUco landing pad ([[camera-mount-calibration]]). ARM build stale; objects_plan.md §21 documents ±2-5° per-unit mount tolerance.
- **Effort**: ~1 week. Port `_shared/camera_calibration.py` logic into C; write to `/system/cam_calib.json` via FxUser. Run at boot if calibrated marker visible.
- **Why now**: any real-hardware demo requires this — uncalibrated mount = systematic bias in PnP-derived pose.

---

## B. Research-driven improvements (5 directions from research_review_2026_05_16.md)

### B1. Schmidt-EKF upgrade for sentai_object_lifter [impact: high, effort: 2-3 wk]
- **Source**: Geneva CVPR 2019 (arXiv 1903.08636), MDPI Machines 13(7):582 (2025).
- **What**: Replace independent per-slot 6-state EKFs with a coupled formulation that treats matured landmarks as nuisance parameters (kept in covariance, never updated). Linear cost in map size; bounded-error consistency — the property reviewers will demand.
- **Files**: new `sentai_object_lifter_schmidt.cc` (flag-switchable); reproduce s131 math-validation harness with Schmidt variant; FlowBaseline gate after.
- **Risk**: numerical stability — Joseph form + ρ_min clamp already in baseline lifter; verify Schmidt preserves them.

### B2. SeqSLAM-style sequence VPR over Track A [impact: high, effort: 1-2 wk]
- **Source**: Milford & Wyeth ICRA 2012; MDPI Drones 8(7):322 (2024).
- **What**: Match K=5 consecutive PHOG/GIST descriptors as a DP-aligned sequence rather than single-frame. Suppresses spurious single-frame false-positives under viewpoint drift. K×168 floats ~3.4 KB ringbuffer. Zero TPU cost.
- **Files**: new `sentai_places_sequence.cc`; expose `sentai.places.match_sequence(K, descs[])`; re-run s143 loop-closure with K=5; measure FP reduction.
- **Promotion**: lives in FutureWork FW2 today; this lifts it into the implementation tree.

### B3. SQ-SLAM superquadric upgrade to lifter [impact: high, effort: 3-4 wk]
- **Source**: Cao/Han/Yang JIRS 2023 (arXiv 2209.10817).
- **What**: Replace 3D-point landmark with superquadric (5-10 params: pose + extent + shape ε). Gives orientation AND footprint, exploitable by `sentai.explore` (near/far inspect, footprint class). Fits 64-byte L3 slot at 8×float16.
- **Files**: extend `sentai_object_lifter.cc`; SIM-first validation on s132 Gazebo Aruco scene; ARM port after.
- **Risk**: Gauss-Newton iteration on 5-param residual — bounded by 8 iters / 1 ms M7 budget.

### B4. Binary-descriptor short-circuit for VPR [impact: medium, effort: 2 wk]
- **Source**: SuperVLAD (NeurIPS 2024), BinVPR (MDPI Sensors 24(13):4130, 2024).
- **What**: Derive 256-bit signature via `sign(GIST - median)`. Hamming popcount over 64 slots = ~64×4 instructions on M7 SIMD (`__USADA8` pattern from s111). Short-circuit ahead of full L1 ranking; only top-K candidates pay full distance.
- **Files**: `sentai_places_binary.cc`; `sentai.places.bin_match()` API; gate via Hamming-recall@K vs L1 sweep, target ≤5% recall drop.

### B5. Thesis writeup — sentai.servo as publishable artifact [impact: medium-high, effort: 1 wk writing]
- **Source**: Panerati 2026 (arXiv 2602.07264), UAL 2020, Aerostack2 2023.
- **What**: §3 "Action layer" subsection + 2-column comparison table (CPU/MEM/firmware/backends/transport) positioning sentai.servo as the on-MCU counterpart of aerial-autonomy-stack/UAL/Aerostack2. Cross-section unoccupied in literature.
- **No code**. Push into objects_plan.md §22 or new ideas/thesis_chapter_action_layer.md.

---

## C. Hardening / tech debt

### C1. Refactor T2 — split sentai_runtime.cc (3321 LoC) + sentai_crazy.cc (2007 LoC)
- **State**: Sim.md §10y notes the planned split; not started.
- **Why**: before Stage 9 ARM bringup, monoliths become merge-conflict magnets and obscure ownership.
- **Effort**: ~2 days. Split by concern (boot, USB CDC, MP REPL, FS, watchdog for runtime; CPX framing, CRTP routing, HL Commander, telemetry for crazy).

### C2. Document sentai.servo Stage 4.A in objects_plan.md
- **State**: §3 Stage 4 still describes skeleton-only. Update with the wiring that landed (#45 + #47), the SERVO_FAULT_TX_FAIL code, the dispatch table.
- **Effort**: 1h.

### C3. Migrate ARM build out of WICED pre-existing breakage
- **State**: `bash build.sh` fails on `wwd_rtos.c` (vTaskStackOverflowHook type conflict) — unrelated to sentai code. We build via `cmake --build build --target sentai_runtime` to bypass.
- **Effort**: 1-2h. Either patch wwd_rtos.c locally or disable WICED in apps/CMakeLists.txt.
- **Why**: CI gate on full `bash build.sh` is broken until fixed.

---

## D. Recommended sequencing (8-week horizon)

| Week | Focus |
|---|---|
| 1 | A1 (migrate s147/s148/s150/s151 to servo) + C2 (docs) |
| 2 | A2 (ARM LOG routing) + smoke pose on real Crazyflie |
| 3 | B5 (thesis writeup §3 + comparison table) |
| 4-5 | B1 (Schmidt-EKF lifter upgrade) |
| 6 | B2 (SeqSLAM over PHOG/GIST) + B4 (binary short-circuit) — parallel |
| 7 | A3 (ARM camera auto-calib) |
| 8 | B3 (SQ-SLAM superquadric) start; spills into next sprint |

C1 (refactor T2) and C3 (WICED fix) are slot-in anytime, ideally weeks 1-3.

---

## E. Out of scope (anti-list — do NOT pull in)

Per memory `[[objectsplan-vs-futurework]]`:

- NetVLAD/MixVPR/AnyLoc on EdgeTPU — anti-recommendation #1 in research review.
- Loopy-SLAM / dense neural SLAM — GPU-class.
- Tightly-coupled VIO with HW timestamp sync — RT1176 lacks the silicon.
- ROS / MAVROS bridge for sentai.servo — gives up the "no-SBC" thesis claim.
- L1 ego-motion EKF own on M7 — redundant with PX4/cf2 internal KF.
- XfeatSLAM (g2o won't fit 1 MB OCRAM).

---

## F. Cross-references

- Research review: `ideas/research_review_2026_05_16.md`
- Thesis-MVP scope: `ideas/objects_plan.md` §23
- FutureWork parking lot: `ideas/FutureWork.md`
- Mission canonical pattern: `examples/sentai_runtime/experiments/s152_hex_patrol_servo/`
- SIL plan: `Sim.md` §10y (file org) + §10z (execution model)
