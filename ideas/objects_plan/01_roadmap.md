<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 119,622. -->

# Chapter 01_roadmap — 10-stage roadmap (§3) — primary anchor for OP-S{N}

WBS anchors: OP-S1..OP-S10 + OP-S4.5 — FROZEN per WBS hard rule

## 3. Foaie de drum — 10 stagii

### Stage 1 — Data layer: `sentai.objects.*` (object map)

**Goal**: structura statică de date care va trăi maparea + interogarea ei din MicroPython. Fără EKF încă; doar "add / get / list / remove / mark visited".

**Existing reuse**:
- `examples/sentai_runtime/sentai_tracker.{h,cc}` — pattern static array, MP binding
- `examples/sentai_runtime/sentai_aruco_shim.h` — pattern struct + shim

**Files to add**:
- `examples/sentai_runtime/sentai_objects.h` — contract API
- `examples/sentai_runtime/sentai_objects.cc` — ARM impl (real, lives in `.sdram_text`)
- `examples/sentai_runtime/modsentai_objects.c` — MP binding (`#include`'d în `modsentai.c`)
- QSTR regen (`agent.md §6`)

**API surface**:
```python
sentai.objects.add(class_id, x_m, y_m, z_m, cov6=None) -> obj_id    # alias for tests
sentai.objects.get(obj_id) -> dict { class_id, status, x, y, z, cov, age_ms, observations }
sentai.objects.list() -> list[dict]                                  # all non-FREE slots
sentai.objects.mark_visited(obj_id) -> int
sentai.objects.remove(obj_id) -> int
sentai.objects.clear() -> int                                        # reset map (debug)
sentai.objects.stats() -> dict { used, confirmed, coasting, stale }
```

**Struct (per concept §4.1, hardened)**:
```c
typedef enum : uint8_t {
    OBJ_FREE       = 0,
    OBJ_TENTATIVE  = 1,
    OBJ_CONFIRMED  = 2,
    OBJ_COASTING   = 3,
    OBJ_STALE      = 4,
    OBJ_LOST       = 5,        // transient → FREE on next sweep
} obj_status_t;

typedef struct {
    uint8_t        id;
    obj_status_t   status;
    uint8_t        class_id;
    uint8_t        observations;
    float          p_W[3];          // metres, ENU, world frame
    float          cov_uppertri[6]; // 3×3 upper-triangular (xx, xy, xz, yy, yz, zz)
    uint32_t       last_seen_ms;
    uint32_t       last_updated_ms;
    uint32_t       descriptor[2];   // color hist hash for re-association
    uint16_t       tracklet_id;     // link back to sentai_tracker if alive
    uint8_t        visited;
    uint8_t        _pad;
} sentai_object_t;

#define SENTAI_OBJECTS_MAX 32
```

**Fault model (per `embeded.md §A`)**:
| Fault | Action | Counter |
|---|---|---|
| F1 add with non-finite floats | reject, return -1 | `oob_rejected` |
| F2 add with class_id ≥ DICT_MAX | reject, return -2 | (same) |
| F3 add when map full | evict OBJ_STALE oldest; if none, return -3 | `evictions` |
| F4 remove with bad obj_id | return -1, no state change | `bad_ids` |
| F5 cov non-PSD on input | clamp diagonal to ≥ σ_min² | `cov_clamped` |

**SIM↔ARM parity**: same `.cc` compiles for both (pure data + math, no HW deps).

**T1 wire test** — `struct.pack` add → list → remove round-trip via REPL.

**Pass criteria**:
- Add/get/list/remove cycle works at REPL
- `oob_rejected` increments on NaN insert
- Sequential add of 33 markers triggers eviction logic without crash
- `stats()` reflects population accurately
- ARM build OK with `.sdram_text` placement
- Stack high-water visible via `sentai.objects.stats()` (NEW: add hwm field)

**Time estimate**: 2-3 zile efort focusat.

---

### Stage 2 — TPU model task-specific

**Goal**: YOLOv8n quantizat INT8 antrenat pe scenariul concept §12.1 (cub roșu + distractoare + obstacole). Compatibil cu pipeline-ul existent `sentai.tpu.*`.

**Existing reuse**:
- `examples/sentai_runtime/detection_task.cc` — invokes loaded model at 41 fps
- `sentai.tpu.load("/models/<name>.tflite")` — already works
- s080+ benchmark infrastructure (`paper/models.md`)

**What to add**:
1. **Sintetic dataset generator** — Gazebo Garden + Domain Randomization Python script (rulează în distrobox + venv)
2. **Training pipeline** — host-side Python (PyTorch/Ultralytics → ONNX → TFLite → edgetpu_compile)
3. **Class definition file** — `examples/sentai_runtime/models/red_cube_v1.classes.txt`
4. **Validation set** — 200+ real-rendered Gazebo frames cu ground-truth bbox

**Files to add** (under `examples/sentai_runtime/experiments/s114_red_cube_model/`):
- `gen_dataset.py` — Gazebo→PNG with bbox JSON (Domain Randomization)
- `train.sh` — wraps Ultralytics CLI
- `convert.sh` — ONNX→TFLite→edgetpu_compile
- `validate.py` — recall + FP rate + fps benchmark on board
- `README.md` — recipe + pass numbers

**Pass criteria (concept §11 step 3)**:
- recall > 90% pe valset
- FP rate < 5%
- on-board fps > 20 (using `sentai.diag.tpu_bench()`)
- model file < 2 MB (FileX user partition budget)

**Fault model**:
- Model file corrupt → `tpu.load` returns -1, log dmesg `E:0B41:N`, drone refuses MISSION_START
- TPU USB enum fail → restart attempt 3× → degraded mode (no detection)

**Time estimate**: 5-7 zile (heavy in data generation + training cycles).

---

### Stage 3 — Mission SM minimal (`sentai.mission.*`)

**Goal**: SM2 redus (SEARCH → APPROACH → FINAL → DONE) ca FreeRTOS task pe M7, observabil din REPL. Fără COAST/ALIGN/INITIALIZE inițial (per concept §11 recommendation). Test pe cf2 SITL pentru iterare rapidă (cf2 KalmanFilter accepts ext_position deja prin auto-forwarder).

**Existing reuse**:
- `sentai_anchor_forward.cc` — task-pattern + start/stop + stats + liveness check + fault gates (clone-able skeleton)
- `sentai_tracker.cc` — provides `get_tracks()` for "is there a confirmed track of class_id=X?"
- `sentai.flow.anchor_pose()` — drone world pose (input for SM transitions)
- `sentai.objects.*` (Stage 1) — target object position

**Files to add**:
- `examples/sentai_runtime/sentai_mission_sm.h` — public API
- `examples/sentai_runtime/sentai_mission_sm.cc` — task body, gated state machine
- `examples/sentai_runtime/modsentai_mission.c` — MP binding

**API surface**:
```python
sentai.mission.start(target_class_id, mission_id_str="default")
sentai.mission.stop()
sentai.mission.state()    # str: "IDLE" | "SEARCH" | ... | "DONE" | "ABORT"
sentai.mission.intent()   # dict: { type, target_pos, target_yaw, speed_cap, pixel_target }
sentai.mission.stats()    # dict: state, time_in_state_ms, transitions[], aborts, ...
```

**State table** (initially 5 states only):
| State | Output intent | Tranziții |
|---|---|---|
| IDLE | none | start() → MISSION_START |
| MISSION_START | none, check preconditions | EKF stable + battery OK → SEARCH; else → ABORT |
| SEARCH | spiral pattern intent | track_confirmed(class_id) ≥ 5 frames → APPROACH |
| APPROACH | GOTO p_obj_W intent | dist < 1.5 m → FINAL; det_age > 5s → SEARCH |
| FINAL | SERVO_IMAGE intent | pixel_err < 20 px sustained 1s → DONE |
| DONE | hold-and-log | (terminal) |
| ABORT | hover-safe intent | (terminal) |

**Fault model (load-bearing)**:
| Fault | Action |
|---|---|
| F1 No tracker confirmed track in SEARCH > 60s | ABORT |
| F2 APPROACH dwells > 30s | ABORT |
| F3 trace(P_obj) crește peste 1.0 m² | ABORT |
| F4 EKF covariance trace > prag | ABORT (cedare la SM1) |
| F5 sentai.imu / sentai.flow not running | refuse start() |
| F6 sentai.objects map empty when start() called | refuse start() with -2 |

**Resource budget**:
- Task @ 20 Hz, stack `configMINIMAL_STACK_SIZE * 2`
- Per tick < 200 µs (no heavy math; reads stats + decides)
- ITCM 0 (full `.sdram_text`)

**SIM↔ARM**: ARM = real task; SIM = identical task on POSIX port (only the action layer differs).

**Pass criteria**:
- `sentai.mission.start(1)` while cf2 SITL hovers near ArUco markers → enters SEARCH → APPROACH → FINAL → DONE within 90s
- Stats track all transitions (`transitions[]` queue)
- All 6 fault paths trigger ABORT correctly when injected
- 0 phantom transitions during 30s IDLE
- Stack hwm < 50% capacity

**Time estimate**: 3-4 zile.

---

### Stage 4 — Action layer (`sentai.servo.*`) — intent → velocity

**Goal**: separate "what we want" (intent from mission SM) from "how we achieve it" (velocity setpoint). PBVS + IBVS controllers + APF obstacle avoidance. Decoupling permite testarea SM cu mock action layer.

**Existing reuse**:
- `sentai_anchor_forward.cc` — auto-VPE forward (publish path to PX4/cf2 already proven)
- `sentai.link.send_velocity_*` (NEW — needs adding similar to `send_vpe`)
- `sentai.crazy.send_velocity_*` (NEW — needs adding)

**Files to add**:
- `examples/sentai_runtime/sentai_action_layer.h`
- `examples/sentai_runtime/sentai_action_layer.cc` — task @ 50 Hz
- `examples/sentai_runtime/modsentai_servo.c` — MP binding (debug + manual override)
- `sentai_link.{h,cc}` — `sentai_link_send_velocity_setpoint(vx, vy, vz, yaw_rate)` (MAVLink SET_POSITION_TARGET_LOCAL_NED with velocity mask)
- `sentai_crazy.{h,cc}` — `sentai_crazy_send_velocity(vx, vy, yaw_rate, z)` (CPX velocity setpoint port)

**API surface**:
```python
sentai.servo.start(mode="auto"|"manual"|"off", rate_hz=50)
sentai.servo.stop()
sentai.servo.stats()      # dict: mode, sent_count, intent_type_seen, ...
sentai.servo.override(vx, vy, vz, yaw_rate)   # debug: manual hijack
sentai.servo.gains()      # dict: kp_pbvs, kp_ibvs_pix, kd_ibvs_pix, v_max, ...
sentai.servo.set_gains(...)
```

**Internal flow**:
```
mission_sm.intent()  →  action_layer (50 Hz tick)
                            │
                            ▼
                  switch(intent.type) {
                    GOTO          → PBVS: v = K_p · (target - pose)
                    SERVO_IMAGE   → IBVS: roll/pitch from pixel error
                    SEARCH_SPIRAL → spiral pattern generator
                    HOLD          → position hold
                  }
                            │
                            ▼
                    APF obstacle avoidance (sentai.objects with class_id=OBSTACLE)
                            │
                            ▼
                   sentai_link_send_velocity_setpoint  (PX4 path)
                   sentai_crazy_send_velocity          (cf2 path)
```

**Fault model**:
| Fault | Action |
|---|---|
| F1 intent.target non-finite | hover-in-place, count `bad_intents` |
| F2 transport not running | skip send, count `skipped` |
| F3 v_cmd exceeds v_max | saturate, count `saturated` |
| F4 PBVS error > 100 m (lost EKF?) | abort, signal mission SM ABORT |
| F5 IBVS pixel err > 200 px sustained | revert to PBVS, count `ibvs_lost` |
| F6 send_failed > 10 consecutive | enter degraded mode (warn) |

**Resource budget**:
- @ 50 Hz, stack `configMINIMAL_STACK_SIZE * 2`, per tick < 500 µs

**Pass criteria**:
- PBVS step response: 2m to 0.2m within 5s, overshoot < 10%
- IBVS centering: pixel error reduced from 80 → < 20 within 3s
- PBVS↔IBVS smooth blend in zone 1.3-1.7 m (no velocity jump > 0.5 m/s)
- APF: drone avoids known pillar without hitting (test in concept world)
- Saturation correctly capped on v_max

**Time estimate**: 4-5 zile.

---

### Stage 4.5 — Multi-marker constellation localization (blind-nav between objects) — added 2026-05-14

**Context**: between L4 (action layer + closed-loop tests s128/s129)
and Stage 5 (inverse-depth EKF) there is a missing primitive operator
flagged during the s129 multi-marker IBVS work: when the target marker
is NOT in the current camera view, the drone needs to NAVIGATE to it
using world memory + a re-localization from the markers it CAN see.

The s129 test as shipped fakes this — `APPROACH_WORLD_XY` is a static
table consulted via dead-reckoning on cf2's EKF.  That works when all
markers are simultaneously visible (compact ±0.15×±0.10 pattern at
z=1 m), but breaks immediately when markers span more than one FOV.

**Goal**: drone with N seeded objects (positions in `sentai.objects`)
can navigate from any visible-subset to a target object even if the
target itself is outside FOV during the transit.  Localization is
done by multi-marker PnP on whatever subset is currently visible,
re-evaluated at every detection.

**Existing reuse — most of the plumbing is already shipped**:
- `sentai.objects` (Stage 1 / L2) — world coords of known markers.
- `aruco_detector.estimate_drone_world_pose(dets, known_positions, drone_yaw)`
  — multi-marker PnP localization, proven in s091 aruco_hover.py.
- `sentai.servo.move()` (Stage 4 / L4) — intent recording.
- cf2 cflib MotionCommander — until Stage 4.A wires transport.

**What's new**:
- The localization step in the control loop: every iter, detect visible
  markers, look up world positions in `sentai.objects.list()`, compute
  drone world pose from the visible subset, plan the route to target.
- When target enters FOV: switch to IBVS (L4.2) for fine centering.

**No new firmware module needed for v1**.  Implementation is a host-
side mission script that composes existing primitives.  Stage 5
(`sentai_object_lifter` with inverse-depth EKF) is a strict superset
that ALSO handles the case where markers are not pre-known — but Stage
4.5 specifically validates the case where world coords ARE known from
the seed phase.

**Pass criteria (s130, proposed)**:
- Markers placed far apart in world.sdf (e.g. ±0.5×±0.5 m) so only a
  subset is visible at any drone position at z=1 m hover.
- Drone visits each marker; at least 50% of the path between markers
  the target is NOT detected in camera (proving "blind" transit).
- Re-localization via PnP on visible markers keeps world-pose error
  < 5 cm during transit (compared to cf2 EKF ground truth).
- Final IBVS-centered px_dist < 30 px on each marker (same as s129).

**Time estimate**: 1-2 zile (composition test, no new firmware).

**Relation to s129**: s129 proves IBVS works (visual-only fine centering).
Stage 4.5 / s130 proves NAVIGATION USING THE WORLD MODEL works (target
position recalled from `sentai.objects` while in transit).  Both are
prerequisites for L6 explore SM.

---

### Stage 5 — Object lifter (2D→3D bearing → inverse-depth EKF)

**Goal**: convert each mature 2D tracklet into a 3D landmark estimate using bearing + class-prior pseudo-depth. Initial inverse-depth parameterization (Civera/Davison/Montiel TRO 2008) converges with parallax over a few seconds of lateral motion.

**Existing reuse**:
- `sentai_tracker.cc` — provides mature tracklets with TENTATIVE/CONFIRMED state + age
- `sentai.flow.anchor_pose()` — drone pose at observation time (when available)
- `sentai.imu.*` — IMU-derived attitude (always available)
- CMSIS-DSP matrix functions (already linked, including `arm_mat_*`)
- `sentai_objects.cc` from Stage 1 — destination after convergence

**Files to add**:
- `examples/sentai_runtime/sentai_object_lifter.h`
- `examples/sentai_runtime/sentai_object_lifter.cc` — runs in detection_task after tracker
- (no new MP binding initially — observable via `sentai.objects.list()`)

**Camera model (constants in header)**:
```c
#define LIFTER_FX_PX  240.0f
#define LIFTER_FY_PX  240.0f
#define LIFTER_CX_PX  160.0f
#define LIFTER_CY_PX  120.0f
// Body→Camera extrinsic (downward 25° tilt, per cf2 mount):
#define R_B_C_R00 ...   // hardcoded calibration matrix (TBD)
```

**Class-prior table** (extensible):
```c
static const float k_class_real_size_m[CLASS_MAX] = {
    [CLASS_RED_CUBE] = 0.30f,
    [CLASS_CARDBOARD] = 0.40f,
    [CLASS_CYLINDER] = 0.25f,
    // ...
};
```

**Per-tracklet algorithm** (called from detection_task.cc after tracker.update()):
1. For each CONFIRMED track with age ≥ 3 frames:
2. Compute bearing `r_C` from bbox center via `K^-1 [cx, cy, 1]^T`
3. Rotate to world: `r_W = R_W_B · R_B_C · r_C`
4. Compute pseudo-depth: `d_est = f · real_size_class / pixel_w_bbox`
5. If not yet in object map: initialize with inverse-depth `(o_W, θ, φ, ρ=1/d_est, σ_ρ large)`
6. Otherwise: update with paralaxa observation → ρ converges
7. When σ_ρ < threshold (e.g. 0.5/d²): convert to Cartesian `(x_W, y_W, z_W)`, mark CONFIRMED in object map

**Fault model**:
| Fault | Action |
|---|---|
| F1 No drone pose available (anchor_pose stale > 200 ms) | skip frame |
| F2 IMU attitude inconsistent (roll/pitch > 60°) | skip (drone tilted, bearing math invalid) |
| F3 Bbox area < 100 px² | skip (too small for reliable pseudo-depth) |
| F4 Tracklet age < 3 frames | skip (immature) |
| F5 ρ becomes negative (numerical blow-up) | reset tracklet's EKF state |
| F6 trace(P) grows after update (filter divergence) | drop tracklet from objects map |

**Test (concept §11 step 5)**:
- Drone hovers at z=1.5 m, then moves laterally ±2 m at 1 m/s for 5 s with target in FOV
- After 5 s: `||p_obj_est − p_obj_GT||` < 0.3 m, `trace(P)` < 0.5

**Time estimate**: 5-7 zile (heavy on numerical-tuning + frame conventions).

---

### Stage 6 — Loop closure on yaw (anti-drift)

**Goal**: when a previously-mapped object is re-observed, the observation acts as **pose correction on the drone** (not on the object) — corrects yaw drift, which is otherwise unobservable without magnetometer.

**Existing reuse**:
- Stage 5 lifter publishes per-frame "this tracklet maps to map slot N" event
- `sentai.flow.anchor_forward(rate, target)` — already publishes VPE; we extend to publish CORRECTED pose
- PX4 EKF2 / cf2 Kalman both accept external pose corrections

**What changes**:
- In `sentai_object_lifter.cc`, after EKF update for a CONFIRMED object:
  - Compute innovation = observed_bearing − expected_bearing_from_map_position
  - If |innovation| > threshold (say 5° in yaw equivalent): this is loop closure evidence
  - Push **corrected drone pose** into `sentai_aruco_pose_t` (or new `sentai_loop_closure_pose_t`)
  - Auto-forwarder picks it up and sends as VPE with covariance scaled accordingly

**Files to modify**:
- `examples/sentai_runtime/sentai_aruco_shim.h` — add `is_loop_closure` flag in pose snapshot
- `examples/sentai_runtime/sentai_object_lifter.cc` — emit loop closure events
- `examples/sentai_runtime/sentai_anchor_forward.cc` — promote LC events at higher priority

**Fault model**:
| Fault | Action |
|---|---|
| F1 Loop closure suggests yaw correction > 45° | reject (likely false ID); count `lc_rejected_large` |
| F2 LC against COASTING-age object > 60s | only accept if descriptor match high; count `lc_stale` |
| F3 LC frequency > 5/s | rate-limit to 2 Hz, drop excess |

**Test (concept §11 step 6)** — CRITIC:
- 2-minute square flight pattern: takeoff (0,0,1.5) → (3,0,1.5) → (3,3,1.5) → (0,3,1.5) → repeat
- Pre-place a red cube at (1.5, 1.5, 0)
- Without LC: yaw drift expected 10-30° after 2 min
- With LC: yaw drift < 2°

**Time estimate**: 3-4 zile.

---

### Stage 7 — Full SM3 (track health) + COAST/ALIGN integration

**Goal**: add the resilience states (COAST, ALIGN, INITIALIZE) + full SM3 (STALE, COASTING per-object). Activate once Stage 3 minimal SM2 is end-to-end working and we see the symptoms that justify each state.

Per concept "recomandare practică": adăugăm aceste stări DOAR când vedem comportamentul rău, NU înainte.

**Trigger conditions**:
- **COAST**: when SEARCH↔APPROACH oscillates ≥ 3× per minute (detection flickering)
- **ALIGN**: when PBVS→FINAL transition jumps velocity > 0.5 m/s
- **INITIALIZE**: when EKF object converges slow (trace stays > 0.5 after 5s)
- **STALE/COASTING** in SM3: when extending to multi-object mission (concept §9.3)

**Files to modify**:
- `sentai_mission_sm.cc` — add COAST/ALIGN/INITIALIZE branches with hysteresis thresholds
- `sentai_objects.cc` — add STALE/COASTING transitions on `tick()` (timeout-driven)

**Pass criteria**:
- After adding COAST: 0 oscillations in 10 trials
- After adding ALIGN: velocity jump < 0.2 m/s at transition

**Time estimate**: 3-5 zile (incremental, debug-driven).

---

### Stage 8 — End-to-end integration in SITL (cf2 + PX4)

**Goal**: run the canonical scenario from concept §12 — 10 randomized cube placements, measure all 5 PASS metrics.

**Files to add** (under `examples/sentai_runtime/experiments/s115_endtoend_cube_landing/`):
- `world_red_cube.sdf` — Gazebo arena with cube + distractoare + pillars (concept §12.1)
- `run_cf2.sh` — CrazySim path: brings up cf2 SITL + sentai_sim with all modules
- `run_px4.sh` — PX4 SITL path: parallel validation
- `analyze.py` — parse CSVs, compute 5 metrics, plot
- `README.md` — pass numbers (target: 8/10 success)

**Required pre-conditions (gate)**:
- All Stages 1-7 PASS individually
- Per-stage smoke tests integrated into `make test` (or equivalent)

**Pass criteria (concept §12.3, on 10 runs)**:
- Detection inițială: < 30 s after arm
- Mission total: < 90 s
- Horizontal error final: < 0.15 m on ≥ 8/10
- 0 collisions with pillars
- Yaw drift final: < 5°

**Time estimate**: 5-7 zile (most time is debugging integration issues — concept warns specifically).

---

### Stage 9 — ARM bring-up + timing budget validation

**Goal**: port the system from SIM to RT1176, profile each loop, fit into 20 ms budget at 50 Hz.

**Existing reuse**:
- All sentai_*.cc files are SIM↔ARM portable by design
- `sentai.diag.aruco_bench` precedent — bench infra already in place

**What to add** (under `experiments/s116_arm_timing_budget/`):
- `bench_per_loop.py` — measure DWT cycles for each task (mission_sm, action_layer, object_lifter, lifter EKF)
- Per-task `sentai.<task>.cycles()` stat exposing min/max/avg
- README with measured numbers + comparison to concept budget

**Budget table (target)**:
| Task | @ Hz | Budget µs | Action if exceeded |
|---|---:|---:|---|
| IMU + flow forward | 200 | 5 ms ⇒ 1000 µs per Hz | move to ITCM if hot |
| Action layer | 50 | 500 µs | optimize PBVS gain calc |
| Mission SM | 20 | 200 µs | unlikely bottleneck |
| Detection + tracker + lifter | 10-30 | 8-10 ms | already proven (s111) |

**Pass criteria**:
- All tasks fit in budget
- 30 s board uptime under full mission load: 0 watchdog kicks missed
- 30 s board uptime: stack hwm of every task < 60% capacity
- Pipeline FPS preserved (≥ 42 fps yolo_1 pure-TPU)

**Time estimate**: 3-5 zile.

---

### Stage 10 — Hardening + documentation + paper-grade evidence

**Goal**: production-grade release notes, design doc per concept §13, evidence trail for `paper/` series.

**Deliverables**:
- `examples/sentai_runtime/paper/objects_nav.md` — full system design doc (5-10 pages)
- `experiments/s115_*/README.md` — 10-run results + plots
- Update `agent.md §11` with new load-bearing invariants
- Update `embeded.md` if new patterns emerged (track-health-states, intent emitter, …)
- Memory entries in `/.claude/projects/.../memory/` for each new module

**Time estimate**: 2-3 zile.

---

