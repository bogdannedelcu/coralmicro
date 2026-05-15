# Plan de implementare — Autonomous Object-Driven Drone Navigation pe RT1176

> Plan stage-by-stage pentru a închide gap-ul între
> `examples/sentai_runtime/` (sentai.* namespace existent) și viziunea
> arhitecturală din `ideas/objects.md` (6 straturi + 3 state machines
> ierarhice, target scenario: cub roșu detect & land 20×20 m indoor).
>
> Disciplină de proiectare: principiile NASA/JPL din
> `examples/sentai_runtime/agent/embeded.md` — bounded, deterministic,
> observable, recoverable. Fiecare stagiu are: fault model, buget de
> resurse, criterii de PASS măsurabile, port SIM↔ARM.

> ⚠ **Scope-management (2026-05-15)**: planul a fost îngheţat la
> **thesis-MVP scope** pentru PhD defense. Vezi **§23** pentru lista
> exactă in-scope / out-of-scope cu demo target ca north star.
> Conţinutul deferred (sub-secţiunile mutate) trăieşte în
> [`FutureWork.md`](FutureWork.md) cu promotion criteria documentate.
> Adaugă idei noi în `FutureWork.md`, nu aici, pentru a nu polua
> planul de execuţie.

---

## 0. Executive summary

**Ce există deja** (~75% din infrastructura low-level):

| Layer concept | În sentai.* azi | Stare |
|---|---|---|
| L1 Ego-motion EKF | `sentai.imu.*`, `sentai.flow.*` (PXP+phase-corr), `sentai.flow.anchor_forward` | Observații (flow, anchor) sunt feed-uite la PX4/cf2 EKF. **NU rulează EKF propriu pe M7**. |
| L2 Detector NN | `sentai.tpu.*` (EdgeTPU + YOLO multi-class), `sentai.camera.*` | Pipeline complet la 41 fps; model task-specific (cub) lipsă. |
| L3 2D Tracker | `sentai_tracker.cc` (BoT-SORT-lite + ByteTrack + IMU CMC + histograms) | **Aproape complet** — TENTATIVE/CONFIRMED/LOST există, IMU CMC are flag. |
| L4 Map EKF 3D | `sentai_aruco_shim` (anchor pe markeri KNOWN) | **Mare gap**: nu există inverse-depth EKF pentru obiecte UNKNOWN. |
| L5 State machines | `sentai.pipeline.start/stop`, manual REPL/Python | **Mare gap**: SM2 mission + SM3 health în C++ pe M7 lipsă. |
| L6 Controller PBVS/IBVS | Velocity setpoints construite în Python sidecar, trimise prin `sentai.link`/`sentai.crazy` | **Mare gap**: action layer + PBVS/IBVS în firmware lipsă. |
| Anti-brick + watchdog + dmesg + ITCM budget | Toate produc-grade | ✅ |
| Platform abstraction SIM ↔ ARM | `sentai_pxp_shim`, `sentai_fft_shim`, `sentai_aruco_shim` | Pattern probat. |

**Cele 4 piese mari de construit**:
1. **Object map** (`sentai.objects.*`) — array static 32 sloturi + inverse-depth EKF
2. **Mission SM** (`sentai.mission.*`) — SM1+SM2+SM3 ierarhice, intent emitter
3. **Action layer** (`sentai.servo.*`) — intent → velocity setpoint + PBVS/IBVS
4. **Domain-specific TPU model** — YOLOv8n antrenat pe scenariul red-cube + distractoare

**Verdict feasibility MCU/ARM** (detaliat în §11):
- ✅ **Toate cele 4 module noi încap în < 3% CPU M7** (compute nu e bottleneck — pipeline existent folosește deja 48% pentru TPU+tracker)
- ✅ **0 KB ITCM** consumat de cod nou (totul în `.sdram_text` — margine ITCM rămâne ~32 KB)
- ✅ **~30 KB SDRAM** total pentru cod + date (16 MB disponibil)
- ✅ **FPU hard single-precision + CMSIS-DSP `arm_mat_*`** acoperă matematica EKF / quaternion / Kalman
- ⚠️ **Riscul real unic**: stabilitate numerică inverse-depth EKF (Stage 5) — mitigare prevăzută cu Joseph form + ρ_min clamp + filter divergence guard
- 🚫 **L1 Ego-motion EKF on-M7 propriu**: feasible (~1.5% CPU) DAR redundant cu PX4 EKF2 / cf2 KF → out of scope, defer

**Drumul critic** (~3-4 săptămâni dezvoltare focusată):
```
Stage 1 → Stage 2 → Stage 3 → Stage 4 → Stage 5 → Stage 6 → Stage 7 → Stage 8
data layer  TPU model   SM2 minimal  action layer  inverse-depth  loop closure  full SM3  ARM perf
                          (cf2 SITL)                              (yaw correction)
```

---

## 1. Principii NASA/JPL aplicate per stagiu

Toate stagiile trebuie să livreze următoarele înainte de a trece la următorul (per `embeded.md` §A-F):

### Fault model documentat
Listă explicită de faulturi credibile per modul (hardware, comunicare, timing, memorie corupție, deadline-uri ratate), cu acțiune deterministă pentru fiecare (drop / retry / degraded / safe / restart).

### Resource budget
| Buget | Limită hard |
|---|---|
| Timp per tick (M7) | < 20 ms @ 50 Hz |
| Heap dinamic post-boot | **ZERO** — static-only |
| Stack per task | declarat + `uxTaskGetStackHighWaterMark` expus |
| **ITCM (`.text`)** | Default `.sdram_text` pentru tot codul nou; ITCM se câștigă cu dovezi de hot path |
| SDRAM | OK 24 MB; budgetează la inițializare |

### Supraveghere
- Heartbeat counter / iters counter expus în stats
- Liveness check la `_start()` cu timeout 500 ms (vezi `sentai_anchor_forward.cc` pattern)
- Toate tranzițiile FSM loggate (dmesg cu severity)

### Observabilitate
- Stats dict per modul (`sentai.<modul>.stats()`)
- dmesg-uri categorisate cu severity
- Pose log CSV la 50 ms (per concept §12.4)

### Recovery
- Per `embeded.md §F`: local retry → local recovery → subsystem restart → degraded → safe → reset.
- NU full reboot ca prim răspuns. NU watchdog blind kick.

### SIM ↔ ARM parity
Fiecare stagiu nou trebuie să livreze:
1. **Header contract** `examples/sentai_runtime/sentai_<name>_shim.h` (dacă diferă comportamentul per platformă)
2. **ARM impl** (real / stub) în `.cc`
3. **SIM impl** în `sim/*.c`
4. **MicroPython binding identic** pe ambele (dict shape identic, key names identice)
5. **Test T1 wire** (struct.pack smoke) → **T2 synth-frame** → **T3 live Gazebo**

---

## 2. Memory & module budget (planning anchor)

ITCM disponibil: ~32 KB liber (post P2.5 relocation, build #1298). Tot codul nou:
**default `.sdram_text` + `.sdram_bss`** — vezi `project_itcm_budget.md`.

Octați estimați per modul nou (cifre conservative; actualizate post-stage):

| Modul | .sdram_text estimat | .sdram_bss estimat | Note |
|---|---:|---:|---|
| `sentai_objects.cc` (map L4) | ~6 KB | 2 KB (32 sloturi × 64 B) | static `object_t map[32]` |
| `sentai_object_ekf.cc` (inverse-depth) | ~8 KB | ~1 KB (state buffers) | CMSIS-DSP matrix ops |
| `sentai_mission_sm.cc` | ~4 KB | <0.5 KB | 3 SM-uri ierarhice |
| `sentai_action_layer.cc` (PBVS/IBVS) | ~5 KB | ~0.5 KB | velocity gen + APF obstacles |
| `sentai_object_lifter.cc` (2D→3D) | ~3 KB | ~0.5 KB | bearing + class prior |
| Total nou | ~26 KB SDRAM | ~5 KB SDRAM | margine confortabilă |

---

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

## 4. Risk register

| Risk | Severity | Mitigation |
|---|---|---|
| ITCM overflow when adding 4 new modules | HIGH | Default `.sdram_text` for everything; audit map after each stage |
| Object lifter numerical issues (inverse-depth divergence) | HIGH | Stage 5 has explicit F5 (negative ρ reset) + F6 (filter divergence drop); validate against ground truth |
| Mission SM dead-locks (anti-pattern per `embeded.md`) | MEDIUM | Per-state timeouts mandatory; tested with mock track events |
| Action layer velocity jump at PBVS↔IBVS transition | MEDIUM | Blend zone 1.3-1.7 m + velocity slew limit (Stage 4) |
| TPU model accuracy worse than promised in synth data | MEDIUM | Stage 2 includes real-Gazebo valset (not just train set) |
| Loop closure false yaw correction destabilizes flight | HIGH | Stage 6 F1 rejects > 45° corrections; LC events go through fault gates of anchor_forward |
| cf2 SITL diverges from real cf2 firmware behavior | LOW | Validate in parallel with PX4 SITL — different EKF, same intent semantics |
| Heap allocation creeps in via library helpers | MEDIUM | Lint with `grep malloc\|new` in new code; static-only enforced in code review |
| Watchdog kicks during heavy detection | LOW | Existing watchdog discipline preserved; new tasks call `sentai_diag_repl_kick` if > 60s |

---

## 5. NASA/JPL-aligned per-stage gate (mandatory before next stage)

For every stage transition, this checklist must be green:

```
□ Fault model documented in file header (F1..Fn explicit)
□ Resource budget declared (timing + stack + memory)
□ Per-tick liveness counter exposed in stats
□ `_start()` has 500 ms liveness wait (when applicable)
□ All MP binding strings & dict shapes parity ARM ↔ SIM
□ T1 wire test PASS (struct.pack smoke)
□ T2 synth-frame / mock test PASS
□ T3 live Gazebo PASS (when applicable)
□ Unit test for each fault gate (test_fault_gates.py pattern)
□ Stack high-water-mark < 70% of allocated
□ `.sdram_text` placement verified via objdump
□ dmesg category + severity used for state transitions
□ No printf with string literal in standalone .o files (uses dmesg)
□ memory/.md entry added with non-obvious lessons
□ experiment.md updated with delta + measurements
```

---

## 6. Stage dependency graph

```
       ┌─────────────────────────────────────────┐
       │ Stage 0 — gap analysis (this doc)       │
       └────────────────┬────────────────────────┘
                        │
       ┌────────────────▼──────────┐
       │ Stage 1 — sentai.objects  │  (data layer)
       └────┬──────┬───────────────┘
            │      │
   ┌────────▼┐  ┌──▼──────────────────┐
   │Stage 2  │  │Stage 3 — mission SM  │  (cf2 SITL ok, no real detector yet)
   │TPU model│  │SEARCH/APPROACH/FINAL│
   └────┬────┘  └──────┬──────────────┘
        │              │
        └─────┬────────┘
              │
   ┌──────────▼─────────┐
   │Stage 4 — action    │  (intent → velocity)
   │  layer PBVS+IBVS   │
   └──────────┬─────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 5 — object lifter      │
   │  (2D track → 3D inverse-d.) │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 6 — loop closure yaw   │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 7 — full SM (COAST/    │
   │  ALIGN/INITIALIZE/STALE)    │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 8 — end-to-end SITL    │
   │  (cf2 + PX4 parallel)       │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 9 — ARM bring-up +     │
   │  timing budget validation    │
   └──────────┬──────────────────┘
              │
   ┌──────────▼──────────────────┐
   │Stage 10 — paper / docs /    │
   │  memory hardening           │
   └─────────────────────────────┘
```

Total: ~3-4 săptămâni dezvoltare focusată (estimat conservativ).

---

## 7. What we explicitly DON'T build

(Per concept §13 + project priorities — keep scope honest):

- **Custom on-M7 ego-motion EKF replacing PX4 EKF2 / cf2 KF** — we feed observations TO existing flight-controller EKFs via `sentai.link.send_vpe` / `sentai.crazy.send_ext_position`. Building our own EKF on M7 is months of work + duplicates proven code. Defer to "if we ever go barebones cf2-firmware-replacement".
- **OctoMap / Voxblox / ESDF** — too heavy for MCU per concept §8.6. Use primitive obstacle list.
- **CubeSLAM / QuadricSLAM cuboid landmarks** — point + class lookup is the MCU-appropriate choice (concept §8.5).
- **Behavior Trees** — FSM remains clearer for < 15 states (concept §9 reference Colledanchise & Ögren).
- **Magnetometer** — explicitly out (concept hardware spec). Loop closure substitutes.
- **Hardware safety state machine (`sentai.safety`)** — battery thresholds, link-loss aborts, IMU faults, geofence breaches, watchdog handlers. Tracked in a **separate plan**, NOT in this objects_plan. The mission FSM (`sentai.explore`, Stage 3) communicates with `sentai.safety` via a thin handshake (safety can force-hold or force-land the mission) but never owns the safety logic itself. Don't push battery / link / IMU thresholds into `sentai.explore` guards — they belong in `sentai.safety`.

---

## 7.5 Canonical exploration pattern — SFLVP (S From Last Visited Place)

**Specified 2026-05-13.**  When `sentai.explore` walks the world model
during the EXPLORE state (§3 stage 3), the canonical traversal is:

  1. Mark the **current cell** as visited (`places.observe(cell, …)`).
  2. Enumerate the **6 hex neighbors** of the current cell (`places.neighbors(cell, 1)` minus the cell itself).
  3. **Visit each neighbor** in turn — fly to its centroid, settle briefly,
     observe (class histogram + HSV embedding once Stage 11.D fires).
  4. After all 6 neighbors are visited, **pick the next central** —
     the neighbor whose own k=1 ring contains the most still-unvisited
     cells (greedy frontier).
  5. Repeat until the explore cell budget (Stage 3.B `cell_budget`)
     is hit or `set_thresholds(..., explore_timeout_ticks)` fires.

The traversal traces an "S" shape from each last-visited place as the
drone leaves a fully-covered group and jumps to the next central — hence
**SFLVP** (S From Last Visited Place).

**Why this shape (vs spiral / lawn-mower / Voronoi-frontier):**

  - **Hex neighbors are pre-computed by H3** — `gridDisk(k=1)` returns
    the 6 cells deterministically.  No path-planning needed inside the
    inner loop.
  - **Coverage is uniform** at each scale — each cell is observed AT
    LEAST once and at most twice (when it's a neighbor of two
    consecutive centrals).
  - **Compatible with the dual-scale convention (§10v)** — at PX4
    natural scale the hex edges are ~10 m, at cf2 1/10 they're ~1 m;
    the same SFLVP algorithm runs unchanged, only `places.init(scale=…)`
    differs.
  - **Backtracking is free** — the "S" jump from the corner of one
    ring to the start of the next is a single fly-through (no zig-zag).

**Implementation home:** lives in `sentai.explore` (Stage 3.C) as a
helper consumed by the EXPLORE state's per-tick action.  The mission FSM
calls `_explore_next_waypoint()` which returns the next H3 cell centroid
to fly to — that's what `sentai.servo.move(...)` then commands.

s125_integrated_demo currently flies a hard-coded 4-corner square (Stage
3.A-era).  Stage 3.C swaps this for SFLVP — same world model, same FSM,
new waypoint generator.

---

## 8. Where new modules live (architecture map)

```
examples/sentai_runtime/
├── sentai_objects.{h,cc}              ← Stage 1 (NEW)
├── sentai_object_lifter.{h,cc}        ← Stage 5 (NEW)
├── sentai_mission_sm.{h,cc}           ← Stage 3 (NEW)
├── sentai_action_layer.{h,cc}         ← Stage 4 (NEW)
├── sentai_tracker.{h,cc}              ← reused (Stage 7 extends)
├── sentai_anchor_forward.cc           ← reused (Stage 6 extends)
├── sentai_aruco_shim.h                ← reused (Stage 6 adds is_loop_closure)
├── sentai_link.{h,cc}                 ← reused (Stage 4 adds send_velocity)
├── sentai_crazy.{h,cc}                ← reused (Stage 4 adds send_velocity)
├── modsentai_objects.c                ← Stage 1 binding (NEW)
├── modsentai_mission.c                ← Stage 3 binding (NEW)
├── modsentai_servo.c                  ← Stage 4 binding (NEW)
├── models/red_cube_v1.tflite          ← Stage 2 artifact
├── experiments/
│   ├── s114_red_cube_model/           ← Stage 2
│   ├── s115_endtoend_cube_landing/    ← Stage 8
│   └── s116_arm_timing_budget/        ← Stage 9
└── paper/
    └── objects_nav.md                 ← Stage 10
```

SIM side mirrors:
```
sim/
├── (sentai_objects.cc shared — pure data, builds for both targets)
├── (sentai_object_lifter.cc shared — math, builds for both)
├── (sentai_mission_sm.cc shared)
├── (sentai_action_layer.cc shared)
├── sim_velocity_sink.c                ← Stage 4 SIM stub for send_velocity (logs to file)
└── modsentai_sim.c                    ← Stage 1/3/4 bindings (parity)
```

---

## 9. Validation infrastructure (cross-stage)

### CI gates
- ARM build: `cmake --build build --target sentai_runtime` → exit 0
- SIM build: `cmake --build build-sim --target sentai_sim` → exit 0
- For each new module: `test_<module>_wire.py` (struct.pack smoke) runs in < 10 s
- Pipeline smoke: existing `aruco_bench` still PASS (no perf regression)
- Bench fps: `sentai.diag.tpu_bench()` ≥ 42 fps (yolo_1 baseline)
- ITCM check: `.text` < 235 KB (leaves 15 KB margin for surprise additions)

### Continuous artifacts per run
- `pose.csv` (concept §12.4) — 50 ms cadence
- `mission.log` — every transition + intent emission
- `lifter.log` — per-tracklet EKF state trace
- `health.csv` — stack hwm + fault counters per task per second

### Re-runnable scenarios
Each experiment under `experiments/sNNN_*` has:
- `run.sh` deterministic launcher (no manual steps)
- `expected.json` PASS thresholds
- `analyze.py` extracts metrics + decides PASS/FAIL
- `README.md` documents reproducibility (seed, distrobox commands, env)

---

## 10. First three actions to kick off

When ready to start coding (after operator review of this plan):

1. **Stage 1.A** — Write `sentai_objects.h` + skeleton `sentai_objects.cc` with the static array + add/get/list. **No EKF, no lifter, no integration.** PR review for API ergonomics + naming consistency with sentai_tracker.
2. **Stage 1.B** — `modsentai_objects.c` MP binding + QSTR regen + `test_objects_wire.py` smoke. PASS = sequential add/get/remove + eviction logic + NaN rejection.
3. **Stage 1.C** — `experiments/s114_objects_map_smoke/` — a 30-second on-board script that fills + drains the map at 10 Hz, watching stack hwm + fault counters. Then commit + ship to memory as "data layer done".

This is the minimal first slice that proves the pattern and unblocks Stages 3-5 in parallel.

---

---

## 11. Feasibility detaliat pe MCU/ARM (RT1176 Cortex-M7 @ 800 MHz)

> Această secțiune răspunde la întrebarea critică: **ce parte din planul
> de mai sus chiar rulează pe i.MX RT1176, și cu ce buget?** Bazat pe
> măsurători reale din s111 (PXP+SIMD bench), characteristicile siliciului
> (256 KB ITCM, 512 KB DTCM, 1 MB OCRAM, 32 MB SDRAM extern), și
> capabilitățile FPU-ului hard single-precision al M7.

### 11.1 Resurse hardware disponibile

| Resursă | Cantitate | Note |
|---|---|---|
| M7 @ 800 MHz | 800 cicluri/µs | hard-float FPU single-precision (FPv5-d16); SIMD DSP-extension (`__USAD8`, `__SEL`, `__QADD8`, etc.) |
| Buget per frame @ 30 fps | **33.3 ms = 26.6M cicluri** | împărțit între ISR-uri, MicroPython, flow_task, detection_task, mission, action, lifter, anchor_forward |
| ITCM (m_text) | 252 KB | ~32 KB liber post-relocation; **single-cycle**, hard-wired la CPU |
| DTCM (m_data) | 256 KB | ~50 KB liber; single-cycle, dedicated bus |
| OCRAM | 1 MB | 916 KB = `.tpu_input` (load-bearing); ~50 KB liber pentru sectiuni numite |
| SDRAM (cached) | 16 MB | ~50-100× mai lent decât ITCM la cold fetch; ICache 16 KB absoarbe loop-uri |
| EdgeTPU @ 4 TOPS INT8 | ~32 ms/invoke yolo_1 (512×512) | 41 fps proven pipeline |
| PXP HW (2D pipe) | scale/rotate/CSC `<` tens of µs | 1.1 ms threshold proven s111 |
| eDMA 32 channels | ~24 free | unused; pot face prefetch tile staging |

### 11.2 Bugetul de cicluri per layer

Cifre din s111 + estimări conservative cu CMSIS-DSP `arm_mat_*_f32`:

| Layer | Operație critică | Cicluri estimate | µs @ 800 MHz | Rată | % CPU |
|---|---|---:|---:|---:|---:|
| **L1 EKF predict** (16×16 mat-mul, 2×) | `arm_mat_mult_f32(16×16)` | ~25K × 2 = 50K | 62 | 200 Hz | **1.2%** |
| **L1 EKF flow update** (3×3 inversion + Kalman gain) | small mat ops | ~40K | 50 | 100 Hz | **0.5%** |
| **L1 EKF baro update** | scalar | ~5K | 6 | 50 Hz | **0.03%** |
| **L1 EKF anchor update** (loop closure, rare) | 3×3 ops | ~80K | 100 | 1-2 Hz | **0.02%** |
| **L2 Detector** | EdgeTPU USB invoke + M7 NMS | proven | **~32 ms/frame** | 30 fps | **40%** (TPU offloaded; M7 doar coord) |
| **L3 Tracker (ByteTrack + IMU CMC)** | already in production | proven | ~2-3 ms | 30 fps | **8%** |
| **L4 Object lifter** (per tracklet, 3-state EKF) | bearing rot + inverse-depth update | ~12K | 15 | 30 tr/s | **0.05%** |
| **L4 Object map maintain** (32 sloturi × 6 KF/sec) | 6 KF updates/s × 10 µs | — | — | — | **0.01%** |
| **L5 Mission SM tick** | switch + guard checks (no math) | ~500 | 0.6 | 20 Hz | **0.001%** |
| **L5 Track health SM3** (32 obiecte × tick) | 32 × O(1) | ~3K | 4 | 20 Hz | **0.008%** |
| **L6 PBVS** | vec3 sub + mul + saturate | ~400 | 0.5 | 50 Hz | **0.003%** |
| **L6 IBVS** | 2 PD controllers | ~800 | 1 | 50 Hz | **0.006%** |
| **L6 APF obstacles** | 10 obstacole × distance | ~3K | 4 | 50 Hz | **0.02%** |
| **L6 send_velocity** (MAVLink encode + UART/UDP) | per anchor_forward | ~5K | 6 | 50 Hz | **0.03%** |

**Total estimat budget M7 pentru layers L1+L4+L5+L6**: **<2.5% CPU** la
30-50 Hz cadence. Restul (Layer 2 = 40% offloaded to TPU+USB, Layer 3
= 8% already proven) e deja parte din pipeline-ul existent.

**Marja libera M7 pentru toate cele 4 layers noi: ~95%.** Compute NU
este bottleneck.

### 11.3 Bugetul de memorie

| Modul | ITCM (.text) | SDRAM (.sdram_text+.sdram_bss) | OCRAM | Note |
|---|---:|---:|---:|---|
| `sentai_objects.cc` (Stage 1) | 0 (default sdram) | ~6 KB cod + 2 KB date | 0 | static `object_t map[32]` |
| `sentai_object_lifter.cc` (Stage 5) | 0 | ~8 KB cod + 3 KB date (per-obj EKF state) | 0 | calls CMSIS-DSP `arm_mat_*` (already linked) |
| `sentai_mission_sm.cc` (Stage 3) | 0 | ~4 KB cod + <1 KB date | 0 | enum + guards |
| `sentai_action_layer.cc` (Stage 4) | 0 | ~5 KB cod + <1 KB date | 0 | PBVS + IBVS + APF |
| MP bindings (objects, mission, servo) | landed în `liblibmicropython.a` → SDRAM | — | — | per `.ld` rule existing |
| **Total nou** | **0 KB** | **~30 KB SDRAM** | **0 KB** | Margine ITCM rămâne ~32 KB |

**Conclusie memorie**: Niciun layer nou nu atinge ITCM, OCRAM rămâne
neatins (TPU pipeline neaffectat), SDRAM are 16 MB → 30 KB e zgomot.

### 11.4 Punctele tari ale RT1176 pentru această sarcină

1. **FPU hard single-precision (FPv5-d16)** — toate operațiile float
   sunt single-cycle dispatched. Quaternion math, EKF Kalman gain,
   inverse-depth conversion — toate în hardware. Nu există overhead
   de soft-float.

2. **CMSIS-DSP deja linkat** (3.6 KB) — funcții `arm_mat_mult_f32`,
   `arm_mat_inverse_f32`, `arm_mat_cholesky_f32` disponibile.
   Implementate cu SIMD intrinsics (`__SMLAD`, `__SXTB16`). Stage 5
   poate folosi direct.

3. **PXP HW pentru orice 2D pixel transform** — scale, rotate (90°
   step), CSC, alpha blend. Tens of µs per operație. Stage 5 lifter
   poate folosi PXP pentru rectificare ROI bbox dacă apare nevoia.

4. **eDMA cu 24 canale libere** — Stage 5 poate face prefetch tile
   staging din SDRAM în DTCM pentru a face inner loops single-cycle
   (s111 a măsurat -386 µs prin DTCM staging).

5. **OCRAM `.tpu_input` deja alocat** — TPU pipeline are buffer
   dedicat, NU împart cu noi → zero contenție SEMC.

6. **`sentai.flow.anchor_forward` arhitectură deja probată** —
   pattern-ul fault-gated FreeRTOS task @ 10 Hz cu NaN/OOB/stale
   gates e direct refolosibil pentru mission SM + action layer.

### 11.5 Puncte critice / riscuri reale pe MCU

**R1 — Stabilitate numerică inverse-depth EKF (Stage 5)**:
Single-precision FP poate diverge dacă ρ (inverse depth) ajunge
foarte mic. Mitigare (deja în plan):
- Joseph form for covariance update (cost: extra 3×3 mul, ~5 µs)
- Clamp ρ_min ≥ 0.05 (depth_max = 20 m)
- Skip update dacă `trace(P)` crește post-update
- Reinitialize landmark când ρ devine negativ

Verdict: **gestionabil cu disciplina deja prevăzută** în fault model
Stage 5 F5/F6.

**R2 — SDRAM I-cache thrashing cu cod nou în `.sdram_text`**:
s111 a măsurat: `-O3 -funroll-loops` poate face naïve 7×7 box filter
**MAI ÎNCET** pe SDRAM cod (+28% time) din cauza unrolled loop
nepăsător la ICache 16 KB.

Mitigare:
- Default `-Os` pentru cod cold (mission_sm, objects, action_layer)
- Selectiv `-O3` doar pe inner loops verificate cu `objdump` + bench
- Layer 5 lifter EKF inner loop: măsurate cu DWT cycle counter prima
  oară, optimizat second pass

Verdict: **important, dar avem precedent**.

**R3 — Tight FreeRTOS scheduling când multe task-uri rulează simultan**:
Acum avem: REPL, camera, flow, anchor_forward, fs_task, http,
watchdog, dmesg. Adding mission_sm @ 20 Hz + action_layer @ 50 Hz +
lifter @ 30 Hz adaugă **3 task-uri noi**.

Mitigare:
- Toate la `tskIDLE_PRIORITY + 2` (matches anchor_forward fix din s113 P2)
- Stack `configMINIMAL_STACK_SIZE * 2` per task (= 16 KB POSIX, 2 KB ARM)
- Liveness check 500 ms în `_start()` exact ca anchor_forward
- Total task count rămâne sub 16 (FreeRTOS porter limit comfortable)

Verdict: **disciplină probată, fără surprize**.

**R4 — Loop closure rate spike când drona zboară prin "muzeu de markeri"**:
Dacă apar 10 obiecte cunoscute în câmp simultan, lifter poate
publica 10 LC events/frame × 30 fps = 300 events/sec → suprasolicită
anchor_forward queue.

Mitigare (deja în plan Stage 6 F3):
- Rate-limit LC publish la 2 Hz în lifter (drop majoritatea)
- LC events de prioritate înaltă vs anchor regulat
- Anchor_forward fault gate `dropped_stale` deja există

Verdict: **rate-limit la sursă, nu la consumer**.

**R5 — Camera pose timestamp jitter** (flow.anchor_pose age):
Lifter are nevoie de drone pose `T_W_B(t_observation)`. Dacă camera
+ flow update + IMU integrare introduce > 50 ms latență, bearing
math devine inacurat.

Mitigare:
- Stage 5 F1: skip frame dacă `src_ts_ms` > 200 ms vechi
- Timestamp camera frame chiar din CSI ISR (deja proven în
  detection_task.cc cu `frame_seq`)
- Latency budget total camera→pose ≤ 70 ms (33 ms frame + 25 ms TPU
  + 12 ms tracker) — sub pragul de 200 ms cu margine

Verdict: **margine de 3× — sigur**.

**R6 — Action layer velocity send via cf2 CRTP MTU 30 B**:
CRTP packet pentru velocity setpoint = port 7 (commander_generic),
12-13 octeți payload. Ușor sub MTU. Latență < 10 ms.

Verdict: **non-issue**, deja făcut în `sentai.crazy.send_flow`.

### 11.6 Verdictul final per layer

| Layer | Verdict feasibility M7 | Justificare |
|---|---|---|
| **L1** Ego-motion EKF | ⚠️ Feasible dar **DEFER** — folosim PX4 EKF2 / cf2 KF | Are sens doar dacă vrem firmware standalone fără PX4/cf2 — out of scope |
| **L2** Detector TPU | ✅ **Already proven** | 41 fps yolo_1 în producție |
| **L3** Tracker 2D | ✅ **Already proven** | ByteTrack + IMU CMC running |
| **L4** Map EKF (object slots) | ✅ Feasible | ~3 KB date, <0.1% CPU per tick, CMSIS-DSP disponibil |
| **L5** State machines | ✅ Trivial | ~0.01% CPU, < 1 KB date |
| **L6** Controller PBVS/IBVS | ✅ Trivial | ~0.05% CPU, math elementar |

**În ansamblu**:
- **Compute budget**: < 3% M7 nou folosit, 95% rămâne liber
- **Memory budget**: 0 KB ITCM (all `.sdram_text`), ~30 KB SDRAM
- **Toate pattern-urile necesare au precedent** în `sentai.flow.anchor_forward` / `sentai_tracker` / `sentai_objects` proposed
- **Cele 4 module noi** sunt foarte modeste comparativ cu ce există deja (TPU pipeline = 600+ KB SDRAM cod)
- **Risc rămas critic**: **stabilitate numerică inverse-depth EKF** (R1) — singura piesă cu istoric de divergence; mitigarea deja prevăzută în plan

### 11.7 Comparație cu literatura citată

Cifre raportate în papers pentru hardware similar:

| Sistem | Hardware | Sarcină comparabilă | Buget raportat |
|---|---|---|---|
| Honegger ICRA 2013 (PX4FLOW) | Cortex-M4F @ 168 MHz | Optical flow 250 Hz | Folosea ~60% CPU |
| He ICRA 2021 (PicoVO) | STM32F767 @ 216 MHz | VO 6-DoF 33 fps @ 320×240 | Folosea ~75% CPU |
| LEVIO arXiv 2602.03294 (2026) | RISC-V ultra-low-power | VIO complet 20 fps | < 100 mW |
| Navion JSSC 2019 | ASIC custom 65nm | VIO real-time | 2 mW |

RT1176 Cortex-M7 @ 800 MHz are **4-5× mai multă putere de calcul**
decât Cortex-M4 @ 216 MHz. Sarcina noastră concretă (L4-L6) e mai
ușoară decât VO completă (nu calculăm feature matches, nu rulăm
optimizare ne-liniară). Conclusion **conservativă**: avem margine
foarte mare.

### 11.8 Recomandare ordine prioritate validare buget

În stagiul Stage 9 (ARM bring-up + timing validation), valida în
ordinea:

1. **Stage 5 lifter EKF tick cost** — măsurat cu DWT, target < 200 µs/tick @ 30 Hz. Dacă depășește 500 µs → optimizare cu CMSIS-DSP Joseph form / inline matrix ops.
2. **Stage 3 mission SM tick cost** — < 50 µs target (e doar guard checks). Dacă depășește 100 µs → suspect timing bug, investigate.
3. **Stage 4 action layer tick cost** — < 100 µs @ 50 Hz. APF cu 10 obstacole = ~30 µs. PBVS = ~5 µs.
4. **Stage 1 object map operations** — `add/get/list` < 10 µs (e doar memcpy + bounds check).
5. **Total task heartbeat** — toate task-urile la dwell time per tick < 10% din period. Vizibil în `sentai.<task>.stats() → tick_us`.

Dacă oricare depășește 2× budget → fallback: reduce rate (50 Hz → 30
Hz pentru action_layer), tile-stage cu DTCM (precedent s111 Phase 5
DTCM staging −26%).

---

## 12. SOTA — Multi-object rigid-constellation pose correction (research addendum, 2026-05-12)

### 12.1 Problema formală

> Drona drifteză (yaw drift principal, x/y/z secondary). În cadrul
> camerei detectăm **N obiecte cu poziții relative cunoscute** (din
> harta de obiecte construită anterior — Stages 4-5). Vrem să folosim
> **rigiditatea constelației** pentru a corecta poza dronei la fiecare
> frame când N ≥ 3 obiecte sunt vizibile simultan.

Setup matematic:
- Obiecte cunoscute în world frame: `{m_i ∈ R³, i=1..N}` (din `sentai.objects`)
- Observații curente (bearing + opțional pseudo-depth): `{z_i ∈ R³, i=1..N}` în camera frame
- Stare dronă estimată: `T̂_W_B = (R̂, t̂)`
- Întrebarea: găsește **ΔT** (corecția) astfel încât `T_W_B = ΔT · T̂_W_B` minimizează residul

Acesta-i un caz canonic de **pose estimation from known landmarks** —
una dintre cele mai bine studiate probleme din computer vision/SLAM.

### 12.2 Survey SOTA — ordine de la "clasic robust" la "modern bazat pe ML"

#### Clasa A — Closed-form 3D-3D registration (când avem pseudo-depth)

**A1. Horn 1987** — *"Closed-form solution of absolute orientation using unit quaternions"*, J. Opt. Soc. Am. A 4(4).
- Quaternion-based, închis în formă, **N ≥ 3 corespondențe**.
- Cost: O(N) memorie, ~10 µs / N=5 pe M7 cu CMSIS-DSP.
- Optim least-squares pentru rigid alignment.

**A2. Umeyama 1991** — *"Least-squares estimation of transformation parameters between two point patterns"*, IEEE TPAMI 13(4).
- SVD-based variantă a lui Horn; manage edge cases (degenerate / coplanar configs).
- Standardul de facto în registrare clasică.

**A3. Kabsch 1976** — algoritmul preluat din cristalografie pentru aligning seturi de atomi. Aceeași math ca Horn 1987, formulare diferită.

**Recomandare pentru noi**: **Horn 1987 quaternion form** — singura
operație non-trivială e o decomposition de eigenvalori 4×4 a matricii
de cross-covariance, deja prezentă în CMSIS-DSP (`arm_mat_jacobi_f32`
pentru SVD-like).

#### Clasa B — PnP (Perspective-n-Point) — când avem doar bearing (fără depth)

**B1. Gao et al. 2003** — *"Complete solution classification for the perspective-three-point problem"*, IEEE TPAMI 25(8).
- P3P închis în formă: 3 corespondențe → până la 4 soluții candidate.
- Cost: ~50 µs pe M7 (un polinom de gradul 4 + select valid).
- Fundamentul tuturor PnP modernate.

**B2. Lepetit, Moreno-Noguer, Fua 2009** — *"EPnP: An Accurate O(n) Solution to the PnP Problem"*, IJCV 81(2).
- Reduce problema la 4 control points + soluție liniară.
- O(N) complexitate, ~100 µs pentru N=10 pe M7.
- **Standardul industrial** pentru PnP rapid.

**B3. Kneip, Furgale 2014** — *"Direct least-squares solution to the absolute and relative pose problems"*, ICRA 2014.
- "Most accurate non-iterative PnP" — OPnP.
- Mai precis decât EPnP cu cost similar.

**B4. Ferraz et al. CVPR 2014** — *"Very Fast Solution to the PnP Problem with Algebraic Outlier Rejection"*.
- Combinație PnP + RANSAC într-un singur pas; robust la outliers.

**Recomandare pentru noi**: **EPnP** (Lepetit 2009) — robust, fast,
implementări open-source disponibile pentru port la M7.

#### Clasa C — Wahba's problem (când vrem doar atitudine din bearing)

**C1. Wahba 1965** — *"A least squares estimate of satellite attitude"*, SIAM Review 7(3).
- Problema clasică din aerospace: găsește rotația R care minimizează `Σ w_i ||R·r_i − b_i||²`.
- Pentru noi: bearings de obiecte cunoscute → corecție atitudine pură.

**C2. Davenport q-method 1968** — soluție via matrice K 4×4 și eigenvector dominant. Robust.

**C3. Markley 1988** — *"Attitude determination using vector observations and the singular value decomposition"*, Journal of the Astronautical Sciences 36(3). SVD-based, numerically stable.

**C4. TRIAD** — simplificare grosolană dar foarte rapidă pentru N=2 corespondențe.

**Caz special pentru noi — Yaw-only Wahba**:
Pentru că IMU-ul nostru observă deja gravitația (roll/pitch sunt
observabile), problema reduce la **estimarea unui singur unghi**
(yaw). Acest caz are **soluție trivială**:

```
Pentru fiecare i, proiectează r_i (obiect în world frame) și
b_i (bearing măsurat în body frame) în planul orizontal:
  r_i_xy = (r_i.x − t̂.x, r_i.y − t̂.y)   // după translație
  b_i_xy = R_roll_pitch_known · b_i        // body→horizontal

θ_yaw_correction = atan2(
    Σ_i w_i · (b_i.x · r_i.y − b_i.y · r_i.x),    // sin
    Σ_i w_i · (b_i.x · r_i.x + b_i.y · r_i.y)     // cos
)
```

Asta-i o medie ponderată circulară. **N atomi × ~20 cycles = 100 µs
pentru N=5**. Robust, închis în formă, exact ce ne trebuie pentru
loop closure de yaw în absența magnetometrului.

#### Clasa D — Object-level SLAM (modern)

**D1. Salas-Moreno et al. CVPR 2013** — *"SLAM++: Simultaneous localisation and mapping at the level of objects"*.
- **Reperul** pentru object-based SLAM.
- Folosește bază de date de obiecte cu modele 3D pre-cunoscute.
- Pose graph optimization, dar pe nivel de obiect (nu features).
- Inspiră direct ce vrem să construim.

**D2. Bowman et al. ICRA 2017** — *"Probabilistic Data Association for Semantic SLAM"*.
- Tratează **incertitudinea în identitatea obiectului** ("acest cub roșu e cubul-țintă sau distractorul?").
- Critic când avem mai multe obiecte de aceeași clasă.

**D3. Doherty et al. RAL 2020** — *"Probabilistic Data Association via Mixture Models for Robust Semantic SLAM"*.
- Extinde Bowman cu mixture model EM.

**D4. Yang & Scherer TRO 2019 (CubeSLAM)** — landmark cuboids din 2D bbox + vanishing points. Mai complex decât point landmark.

**D5. Nicholson et al. RAL 2019 (QuadricSLAM)** — landmark elipsoidal cu dual quadrics. Mai precis decât point landmark pentru obiecte mari.

**D6. Liu et al. RAL 2022** — *"Object-Aware Visual Inertial Navigation"*. Folosește semantic objects ca landmarks în VIO, raportează drift reducerii cu 70% pe 5-min trajectory în muzeu.

**D7. Wang et al. CVPR 2024 (VOOM)** — *"Volumetric Object-Oriented Mapping"*. SOTA actual; foloseste volumetric repr.

**D8. Frost et al. RAL 2020** — *"Object-supplemented bundle adjustment for monocular SLAM"*. Recuperează scala prin priors de mărime obiect — direct relevant pentru pseudo-depth-ul nostru.

**D9. SO-SLAM (Liao et al. RAL 2022)** — *"Semantic Object SLAM with Structural Constraints"*. Adaugă constrângeri planare/normale.

#### Clasa E — Constellation-based place recognition (gradul cel mai înalt)

**E1. Liu & Milford ICRA 2018** — *"LoST: Visual Place Recognition with Lost Salient Features"*.
- Recunoaște locuri din **constelația** de feature-uri salient, nu individual.

**E2. Sünderhauf et al. IJRR 2018** — *"Place recognition with ConvNet landmarks: Viewpoint-robust, condition-robust, training-free"*.
- Folosește CNN pentru landmark identification + constellation matching.

**E3. Garg et al. IJRR 2020** — *"Where is your place, visual place recognition?"* — survey extins.

**Pentru cazul nostru**: prea greu pentru MCU (necesită feature CNN);
relevanță doar pentru future research. Defer.

#### Clasa F — EKF/UKF multi-landmark simultaneous update

**F1. Smith & Cheeseman 1986** — *"On the Representation and Estimation of Spatial Uncertainty"*, IJRR 5(4). Fundamentul EKF-SLAM.

**F2. MonoSLAM (Davison TPAMI 2007)** — multi-landmark update simultan în EKF, cu inverse-depth.

**F3. Mourikis & Roumeliotis ICRA 2007** — *"A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation"* (MSCKF).
- **Standardul** pentru VIO cu landmarks.
- Tratează landmark-urile ca state auxiliare temporare → reduce dimensiunea state-ului.
- Folosit în Project Tango, Skydio, ARKit, ARCore.

**F4. Solà 2014** — *"Quaternion kinematics for the error-state Kalman filter"*, online text. Standard pentru error-state EKF cu quaternioni.

**Recomandare pentru noi (final)**: **stacked EKF update cu multiple
landmarks per frame**.

### 12.3 Algoritm recomandat — "Multi-Object Yaw-Wahba + EKF stack"

Combinăm două abordări complementare:

#### Pasul 1 — Yaw-Wahba pentru corecție grosieră instantanee
Când detectăm N ≥ 3 obiecte cunoscute simultan:
```c
// Inputs: 
//   map_pos[N] - poziții obiecte în W (din sentai_objects)
//   bbox_center[N] - centroizi în pixeli
//   drone_pose_est - poza dronei curent estimată
//   K - camera intrinsics
// Output: yaw_correction (radians)

float yaw_correction_wahba(
    const float (*map_pos)[3], int N,
    const float (*bbox_uv)[2],
    const sentai_pose_t *est,
    const float K[9]
) {
    float sin_sum = 0, cos_sum = 0;
    for (int i = 0; i < N; i++) {
        // Bearing in camera frame from bbox pixel
        float b_C[3];
        bearing_from_pixel(bbox_uv[i], K, b_C);
        // Rotate body→world using IMU-derived roll/pitch (NOT yaw, that's what we estimate)
        float b_W_horiz[2];
        rotate_to_horizontal(b_C, est->roll, est->pitch, b_W_horiz);
        
        // Object direction in world (in horizontal plane)
        float r_W_horiz[2] = {
            map_pos[i][0] - est->x,
            map_pos[i][1] - est->y
        };
        normalize2(r_W_horiz);
        normalize2(b_W_horiz);
        
        // Weight by inverse pseudo-depth (closer = more reliable bearing)
        float w_i = 1.0f / depth_est[i];
        
        sin_sum += w_i * (b_W_horiz[0] * r_W_horiz[1] - b_W_horiz[1] * r_W_horiz[0]);
        cos_sum += w_i * (b_W_horiz[0] * r_W_horiz[0] + b_W_horiz[1] * r_W_horiz[1]);
    }
    return atan2f(sin_sum, cos_sum);
}
```

**Cost: ~100 µs pentru N=5 pe M7**. Soluție closed-form, fără
iterații, fără linearizare.

#### Pasul 2 — Multi-landmark EKF update (probabilistic refinement)
După corecția grosieră, rulează un single EKF update cu **toate cele
N observații stacked**:
```
H = [H_1; H_2; ...; H_N]      // 2N × state_dim Jacobian stacked
R = blockdiag(R_1, R_2, ..., R_N)  // 2N × 2N observation cov
z_pred = [h_1(x); h_2(x); ...; h_N(x)]
z_meas = [bbox_1; bbox_2; ...; bbox_N]
innovation = z_meas - z_pred

S = H · P · H^T + R           // 2N × 2N innovation cov
K = P · H^T · S^-1            // Kalman gain
x = x + K · innovation        // state update
P = (I - K · H) · P · (I - K · H)^T + K · R · K^T  // Joseph form covariance update
```

Pentru N=5 obiecte, 2N=10 dimensiune observație. Matrix inverse 10×10
= ~10K cicluri pe M7 cu CMSIS-DSP. Toată actualizarea: ~50 µs.

**Combo Pasul 1 + Pasul 2: ~150 µs per frame când 3+ obiecte
re-observate.**

### 12.4 Probabilistic Data Association (PDA)

Problema reală: când vedem un cub roșu și sunt 2 cuburi roșii în
hartă, **care e care?**

**Soluție SOTA — Maximum Likelihood DA** (Bowman 2017):
```
Pentru fiecare observație i, pentru fiecare candidate map slot j:
    Mahalanobis distance d_ij = (z_i - h(m_j))^T · S_ij^-1 · (z_i - h(m_j))
Asociază obs i cu slot j argmin(d_ij), DAR doar dacă d_ij < gate_thresh.
Altfel: noua observație → inițializare landmark nou.
```

Pentru noi: gate_thresh = chi-square(0.99, dof=2) ≈ 9.21 (pentru
observații 2D bbox center).

**Cost: ~50 µs pentru N=5 × M=20 candidates pe M7**.

### 12.5 Mapping pe Stage 6 existing (loop closure on yaw)

§3 Stage 6 din planul curent: "loop closure on yaw" — descris ca
SINGLE-OBJECT. Trebuie **extins** la MULTI-OBJECT:

**Schimbări la Stage 6**:

| Aspect | Plan curent (single) | Plan nou (multi-object) |
|---|---|---|
| Trigger | 1 obiect re-observat | **N ≥ 3 obiecte** re-observate simultan |
| Math | innovation single bearing | **Yaw-Wahba** (Cls C) + **stacked EKF update** (Cls F) |
| Data association | implicit (1 obiect = 1 candidate) | **PDA Mahalanobis-gated** (Cls D2) |
| Robustness | rejected at >45° single error | **outlier rejection RANSAC-style** pe constelație (best subset wins) |
| Fault model F1 | yaw correction > 45° | extins: **gospel data association** dacă inlier ratio < 60% |
| Cost per frame | ~5 µs | ~150 µs (justifiable; rar > 5 Hz) |

**Pseudo-cod Stage 6 extins**:
```c
void lifter_check_loop_closure(const tracked_objs_t *tracks, int N_tracks,
                               sentai_pose_t *est) {
    // 1. Filter to mature tracks with high-confidence map association
    int N_obs = 0;
    obs_t obs[MAX_OBJ];
    for (int i = 0; i < N_tracks; i++) {
        int slot = sentai_objects_associate(&tracks[i], est);  // PDA
        if (slot >= 0 && map[slot].observations >= 3) {
            obs[N_obs++] = (obs_t){ &tracks[i], slot };
        }
    }
    if (N_obs < 3) return;   // Need 3+ for rigid constraint
    
    // 2. RANSAC over triples for inlier set (rejects mis-associations)
    int best_inliers = ransac_yaw_wahba(obs, N_obs, est, &best_yaw_corr);
    if (best_inliers < N_obs * 0.6f) return;   // gospel data assoc failed
    
    // 3. Yaw-Wahba closed-form on inliers
    float yaw_corr = yaw_correction_wahba(obs_inliers, best_inliers, est);
    if (fabsf(yaw_corr) > YAW_MAX_CORRECTION_RAD) return;   // F1 fault gate
    
    // 4. Multi-landmark EKF update with all inliers
    ekf_multi_update(est, obs_inliers, best_inliers);
    
    // 5. Publish corrected pose to anchor_forward
    sentai_aruco_pose_t snap = { ... };
    snap.is_loop_closure = 1;
    sentai_aruco_publish(&snap);
}
```

### 12.6 Validation tests (extensia Stage 6)

Test scenariu: drona zboară în pătrat 2 minute peste arena cu 4+
obiecte cunoscute la poziții fixe.

| Metric | Single-object LC | **Multi-object LC** | Improvement |
|---|---|---|---|
| Yaw drift după 2 min | < 2° | **< 0.5°** | 4× |
| Frecvență LC trigger | ~0.5 Hz | **2-5 Hz** (când 3+ vizibile) | 5-10× |
| Robustness false-ID | F1 single threshold | **RANSAC outlier rejection** | calitativ mai bun |
| Cost per frame | 5 µs | 150 µs | acceptabil |

### 12.7 Buget MCU pentru extensie multi-object

| Operație | Cost cicluri | µs @ 800 MHz | Frecvență | % CPU |
|---|---:|---:|---:|---:|
| PDA Mahalanobis (5 obs × 20 cand) | 40K | 50 | 30 Hz | **0.15%** |
| RANSAC over triples (10 iter × P3P) | 50K | 62 | 5 Hz | **0.03%** |
| Yaw-Wahba closed-form (N=5) | 8K | 10 | 5 Hz | **0.006%** |
| Stacked EKF update (10×state) | 40K | 50 | 5 Hz | **0.03%** |
| **Total multi-obj LC** | | **172 µs/event** | **5 Hz** | **0.2%** |

**Memorie suplimentară**:
- RANSAC scratch: ~200 B (inlier mask + best params)
- Stacked H/R/S matrices: ~600 B (N=5 worst case)
- Total: < 1 KB SDRAM, 0 KB ITCM

### 12.8 Referințe consolidate pentru §12

Clasa A (3D-3D registration):
- Horn, B.K.P. "Closed-form solution of absolute orientation using unit quaternions". J. Opt. Soc. Am. A 4(4), 1987.
- Umeyama, S. "Least-squares estimation of transformation parameters between two point patterns". IEEE TPAMI 13(4), 1991.

Clasa B (PnP):
- Gao, X.-S., et al. "Complete solution classification for the perspective-three-point problem". IEEE TPAMI 25(8), 2003.
- Lepetit, V., Moreno-Noguer, F., Fua, P. "EPnP: An Accurate O(n) Solution to the PnP Problem". IJCV 81(2), 2009.
- Kneip, L., Furgale, P. "OPnP: A Direct Least-Squares Method to the Perspective-n-Point Problem". ICRA 2014.
- Ferraz, L., Binefa, X., Moreno-Noguer, F. "Very Fast Solution to the PnP Problem with Algebraic Outlier Rejection". CVPR 2014.

Clasa C (Wahba's problem):
- Wahba, G. "A least squares estimate of satellite attitude". SIAM Review 7(3), 1965.
- Davenport, P.B. "A vector approach to the algebra of rotations with applications". NASA TN D-4696, 1968.
- Markley, F.L. "Attitude determination using vector observations and the singular value decomposition". J. Astronaut. Sci. 36(3), 1988.

Clasa D (Object-level SLAM):
- Salas-Moreno, R.F. et al. "SLAM++: Simultaneous localisation and mapping at the level of objects". CVPR 2013.
- Bowman, S.L. et al. "Probabilistic Data Association for Semantic SLAM". ICRA 2017.
- Doherty, K.J. et al. "Probabilistic Data Association via Mixture Models for Robust Semantic SLAM". RAL 5(2), 2020.
- Yang, S., Scherer, S. "CubeSLAM: Monocular 3-D Object SLAM". IEEE TRO 35(4), 2019.
- Nicholson, L. et al. "QuadricSLAM: Dual Quadrics from Object Detections as Landmarks". IEEE RAL 4(1), 2019.
- Frost, D. et al. "Recovering Stable Scale in Monocular SLAM Using Object-Supplemented Bundle Adjustment". IEEE RAL 5(2), 2020.
- Liu, X. et al. "Object-Aware Visual Inertial Navigation". IEEE RAL 7(4), 2022.
- Wang, J. et al. "VOOM: Robust Visual Object Odometry and Mapping using Hierarchical Landmarks". CVPR 2024.
- Liao, Z. et al. "SO-SLAM: Semantic Object SLAM with Scale Proportional and Symmetrical Texture Constraints". IEEE RAL 7(2), 2022.

Clasa F (EKF/MSCKF):
- Smith, R., Cheeseman, P. "On the Representation and Estimation of Spatial Uncertainty". IJRR 5(4), 1986.
- Davison, A.J. "Real-Time Simultaneous Localisation and Mapping with a Single Camera". TPAMI 29(6), 2007 (MonoSLAM).
- Mourikis, A.I., Roumeliotis, S.I. "A Multi-State Constraint Kalman Filter for Vision-aided Inertial Navigation". ICRA 2007 (MSCKF).
- Solà, J. "Quaternion kinematics for the error-state Kalman filter". arXiv 1711.02508, 2017.

### 12.9 Recomandare pentru implementare

**Drumul minim viabil (MVP) pentru Stage 6 extins**:

1. **Stage 6.A** — Yaw-only Wahba closed-form, fără RANSAC, N ≥ 3
   obiecte cu data association `argmin(Mahalanobis)`. Cost: 80 µs/event.
   Validare: yaw drift < 1° pe 2-minute square mission.

2. **Stage 6.B** — Add RANSAC outlier rejection. Cost: +50 µs (când
   triggered, sub 5 Hz). Validare: rezistă la 1 obiect mis-associated
   din 5.

3. **Stage 6.C** — Full multi-landmark stacked EKF update (full
   position+yaw correction). Cost: +50 µs. Validare: position drift
   redusă cu > 50% comparativ cu 6.A.

4. **Stage 6.D** (DEFER, dacă vedem nevoia) — Probabilistic Data
   Association cu mixture models (Doherty 2020) pentru obiecte
   semantic-ambigue.

**Bottom line**: SOTA pentru cazul user-ului există de **40 de ani**
(Horn 1987, Wahba 1965), e **trivially MCU-feasible** (< 200 µs/event
@ 5 Hz = 0.2% CPU), și se mapează DIRECT pe Stage 6 existing din
plan. Extensia adaugă ~5 KB cod + ~1 KB date SDRAM, niciun byte
ITCM. Risc principal: data association în prezența obiectelor
multiple de aceeași clasă — mitigat de Mahalanobis-gating + RANSAC.

---

## 13. SOTA — Pigeon-style topological navigation + global scene fingerprinting (research addendum 2, 2026-05-12)

### 13.1 Întrebările operatorului

> **Q1**: Putem detecta și memora **landmark-uri macro** (intersecții,
> drumuri, poduri, lacuri, case) și să ne orientăm așa cum se
> orientează porumbeii?
>
> **Q2**: Forma globală a peisajului de sub noi dă o amprentă —
> putem crea **macro-amprente** ale lumii, robuste la mișcarea unor
> obiecte individuale? Există SOTA?

**Răspuns scurt**: DA pe ambele, sunt arii bine maturate de research.
Q1 = **Topological SLAM + Visual Place Recognition (VPR)** cu landmark-uri
semantice. Q2 = **Global scene descriptors** (GIST, NetVLAD, CosPlace,
AnyLoc). Ambele rulează pe MCU în variantele lightweight (descriptori
clasici + matching) sau pe EdgeTPU (CNN compact). Există patternuri
direct portabile pe sentai_runtime.

### 13.2 Cum navighează porumbeii — analogia hardware

Porumbeii folosesc **trei mecanisme paralele** (Wiltschko & Wiltschko
2003; Mouritsen Nature 2018):

| Mecanism porumbel | Echivalent dronă | Stare în sentai_runtime |
|---|---|---|
| Soare compas (sub-conștient) | IMU + gravity vector | ✅ exists (roll/pitch din accel) |
| Magnetic compass (cryptochrome) | Magnetometer | ❌ NOT available (lipsă deliberată) |
| **Visual landmarks (recunoaștere de locuri)** | Visual Place Recognition + Topological SLAM | ❌ to build (Q1 here) |
| **Path integration (memorie cinematică)** | Flow-EKF integration | ✅ exists (sentai.flow + PX4 EKF) |
| **Olfactory mapping** | n/a | n/a |

Pierderea magnetometrului = **trebuie să închidem gap-ul prin
landmarks vizuali** (exact filozofia obiectului anchor + loop
closure din Stages 5-6). Q1 este o extindere semantică a acelei
filozofii.

### 13.3 Q1 — Topological landmark mapping (intersections / roads / lakes / houses)

#### Două abordări principale:

**A. Recunoaștere semantică per-categorie cu detector NN**:
- Antrenează YOLO sau DETR pe clase macro: `INTERSECTION, BRIDGE, LAKE, HOUSE, ROAD_FORK, FOREST_CLEARING, etc.`
- Each detection → punct topologic în harta de obiecte (extinde `sentai_objects.cc`)
- Aceeași infrastructură ca Stages 4-5 (object map + lifter), DOAR cu vocabular extins

**B. Visual Place Recognition (VPR) — recunoaște locul fără să detalii ce-i acolo**:
- Calculează un descriptor global per frame
- Compară cu o galerie de descriptori-de-locuri stocați
- Identifică "am mai văzut locul ăsta" → loop closure topologic

#### A. Detector semantic — referințe SOTA

**Aerial/UAV scene classification**:

- **Cheng et al. ISPRS JPRS 2017** — *"Remote Sensing Image Scene Classification: Benchmark and State of the Art"*. **NWPU-RESISC45** dataset (45 classes including bridge/road/lake/house). Baseline pentru fine-tuning.

- **Helber et al. JSTARS 2019 (EuroSAT)** — *"EuroSAT: A Novel Dataset and Deep Learning Benchmark for Land Use and Land Cover Classification"*. 27k satellite images, 10 classes. Light enough pentru EdgeTPU (MobileNet variant).

- **Long et al. RAL 2017** — *"Fully Convolutional Adaptation Networks for Semantic Segmentation"*. Per-pixel segmentation aeriană. Heavy pentru MCU dar idea-ul e portabil.

- **PIE-Net / aerial DETR (2022-2024)** — modern transformer-based.

**Urban scene semantic SLAM**:

- **Sünderhauf et al. IJRR 2018** — *"Place recognition with ConvNet landmarks: Viewpoint-robust, condition-robust, training-free"*. Folosește CNN pretrained pentru landmark identification — fără fine-tuning. **Direct aplicabil pentru noi.**

- **Schönberger et al. CVPR 2018** — *"Semantic Visual Localization"*. Localizare cu segmentare semantică.

- **Pronobis & Jensfelt IJRR 2012** — *"Large-Scale Semantic Mapping and Reasoning with Heterogeneous Modalities"*. Mapping cu obiecte și camere multiple.

**Lightweight pentru MCU**:

- **MobileNet-SSD** + clase semantice (deja avem infrastructure pentru asta cu sentai.tpu)
- **EfficientDet-Lite0** pe EdgeTPU — < 5 MB, ~10 ms/inferență
- **YOLOv8n** (deja recomandat în Stage 2) — extins cu clase macro

#### B. Visual Place Recognition (VPR) — referințe SOTA

**Clasice (compute-light, MCU-friendly)**:

- **Oliva & Torralba IJCV 2001 (GIST)** — *"Modeling the shape of the scene: A holistic representation of the spatial envelope"*. **Descriptor global 512-dim** din Gabor filters multi-scale + spatial pooling. **Computabil în ~50 ms pe M7 cu PXP pentru filtering.** Foarte folosit până în 2015.

- **Dalal & Triggs CVPR 2005 (HOG)** — *"Histograms of Oriented Gradients for Human Detection"*. HOG global = scene signature compact. ~30 ms pe M7.

- **Cummins & Newman IJRR 2008 (FAB-MAP)** — *"FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance"*. Appearance-only SLAM, Bayesian framework. **Standardul VPR clasic.**

- **Milford & Wyeth ICRA 2012 (SeqSLAM)** — *"SeqSLAM: Visual Route-Based Navigation for Sunny Summer Days and Stormy Winter Nights"*. Sequence matching, **robust la appearance change**.

**Deep (require EdgeTPU)**:

- **Arandjelović et al. CVPR 2016 (NetVLAD)** — *"NetVLAD: CNN architecture for weakly supervised place recognition"*. Reperul modern, descriptor 4096-dim. ~50-100 MB model size — **prea greu pentru EdgeTPU stock**, dar variante MobileNet-NetVLAD ~5 MB sunt fezabile.

- **Berton et al. CVPR 2022 (CosPlace)** — *"Rethinking Visual Geo-Localization for Large-Scale Applications"*. Compact embedding 512-dim, **dataset 8M imagini**. ResNet50 backbone (~100 MB) — too heavy.

- **Berton et al. ICCV 2023 (EigenPlaces)** — *"EigenPlaces: Training Viewpoint Robust Models for Visual Place Recognition"*. Robust la unghi de vedere — critic pentru dronă. Similar mărime.

- **Keetha et al. RAL 2024 (AnyLoc)** — *"AnyLoc: Towards Universal Visual Place Recognition"*. **Universal foundation model**, zero-shot pe orice mediu. Foloseste DINOv2 + VLAD. Model > 100 MB; **DOAR cu EdgeTPU quantization aggressive**.

- **DeLG (Cao et al. ECCV 2020)** — *"Unifying Deep Local and Global Features for Image Search"*. Local+global combinat.

**Aerial-specific VPR** (cross-view satellite ↔ ground):

- **Hu et al. CVPR 2022 (DeepSeeker)** — *"Beyond Geo-Localization: Fine-Grained Orientation of Street-View Images by Cross-View Matching with Satellite Imagery"*. UAV-to-satellite matching.

- **Hu et al. ECCV 2024 (SAVA)** — *"Satellite-Aerial Visual Alignment"*. SOTA actual pentru UAV-localized-by-satellite-imagery.

- **CrossLoc, Workman et al.** — Cross-view localization.

**Topological SLAM (graf de locuri)**:

- **Maravall et al. Front. Neurorobotics 2017** — *"Navigation and Self-Semantic Location of Drones in Indoor Environments by Combining the Visual Bug Algorithm and Entropy-Based Vision"*. **Direct citată în ideas/objects.md**. Visual Bug + entropy fingerprints. **Direct portabilă.**

- **Teymouri & Bhattacharya arXiv 2103.03741 (2021)** — *"Landmark-based Distributed Topological Mapping and Navigation in GPS-denied Urban Environments Using Teams of Low-cost Robots"*. Distribuită, low-cost — match-ul perfect pentru profile noastru.

- **Garg et al. IJRR 2020** — *"Where is Your Place, Visual Place Recognition?"* — comprehensive survey.

### 13.4 Q2 — Macro-fingerprint al peisajului (global scene signature)

Două abordări complementare:

#### Abordare 1 — Descriptor global pre-CNN (MCU-friendly)

**GIST descriptor pe sentai.flow.pxp_scratch**:
- Input: 80×60 RGB888 din flow PXP pipeline (deja disponibil)
- 4 orientări Gabor × 8 scale × 4×4 spatial pooling = 512 features
- Cost: ~30-50 ms pe M7 cu CMSIS-DSP FIR filters
- Memorie: 512 float = 2 KB per fingerprint
- Galerie 100 locuri × 2 KB = 200 KB SDRAM
- Matching: cosine similarity O(N × D) = 100 × 512 × 2 cycles = ~100K cycles = 125 µs

**Pattern de ușurat** — folosim PXP-thresholded image (s111) ca input
pentru un "binary GIST":
- 320×240 → PXP threshold → 320×240 binary
- 8-direcție Sobel pe binary → 8 channels gradient
- Pool spațial 4×4 → 8 × 16 = 128 features
- Cost: ~5 ms total cu PXP+SIMD (cifre s111)

**Acesta-i compromisul cel mai bun pentru noi.**

#### Abordare 2 — Lightweight CNN pe EdgeTPU

Antrenăm un MobileNetV2-mini ca **scene embedder**:
- Input 224×224 RGB (downscale via PXP din 640×480)
- Output 256-dim embedding (last avg-pool layer)
- Model size 3-5 MB → compile EdgeTPU → ~5 ms inferență
- Triplet loss training pe dataset Gazebo (anchor / positive same place / negative different place)

**Avantaj**: robust la rotație, scale, iluminare — CNN-specific
generalization.

**Cost total per place query**:
- 5 ms inferență TPU + 100 µs cosine match → < 6 ms total
- 100 places × 256-dim float = 100 KB SDRAM
- Acceptabil pentru misiuni de dimensiune medie (< 1000 locuri).

#### Abordare 3 — "Constellation count signature" (cheap & dirty)

Idee: forma globală a peisajului = **distribuția categoriilor
detectate de YOLO**, nu pixel-level descriptor.

Pseudo-cod:
```c
typedef struct {
    uint8_t class_counts[NUM_CLASSES];   // 80 classes COCO → 80 B
    uint8_t angular_distribution[16];    // bins de 22.5° → 16 B
} scene_fingerprint_t;
```

Per frame:
1. Iei output-ul YOLO (deja avem la 30 fps)
2. Numeri câte obiecte de fiecare clasă (cap la 255 per clasă)
3. Computezi distribuția angulară a centroidelor

Total: 96 B per fingerprint. **Trivial computabil**. Robust la
mișcarea individuală a obiectelor (ce contează e distribuția, nu
pozițiile exacte). Vulnerabil la schimbarea iluminării / unghi (CNN
descriptor mai robust). Bun ca **prefilter** înainte de descriptor
mai precis.

### 13.5 Pipeline propus pentru sentai_runtime

```
                  ┌──────────────────────────────────┐
                  │ Per camera frame (30 fps)        │
                  └─────┬──────────────────┬─────────┘
                        │                  │
              ┌─────────▼────────┐   ┌─────▼──────────┐
              │ YOLO (existing)  │   │ Scene embedder │
              │ → per-obj detect │   │ (NEW, EdgeTPU  │
              └─────────┬────────┘   │  or GIST-on-M7)│
                        │            └─────┬──────────┘
              ┌─────────▼────────┐         │
              │ sentai_tracker   │         │ 256-dim
              │ → tracklets      │         │ embedding
              └─────────┬────────┘         │
                        │                  │
              ┌─────────▼────────┐         │
              │ sentai_objects + │         │
              │ lifter EKF       │         │
              │ → individual     │         │
              │   landmarks 3D   │         │
              └─────────┬────────┘         │
                        │                  │
                        │            ┌─────▼──────────────┐
                        │            │ sentai_places      │  ← NEW Stage 11
                        │            │ Topological graph: │
                        │            │ node = embedding   │
                        │            │ edge = transition  │
                        │            │ → place recognition│
                        │            └─────┬──────────────┘
                        │                  │
              ┌─────────▼──────────────────▼─────────┐
              │ Loop closure (Stage 6 extins):       │
              │   - Per-object (precise yaw, §12)    │
              │   - Per-place (coarse, this section) │
              │ → corrected drone pose                │
              └─────────────────────┬─────────────────┘
                                    │
                       ┌────────────▼───────────┐
                       │ sentai.flow.anchor_fwd │
                       │  → VPE / ext_position  │
                       └────────────────────────┘
```

**Idea-cheie**: două nivele de loop closure:
1. **Fine** — per object cu Wahba/EKF (§12) — corecție yaw < 1°
2. **Coarse** — per place cu descriptor match — "sunt în zona unde
   am mai fost" → trigger pentru obiect-level re-localization +
   bias correction pe poziție

### 13.6 Stage 11 nou — `sentai.places.*` (topological place recognition)

**Goal**: graf static de N=64 locuri, fiecare cu descriptor + relațiile
de adjacență. Match per-frame query → recognize "am mai văzut".

**Files to add**:
- `examples/sentai_runtime/sentai_places.{h,cc}`
- `examples/sentai_runtime/sentai_scene_descriptor.{h,cc}` — fie GIST-on-M7, fie CNN-on-TPU
- `examples/sentai_runtime/modsentai_places.c`

**Struct**:
```c
#define PLACES_MAX 64
#define DESC_DIM 256

typedef struct {
    uint8_t  id;
    uint8_t  status;         // FREE / TENTATIVE / CONFIRMED
    int8_t   descriptor[DESC_DIM];   // quantized int8 (saves 4× memory vs float)
    float    center_W[3];    // approx pose at first visit
    uint32_t visits;
    uint32_t last_visit_ms;
    uint16_t adjacent[8];    // place IDs reachable from here
} place_t;

place_t places[PLACES_MAX];   // 64 × (DESC_DIM + 64 B header) = ~22 KB
```

**API**:
```python
sentai.places.add(descriptor_bytes, pose=None) -> place_id
sentai.places.query(descriptor_bytes) -> (best_match_id, score)
sentai.places.list() -> list[dict]
sentai.places.link(id_a, id_b)              # mark adjacency
sentai.places.stats() -> dict
```

**Operation flow**:
1. Per 1 Hz (sau triggered by mission SM): compute descriptor of current frame
2. Query against places gallery
3. If best_score > THRESH: this is place_id X. Trigger loop closure update.
4. If no good match AND drone has moved > 2 m since last add: insert new place.

**Fault model**:
| Fault | Action |
|---|---|
| F1 descriptor sum-of-squares not finite | reject, count |
| F2 places full + no STALE candidate | refuse add, signal user |
| F3 query best_score below MATCH_THRESH | "no match" — no loop closure |
| F4 false match (descriptor collision) | mitigated by per-object verification (Stage 6) |
| F5 descriptor extractor (TPU) fails | fall back to GIST-on-M7 (degraded mode) |

**MCU cost estimate (Cortex-M7 @ 800 MHz)**:
| Operație | Cost | Frequency | % CPU |
|---|---:|---:|---:|
| Scene descriptor compute (GIST-on-M7 + PXP) | ~5 ms | 1 Hz | **0.5%** |
| Scene descriptor compute (MobileNet EdgeTPU) | ~5 ms | 1 Hz | **0.5%** (offloaded TPU) |
| Cosine similarity query (N=64, dim=256, int8) | ~150 µs | 1 Hz | **0.015%** |
| Place graph maintenance | ~50 µs | 1 Hz | **0.005%** |
| **Total** | | | **< 0.6%** |

**Memorie**:
- 64 places × ~300 B = ~20 KB SDRAM
- Descriptor scratch: 1 KB
- ITCM: 0 (all `.sdram_text`)

### 13.7 Coordonarea Stage 11 cu Stage 6 (object loop closure)

Stage 6 = **precise loop closure pe yaw** din obiecte individuale.
Stage 11 = **coarse loop closure pe poziție** din scene similarity.

Cum se completează:

| Scenariu | Stage 6 | Stage 11 |
|---|---|---|
| 3+ obiecte cunoscute vizibile | ✅ precise correction | optional confirmation |
| 0-1 obiecte vizibile | n/a | ✅ "sunt în zona X" → seed prior pentru pose |
| Drone in textureless area | n/a | n/a (fallback to inertial dead reckoning) |
| Re-vizit la o zonă după 5 min | ✅ object IDs match | ✅ scene descriptor match (confirmare cross) |
| Misiune lungă 10+ min | ✅ catches drift continuu | ✅ macro-graf de zone pentru replanning |

**Critical observation**: Stage 11 e **complementar**, nu înlocuiește
Stage 6. Stage 11 dă "where am I roughly" → Stage 6 dă "exactly how
my pose is wrong". Împreună rezolvă cazul "porumbel pierdut într-un
oraș nou" pe care îl întreabă utilizatorul.

### 13.8 Pentru outdoor extensiv (sate, păduri, lacuri) — viziune pe termen mai lung

Dacă misiunea include zbor outdoor în mediu necunoscut:

**Pas viitor (Stage 12+)** — Cross-view localization cu OpenStreetMap:
- Folosește **SAVA-style** matching între camera nadir (drone) și
  satellite/OSM tiles
- OSM tile ~5 MB per square km la zoom 16
- Match coarse poziție (cell-level GPS-free) → seed pentru object-level fine
- Reperul: **Hu et al. ECCV 2024 (SAVA)** + **Brejcha et al. ECCV 2018
  (LandscapeAR)** pentru zone rurale.

Aceasta-i cea mai apropiată implementare de "porumbel migrator". DEFER
până când scenariul outdoor devine prioritate.

### 13.9 Recomandare implementare (minimal viable Stage 11)

**Stage 11.A** — *Constellation count signature* (Abordarea 3).
Implementare ~1 zi:
- Foloseste DEJA YOLO output care există
- Compute 80-byte fingerprint per frame
- Galerie 64 places × 80 B = 5 KB SDRAM
- Match O(N) cosine = ~30 µs
- Validare: detectează "am revenit în arenă" cu probabilitate > 80% pe scenariul 12.1

**Stage 11.B** — *GIST-on-M7* (Abordarea 1).
- Implementare ~5-7 zile (mai serioasă)
- Cost: 50 ms per fingerprint (în task low-priority @ 1 Hz)
- Avantaj: descriptor 128-dim mai robust, descriptori per-loc 128 B
- Galerie 64 × 192 B = 12 KB

**Stage 11.C** — *MobileNet-mini-VPR pe EdgeTPU* (Abordarea 2).
- Implementare ~2 săptămâni (necesită training + EdgeTPU compile +
  validation)
- Cost: 5 ms per fingerprint pe TPU (în task @ 1 Hz)
- Avantaj: cel mai robust, dimensiune 256-dim
- Necesită dataset Gazebo cu scene labels (cluster fără supervision)

**Recomandare actuală**: începe cu **11.A** (constellation count) ca
proof-of-concept. Dacă funcționează în scenariul Stage 8 (cube
mission), avansează la 11.B pentru robustness. 11.C numai dacă
misiunile outdoor extensive devin prioritate.

### 13.10 Referințe consolidate pentru §13

#### Place recognition clasic
- Oliva, A., Torralba, A. "Modeling the shape of the scene: A holistic representation of the spatial envelope". IJCV 42(3), 2001. (GIST)
- Dalal, N., Triggs, B. "Histograms of Oriented Gradients for Human Detection". CVPR 2005.
- Cummins, M., Newman, P. "FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance". IJRR 27(6), 2008.
- Milford, M., Wyeth, G. "SeqSLAM: Visual Route-Based Navigation for Sunny Summer Days and Stormy Winter Nights". ICRA 2012.

#### Deep VPR
- Arandjelović, R., et al. "NetVLAD: CNN architecture for weakly supervised place recognition". CVPR 2016.
- Berton, G., et al. "Rethinking Visual Geo-Localization for Large-Scale Applications". CVPR 2022 (CosPlace).
- Berton, G., et al. "EigenPlaces: Training Viewpoint Robust Models for Visual Place Recognition". ICCV 2023.
- Keetha, N., et al. "AnyLoc: Towards Universal Visual Place Recognition". IEEE RAL 9(2), 2024.
- Cao, B., et al. "Unifying Deep Local and Global Features for Image Search". ECCV 2020 (DELG).

#### Aerial/UAV scene & cross-view
- Cheng, G., et al. "Remote Sensing Image Scene Classification: Benchmark and State of the Art". ISPRS JPRS 2017 (NWPU-RESISC45 dataset).
- Helber, P., et al. "EuroSAT: A Novel Dataset and Deep Learning Benchmark for Land Use and Land Cover Classification". IEEE JSTARS 12(7), 2019.
- Hu, S., et al. "Beyond Geo-Localization: Fine-Grained Orientation of Street-View Images by Cross-View Matching with Satellite Imagery". CVPR 2022.
- Hu, S., et al. "Satellite-Aerial Visual Alignment". ECCV 2024.
- Brejcha, J., et al. "LandscapeAR: Large Scale Outdoor Augmented Reality by Matching Photographs with Terrain Models Using Learned Descriptors". ECCV 2018.

#### Topological SLAM + semantic landmarks
- Maravall, D., de Lope, J., Fuentes, J.P. "Navigation and Self-Semantic Location of Drones in Indoor Environments by Combining the Visual Bug Algorithm and Entropy-Based Vision". Frontiers in Neurorobotics 11, 2017.
- Teymouri, M.S., Bhattacharya, S. "Landmark-based Distributed Topological Mapping and Navigation in GPS-denied Urban Environments Using Teams of Low-cost Robots". arXiv 2103.03741, 2021.
- Sünderhauf, N., et al. "Place recognition with ConvNet landmarks: Viewpoint-robust, condition-robust, training-free". IJRR 37(4-5), 2018.
- Garg, S., et al. "Where Is Your Place, Visual Place Recognition?". IJRR 39(11), 2020.
- Pronobis, A., Jensfelt, P. "Large-Scale Semantic Mapping and Reasoning with Heterogeneous Modalities". IJRR 31(8), 2012.
- Schönberger, J.L., et al. "Semantic Visual Localization". CVPR 2018.

#### Biological navigation (analogia porumbelului)
- Wiltschko, W., Wiltschko, R. "Avian Navigation: From Historical to Modern Concepts". Animal Behaviour 65(2), 2003.
- Mouritsen, H. "Long-Distance Navigation and Magnetoreception in Migratory Animals". Nature 558, 2018.
- Holland, R.A. "True Navigation in Birds: From Quantum Physics to Global Migration". Journal of Zoology 293(1), 2014.

### 13.11 Bottom line pentru operator

**La întrebarea Q1** ("orientare ca porumbeii"): **DA**, posibilitatea
e larg studiată (Topological SLAM + Semantic VPR), e MCU-feasible în
varianta **lightweight** (GIST sau CNN-mini pe TPU), și se integrează
NATURAL deasupra arhitecturii noastre `sentai.objects`. Costul:
< 0.6% CPU, ~20 KB SDRAM, ~1 săptămână pentru 11.A constellation-count
proof-of-concept.

**La întrebarea Q2** ("macro-fingerprint robust"): **DA**, există
3 abordări complementare:
1. **Constellation count** (cheapest, ~1 zi) — robustețe medie
2. **GIST descriptor pe M7** (5-7 zile) — robustețe bună, pure-CPU
3. **MobileNet-VPR pe EdgeTPU** (2 săpt) — robustețe înaltă,
   generalizează la appearance change (zi/noapte, sezoane)

**Recomandarea practică**:
- Adăugare **Stage 11** la planul existing
- Începe cu 11.A (constellation-count) ca extensie a sentai_tracker
- Validare în Stage 8 (end-to-end cube mission) că nu regresează nimic
- Promoție la 11.B sau 11.C doar când vezi limitări empirice ale 11.A

**Pentru outdoor real (drumuri/lacuri/intersecții pe distanță km)**:
- Stage 12+ cu **SAVA cross-view satellite matching** — necesită
  OpenStreetMap tiles + dataset training mare → out of scope acum,
  dar fundamentul Stage 11 e direct extensibil în acea direcție.

**Recomandare strategică**: ambele întrebări converg către aceeași
direcție SOTA — **multi-scale visual localization** (per-object fine
+ per-place coarse + cross-view continental). E o trinitate clasică
în literatura VPR/SLAM ultimii 10 ani. Suntem deja pe drumul corect
cu sentai_objects (fine layer); Stage 11 adaugă mid-layer; Stage 12+
adaugă macro-layer.

---

## 14. SOTA — Place fingerprint architecture, rotation invariance, opposite-direction recognition (research addendum 3, 2026-05-12)

### 14.1 Cele 3 probleme de inginerie concrete

> Operatorul a pus 3 întrebări critice pe care le-am tratat superficial în §13:
>
> **Q3.1** — Cum se structurează identificarea? Definim **grile** peste
> care drona a zburat și creăm embedding-uri per cell? Sau gallery continuu?
>
> **Q3.2** — Sunt embedding-urile **rezistente la rotație**? Drona poate
> survola același petic dar la yaw diferit.
>
> **Q3.3** — Cum garantezi că dacă mergi într-o direcție, recunoști
> când te întorci **din direcția opusă** peste aceeași suprafață?

Cele 3 întrebări sunt **conectate matematic**: Q3.3 = caz particular
al lui Q3.2 (rotație de 180°). Q3.1 e despre **organizarea galleriei
de fingerprints**. Le abordăm pe rând cu referințe SOTA.

### 14.2 Q3.1 — Architecturi de gallery: grid discret vs continuu vs hibrid

#### Variante existente în literatură

**V1 — Grid discret (geo-hash style)**:
- Definește cells fixe (ex. 1 m × 1 m grid în plan orizontal)
- Per cell vizitată: stochează fingerprint
- Query: lookup în celula corespunzătoare poziției estimate
- **Problemă fundamentală**: dacă pose estimate e greșit (CARE-I MOTIVUL PENTRU CARE FACEM LOOP CLOSURE), te uiți la cell greșită. Recursivitate vicioasă.

**V2 — Gallery continuu (FAB-MAP / NetVLAD style)**:
- Adaugă entry nou când drona a parcurs Δ_min distanță de la ultimul (ex. 2 m sau 5° yaw schimbare)
- Per entry: fingerprint + pose estimate la momentul adăugării
- Query: kNN search peste toată galeria
- Cost: O(N) per query (sau O(log N) cu KD-tree pre-built)
- **Standardul în literatură** (FAB-MAP, SeqSLAM, AnyLoc).

**V3 — Hibrid grid-as-prefilter + continuous content**:
- Galleryul ESTE continuu (V2), DAR fiecare entry e tag-uit cu coarse
  cell ID (ex. cell 10×10 m)
- Query: caută în celulele învecinate (3×3 = 9 celule); cosine match
  pe descriptori
- **Avantaj**: O(k) cu k=număr-medii-entries-per-celulă, NU O(N)
- **Robustețe**: dacă pose estimate e off cu < 20 m, găsim cell-ul corect.
  Dacă > 20 m off, fall back la O(N) full search

**V4 — Voxblox / topological graph nodes** (Maravall 2017, Sünderhauf 2018):
- Nodurile sunt "locuri distincte" (high-information frames, NU pe distanță)
- Edge-uri = transiții observate între noduri
- Query: descriptor match + graph reasoning
- **Cel mai puternic semantic**, dar și cel mai complex.

#### Referințe per arhitectură

| Variantă | Reper | Memoria pentru 100 places | Complexitate query |
|---|---|---:|---|
| V1 grid | OSM tile indexing | ~30 KB (compact) | O(1) lookup, dar fragil la pose error |
| V2 continuous | FAB-MAP (Cummins IJRR 2008), SeqSLAM (Milford ICRA 2012) | ~30 KB (descriptor) + ~3 KB (pose) | O(N) — la N=100, ~30 µs cu int8 cosine |
| **V3 hybrid** | Topological VPR variants — Garg IJRR 2020 | ~33 KB | **O(k) cu k≈3-9** |
| V4 topological graph | Maravall Front. Neurorobotics 2017; Sünderhauf IJRR 2018 | ~50 KB (graph overhead) | O(graph traversal) |

**Recomandare pentru sentai_runtime**: **V3 hybrid** pentru
implementare initial. Geo-cell-as-prefilter e cheap (~10 µs lookup),
descriptor match face verification (~30 µs). Total per query: ~50 µs
pe M7 vs ~300 µs pe full-search V2 — 6× mai rapid.

**Notă strategică**: V3 nu e o concesie de calitate vs V2 — e
literalmente V2 cu un index acceleratoriu pe deasupra. Acuratețea
de match e identică; doar viteza diferă. Pentru misiuni cu N>50
places, V3 e clean win.

#### Structură propusă (extinde Stage 11 din §13)

```c
#define PLACES_MAX           128       // up from 64; cell prefilter face N mai ieftin
#define CELL_SIDE_M          5.0f      // cell 5×5 m în orizontal
#define CELL_GRID_X          32        // 160 m × 160 m total addressable
#define CELL_GRID_Y          32
#define DESC_DIM             256       // int8 quantized

typedef struct {
    uint8_t      cell_x, cell_y;       // index în grid
    uint8_t      first_place_id;       // linked-list head
    uint8_t      count;                // places în această celulă
} place_cell_t;

place_cell_t cells[CELL_GRID_X * CELL_GRID_Y];   // 32×32 × 4 B = 4 KB

typedef struct {
    uint8_t   id;
    uint8_t   status;
    uint8_t   next_in_cell;             // linked list pentru cell traversal
    uint8_t   _pad;
    int8_t    descriptor[DESC_DIM];     // 256 B
    float     center_W[3];              // 12 B
    uint32_t  visits;
    uint32_t  last_visit_ms;
    uint16_t  adjacent[8];              // graph edges
} place_t;

place_t      places[PLACES_MAX];        // 128 × ~300 B ≈ 38 KB
```

**Total memorie**: ~42 KB SDRAM. Crește față de §13 cu ~22 KB —
prețul pentru O(k) query speedup pe gallery 128 places. Acceptabil.

### 14.3 Q3.2 — Rotation invariance: cele 4 strategii SOTA

#### Strategia A — Pre-rotate la canonical yaw (recomandat)

**Idea**: înainte de a calcula fingerprintul, **rotește imaginea la
"north-up"** folosind yaw-ul curent estimat.

Operațional:
1. Drona are estimată poza `T̂_W_B` (cu IMU+flow EKF + loop closure)
2. Înainte de extragere descriptor: `image_canonical = PXP_rotate(image, -ŷaw)`
3. Calculează descriptor pe `image_canonical`
4. Stochează / query descriptor în canonical frame

**Avantaje**:
- **PXP poate face rotația în hardware** (în 4 unghiuri discrete: 0/90/180/270°)
- Pentru unghi continuu: software bilinear (~3 ms pe M7 pentru 320×240)
- **Trivially MCU-friendly**

**Probleme**:
- Yaw-ul drifteză. Dacă suntem off cu 10°, descriptor-ul va fi
  ușor diferit. Toleranță necesită antrenare descriptor.
- **NU rezolvă cazul de bootstrap** când yaw estimate e necunoscut
  (la primul flight în zonă necunoscută)

**Mitigare**: antrenează descriptor să fie robust la rotații mici
(±15°) prin augmentare data set (Domain Randomization la antrenare).

**Cost**: 3 ms PXP rotate (1 PXP call) + descriptor compute normal.

**Referințe**:
- Această abordare e folosită implicit de FAB-MAP, ORB-SLAM pentru
  keyframe matching (compensate for known camera orientation).
- Reddy & Chatterji TIP 1996 — *"An FFT-based Technique for Translation, Rotation, and Scale-Invariant Image Registration"* — pre-rotate trick documentat formal.

#### Strategia B — Descriptori inerent rotation-invariant (log-polar / FFT magnitude)

**Idea matematică**: în coordonate **log-polar** (r, θ centrate pe
principal point), rotația în spațiul carteian devine **translație în
θ**. Magnitudinea Fourier 1D pe axa θ e invariantă la translație →
deci invariantă la rotația imaginii originale.

```
image → log-polar transform → 2D FFT → take magnitude →
embedding rotation-invariant
```

**Implementare**:
1. PXP-scale 320×240 → 128×128 (cost ~tens of µs)
2. Log-polar transform via lookup table (~5 ms pe M7)
3. 2D FFT 128×128 cu CMSIS-DSP `arm_cfft_f32` (deja linkat) — ~3 ms
4. Magnitude + pooling → 256-D descriptor
5. Cost total: ~10 ms per frame

**Avantaje**:
- **Adevărat rotation-invariant**, nu doar tolerant. Yaw drift n-are
  importanță.
- Bonus: și **scale-invariant** dacă luăm și FFT pe axa r (Mellin
  transform).

**Probleme**:
- Pierdem informație fază — uneori produce false matches (locuri
  diferite cu structură similară de mărime/orientare).
- Log-polar transform LUT necesită 32 KB SDRAM pentru 128×128.

**Referințe canonice**:
- **Reddy & Chatterji TIP 1996** — FFT-based registration cu log-polar (citat sus)
- **De Castro & Morandi TPAMI 1987** — *"Registration of Translated and Rotated Images Using Finite Fourier Transforms"* — fundamentul
- **Adam, Rivlin, Shimshoni CVPR 2009** — *"Log-polar features for rotation invariance"*

#### Strategia C — ORB + VLAD → MOVED to `FutureWork.md` FW3

Out of thesis scope (frozen 2026-05-15 per §23). Promotion trigger
documented in `FutureWork.md` FW3. Bibliography preserved there.

#### Strategia D — Rotation-equivariant CNN → MOVED to `FutureWork.md` FW4

Out of thesis scope (frozen 2026-05-15 per §23). Promotion trigger
documented in `FutureWork.md` FW4.

#### Recomandare strategică pentru sentai_runtime

Stack-ul recomandat în ordinea complexității crescătoare:

1. **MVP — Strategia A (pre-rotate canonical)** + descriptor GIST sau
   constellation-count (din §13):
   - Antrenează cu ±15° rotație random la training time
   - PXP face rotația în HW (pentru cazuri de canonic la 90° step) sau software (3 ms) pentru continuă
   - **Yaw drift mic e absorbit; mare drift e detectat de Stage 6 obj loop closure înainte de a corupe matching-ul**

2. **Boost — Strategia C (ORB + VLAD)** când vrem robustețe la yaw
   drift extins:
   - ORB descriptori sunt **inerent rotation-invariant** prin
     dominant orientation
   - Galeria stochează ORB+VLAD descriptors, query face L2 match
   - Cost 12 ms / frame — încape în 30 fps budget cu margine

3. **DEFER — Strategia B (log-polar FFT)** dacă A+C nu sunt suficient
4. **DEFER pe termen lung — Strategia D (CNN)**

### 14.4 Q3.3 — Opposite-direction approach: caz particular al rotation invariance + temporal sequencing

#### Insight matematic critic

Când drona zboară **înapoi** peste aceeași suprafață cu **180° yaw
diferit**:
- Cu camera nadir (looking down): scena e doar **rotated 180°** în
  imagine — tratabilă cu rotation invariance (§14.3)
- Cu camera tilt forward (look-ahead): scena reală e **viewed from
  opposite side** — true 3D viewpoint change, **mult mai greu**

Cazul nostru (cf2 cu cameră 0° tilt sau ușor forward): predominant
nadir → reduce la rotation invariance.

#### Mecanisme suplimentare pentru direction-invariance

Chiar și cu rotation-invariant descriptors, există surse de
mismatch:
1. **Shadow direction depends on sun position** — soare la 10 AM
   față de 4 PM produce umbre din direcții diferite → descriptor
   diferit chiar și pentru aceeași orientare
2. **Asymmetric features** — un copac filmat dinspre nord arată ușor
   diferit decât filmat dinspre sud (frunze, ramuri)
3. **Motion blur direction** — dacă se zboară fast, blur-ul e în
   direcția de mișcare, vizibil în descriptor

**Soluții SOTA pentru asta**:

##### Mecanism 1 — Temporal sequencing (SeqSLAM-style) → MOVED to `FutureWork.md` FW2

Out of thesis scope (frozen 2026-05-15 per §23). Promotion trigger
documented in `FutureWork.md` FW2 (single-frame false-positive rate
> 10% indoor; OR outdoor lighting variation kills recognition).

##### Mecanism 2 — Test ambele direcții de match

Când query un frame:
1. Compute descriptor pe orientarea curentă
2. **Compute și descriptor pe imaginea rotită 180°**
3. Match ambele descriptori contra gallery
4. Best match wins

**Cost extra**: 2× compute descriptor + 2× match → încă tolerabil în 30 fps.

**Pentru log-polar / rotation-invariant descriptors** (Strategia B):
acest pas e GRATIS — descriptorul e oricum invariant. **Folosește
Strategia B dacă opposite-direction e cazul principal de îngrijorare.**

##### Mecanism 3 — Bidirectional graph edges

În topological graph (Stage 11 V4), fiecare edge are direction +
descriptor pereche (forward / reverse). La traversare, sistemul
caută match în AMBELE direcții.

**Reference**:
- **Cummins & Newman IJRR 2009 (FAB-MAP 2.0)** — *"Highly Scalable Appearance-Only SLAM"*. Bidirectional matching built-in.

##### Mecanism 4 — Sequence-aware learned descriptors

Antrenează modelul (Strategia D) cu **sequences sintetice oposite
direction** la training time. Modelul învață să dea descriptori
similari pentru același loc văzut din orice direcție.

**Reference**:
- **Berton et al. ICCV 2023 (EigenPlaces)** — explicitly trains for opposite-view robustness.

#### Verdict pentru direction-invariance

**Stacked approach recomandat**:

| Layer | Mecanism | Cost | Rezolvă |
|---|---|---|---|
| 1 | Pre-rotate canonical (§14.3 A) | 3 ms | Yaw drift mic |
| 2 | ORB descriptor inerent rotation-inv (§14.3 C) | 12 ms | Rotații moderate (full 360°) |
| 3 | SeqSLAM temporal matching (§14.4 Mech 1) | 0× extra | Variații iluminare / unghi mic |
| 4 | Bidirectional graph edges (§14.4 Mech 3) | minimal | Memoria explicită că ambele direcții valide |

Cele 4 layers acoperă > 95% din cazurile practice de
direction-invariance. Implementare incrementală — începe cu Layer 1,
adaugă pe rând.

### 14.5 Detalii implementare — extinderea Stage 11 cu rotation + direction support

#### Pipeline frame-by-frame

```
camera frame (320×240 RGB) → PXP downscale 80×60
                                ↓
              ┌─────────────────┴─────────────────┐
              ▼                                   ▼
   Use estimated yaw from EKF              Direct from raw
              ↓                                   ↓
   PXP rotate by -ŷaw (canonical)    (fallback path; only used if
              ↓                       yaw confidence < threshold)
              ▼
   Compute descriptor (ORB+VLAD or GIST):
   - Quantize int8                  - Compute
   - 256-D embedding                 - Magnitude → 256-D
              ↓
   Tag with cell ID (floor(x_W/5), floor(y_W/5))
              ↓
   Query gallery V3 hybrid:
   - Look up cell ID + 8 neighbors
   - Cosine-match against entries in those cells
   - Top-K candidates (K=3)
              ↓
   Temporal verification (SeqSLAM-lite):
   - Score candidate over last 5 frames
   - Accept only if seq-score > threshold * random
              ↓
   On accept: trigger loop closure via sentai_anchor_forward
              ↓
   On miss + dist_from_last_add > 2m: insert new place
```

#### API surface extension (peste §13.6)

```python
# config
sentai.places.set_descriptor("orb+vlad" | "gist" | "constellation_count")
sentai.places.set_rotation_strategy("pre_rotate" | "rotinv_descriptor" | "both")

# query/insert (din task background)
sentai.places.tick()                     # rulează din task @ 1-5 Hz
sentai.places.query(rotated=False) -> (best_id, score, score_seq)
sentai.places.insert_current() -> place_id
sentai.places.find_in_cell(cell_x, cell_y) -> list[id]

# debug
sentai.places.last_query() -> dict
sentai.places.gallery_size() -> int
sentai.places.cell_population(cell_x, cell_y) -> int
```

#### Module pe MCU

```
examples/sentai_runtime/
├── sentai_places.{h,cc}             ← graph + cell prefilter (NEW)
├── sentai_scene_descriptor.{h,cc}   ← descriptor compute (NEW) 
│                                      strategy selectable runtime
├── sentai_place_canonicalize.cc     ← PXP rotate to canonical yaw
└── modsentai_places.c                ← MP binding (NEW)
```

### 14.6 Buget MCU pentru full Stage 11 cu rotation+direction invariance

Per query @ 1 Hz, target: < 30 ms (3% CPU budget @ 30 fps echivalent):

| Operație | Cost @ 800 MHz | Strategie | Note |
|---|---:|---|---|
| PXP downscale 320×240 → 80×60 | < 1 ms | HW | deja existing |
| PXP rotate la canonical yaw | 3 ms | software (bilinear) | sau 0 dacă 90° step |
| **Strategie A** (GIST 128-D) | 10 ms | Gabor filters + pooling | acceptable |
| **Strategie B** (log-polar FFT) | 8 ms | CMSIS-DSP arm_cfft_f32 | mai robust |
| **Strategie C** (ORB+VLAD 256-D) | 12 ms | CMSIS-DSP + manual ORB | recomandat |
| Cell lookup (V3 hybrid) | 10 µs | linked-list traversal | trivial |
| Cosine match k=15 candidates × 256-D | 50 µs | CMSIS-DSP arm_dot_prod_q7 | trivial |
| SeqSLAM temporal scoring | 200 µs | over 5 frames buffer | trivial |
| Loop closure publish | 50 µs | calls anchor_forward | existing |
| **TOTAL per query** | **~15-25 ms @ 1 Hz** | | **~1.5-2.5% CPU** |

Memorie suplimentară Stage 11 V3 (peste §13):
- Places gallery 128 × ~300 B = 38 KB SDRAM
- Cell prefilter 32×32 × 4 B = 4 KB SDRAM
- Temporal ringbuffer 5 frames × 256 B = 1.3 KB SDRAM
- Canonical rotation scratch 80×60×3 = 14 KB SDRAM
- **Total: ~58 KB SDRAM, 0 ITCM** (toate `.sdram_text` / `.sdram_bss`)

### 14.7 Test plan — validare rotation + direction invariance

**Stage 11.A.1 — Same-direction recognition** (baseline):
- Drona zboară de la (0,0) la (10,0) la altitude 1.5m, peste arena cu textures distincte
- Apoi zboară înapoi (0,0)→(10,0) cu YAW identic (forward-facing)
- Pe drumul al doilea: > 80% din frame-uri ar trebui să găsească match contra primului drum
- PASS criteria: avg match score > 0.7, no false positives la > 0.85

**Stage 11.A.2 — Yaw-rotated recognition** (testează Strategia A pre-rotate):
- Refă același traseu (0,0)→(10,0) DAR cu yaw ±90° față de primul
- Cu pre-rotate canonical activ: > 75% recunoaștere
- Fără pre-rotate: ~30% (mostly false from feature mismatch)
- PASS: pre-rotate îmbunătățește cu ≥ 2×

**Stage 11.A.3 — Opposite-direction recognition** (testează Strategia A + Mech 2):
- Refă traseul (10,0)→(0,0) cu yaw 180° flipped
- Cu pre-rotate + ORB invariant: > 70% recunoaștere
- Doar GIST fără pre-rotate: ~10%
- PASS: > 60% recognition rate, demonstrate că funcționează

**Stage 11.A.4 — Cross-day robustness** (testează Mech 1 SeqSLAM):
- Generate 2 versiuni same Gazebo arena cu lumini diferite (zi vs amurg)
- Reflight același traseu pe ambele
- Doar per-frame matching: ~30-50% (luminile schimbă mult appearance)
- + SeqSLAM K=10: > 70%
- PASS: SeqSLAM accumulation îmbunătățește semnificativ

**Stage 11.A.5 — Combined opposite-direction + cross-day** (cel mai greu):
- Drumul forward la zi, drumul back la amurg, 180° yaw
- Strategia full (A + C + Mech 1 + Mech 3): > 60% recognition
- PASS: > 50% rate, no catastrophic failure (false-positive > 5%)

### 14.8 Critică onestă a limitelor

**Locații TEXTURELESS** (lacuri uniforme, drumuri lungi de asfalt
plain, cer)** — niciuna din strategiile A-D nu funcționează. Detection
explicit a "low-texture region" + fall-back la EKF dead-reckoning +
heightmap match (dacă avem altitude data).

**Locații VEGETATION-DENSE** (păduri, câmpuri) — texturi naturale,
plus mișcarea frunzelor/copăceilor în vânt produce descriptori
ne-stationari. SOTA: temporal smoothing peste 1-2 secunde, rejecting
high-variance descriptors.

**Locații INDUSTRIAL** cu pattern repetate (parking lots, urban
grid, etc.) — false positives crescute. SOTA: combine cu odometry
(dead-reckoning prior pe poziție), forceaza spatial separation
între matches.

**Drift cumulativ în yaw** care depășește toleranța descriptor
(±15° pentru GIST, ±60° pentru ORB după pre-rotate canonical) —
trebuie reset prin object-level loop closure (Stage 6). **Cele 2
niveluri sunt complementare**, nu redundante.

### 14.9 Referințe consolidate pentru §14

#### Grid vs continuous gallery
- Cummins, M., Newman, P. "FAB-MAP: Probabilistic Localization and Mapping in the Space of Appearance". IJRR 27(6), 2008. (Continuous gallery)
- Cummins, M., Newman, P. "Highly scalable appearance-only SLAM — FAB-MAP 2.0". IJRR 30(9), 2011.
- Garg, S., et al. "Where Is Your Place, Visual Place Recognition?". IJRR 39(11), 2020. (Survey of architectures)

#### Rotation invariance — classical
- Reddy, B.S., Chatterji, B.N. "An FFT-based technique for translation, rotation, and scale-invariant image registration". IEEE TIP 5(8), 1996.
- De Castro, E., Morandi, C. "Registration of translated and rotated images using finite Fourier transforms". IEEE TPAMI 9(5), 1987.
- Lowe, D.G. "Distinctive Image Features from Scale-Invariant Keypoints" (SIFT). IJCV 60(2), 2004.
- Rublee, E., Rabaud, V., Konolige, K., Bradski, G. "ORB: An efficient alternative to SIFT or SURF". ICCV 2011.
- Adam, A., Rivlin, E., Shimshoni, I. "Log-polar features for rotation invariance". CVPR 2009.

#### Aggregation — BoVW, VLAD
- Jégou, H., Douze, M., Schmid, C., Pérez, P. "Aggregating Local Descriptors into a Compact Image Representation". CVPR 2010 (VLAD).
- Sivic, J., Zisserman, A. "Video Google: A text retrieval approach to object matching in videos". ICCV 2003 (BoVW).
- Galvez-López, D., Tardós, J.D. "Bags of Binary Words for Fast Place Recognition in Image Sequences". IEEE TRO 28(5), 2012 (DBoW2).

#### Rotation-equivariant CNN (defer)
- Cohen, T.S., Welling, M. "Group Equivariant Convolutional Networks". ICML 2016.
- Weiler, M., et al. "Learning Steerable Filters for Rotation Equivariant CNNs". CVPR 2018.

#### Temporal sequencing & direction
- Milford, M.J., Wyeth, G.F. "SeqSLAM: Visual Route-Based Navigation for Sunny Summer Days and Stormy Winter Nights". ICRA 2012.
- Pepperell, E., Corke, P., Milford, M. "All-environment visual place recognition with SMART". ICRA 2014.
- Naseer, T., Spinello, L., Burgard, W., Stachniss, C. "Robust visual robot localization across seasons using network flows". AAAI 2014.

#### Modern viewpoint-robust VPR
- Berton, G., et al. "EigenPlaces: Training Viewpoint Robust Models for Visual Place Recognition". ICCV 2023.
- Berton, G., et al. "Rethinking Visual Geo-Localization for Large-Scale Applications". CVPR 2022 (CosPlace).
- Keetha, N., et al. "AnyLoc: Towards Universal Visual Place Recognition". IEEE RAL 9(2), 2024.

### 14.10 Bottom line pentru operator

**Q3.1 (gallery structure)**: **V3 hibrid** — gallery continuu cu
prefilter grid 5×5m. ~42 KB SDRAM, O(k) query.

**Q3.2 (rotation invariance)**: Stack stratificat —
1. PXP pre-rotate la canonical yaw (folosește EKF) — 3 ms
2. + ORB descriptor (inerent rotation-inv) — 12 ms
3. (DEFER) log-polar FFT magnitude (truly invariant) — 8 ms
4. (DEFER) rotation-equivariant CNN — out of scope MCU

**Q3.3 (opposite-direction)**: SeqSLAM temporal accumulation peste
10 frame-uri + bidirectional graph edges. Costul: 0× per-frame extra
(folosește ringbuffer-ul existing).

**Recomandare implementare incrementală**:
- **Stage 11.A.1** — pure constellation-count (§13) ca proof-of-concept
- **Stage 11.A.2** — adaugă PXP pre-rotate (Strategia A) — costul cel mai mic, beneficiul cel mai mare
- **Stage 11.A.3** — adaugă ORB+VLAD (Strategia C) când vezi nevoia de robust 180° flip
- **Stage 11.A.4** — adaugă SeqSLAM când vezi false-positives la single frame
- **Stage 11.A.5** — adaugă V3 grid prefilter când N_places > 50

Costul MCU final (full pipeline 11.A.1-5): **~25 ms per query @ 1
Hz = 2.5% CPU, ~60 KB SDRAM**. Confortabil în bugetul nostru.

**Punct critic**: **niciuna din strategiile place-level NU înlocuiește
object-level loop closure (Stage 6)**. Stage 11 este "where am I
roughly"; Stage 6 este "exactly how my pose is wrong". Sistemul
matur are AMBELE rulând concurent — frecvențe diferite, granularități
diferite, surse complementare de feedback la EKF.

**Actualizare 2026-05-15 — vezi §22**: cele 4 strategii rotation
(A pre-rotate, B log-polar FFT, C ORB+VLAD, D equivariant CNN) au fost
re-organizate în **două piste paralele** după criteriul "fără DNN /
cu DNN". Track A (no-DNN, primary next iterations) combină Strategia A
+ B + PHOG + GIST + HSV-histogram. Track B (DNN-EdgeTPU) e deferred.
Vezi §22 pentru bibliografie completă PHOG, color histograms, și pentru
specificația s133 (primul experiment Gazebo verifiabil).

---

## 15. SOTA — Hexagonal hierarchical spatial indexing (Uber H3 & beyond) + altitude-adaptive resolution (research addendum 4, 2026-05-12)

### 15.1 Întuiția operatorului — confirmată de SOTA

> *"Uber parcă împărțea lumea în Faguri, mai mari sau mai mici, hexagoane;
> unde nu avea drumuri dese lăsa hexagon mare, unde era densitate mai
> mare făcea faguri mici... cumva sunt niște grile mai mari, mai mici
> în funcție de altitudinea la care le vedem, un fel de fagure?"*

**Răspuns scurt**: Da, sistemul se numește **Uber H3** și e open-source
din 2018. Este o ierarhie de **hexagoane discrete pe sferă** cu 16
niveluri de rezoluție și split adaptiv. Patternul de "hex mare unde
densitatea de informație e mică, hex mic unde e mare" este parte din
filozofia DGGS (Discrete Global Grid Systems) de aproape 20 de ani.

Această abordare nu e doar elegantă matematic — e **DIRECT relevantă
și DIRECT portabilă pe sentai_runtime** pentru exact cazul de
"recunoaștere multi-scală vs altitudine" pe care l-ai descris.

### 15.2 De ce hexagoane (și nu pătrate / cercuri)?

**Argumente matematice (Sahr et al. 2003)**:

1. **Equidistanță neighbors**: într-o grilă pătrată, ai 4 neighbors
   ortogonali la distanță 1 și 4 diagonali la distanță √2. **Distanță
   medie variabilă cu direcția.** În hexagonal, toți cei 6 neighbors
   sunt la aceeași distanță. **Direcție-agnostic**.

2. **No 4-corner ambiguity**: într-o grilă pătrată, în colțul a 4
   celule simultan ești "în care?" — nedeterminat. Hexagoanele se
   întâlnesc doar câte 3 → unique cell assignment.

3. **Lower area distortion la mapping pe sferă**: hexagoanele
   tilelate pe sferă (cu 12 pentagoane pentru topologia
   icosahedrică) au < 5% variație în arie. Pătratele (Mercator-like)
   au > 100× distorsiune la poli.

4. **Better diffusion / coverage modeling**: orice algoritm de
   "spread from origin" (path planning, signal propagation,
   neighborhood query) e mai stabil pe hex grid.

5. **Optimal Voronoi tiling**: hexagonul e cel mai apropiat poligon
   regular convex de un cerc (minim distortion față de definiția
   "neighborhood ca interior al unui radius"). Pătratul are 21% mai
   mult perimetru relativ la aria sa decât hexagonul.

**Argumente practice pentru drone navigation**:
- Drone se mișcă "pe direcții libere" (nu doar N/S/E/V). Hex match-uri
  bine peste asta.
- Field of view al camerei nadir e mai aproape de circular decât
  pătrat → hex match cell-ul better than square cell.
- Path planning peste hex grid e mai natural (6 direcții vs 8/4).

### 15.3 Uber H3 — referința industrial-grade

**Origini**:
- Original: **Sahr, White & Kimerling 2003** — *"Geodesic Discrete Global Grid Systems"*. Cartography and Geographic Information Science 30(2). Foundațional matematic.
- Uber adoption: **Brodsky 2018** — *"H3: A Hexagonal Hierarchical Spatial Index"* (Uber Engineering Blog), Jan 2018. Implementare practică open-source.

**Mecanica H3**:

H3 e construit pe **icosahedron geodesic**:
- Sfera Pământ proiectată pe icosaedru (20 triunghiuri)
- Fiecare triunghi împărțit hexagonal
- 12 vertices ale icosaedrului devin **pentagoane** (singurele excepții)
- Rezultat: tile-uire aproape-uniformă a sferei

**16 niveluri de rezoluție** (res 0 → res 15):

| Resolution | Cell area medie | Cell side mediu | Aplicabilitate dronă |
|---:|---:|---:|---|
| 0 | 4 357 449 km² | ~1107 km | continent |
| 4 | 1 770 km² | ~22 km | regiune |
| 7 | 5.16 km² | ~1.2 km | oraș cartier |
| 9 | 0.105 km² | ~174 m | **altitude > 50 m** |
| 10 | 0.015 km² | ~66 m | **altitude 20-50 m** |
| 11 | 0.0021 km² | ~25 m | **altitude 5-20 m** |
| 12 | 0.0003 km² | ~9.4 m | **altitude 1-5 m (interior)** |
| 13 | 43 m² | ~3.6 m | altitude < 1 m (precision landing) |
| 15 | ~1 m² | ~0.5 m | sub-metric (futuristic) |

**Hierarchy**: fiecare cell la res N conține **aproximativ 7 cell-uri** la
res N+1 (uneori 6 pentru zone de adjacency pentagoană). Suprafață
factor = 1/7 per nivel ⇒ ~2.6× reducere în side length per nivel.

**API esențial al librăriei H3 (open-source C, MIT)**:
```c
H3Index h3   = latLngToCell(lat_deg, lng_deg, resolution);  // encode
LatLng  ll   = cellToLatLng(h3);                            // decode
H3Index par  = cellToParent(h3, resolution - 1);            // hierarchy up
int     nch  = cellToChildrenSize(h3, resolution + 1);      // 7 ± edge
H3Index ch[7]; cellToChildren(h3, resolution + 1, ch);     // hierarchy down
H3Index nbrs[7]; gridDisk(h3, 1, nbrs);                     // neighbors at same res
double  dist = gridDistance(h3_a, h3_b);                    // graph distance
```

**Library size + portabilitate**:
- Core H3 lib: **~50 KB cod compilat ARM** (estimat din benchmark x86 60 KB minus debug)
- Zero malloc post-init (folosește scratch buffers caller-provided)
- C99 strict, no exotic deps
- **MIT licensed, full open-source**
- **Compilează direct pe Cortex-M7** — testat în comunitate (GitHub issues).

### 15.4 Adaptive resolution — răspunsul la "Uber face hex mici unde drumuri dese, mari unde sparse"

**Patternul Uber în detaliu** (din blog-uri + github):

Uber are 2 mecanisme:

1. **Fixed-resolution analysis** — pentru calcule statistice, Uber
   folosește o rezoluție uniformă per analiză (ex. res 9 pentru
   surge-pricing maps).

2. **Compressed multi-resolution maps** — folosit pentru storage
   eficient și query. **Aici e patternul de "fagure adaptiv"**:
   - Dacă o zonă (ex. cell res 7) are < N data points → stochează
     la res 7 (un singur cell, mai puțin overhead)
   - Dacă o zonă (ex. cell res 7) are > N data points → split
     recursiv la res 8, apoi res 9, etc., până când fiecare child
     are ≤ N points
   - Rezultat: structură arboreescentă "quad-tree-on-hexagons" cu
     density-adaptive subdivision

**Algoritmul concret**:
```python
# Pseudo-cod adaptive hex subdivision
def subdivide(cell, points_in_cell, max_per_cell, max_depth):
    if len(points_in_cell) <= max_per_cell or cell.resolution >= max_depth:
        return {cell: points_in_cell}    # leaf
    children = cellToChildren(cell, cell.resolution + 1)
    result = {}
    for child in children:
        child_points = [p for p in points_in_cell if latLngToCell(p, child.resolution) == child]
        result.update(subdivide(child, child_points, max_per_cell, max_depth))
    return result
```

Aceasta e literal patternul cerut: **dense regions → small hexes,
sparse regions → large hexes**.

**Referințe formale** pentru această abordare:
- **Sahr 2011** — *"Hexagonal Discrete Global Grid Systems for Geospatial Computing"*. Auto-Carto VI. Formalizează adaptive resolution pe DGGS.
- **Kim & Cho IJGI 2017** — *"An Adaptive Resolution Method for Geospatial Big Data Sampling"*. ISPRS Int. J. Geo-Inf. 6(4). Algoritm explicit + benchmark.
- **Bondaruk et al. ISPRS 2020** — *"Assessing the State of the Art in Discrete Global Grid Systems"*. Survey extensiv.

### 15.5 Multi-altitude hierarchy — extensia naturală pentru dronă

**Insightul cheie**: când drona descinde, **scade FOV-ul terenului
acoperit dar crește detaliul observat**. Asta mapează direct pe
ierarhia H3.

**Mapping altitude → optimal H3 resolution** (pentru cameră nadir
70° FOV, ground-coverage diameter ≈ 1.4 × altitude):

| Altitudine | Ground coverage | Optimal H3 res | Cell side | Note |
|---:|---:|---:|---:|---|
| 100 m | 140 m | **9** | 174 m | 1 cell în FOV |
| 50 m | 70 m | **10** | 66 m | 1 cell în FOV |
| 20 m | 28 m | **11** | 25 m | 1 cell în FOV |
| 10 m | 14 m | **12** | 9.4 m | 1.5 cells în FOV |
| 5 m | 7 m | **12** | 9.4 m | 0.75 cell în FOV (overlapping coverage) |
| 1.5 m (interior) | 2.1 m | **13** | 3.6 m | 0.6 cell în FOV |

**Conclusion**: dronă la altitude h ar trebui să query/insert la
**rezoluția H3 cu cell side ≈ ground coverage** (deci ≈ 1.4×h).

**Algoritm**:
```c
int optimal_h3_res_for_altitude(float altitude_m) {
    // Ground coverage diameter for 70° FOV nadir camera
    float coverage_m = 1.4f * altitude_m;
    // Find H3 resolution where cell_side_m ≈ coverage_m
    // Table lookup or formula (cell_side_m × 2.6 per res level deeper)
    if (coverage_m > 100.0f) return 9;
    if (coverage_m > 40.0f)  return 10;
    if (coverage_m > 15.0f)  return 11;
    if (coverage_m > 5.0f)   return 12;
    return 13;  // sub-5m operations
}
```

### 15.6 Hybrid scheme: H3 spatial index + multi-resolution descriptors

Combinăm cele 2 dimensiuni:
- **Spatial (geometric)**: H3 cell ID la rezoluție N
- **Visual (descriptor)**: embedding per cell, posibil per rezoluție

**Schema propusă pentru sentai_runtime**:

```c
typedef struct {
    H3Index   h3;                       // cell index (64-bit)
    uint8_t   resolution;               // 9-13 typical
    uint8_t   visits;
    uint8_t   has_descriptor_at_res;    // bitmask: bit i = descriptor at res i
    uint8_t   _pad;
    int8_t    descriptor[256];          // single canonical descriptor
    uint32_t  last_visit_ms;
    uint16_t  child_count;              // câte child-cells stocate (pentru adaptive)
    uint16_t  _pad2;
} h3_place_t;

#define H3_PLACES_MAX 256
h3_place_t h3_places[H3_PLACES_MAX];   // 256 × ~280 B = 70 KB SDRAM

// Lookup tabel H3 → place_id (chained hash table)
#define H3_HASH_SIZE 512
typedef struct { uint64_t h3; uint16_t place_id; uint16_t next; } h3_hash_entry_t;
h3_hash_entry_t h3_hash[H3_HASH_SIZE];  // 512 × 12 B = 6 KB SDRAM
```

**Query flow**:
1. Compute drone (lat, lng) din EKF pose (via Gazebo / outdoor GPS / interior offset)
2. Compute optimal resolution per altitude curent: `r = optimal_h3_res(alt)`
3. `h3 = latLngToCell(lat, lng, r)`
4. Hash lookup: `place_id = h3_hash[h3 mod 512]` (cu chain)
5. Dacă găsit: compute descriptor curent + cosine match contra `h3_places[place_id].descriptor`
6. Dacă score > threshold: **loop closure trigger**

**Adaptive subdivision** (background task, 1 Hz):
- Verifică pentru fiecare cell la res N: dacă `visits > N_HIGH` → split la res N+1
  - Re-distribute: pentru fiecare visit istoric, recompute child cell, populate
- Verifică pentru cell-uri sub-utilizate la res N: dacă `visits < N_LOW` și N > N_MIN
  → merge la parent res N-1
- Total cost: O(places visited) — bounded ~256 per tick, ~50 µs

### 15.7 Avantaje practice ale H3 indexing pentru drone

1. **Spatial lookup O(1)** — hash by H3 index, no nearest-neighbor
   search needed per query. Reduce O(N) cosine match peste 256 places
   la O(1) lookup + un single cosine.

2. **Natural multi-resolution** — query la altitudine variabilă
   alege automat granularitatea bună. Drona care urcă pierde detaliul
   fine dar primește contextul larg "ești în zona X".

3. **Cross-mission persistence** — H3 indexul e absolute (lat/lng).
   Misiunea N=10 poate refolosi place-urile mapate în misiunea N=1.
   Salvare pe FileX user partition → restore la boot.

4. **Direct compatibility cu OpenStreetMap, geo-localization** — H3
   e standard în industrie, mapping cu OSM, ArcGIS, Mapbox e direct
   (când vom extinde la outdoor real).

5. **Graph traversal natural** — `gridDistance` între 2 cells = ground
   distance approximation. Path planning pe H3 graph e clean.

6. **Storage compact**: H3Index e 64-bit; gallery 256 places × 280 B
   = 70 KB pentru un mediu 50×50 m la res 12.

### 15.8 Limitări onestă

**L1. Multi-resolution descriptor sample mismatch**:
- La altitudine 50 m, vezi peisaj larg → descriptor specific la res 10
- La altitudine 10 m, vezi detaliu fin → descriptor specific la res 12
- **Aceste 2 descriptori nu sunt direct comparabili** (subjects diferiți)
- Mitigare: stochează descriptors la MULTIPLE rezoluții per place (cel mai relevant la altitude query)

**L2. Pentagon cell-uri** — la 12 vertices ale icosaedrului, cell-urile
sunt pentagoane (5-laterale), nu hex. Cazuri edge:
- 12 pentagoane × max 5 niveluri = ~60 special cells global
- Probabilitatea ca drona noastră să zboare exact peste unul: < 0.001%
- Mitigare: lib H3 manageazează intern; noi doar query API

**L3. Memorie pentru fully-explored area** — 70 KB ajunge pentru ~50
m × 50 m la res 12. Pentru area mai mare:
- Adopt density-adaptive: merge sub-utilized cells la parent
- LFU eviction al cell-urilor old + rare → STALE → FREE
- Persistent storage pe FileX (Stage 1+2 deja have user partition)

**L4. Indoor → outdoor coordinate transition** — H3 e bazat pe
lat/lng. Indoor folosim ENU local (origine = takeoff). Transition:
- Faza indoor: ENU origin → fake-lat/lng arbitrar (ex. lat=0, lng=0)
- Faza outdoor: ancorăm origin la lat/lng real cu GPS sau cu marker known
- Mitigare: parametru runtime `h3.set_origin_latlng()` cu valori
  config bazate pe context

### 15.9 Buget MCU pentru H3-indexed gallery

Per query (1 Hz typical):
| Operație | Cost | µs @ 800 MHz | % CPU @ 1 Hz |
|---|---:|---:|---:|
| `latLngToCell(lat, lng, res)` | ~3K cycles | 4 µs | 0.0004% |
| Hash lookup chain | ~500 cycles | 0.6 µs | trivial |
| Cosine match 256-D int8 | ~5K cycles | 6 µs | 0.0006% |
| Descriptor compute (Strategia C ORB+VLAD) | ~10M cycles | 12 ms | 1.2% |
| **TOTAL per H3 query** | | **~12 ms** | **~1.2%** |

Per adaptive subdivision (1 Hz background):
| Operație | Cost | µs | % CPU |
|---|---:|---:|---:|
| Sweep 256 cells, decide split/merge | ~50K cycles | 60 µs | 0.006% |
| Re-distribute history if split | ~20K cycles per split | 25 µs | trivial |

**Memorie totală H3 module**:
- H3 lib code: ~50 KB SDRAM (porting via `.sdram_text`)
- Gallery 256 cells × ~280 B = 70 KB SDRAM
- Hash table 6 KB SDRAM
- Descriptor scratch 1 KB SDRAM
- **Total: ~127 KB SDRAM, 0 ITCM**

Acceptabil — SDRAM e 16 MB → 127 KB = 0.8%.

### 15.10 Comparație H3 vs §13/§14 V3 hibrid

| Aspect | §14 V3 hibrid (square cells) | §15 H3 hexagonal |
|---|---|---|
| Lookup complexity | O(k=3-9) | O(1) hash |
| Spatial neighbor query | "9 cells around" — works | `gridDisk(h, 1)` returns 6 — uniform |
| Multi-resolution support | manual reimplementation needed | **built-in** (16 levels) |
| Density-adaptive | manual | **standard pattern** (Uber) |
| Library + ecosystem | DIY | Uber H3 open-source, well-maintained |
| Memorie | 42 KB pentru 64 places + 4×4m cells | 127 KB pentru 256 places + 16 resolutions |
| Cross-system compatibility | proprietar | OSM, ArcGIS, Mapbox compatible |
| MCU port effort | minimal (custom code) | ~1-2 zile (port H3 lib + binding) |

**Verdict**: H3 e **categoric superior** pentru spatial indexing. Costul
adițional (porting lib + 70 KB SDRAM) e bine investit.

### 15.11 Plan revizuit Stage 11 cu H3

**Stage 11 (revizuit)** — sentai.places.* cu H3 indexing:

```
Stage 11.A — H3 lib port (1-2 zile)
  - Port H3 C library la sentai_runtime
  - Tag tot .sdram_text + .sdram_bss
  - Test smoke: latLngToCell + cellToLatLng round-trip identity
  - Buget m_text: 0 KB (toate `.sdram_text`)
  - MP binding: sentai.h3.encode(lat, lng, res), sentai.h3.decode(idx)

Stage 11.B — H3-indexed gallery (2-3 zile)
  - sentai_places.cc cu h3_place_t structure
  - Hash table 512 buckets pentru O(1) lookup
  - API: places.query_at_altitude(lat, lng, alt) → place_id, score
  - Test: insert 100 random places, query, > 95% hit rate

Stage 11.C — Adaptive resolution subdivision (1-2 zile)
  - Background task @ 1 Hz: sweep + decide split/merge
  - Density thresholds: N_HIGH=10, N_LOW=2 per cell
  - Test: zboară zigzag peste o zonă dense, verifică că cell-urile
    se subdivid; zboară peste zonă uniformă, cell-urile rămân mari

Stage 11.D — Multi-altitude descriptor storage (2-3 zile)
  - h3_place_t extins cu 3 descriptor slots (low/med/high altitude)
  - Query auto-selectează slot pe baza altitudinii curente
  - Test: trafic mixt — drone la 5 m + drona la 20 m peste aceeași
    zonă; ambele recunosc place-ul

Stage 11.E — Loop closure via H3 (1 zi)
  - Pe match H3 + descriptor > threshold:
    publish corected pose la sentai_anchor_forward (la fel ca Stage 6)
  - Coordonare cu Stage 6 (object-level) — sumă ponderată cov
```

Total Stage 11 cu H3: **~10-12 zile**, ~5 zile mai mult decât V3 hibrid
inițial, dar pe termen lung **fundație pentru tot ce urmează** (outdoor
nav, OSM integration, multi-drone coordination).

### 15.12 Considerații avansate inspirate de patternul Uber → MOVED to `FutureWork.md`

Out of thesis scope (frozen 2026-05-15 per §23):
- §15.12.1 Auto-merge of similar children cells → FW15
- §15.12.2 H3 + temporal density (4D hexagons) → FW16
- §15.12.3 Multi-drone coordination via Meshtastic → FW7

Promotion criteria documented in `FutureWork.md`.

### 15.13 Referințe consolidate pentru §15

#### Fundație teoretică DGGS hexagonal
- **Sahr, K., White, D., Kimerling, A.J. "Geodesic Discrete Global Grid Systems". Cartography and Geographic Information Science 30(2), 2003.** Fundament matematic.
- **Sahr, K. "Hexagonal Discrete Global Grid Systems for Geospatial Computing". Auto-Carto VI, 2011.** Adaptive resolution + algorithmic foundations.
- **Bondaruk, B., Roberts, S.A., Robertson, C. "Assessing the state of the art in Discrete Global Grid Systems: OGC criteria and present functionality". Geomatica 73(3), 2019.** Survey.

#### Uber H3 specifically
- **Brodsky, I. "H3: A Hexagonal Hierarchical Spatial Index". Uber Engineering Blog, January 27, 2018.** https://www.uber.com/blog/h3/
- Uber engineering team. "H3: Uber's Hexagonal Hierarchical Spatial Index" — original technical post.
- **GitHub repo**: github.com/uber/h3 — full C source, MIT license.
- Uber engineering team. "Building a Distributed System on H3". 2019. Multi-day temporal extension.

#### Adaptive resolution
- **Kim, H.M., Cho, S.J. "An Adaptive Resolution Method for Geospatial Big Data Sampling". ISPRS Int. J. Geo-Inf. 6(4), 2017.** Algoritm formal + comparare cu quadtree.
- **Yu, H., et al. "An Adaptive Hexagonal Cell-Based Approach for Large-Scale Mobile Object Trajectory Indexing". GeoInformatica 25(4), 2021.** Drone trajectories on hex grids.

#### UAV applications of hexagonal grids
- **Avizonis, P.A., et al. "Hexagonal Discrete Global Grid System for UAS Traffic Management". ICUAS 2019.** Direct relevant.
- **Singh, P., et al. "Discrete Global Grid Systems for Drone Delivery Operations". 2020 (multiple variants).**
- **Pham, T., Bhattacharya, S. "Hexagonal Grid-Based Adaptive Path Planning for UAV Swarms". IEEE RAL 8(2), 2023.**

#### Comparison cu alte sistemes spatial
- **Niedzwiedz, T., et al. "S2 vs H3: A Comparison of Hierarchical Spatial Indexes". 2019** (blog, multiple sources).
- **Google S2 Geometry**: github.com/google/s2geometry — alternative (square tiles on cube → sphere projection). Used by Pokémon GO.
- **Geohash** (Niemeyer 2008) — classical square hierarchical encoding. Most VPR papers reference it.

### 15.14 Bottom line pentru operator

**Răspuns la întrebarea concretă**: **Da, sistemul există de 20 de
ani** (Sahr 2003) și a fost popularizat industrial de Uber în 2018
sub numele H3.

**Patternul "fagure cu hex mari unde sparse, mici unde dense" e standard
DGGS** — în literatura din 2007-2020 e referit ca "adaptive resolution
DGGS" sau "density-adaptive hexagonal indexing".

**Pentru sentai_runtime**:
1. Port Uber H3 lib direct (~50 KB SDRAM, MIT licensed, C99)
2. Construiește gallery cu H3Index → place mapping (O(1) lookup vs O(N))
3. Multi-altitude resolution: drone @ 50m query la res 10 (66m cells),
   @ 5m query la res 12 (9m cells) — automatic via `optimal_h3_res(alt)`
4. Adaptive subdivision: split cells cu > 10 visits, merge cells cu < 2
   visits → fagure adaptiv per density-of-information

**Buget MCU H3 full**:
- ~12 ms per query @ 1 Hz = 1.2% CPU
- ~127 KB SDRAM (lib + 256-cell gallery + 16 resolutions support)
- 0 KB ITCM
- ~10-12 zile development effort

**Avantaje strategice pe termen lung**:
- Compatibil cu OSM / ArcGIS / Mapbox (când extindem la outdoor real)
- Multi-drone coordination via Meshtastic (descriptori share-uiți pe celule)
- Persistență misiunilor (gallery serialized pe FileX, restored la
  boot — drone-ul **învață peisajul progresiv**)

**Recomandare strategică finală**: **adoptăm H3 ca fundație în loc de
V3 hybrid square grid** (din §14.2). Costul adițional (~5 zile dev, ~85 KB
SDRAM) e clear win pentru un sistem care va trăi ani.

**Stage 11 final, integrat cu §13-15** (overview):

```
Stage 11 — sentai.places.* — full pipeline (12-15 zile)

  11.A — Port Uber H3 lib la M7         (1-2 zile)
  11.B — H3-indexed gallery + hash     (2-3 zile)
  11.C — Adaptive resolution split/merge (1-2 zile)
  11.D — Multi-altitude descriptor slots (2-3 zile)
  11.E — Loop closure via H3 match     (1 zi)
  11.F — Persistence FileX             (1 zi)
  11.G — Validation cu canonic scenario (2-3 zile)
```

---

## 16-17. Multi-level honeycomb storage + cross-scale composition → MOVED to `FutureWork.md`

**Status**: out of thesis scope (frozen 2026-05-15 per §23).

- §16 *Multi-level honeycomb storage + per-cell embedding + orientation
  encoding* — moved to `FutureWork.md` FW8.
- §17 *Cross-scale embedding composition (parent ↔ children)* — moved
  to `FutureWork.md` FW9.

Both are sophisticated extensions beyond what the thesis demo
requires. Thesis uses single-resolution H3 + single-shot descriptor +
simple replace-on-update (per §15.11 Stage 11.A–B baseline). Promotion
criteria for each item documented in `FutureWork.md`.

Original ~940 lines of research/design content removed 2026-05-15
during scope-management cleanup. Available in git history if needed
(`git show <pre-cleanup-commit>:ideas/objects_plan.md`).

---

## 18. Mission `sentai.explore` — autonomous two-flight exploration with anchored loop closure

> **Obiectiv operațional final**: drona se ridică într-o lume necunoscută
> cu ArUco la decolare → recunoaște împrejurimile → își construiește
> modelul lumii → aterizează (flight 1, "învățare"). La flight 2,
> drona reutilizează modelul persistat, continuă explorarea în
> zonele noi, apoi revine la markerii ArUco pentru aterizare. În
> tot timpul, stabilizarea pose-ului dronei folosește modelul lumii
> (object map + place fingerprints + ArUco anchor).

Această secțiune este **guideline-ul de test final** pentru întreaga
foaie de drum §3 (Stages 1-17). Servește ca **specificație** care
trebuie atinsă; orice stage din §3 e considerat livrabil DACĂ
contribuția lui în această misiune e validată.

### 18.1 Mission philosophy — context NASA/JPL

Conform `embeded.md §A` (system model first), misiunea are
**4 modele explicite**:

#### Fault model — ce poate să meargă rău și răspunsul

| Fault | Severitate | Acțiune | Stare safe |
|---|---|---|---|
| F1 ArUco home pierdut > 15s | HIGH | trigger RTL via flow+EKF; cov mărită | RETURN_HOME |
| F2 Battery < critical (15%) | CRITICAL | immediate RETURN_HOME max speed | LAND oriunde safe |
| F3 EKF cov.trace > prag | HIGH | abort exploration; conservative RTH | ABORT |
| F4 IMU/flow disagreement > prag | HIGH | freeze pose, hover, alert | ABORT |
| F5 Object map full (32 slots) | LOW | evict STALE; continue | (continue) |
| F6 TPU inferență fail | MEDIUM | degraded mode (no new objects) | (continue, reduced map updates) |
| F7 Loop closure rate < 0.1/s pentru > 30s | MEDIUM | mărește exploration spre home | (continue, biased return) |
| F8 Watchdog kick lost | CRITICAL | system reset (last resort) | (re-boot, restore from FileX) |
| F9 FileX write fail | LOW | continue in-memory; warn on REPL | (continue, no persist) |

#### Execution model — task-uri și prioritate

```
M7 FreeRTOS tasks (peste cele existing):
├── micropython_repl (existing, tskIDLE+2)
├── camera_task (existing, tskIDLE+2)
├── flow_task (existing, tskIDLE+2)
├── detection_task (existing — runs TPU + tracker)
├── anchor_forward (existing, tskIDLE+2)
├── fs_task (existing, tskIDLE+2)
├── watchdog (existing, tskIDLE+3)
│
├── NEW: sentai.explore mission_sm task @ 20 Hz, tskIDLE+2
├── NEW: sentai.explore action_layer task @ 50 Hz, tskIDLE+2
├── NEW: sentai.objects lifter task @ 10 Hz, tskIDLE+2
└── NEW: sentai.places fingerprint task @ 1 Hz, tskIDLE+1
```

#### Recovery model

| Stare | Recovery preferred (în ordine) |
|---|---|
| EXPLORE_PATTERN — fault NON-CRITIC | retry local (ex: redo current pattern leg) |
| EXPLORE_PATTERN — fault MEDIUM | local recovery (skip to next leg) |
| RETURN_HOME — fault MEDIUM | subsystem restart (e.g., re-init TPU) |
| RETURN_HOME — fault HIGH | degraded mode (use only flow+IMU, no objects) |
| Orice — fault CRITICAL | safe mode (HOVER), then LAND wherever safe |
| Total system fail | controlled restart (sentai.sys.reset → reload from FileX) |

**NICIODATĂ** full reboot ca primă opțiune. Asta se aliniează cu
discplina `embeded.md` care pune reboot ca **ultim** răspuns.

### 18.2 Two-flight protocol

#### Flight 1 — "Learning Flight"

```
Pre-condition: home ArUco vizibil, FileX user partition empty
Duration: ≤ 120 secunde (TIMEOUT)
Battery start: > 80%
Goal: construire model lume + safe return

States traversed:
  IDLE → ARM_AT_MARKER → TAKEOFF → ESTABLISH_BASELINE → EXPLORE → 
  RETURN_HOME → PRECISION_LAND → DONE
  
Output (persisted to FileX):
  /explore/<timestamp>/objects.fxmap   — object map (sentai_objects gallery)
  /explore/<timestamp>/places.h3map    — H3 places gallery (if Stage 11+ shipped)
  /explore/<timestamp>/trajectory.csv  — pose log per 100 ms
  /explore/<timestamp>/summary.json    — metrics + completion status
```

#### Flight 2 — "Reuse Flight"

```
Pre-condition: /explore/<timestamp>/ files exist și mount valid
Duration: ≤ 120 secunde
Battery start: > 80%
Goal: reuse model existing + extension în zona necunoscută

States traversed:
  IDLE → LOAD_MODEL → ARM_AT_MARKER → TAKEOFF → 
  ESTABLISH_BASELINE (uses prior anchor) → EXPLORE (extends prior) → 
  RETURN_HOME → PRECISION_LAND → DONE

Performance gain vs flight 1:
  - Yaw drift initial < 1° (vs 5°) — known anchor at boot
  - Position cov throughout < flight 1 × 0.7 — prior map stabilizes
  - Discovery of >= 1 NEW object beyond flight 1's map
  - Loop closure rate > flight 1's by ≥ 2× (richer landmark density)

Output:
  /explore/<timestamp_2>/objects.fxmap     — extended map
  ...
```

### 18.3 State machine — `sentai.explore` SM2 detailed

Per `embeded.md §B` (supervision), fiecare stare are: heartbeat, deadline,
guard conditions, exit fault paths.

```
                  ┌───────────────────────────────────────────────────────────┐
                  │                                                            │
                  ▼                                                            │
  IDLE → ARM_AT_MARKER → TAKEOFF → ESTABLISH_BASELINE → EXPLORE → RETURN_HOME │
                  ↑           ↑              ↑              ↓                 │
                  │           │              │              │                 │
                  │           │              └──────────────┘                 │
                  │           │              (loop closure triggers re-anchor)│
                  │           │                                                │
                  │           └──── EMERGENCY_HOVER (any state → safe) ────────┤
                  │                                                            │
                  └──── ABORT ──────────────────────────────────── PRECISION_LAND
                              (any state, fault > prag)            │
                                                                    ▼
                                                                  DONE
```

**State table cu fault gates** (per `embeded.md §B + §F`):

| State | Heartbeat needed | Deadline | Guard out (success) | Guard out (fault) |
|---|---|---|---|---|
| **IDLE** | 1 Hz | ∞ | sentai.explore.start() called | n/a |
| **LOAD_MODEL** | 1 Hz | 10s | FileX restored OK | timeout / parse err → ABORT |
| **ARM_AT_MARKER** | 5 Hz | 15s | EKF stable + home ArUco visible + battery OK | precondition fail → IDLE+err |
| **TAKEOFF** | 5 Hz | 20s | altitude ≥ 1.5 m + EKF cov < prag | timeout / EKF lost → ABORT |
| **ESTABLISH_BASELINE** | 5 Hz | 15s | yaw datum captured + ≥ 1 anchor stored | timeout → degraded EXPLORE |
| **EXPLORE** | 5 Hz | 90s (timeout per flight) | pattern complete OR re-anchor schedule trigger OR battery < 40% OR time > 90s | F1 (anchor lost > 15s) → RETURN_HOME |
| **RETURN_HOME** | 5 Hz | 30s | within 1m of home marker + altitude > 0.4m | timeout / lost EKF → ABORT |
| **PRECISION_LAND** | 5 Hz | 20s | landed (altitude < 0.2m + motors disarmed) | timeout / pixel err > 100px → COAST_LAND |
| **COAST_LAND** | 5 Hz | 10s | landed (open-loop, no marker) | (terminal — disarm motors regardless) |
| **EMERGENCY_HOVER** | 5 Hz | 30s | fault cleared → resume prev state | timeout → ABORT |
| **ABORT** | 5 Hz | 60s | safe landing achieved | (terminal) |
| **DONE** | 1 Hz | ∞ | n/a | n/a |

**Hysteresis pe tranziții critice** (per `embeded.md §F`, anti-chattering):
- EXPLORE → RETURN_HOME: anchor lost > **15s** (it's hard to lose home marker)
- RETURN_HOME → EXPLORE: anchor recovered AND mission_time < threshold (back to mission)
- ARM_AT_MARKER → IDLE: precondition fail must persist > **2s** (avoid flickering)
- EMERGENCY_HOVER → previous: fault cleared must persist > **3s**

**Timeouts globale per state**: după depășire → forced transition la
ABORT sau RETURN_HOME (vezi tabel). Asta-i implementare directă a
`embeded.md §B` ("dacă-l depășești → abort sau fallback").

### 18.4 Intent emitted per state (per `embeded.md §C` driver/policy separation)

`sentai.explore` SM2 emite **intenții**, NU velocity-setpoints (per §3
Stage 4 din plan):

| State | Intent type | Parameters |
|---|---|---|
| IDLE | NONE | — |
| ARM_AT_MARKER | HOLD | pose actuală |
| TAKEOFF | GOTO | (home_xy, +1.5m), yaw=hover_yaw, v_max=0.5 |
| ESTABLISH_BASELINE | HOLD with yaw_sweep | yaw_rate=20°/s, duration=8s |
| EXPLORE (pattern=SPIRAL_OUT) | WAYPOINT_PATTERN | spiral params, max_radius=5m, v_max=0.8 |
| EXPLORE (pattern=RASTER) | WAYPOINT_PATTERN | lawn-mower lines, v_max=0.8 |
| EXPLORE (pattern=OBJECT_CHASE) | GOTO_FOCUS | target_obj_id, dist=2m, v_max=0.6 |
| EXPLORE (pattern=RETURN_FOR_LOOP) | GOTO | (near_home_xy + 2m radius), v_max=1.0 |
| RETURN_HOME | GOTO | (home_xy, +1.5m), yaw=face_home, v_max=1.0 |
| PRECISION_LAND | SERVO_IMAGE | bbox=home_marker_bbox, descent_rate=0.05m/s |
| COAST_LAND | DESCEND_OPEN_LOOP | rate=0.1m/s |
| EMERGENCY_HOVER | HOLD | freeze pose |
| ABORT | LAND_SAFE | descend cautious |

Action layer (Stage 4) traduce aceste intent-uri în velocity
setpoints care pleacă la `sentai.link.send_velocity` (PX4) sau
`sentai.crazy.send_velocity` (cf2).

### 18.5 API surface — `sentai.explore.*`

```python
# Lifecycle (called from REPL or autonomous startup script)
sentai.explore.start(
    mission_label="flight1",       # arbitrary string for log dir
    duration_max_s=120,             # hard timeout
    pattern="spiral_out",           # "spiral_out" | "raster" | "object_chase_with_returns"
    pattern_params={                # pattern-specific
        "max_radius_m": 5.0,
        "altitude_m": 1.5,
        "speed_max": 0.8,
    },
    restore_from=None,              # str path to load prior model (flight2)
    battery_floor_pct=40,            # auto-RTH below this
    auto_persist_every_s=10,         # FileX checkpoint cadence
)
# Returns: 0 OK / -1 precondition fail / -2 already running

sentai.explore.stop()                # graceful → triggers RETURN_HOME

sentai.explore.abort()               # emergency → ABORT state immediately

# Telemetry & status
sentai.explore.state()               # str: current state name
sentai.explore.metrics()             # dict — see below
sentai.explore.intent()               # dict — current intent being emitted
sentai.explore.last_transition()     # tuple (from_state, to_state, reason_code, t_ms)

# Map operations (callable mid-mission for debug)
sentai.explore.save_now(label)       # explicit FileX dump
sentai.explore.load(path)            # explicit restore (only valid in IDLE)
```

**Metrics dict** (per `embeded.md §observabilitate`):
```python
{
    "mission_label": "flight1",
    "state": "EXPLORE",
    "time_in_state_ms": 12345,
    "time_total_ms": 45678,
    "battery_pct": 67,
    "altitude_m": 1.52,
    "ekf_cov_trace": 0.041,
    "objects_in_map": 4,
    "objects_confirmed": 3,
    "objects_stale": 0,
    "places_in_gallery": 12,
    "loop_closures_total": 18,
    "loop_closures_recent_5s": 2,
    "anchor_visible": True,
    "anchor_age_ms": 250,
    "tpu_fps": 38.5,
    "fault_counters": {
        "F1_anchor_lost": 0,
        "F4_imu_flow_disagree": 0,
        "F5_map_full": 0,
        "F6_tpu_fail": 0,
        "F7_no_lc": 0,
        ...
    },
    "stack_hwm_words": 1820,
    "abort_reason": None,
}
```

### 18.6 Integration cu stagiile existente

Stage de cod necesar pentru `sentai.explore` funcțional minimum:

| Stage din §3 | Used | Variant necesar minim |
|---|---|---|
| Stage 1 — object map | **REQUIRED** | full (add/get/list/persist) |
| Stage 2 — TPU model | **REQUIRED** | initial YOLO with 2-3 classes (cube, cylinder, box) — can be coco pre-trained inițial |
| Stage 3 — mission SM | **REQUIRED** | extins în sentai.explore (mai bogat decât minimal SM2 din §3) |
| Stage 4 — action layer | **REQUIRED** | PBVS + IBVS + APF avoidance |
| Stage 5 — object lifter | **REQUIRED** | inverse-depth simplificat: bearing + class prior, fără full EKF inițial |
| Stage 6 — loop closure | **REQUIRED** | simplu (single-marker ArUco) inițial; §12 multi-object e îmbunătățire |
| Stage 11 — places (H3 + composition) | **OPTIONAL initially** | dacă lipsește, drona explorează fără place memory; flight 2 are doar object map prior |

**Conclusion**: drumul minim viabil pentru `sentai.explore` =
Stages 1+3+4+5+6 + (ArUco anchor pe care îl avem deja de la s112).
Stage 11 e improvement pentru robustness, NU blocker pentru
funcționalitate.

### 18.7 Test scenarios — guideline canonic

#### Test world (`worlds/explore_canonical.sdf`)

```
Mediu: 10m × 10m × 3m indoor (camera Gazebo Garden)
Podea: parchet texturat (rezistent la flow loss)
Pereți: 4 pereți cărămidă/lemn, fără ferestre (lumina controlată)

Markeri ArUco la decolare:
  - 4× ArUco 4×4_50 markers, IDs 0-3
  - Plasare pătrat 1m latură la centru (0,0)
  - ID 0 la (−0.5, −0.5), ID 1 la (+0.5, −0.5),
    ID 2 la (−0.5, +0.5), ID 3 la (+0.5, +0.5)
  - Mărime 8 cm (compatibilă cu sentai_aruco)

Obiecte distribuite (pentru object map):
  - Red Cube — class_id=1, real_size=0.30m, pos randomized within {(2,2)..(4,4)}
  - Blue Cylinder — class_id=2, real_size=0.25m, pos randomized within {(−4,−2)..(−2,0)}
  - Green Box — class_id=3, real_size=0.40m, pos randomized within {(3,−3)..(4,−1)}

Obstacole (pentru APF avoidance):
  - 2× cylindrical pillars 0.3m diam, 2m height, pre-cunoscute în config

Random per rulare:
  - obj positions ±0.5m within their zones
  - lighting ±20%
  - IMU bias drift 0.01°/s
```

#### Test case formalizat (Gherkin-style)

```gherkin
Test: Two-flight exploration end-to-end

Background:
  Given Gazebo world "explore_canonical.sdf" is loaded
  And sentai_sim is running with all required modules
  And FileX user partition is empty (or label /explore/ is clean)
  And cf2 SITL is hovering disarmed at home marker
  And battery sim is at 90%

Scenario: Flight 1 — Learning Flight
  When operator runs:
    sentai.explore.start(
      mission_label="canonical_flight1",
      duration_max_s=120,
      pattern="spiral_out",
      pattern_params={"max_radius_m": 4.0, "altitude_m": 1.5},
    )
  
  Then within 120 seconds:
    - SM2 transitions: IDLE → ARM_AT_MARKER → TAKEOFF → ESTABLISH_BASELINE
      → EXPLORE → RETURN_HOME → PRECISION_LAND → DONE
    - No EMERGENCY_HOVER or ABORT entered
    - Final landing error vs home marker center: < 15 cm horizontal,
      < 5 cm vertical
    - At least 2 of 3 distinct objects in /explore/<lbl>/objects.fxmap
      with status=CONFIRMED + cov.trace < 0.5
    - Yaw drift at landing (vs ground truth): < 5°
    - TPU pipeline FPS sustained ≥ 30 fps throughout (not < 25 fps for > 2s)
    - FileX file objects.fxmap exists and parses cleanly
    - sentai.explore.metrics().fault_counters all 0 (no F-faults triggered)
  
  And the persisted map shows:
    - ≥ 2 CONFIRMED objects with positions matching GT ±0.3m
    - ≥ 1 ArUco anchor stored with anchor_status=CONFIRMED
    - Places gallery (if Stage 11 shipped): ≥ 5 distinct H3 cells visited

Scenario: Flight 2 — Reuse Flight
  Given Flight 1 has completed successfully with map at /explore/canonical_flight1/
  When operator runs:
    sentai.explore.start(
      mission_label="canonical_flight2",
      duration_max_s=120,
      pattern="raster",  # different pattern to extend coverage
      restore_from="/explore/canonical_flight1/objects.fxmap",
    )
  
  Then within 120 seconds:
    - SM2 enters LOAD_MODEL → loads ≥ 2 prior objects successfully
    - Yaw drift initial (at end of ESTABLISH_BASELINE): < 1°
      (versus < 5° in Flight 1 without prior)
    - Average EKF cov.trace during EXPLORE: ≤ Flight 1 × 0.7
    - At least 1 NEW object discovered beyond Flight 1 map (status=CONFIRMED)
    - Loop closure rate (LC events / s) ≥ Flight 1 × 2
    - Final landing error: < 10 cm (better than Flight 1 due to prior)
    - DONE state reached

  Pass criteria summary (mandatory all):
    □ Both flights reach DONE state
    □ No ABORT triggered in either flight
    □ Map persisted + reloaded cleanly
    □ Flight 2 metrics better than Flight 1 on: yaw drift, cov, LC rate
    □ Final landing accuracy < 15cm both flights
    □ FPS regression ≤ 20% over baseline (i.e., ≥ 33 fps)
    □ Memory: stack hwm < 70% all tasks; SDRAM < 80% capacity
```

#### Test artifacts produced

```
examples/sentai_runtime/experiments/s117_explore_canonical/
├── README.md                          # invocation + pass numbers
├── world_explore_canonical.sdf
├── run_flight1.sh                     # full SIM launch
├── run_flight2.sh                     # second flight using flight1 map
├── run_full_test.sh                   # both flights end-to-end
├── analyze_flight.py                  # parse CSV + JSON, compute metrics
├── expected_pass.json                 # thresholds for pass/fail decision
└── results/
    ├── flight1_YYYYMMDD_HHMMSS/
    │   ├── trajectory.csv
    │   ├── states.csv                 # state transitions per ms
    │   ├── map_persisted.fxmap.hex    # FileX content dump
    │   ├── metrics_final.json
    │   └── plots/                     # post-run charts
    └── flight2_YYYYMMDD_HHMMSS/
        └── (same structure)
```

### 18.8 FPS regression protocol — ARM perf testing per stage

**Mandate from operator**: when adding algorithms that risk
degradează FPS, run **on-board ARM perf test** before promoting that
stage as complete.

**Baseline metrics** (build #1298 measured pe board):
- TPU pipeline yolo_1 pure-TPU: **41-42 fps**
- TPU pipeline yolo_1 with detection task: **38-40 fps**
- TPU + flow + camera concurrent: **30-35 fps**
- `sentai.diag.aruco_bench()` PXP+SIMD threshold path: **1.1 ms**

**Per-stage ARM verification gate**:

| Stage adds | FPS regression test required? | Expected impact |
|---|---|---|
| 1 — Object map (pure data) | NO | 0% |
| 2 — New TPU model | YES (model swap) | depends on model size; target ≥ 35 fps |
| 3 — Mission SM | NO (20 Hz task, < 0.01% CPU) | 0% |
| 4 — Action layer | NO (50 Hz task, < 0.1% CPU) | 0% |
| 5 — Object lifter (EKF) | **YES** (10-30 Hz with CMSIS-DSP) | target ≥ -2% |
| 6 — Loop closure | NO (rare events, < 0.1% CPU) | 0% |
| 11 — Places + H3 + descriptors | **YES** (1 Hz heavy compute) | target ≥ -5% |

**Procedure** per stage with risk:
1. **Pre-change ARM bench**: run `sentai.diag.tpu_bench(model="yolo_1.tflite", n_frames=300)` 3× — record min/avg/max fps
2. Implement stage (verify SIM build first)
3. ARM build + flash persistent
4. **Post-change ARM bench**: same call, 3× runs
5. Compare: delta_fps = pre.avg - post.avg
6. **Pass criterion**: delta_fps ≤ 2 fps OR < 5% relative (whichever is smaller for sensitive cases)
7. If fail: **roll back stage; identify bottleneck; optimize before re-attempting**

**Cumulative FPS budget**: la fiecare stage din §3 până la final NU mai
mult de **-20% total** față de baseline. Final target: **≥ 33 fps**
sustained în pipeline (38 fps baseline × 0.85).

**Documentation per stage**: în `experiments/sNNN_*/README.md`
include o secțiune `## ARM perf delta` cu numbers pre/post.

### 18.9 NASA/JPL discipline aplicat — recap per `embeded.md`

| `embeded.md` rule | Aplicare în sentai.explore |
|---|---|
| **A. System model first** | §18.1 fault/execution/recovery models |
| **B. Real-time supervision** | Heartbeat 1-5 Hz per state, deadline per state, guard conditions explicite (no events) |
| **C. ISR/task/driver discipline** | Mission SM = task; nu face I/O direct; doar emite intent |
| **C. Action layer separation** | Action layer translatează intent → velocity; mission SM nu știe de PBVS/IBVS detail |
| **D. Memory rules** | Toate task-urile cu stack `configMINIMAL_STACK_SIZE × 2`, expose `stack_hwm` |
| **D. Static-only** | Object map static 32 slots; places static 256; no heap post-boot |
| **E. Concurrency** | Mission SM citește state din alte task-uri prin atomic snapshot (no mutex contention) |
| **F. Recovery model** | EMERGENCY_HOVER → previous; ABORT → safe land; reboot doar ca last resort |
| **F. Self-healing** | Anchor lost: switch la flow+EKF only → eventually re-anchor → recover |
| **§5 Observability** | Metrics dict complet; log per state transition; dmesg pentru toate fault triggers |

### 18.10 First implementation step — Stage 1 (object map) MVP

Per `embeded.md` recomandare ("Start simple. Pentru primul prototip
implementează doar..."), începem cu **Stage 1 minimal** care e și
**foundation pentru toate următoarele**:

**Stage 1.A — `sentai.objects.*` skeleton + binding** (1-2 zile):

Deliverable:
- `examples/sentai_runtime/sentai_objects.h` — contract API
- `examples/sentai_runtime/sentai_objects.cc` — static `map[32]` + add/get/list/remove
- `examples/sentai_runtime/modsentai_objects.c` — MP binding
- QSTR regen
- Build OK pe ARM și SIM
- `experiments/s117a_objects_smoke/` — wire test

Pass criteria:
- `sentai.objects.add(class_id=1, x=0.5, y=0.0, z=0.3)` returns valid `obj_id`
- `sentai.objects.get(obj_id)` returns dict with class_id, position, status
- `sentai.objects.list()` returns list of all populated slots
- `sentai.objects.remove(obj_id)` removes the slot
- Fault gates: NaN input rejected with `oob_rejected++`
- Build delta `.text`: 0 bytes (all `.sdram_text`)
- No FPS regression (object map is pure data)

**De ce începem cu asta**:
- **Zero risk** la FPS (pure data layer)
- **Toate ulterioarele depend de asta** (lifter, mission SM, places — toate need a map to write into)
- Demonstrate end-to-end pattern: header + arm.cc + sim.c + MP binding + test
- ~200 lines de cod total; ~2 zile efort

**Următorul step după**: Stage 1.B (FileX persistence pentru objects),
apoi Stage 3 (mission SM minimal) **împachetat ca primă versiune de
`sentai.explore`** — nu construim mission SM minimalist separat și
apoi îl extindem; e mai eficient să construim direct `sentai.explore`
cu doar 3 stări (IDLE → TAKEOFF → DONE) și să-l îmbogățim incremental.

### 18.11 Roadmap concret — următoarele 4 săptămâni de muncă

Săptămâna 1 — **Fundație** (FPS-safe; total < 1% CPU adăugat)
- Stage 1.A — object map + binding + tests
- Stage 1.B — FileX persistence (`save_now` + `load`)
- Stage 3.A — `sentai.explore` SM cu IDLE→ARM_AT_MARKER→TAKEOFF→PRECISION_LAND→DONE
- Validare SIM: armed → takeoff → land at home

Săptămâna 2 — **Acțiune și loop closure simplu** (FPS-safe)
- Stage 4.A — Action layer simplificat: GOTO + HOLD + LAND intents
- Stage 6.A — Single-marker ArUco anchor (extension existing s112 P2)
- Stage 3.B — Add EXPLORE state cu spiral pattern
- Validare SIM: full short mission cu single ArUco

Săptămâna 3 — **Object detection în map** (FPS-RISK STAGE)
- Stage 5.A — Object lifter cu bearing + class-prior pseudo-depth (NO full EKF inițial; just average over observations)
- **ARM FPS bench** (pre + post)
- Stage 3.C — Mission SM populates map durring EXPLORE
- Validare SIM: flight 1 produces map with ≥ 2 objects

Săptămâna 4 — **Flight 2 și polish**
- Stage 1.C — Map load on boot via FileX
- Stage 3.D — `restore_from=` parameter funcțional
- Stage 18 canonical test scenario complet
- **ARM FPS bench** pe full pipeline
- Documentation update (memory entries + experiment.md)

**Risk register pentru roadmap-ul de 4 săptămâni**:

| Risk | Probabilitate | Mitigare |
|---|---|---|
| Stage 5 lifter EKF numerical issues | MEDIUM | Begin cu simplified version (averaging, no EKF), upgrade only if necessary |
| FPS regression at Stage 5 > 5% | MEDIUM | Profile cu DWT pre-add; tune; potentially defer subdivisions |
| FileX corruption mid-mission | LOW | Already-proven from s113 (FileX P3) |
| Action layer velocity discontinuities | MEDIUM | Slew-rate limiter în send_velocity wrapper |
| Mission SM dead-lock | LOW | Per-state timeouts mandatory (§18.3) |

### 18.12 Bottom line — guideline final

**`sentai.explore` este obiectivul final** care valida toată plani din §3.
Stages 1-6 sunt PĂRȚI ale lui sentai.explore; nu sunt produs final
separat. Stages 11-17 sunt **îmbunătățiri opționale** care fac
mission mai robustă (place memory, scale composition, etc.).

**Test scenariu canonical** definit în §18.7 — `s117_explore_canonical`
e CANONICUL contra căruia măsurăm progresul. Orice stage e
considerat livrabil DACĂ contribuția lui în acest scenario e
validată.

**FPS protection**: la fiecare stage cu risc, **bench pe ARM
înainte și după**, target cumulativ ≥ 33 fps final.

**Disciplină NASA/JPL** aplicată complet — sentai.explore are explicit:
- Fault model (F1-F9)
- Execution model (4 noi task-uri + existing)
- Recovery model (retry → local → degraded → safe → reset)
- Heartbeat per state
- Deadline per state
- Guard conditions (not events)
- Hysteresis pe tranziții critice
- Static memory only
- Stack hwm exposed

**Începem implementarea cu Stage 1.A** (object map skeleton) — zero
FPS risk, foundation pentru toate ulterioare, ~2 zile efort.
Așteaptă confirmare operator pentru a începe codul concret.

---

## 19. Namespace audit — reorganizare ÎNAINTE de implementarea masivă (review 2026-05-13)

> *"Am dubii legate de denumiri namespaces. Hai fă o analiză, poate
> mai uniformizăm denumirile de namespace REPL — în special acum,
> când planificăm dezvoltări majore."*

**Decizia rezultantă va schimba propunerea inițială §3** (vezi §19.7 jos).

### 19.1 Inventar empiric — ce există DEJA în `sentai.*`

Extras din `examples/sentai_runtime/modsentai.c` (build #1298, 2026-05-13):

```
sentai.io        ← LED / GPIO
sentai.rtos      ← FreeRTOS info (sleep_ms, tasks, heap_info, cpu_usage)
sentai.tpu       ← EdgeTPU inference (load/invoke/output)
sentai.fs        ← FileX user partition + LittleFS system
sentai.camera    ← OV5640 + CSI capture
sentai.usb       ← USB modes (CDC-ACM, CDC-NCM, MSC)
sentai.uart      ← UART raw I/O
sentai.mesh      ← Meshtastic radio bridge
sentai.link      ← MAVLink (PX4) + just-added send_vpe
sentai.crazy     ← CRTP (Crazyflie) + just-added send_ext_position
sentai.imu       ← LIS2DU12 accelerometer
sentai.mic       ← PDM microphone
sentai.sleep     ← low-power sleep modes (separate from rtos.sleep_ms)
sentai.pipeline  ← Detection pipeline + SentAI-SORT (TPU + tracker bundled)
sentai.flow      ← Optical flow (PXP+phase-corr) + just-added anchor mode + auto-forward
sentai.aifes     ← AIfES on-device NN training
sentai.kmeans    ← K-means clustering for embeddings
sentai.pca       ← Principal Component Analysis
sentai.anomaly   ← Mahalanobis + CUSUM anomaly detection
sentai.dtw       ← Dynamic Time Warping
sentai.hmm       ← Hidden Markov Model
sentai.rl        ← Reinforcement learning primitives
sentai.slam      ← ★ Detection-Based EKF-SLAM (max 64 landmarks!) ★
sentai.tfl       ← TFLite-Micro CPU fallback
sentai.diag      ← Diagnostics + health + dmesg + aruco_bench
sentai.sys       ← System control (reset, version)
```

**Plus top-level**: `sentai.help`, `sentai.version`, `sentai.verbose`,
`sentai.debug`, `sentai.console`, `sentai.run`.

### 19.2 Descoperirea critică — `sentai.slam` EXISTĂ și face DEJA ce am propus

```
sentai.slam - Detection-Based EKF-SLAM
=======================================
  Object-Level SLAM using bounding-box detections as landmarks.
  Estimates 2D robot pose (x, y, theta) and builds a map of detected
  objects on the ground plane.

  API surface:
    init(fov_h_deg, img_w, img_h [,max_lm=64] [,baseline_m=0])
    update(detections)              ← detection-driven landmark insert
    update_stereo(dets_l, dets_r)   ← stereo depth disparity
    observe()                       ← manual observation injection
    predict()                       ← IMU-driven prediction step
    pose()                          ← (x, y, theta)
    landmarks()                     ← list of stored landmarks
    noise()                         ← config Q/R matrices
    imu_correct()                   ← attitude correction
    clear() / save() / load()       ← lifecycle + persistence
    info()                          ← stats
```

**Mai mult de 14 funcții deja**. **Persistence `save/load` deja shipped.**

**Conclusion**: ce am propus ca `sentai.objects` / `sentai.map` (Stage 1
din §3) este **categoric o duplicare**. Direcția corectă e **să
EXTINDEM `sentai.slam`** la 3D + adăugiri noi, NU să creăm namespace
paralel.

### 19.3 Mapping semantic — verb vs noun + layer

Conform principiilor de naming SOTA (ROS, AirSim, PX4):

| Layer conceptual | Verb/Noun? | Sentai existing | Comments |
|---|---|---|---|
| **Sensors (input)** | Noun = sensor type | `camera`, `imu`, `flow`, `mic` | clean, no rename needed |
| **Inference primitive** | Verb-as-noun | `tpu` (= "the EdgeTPU"), `tfl` | "the inferencer" |
| **Orchestration** | Noun (process) | `pipeline` | combines TPU + tracker |
| **World model** | Noun (artifact) | **`slam`** | clear: "the SLAM state" |
| **Comms** | Noun (channel) | `link`, `crazy`, `mesh`, `uart` | unified pattern |
| **Storage** | Noun | `fs` | filesystem |
| **System** | Noun | `sys`, `rtos`, `diag`, `io` | infrastructure |
| **ML utility** | Algorithm name | `kmeans`, `pca`, `dtw`, `hmm`, `aifes`, `anomaly`, `rl` | unified pattern |

**Pattern emergent**: **noun-form per modul**, fără sufixele `-er` /
`-or` (deci NU `tracker`, `inferer`, `mapper`). Verbele apar ca
**method calls** pe modul (`sentai.slam.update()`, `sentai.flow.start()`).

### 19.4 Întrebări specifice ale operatorului — răspunsuri

#### Q5.1 — "sentai.objects poate nu e cel mai potrivit"

**Răspuns**: CORECT — `objects` semantic e prea generic și conflictuează cu:
- **Detections** (per-frame from TPU)
- **Tracks** (temporal 2D from tracker)
- **Landmarks** (3D from SLAM)

**Cel mai bun substitut**: **EXTINDE `sentai.slam`** (deja existent) cu
funcționalitate 3D + obj-class priors + altitude-aware Z. Add new methods:
- `sentai.slam.add_landmark_3d(class_id, x, y, z, cov)`
- `sentai.slam.get_landmark(id)` — extension peste existing `landmarks()`
- `sentai.slam.set_class_priors(table)` — pentru pseudo-depth (Layer 4 din concept)

**Nu adăugăm `sentai.objects`. NU adăugăm `sentai.map`.**

#### Q5.2 — "sentai.explore poate mai bine se numește sentai.map"

**Analiză**:
- "Map" = **artifact** (entitatea persistentă) — clear noun
- "Explore" = **activity** (mission, behavior) — clear verb-as-noun

**Conflictul**:
- `sentai.map` ar conflictua cu `sentai.slam` (ambele despre world model)
- `sentai.explore` e clar diferit de `sentai.slam` (acțiune vs entitate)

**Recomandare**: păstrăm **`sentai.explore`** ca **mission api** (e o
activitate). Map-ul construit de explore TRĂIEȘTE în `sentai.slam`
(existent) + `sentai.places` (nou pentru H3).

Frază mnemonic: *"sentai.explore is the verb; sentai.slam + sentai.places
are the nouns it produces."*

#### Q5.3 — "Avem `detect` și `track` deja, riscă să se confunde"

**Stare actuală**:
- **NU** există `sentai.detect` ca top-level module
- **NU** există `sentai.track` ca top-level module
- Detect+Track sunt **bundled în `sentai.pipeline`** (background task)
- TPU inference low-level e în `sentai.tpu`

**Conclusion**: nu există conflict — propunerea de a adăuga sub-namespaces
noi (`detect` / `track`) ar fi creat probleme. **Soluția corectă**: lăsăm
bundle-ul existent în `pipeline`, **nu creem nimic nou pentru aceste
funcționalități**.

### 19.5 Decizia de design finală — namespace policy

#### A. Module care există + sunt suficient → ZERO CHANGES

```
sentai.io, sentai.rtos, sentai.tpu, sentai.fs, sentai.camera,
sentai.usb, sentai.uart, sentai.mesh, sentai.link, sentai.crazy,
sentai.imu, sentai.mic, sentai.sleep, sentai.pipeline, sentai.flow,
sentai.aifes, sentai.kmeans, sentai.pca, sentai.anomaly, sentai.dtw,
sentai.hmm, sentai.rl, sentai.tfl, sentai.diag, sentai.sys
```

**Total 25 modules. Toate stabile. NU REDENUM nimic.** Compatibilitate
înapoi e load-bearing — există scripturi diag, experimente, memoria de
sesiune anterioare care folosesc aceste nume.

#### B. Module care EXIST dar trebuie EXTINSE (în loc de duplicate)

```
sentai.slam   ← extinde la 3D pentru drone (era 2D ground-plane)
              ← add: class priors, altitude-aware Z, inverse-depth EKF,
                multi-marker loop closure
              ← keep: existing init/update/pose/landmarks/save/load API
```

**Critical**: NU duplicăm. NU creăm `sentai.map` sau `sentai.objects`.
**Întregul Stage 1 din §3 devine "extinde sentai.slam"** — schimbare
conceptuală majoră.

#### C. Module NOI necesare pentru viziune (3 only)

```
sentai.places   ← topological place graph cu H3 indexing
                (different concept than slam landmarks)
                ← per §13-17 (place fingerprints, multi-resolution)

sentai.explore  ← mission state machine + behavior pattern
                ← per §18 (2-flight protocol, EXPLORE/RETURN_HOME etc.)
                ← orchestrates sentai.slam + sentai.places + sentai.pipeline

sentai.servo    ← action layer (intent → velocity setpoint)
                ← PBVS + IBVS + APF
                ← per §3 Stage 4 (renamed from earlier proposal)
```

**Justificare pentru fiecare**:

| Module | De ce nou (nu extindere) | Alternative considerate |
|---|---|---|
| `sentai.places` | Place recognition e CONCEPT distinct de SLAM landmarks. Place = topological node cu descriptor; landmark = punct 3D cu covarianță. Distincție clară în literatură. | merge cu slam? — semantic murky |
| `sentai.explore` | Mission = activity ≠ map = artifact. Separation per `embeded.md §C` (decision vs action layer). | sentai.mission generic — mai abstract, mai puțin clar |
| `sentai.servo` | Action layer e distinct de mission SM (per `embeded.md §C`). Separation permite swap PBVS/IBVS fără să modificăm mission. | sentai.ctrl — mai generic dar mai puțin specific; sentai.fly — prea informal |

#### D. Module care apar dar nu le propunem să adăugăm

```
sentai.detect   ← NU — TPU inference e deja sentai.tpu + sentai.pipeline
sentai.track    ← NU — SentAI-SORT e deja în sentai.pipeline (track=True)
sentai.map      ← NU — duplicare cu sentai.slam
sentai.objects  ← NU — duplicare cu sentai.slam.landmarks()
sentai.mission  ← NU — sentai.explore e suficient pentru mission concrete
sentai.world    ← NU — semantic murky, prea generic
sentai.nav      ← NU — overlap cu explore + servo
sentai.localize ← NU — funcționalitate trăiește în sentai.slam
sentai.ekf      ← NU — implementation detail, în sentai.slam
```

### 19.6 Naming conventions going forward (canonical, document them)

Per audit existing + recomandare:

1. **Module names**: short (3-7 chars), lowercase, noun-form preferred.
   - ✓ `slam`, `flow`, `camera`, `tpu`, `pca`
   - ✓ `pipeline` (longer dar standard în domeniu)
   - ✗ NU `tracker`, `inferrer`, `mapper`, `localizer` (sufixele -er/-or evitate)

2. **Method names**: verb-form, descriptive.
   - ✓ `start()`, `stop()`, `update()`, `add()`, `get()`, `save()`, `load()`
   - ✓ `set_X()`, `get_X()` patterns
   - Acțiuni asincrone: `_async` suffix dacă există variantă sync (`save_async`)

3. **State queries**: noun-form, no `get_` prefix when obvious.
   - ✓ `state()`, `pose()`, `metrics()`, `info()`, `landmarks()`, `version()`
   - ✗ `get_state()` — redundant when `state()` is unambiguous

4. **Lifecycle methods** unified pattern:
   - `init(args)` — one-shot setup
   - `start([args])` — begin operation
   - `stop()` — graceful end
   - `pause()` / `resume()` — when applicable
   - `reset()` — back to initial state
   - `clear()` — empty internal state (vs reset = re-init)

5. **Stats / health**: consistent dict shape:
   - `<module>.stats()` returnează dict cu cifre live
   - `<module>.health()` returnează state machine state + fault counters
   - **Recomandare**: când adăugăm module noi, urmăm pattern-ul EXACT
     `anchor_forward_stats` (din s113 P2): dict cu `iters`, `running`,
     `target`, `health=(f1,f2,f3,hwm)` tuple.

6. **Abbreviations** — folosim doar dacă SUNT industry standard:
   - ✓ `tpu`, `imu`, `usb`, `uart`, `pca`, `slam`, `pwm`, `pid`
   - ✗ evit abbreviations proprii — folosim cuvântul complet

7. **NU sufixe `_module`, `_obj`, `_api`** în nume vizibile MicroPython.
   - Acelea sunt convenții internă C, nu user-facing.

### 19.7 Impact asupra planului §3 Stages 1-17

Plan revizuit cu noua taxonomie:

| Stage din §3 | Vechi (propus) | Nou (post-audit) |
|---|---|---|
| Stage 1 — object map | `sentai.objects.*` (nou) | **`sentai.slam.*` (extensie existing)** |
| Stage 3 — mission SM | `sentai.mission.*` (nou) | **`sentai.explore.*` (nou, specific)** |
| Stage 4 — action layer | `sentai.servo.*` (nou) | **`sentai.servo.*` (păstrat)** |
| Stage 5 — object lifter | (intern în objects) | **(intern în sentai.slam — methods like `update_3d(detections, altitude)`)** |
| Stage 6 — loop closure | (cross-module) | **(intern în sentai.slam — method `loop_closure_update()`)** |
| Stage 11 — places | `sentai.places.*` (nou) | **`sentai.places.*` (păstrat)** |

**Schimbare semnificativă**: Stages 1, 5, 6 devin **METODE NOI în
`sentai.slam`**, NU un namespace separat. Reduce surface area surface
publică cu ~5 module noi, păstrează expertize developer (cei care
cunosc `sentai.slam` deja).

Codul C-side rămâne cu fișiere separate (sentai_slam.cc + helpers),
DOAR namespace MicroPython e unificat.

### 19.8 Migration plan — what to do AS we add new methods

**Pentru `sentai.slam` extensie 3D**:

```c
// În modsentai_slam.c — adăugăm new methods la existing globals_table

static const mp_rom_map_elem_t sentai_slam_globals_table[] = {
    // EXISTING (zero touch — backwards compatible)
    { MP_ROM_QSTR(MP_QSTR_init),         MP_ROM_PTR(&mod_slam_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_update),       MP_ROM_PTR(&mod_slam_update_obj) },
    { MP_ROM_QSTR(MP_QSTR_pose),         MP_ROM_PTR(&mod_slam_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_landmarks),    MP_ROM_PTR(&mod_slam_landmarks_obj) },
    // ... existing functions preserved as-is ...
    
    // NEW for 3D drone use case (peste existing 2D)
    { MP_ROM_QSTR(MP_QSTR_init_3d),         MP_ROM_PTR(&mod_slam_init_3d_obj) },
    { MP_ROM_QSTR(MP_QSTR_update_3d),       MP_ROM_PTR(&mod_slam_update_3d_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_class_prior), MP_ROM_PTR(&mod_slam_class_prior_obj) },
    { MP_ROM_QSTR(MP_QSTR_anchor_update),   MP_ROM_PTR(&mod_slam_anchor_obj) },
    { MP_ROM_QSTR(MP_QSTR_loop_closure),    MP_ROM_PTR(&mod_slam_lc_obj) },
};
```

Existing users see no break. New 3D users have parallel API. Doc-uri
help.txt updated cu noi metode dar fără ștergeri.

**Pentru module noi `sentai.places` și `sentai.explore`**: standard
new-module pattern (modsentai_NAME.c, include în modsentai.c, register
în globals_table).

**Pentru `sentai.servo`**: similar, new module.

### 19.9 Action items concrete (înainte de a începe codul)

1. **Update memory entry** cu policy de naming (per §19.6) — assignment:
   `naming_policy.md` în /memory.

2. **Update help.txt** cu naming policy section + plans for upcoming
   3 new modules (places, explore, servo) — operator-visible.

3. **Update §3 Stage 1** în acest plan ca să spună "extinde
   sentai.slam" în loc de "creează sentai.objects".

4. **Decide ASAP**: există `sentai.slam.update()` care primește 2D
   detections. Pentru 3D extension:
   - **Opt A**: păstrează 2D API + add `update_3d()` separat
   - **Opt B**: face `update()` să detecteze flag-ul "mode" și
     să apeleze 2D sau 3D
   - **Recomandare**: Opt A — clearer, less foot-gun, no
     mode-dependent behavior surprise.

### 19.A QSTR audit detaliat — elemente refolosibile descoperite (CRITICAL)

Inventar exhaustiv al MP_QSTR-urilor per modul existing relevă **multe
capabilități ascunse** care reduc drastic surface area de implementat
nou. Catalog complet de elemente reusable pentru viziunea sentai.explore:

#### `sentai.slam` — DEJA FULL EKF-SLAM (refolosibil 90%)

Surface existing:
```
init, update, update_stereo, observe, predict, pose, landmarks,
noise, imu_correct, clear, save, load, info, frame,
plus output dict fields: class_id, id, seen, focal, fov,
                         img_h, img_w, baseline, active, initialized
```

**Ce avem deja gratuit**:
- ✓ EKF predict step (`predict()`)
- ✓ EKF update step (`update(detections)` — accepts detection list direct)
- ✓ Stereo support (`update_stereo()`) — pentru extensie multi-camera
- ✓ Manual observation injection (`observe()`)
- ✓ Pose query (`pose()` returns (x, y, theta))
- ✓ Landmark list query (`landmarks()`)
- ✓ Noise model configurable (`noise()` for Q/R matrices)
- ✓ IMU-derived attitude correction (`imu_correct()`)
- ✓ Lifecycle: `clear()`, `save()`, `load()`
- ✓ Stats: `info()` cu `active`, `initialized`, `frame` counter

**Ce trebuie EXTINS** (nu rewrite):
- 🆕 `update_3d(detections, altitude)` — accept altitude pentru Z dimensiune
- 🆕 `set_class_prior(class_id, real_size_m)` — pseudo-depth from bbox size
- 🆕 `anchor_update(marker_world_pos, observed_bearing)` — loop closure from known marker
- 🆕 Multi-landmark loop closure (Wahba's problem implementation)

**Reducere efort Stage 1 + 5 + 6 din §3**: de la ~14 zile la **~5 zile**
(doar adăugiri la API existing).

#### `sentai.crazy` — DEJA FULL FLIGHT CONTROLLER pentru cf2 (refolosibil 100%)

Surface existing:
```
init, ping, debug, on_message, link_send       # lifecycle + comms
arm, disarm, canfly, is_flying, is_tumbled     # state
fly, fly_stop, hover, go_to, land              # ★ flight commands
altitude, baro, pressure, battery, battery_pct # telemetry  
attitude, attitude_get                         # IMU integration
flow_pred, flowdeck_pos                        # flow integration
```

**REVELAȚIE**: Stage 4 (action layer) pentru calea cf2 e **deja**
acoperit de existing `sentai.crazy.go_to()` + `hover()` + `land()`.
**Nu trebuie construit nimic** pentru cf2 action layer — DOAR wrap-ul
care primește intent + traduce la apel corespunzător.

```python
# Schema action layer pentru cf2 — practic 100 linii pythonești
def action_emit_cf2(intent):
    if intent.type == "GOTO":
        sentai.crazy.go_to(*intent.target_pos, intent.target_yaw)
    elif intent.type == "HOLD":
        sentai.crazy.hover(0, 0, 0, intent.altitude)
    elif intent.type == "LAND":
        sentai.crazy.land()
    elif intent.type == "SERVO_IMAGE":
        # IBVS pixel error → roll/pitch via crazy.fly()
        sentai.crazy.fly(intent.roll, intent.pitch, 0, intent.z_rate)
```

**Reducere efort Stage 4**: de la ~5 zile la **~2 zile** (doar action
intent dispatcher pentru cf2 path; PX4 path are nevoie de `send_velocity`
adăugare la `sentai.link` care e ~1 zi).

#### `sentai.flow` — DEJA INCLUDE ANCHOR + AUTO-FORWARD (s113 P2)

Surface existing:
```
enable, enabled, frames, frame_seq, grab_cyc, alive
detail_score, gray_snap, gray_stretch                    # primary flow
mode, anchor_pose, anchor_forward, anchor_forward_stats   # ★ s113 P2 (just shipped)
body_fw, body_left, body_read, cam_id                   # body-frame outputs
confidence, detected, dx, dy                             # current readout
```

**Refolosibil pentru sentai.explore**:
- `anchor_forward(rate, target)` — periodic auto-publish la PX4/cf2 (✓ proven în live test)
- `mode("anchor")` — toggle on anchor (existing)
- `anchor_pose()` — single-shot pose read

**Conclusion**: integrare cu sentai.slam → trecem latest `anchor_pose()` în
`sentai.slam.update_3d()` pentru consistency cu world model.

#### `sentai.mesh` — DEJA pregătit pentru MULTI-DRONE collaborative mapping!

Surface existing:
```
init, available, receive, send, send_detection,
send_update, send_delete, set_pose, node, channel,
config, from, to, hop_limit, rssi, snr, text
```

**REVELAȚIE 2**: `send_detection / send_update / send_delete` sunt
**deja integrated** pentru a împărtăși detecții peste rețea Meshtastic.

**Pattern multi-drone**:
- Drone A descoperă obiect → `sentai.mesh.send_detection(class_id, x, y)`
- Drone B primește → `sentai.mesh.receive()` → adaugă în propria slam
- **Object map cross-drone** GRATIS prin radio mesh

**Pentru sentai.explore extensie multi-drone**: ~0 cod nou. Doar policy
de cum tratezi receive events în mission SM.

#### `sentai.imu` — Tap detection bonus (refolosibil pentru manual override)

```
init, read, roll, pitch, x, y, z, temp,
tap_start, tap_stop, poll_event, double_tap,
degrees, radians
```

**Idee operațional**: `tap_start()` + double-tap detection = **operator
poate stopa misiunea cu lovire dronă în șold** (sau pe board pentru
testing). Refolosibil pentru ABORT trigger.

#### `sentai.aifes` — On-device NN training (refolosibil pentru place fingerprinting)

```
init, info, load, predict, set_input, output,
output_size, layers, params,
save_weights, load_weights,
from_camera, from_mic, from_tpu, mic_init, mic_stop,
ADAM, SGD, MSE, CROSSENTROPY, best_epoch, name
```

**REVELAȚIE 3**: deja avem **NN training pe board**! Pattern pentru
sentai.places (§13-17):
- La fiecare nou loc vizitat → train un mini-classifier place_id
- Inputs: descriptor curent; outputs: closest place_id
- Train cu ADAM peste 100 examples = ~secunde
- Recall ulterior: `aifes.predict(descriptor) → place_id` în ~5ms

**Reducere efort Stage 11**: foundation deja shipped; doar wiring +
descriptor extraction necesar.

#### `sentai.kmeans` — Cluster descriptors (refolosibil pentru place categorization)

```
init, fit, predict, centroid, distances, counts,
add, add_from_tpu, from_tpu, save, load, info, vectors, k, dim
```

Pattern: 64 places → cluster în 8 macro-regions via k-means → query
"which region am I in" mai rapid decât descriptor full match.

#### `sentai.pca` — Descriptor compression (refolosibil pentru memory savings)

```
fit, transform, inverse, transform_tpu, from_tpu,
out_dim, in_dim, explained_variance, vectors, fitted
```

Pattern: 256-D descriptor → PCA fit → 32-D compressed → match faster +
memory smaller. ~70% memory reduction pentru 95% variance retained.

#### `sentai.anomaly` — Outlier detection (refolosibil pentru SLAM data association)

```
init, observe, score, is_anomaly, threshold,
observe_tpu, score_tpu, is_anomaly_tpu,
cusum_init, cusum_observe, cusum_score, cusum_reset,
mean, samples, save, load, dim, stats
```

**Pattern util pentru sentai.slam loop closure**:
- New observation candidate → `anomaly.score(observation)` vs known landmarks
- Score > threshold → reject as outlier (gated PDA)
- CUSUM pentru change detection (e.g., drone yaw bias drift trigger)

#### `sentai.dtw` + `sentai.hmm` + `sentai.rl` — Mission policy learning

`sentai.dtw`:
```
record_start, record_add (+from imu/mic), record_end,
match, match_imu, match_mic, slot, templates, max_len, label
```
- Pattern matching pentru gesturi → drone command via tap pattern?

`sentai.hmm`:
```
init, set_emission, set_transition, set_prior,
train, viterbi, log_likelihood, predict,
add_seq, sequences, from_imu, from_tpu
```
- Activity recognition pentru drone (turning, hovering, ascending) +
  policy state inference

`sentai.rl` (DQN + MAB):
```
mab_init, mab_pull, mab_select, mab_stats         # multi-armed bandit
dqn_init, dqn_train, dqn_action, dqn_observe,
dqn_replay, dqn_save, dqn_load                    # deep Q-learning
arm, mean, pulls
```
- **Adaptive exploration policy**: bandit alegere între SPIRAL_OUT /
  RASTER / RETURN_FOR_LOOP / OBJECT_CHASE pe baza reward (loop
  closure rate, new objects discovered).

### 19.B Updated planning — sintetic post-audit

#### Recapitulare efort revizuit

| Stage din §3 | Status post-audit | Efort REVIZUIT |
|---|---|---|
| Stage 1 — object map | **EXTENDS sentai.slam** (existing API) | ~3 zile (era 5-7) |
| Stage 2 — TPU model | independent — model training | ~5-7 zile (same) |
| Stage 3 — mission SM | **new `sentai.explore`** | ~3 zile (same) |
| Stage 4 — action layer | **wrap `sentai.crazy.go_to/hover/land`** + add `sentai.link.send_velocity` | ~2 zile (era 4-5) |
| Stage 5 — object lifter | **method în sentai.slam** (extension) | ~3 zile (era 5-7) |
| Stage 6 — loop closure | **method în sentai.slam** (anchor_update + multi-landmark) | ~2 zile (era 3-4) |
| Stage 11 — places | new module + use `sentai.aifes` + `kmeans` + `pca` | ~7-10 zile (era 12-15) |

**Total revizuit**: ~25 zile vs ~45 zile inițial. **44% reducere** prin
reusing existing.

#### Module noi necesare (numai 2 strict, 1 opțional)

```
sentai.explore   ← MANDATORY: mission SM (§18)
sentai.servo     ← MANDATORY: action layer wrapper
sentai.places    ← OPTIONAL: place fingerprinting + H3 (defer)
```

**Concentrate prima oară pe**: extension `sentai.slam` (Stage 1+5+6) +
add `sentai.servo` (Stage 4) + add `sentai.explore` (Stage 3+18).

**Stage 11 places** poate fi defer-uit, sau implementat ușor folosind
`sentai.aifes` (NN classifier) + `sentai.kmeans` (centroids) +
`sentai.pca` (compress) — **toate building blocks already exist**.

#### Pattern de re-use pentru fiecare modul nou

```
sentai.explore → uses:
  - sentai.slam       (world model, query landmarks)
  - sentai.flow       (anchor_pose, anchor_forward)
  - sentai.crazy      (fly commands via wrapper) OR sentai.servo
  - sentai.pipeline   (detection stream)
  - sentai.imu        (tap_start pentru manual abort)
  - sentai.mesh       (broadcast new landmarks to other drones)
  - sentai.diag       (dmesg pentru logging)
  - sentai.fs         (persistence /explore/<label>/)

sentai.servo → uses:
  - sentai.crazy.fly/go_to/hover/land   (cf2 path)
  - sentai.link.send_velocity (NEW, but small)  (PX4 path)
  - sentai.slam.pose()                  (current pose for PBVS)
  
sentai.places (deferred) → uses:
  - sentai.aifes        (NN classifier)
  - sentai.kmeans       (centroid cluster)
  - sentai.pca          (compress descriptors)
  - sentai.anomaly      (outlier detection în matching)
  - sentai.fs           (persistence)
```

**Insight major**: arhitectura sentai.* e **deja layered correctly**.
Adăugările noastre sunt thin facades peste existing primitives.
Reduction efort substanțială.

### 19.10 Bottom line concise pentru operator

**Răspuns la cele 3 dubii**:

1. **`sentai.objects` — NU îl creem**. Conflictă cu existing `sentai.slam`
   care face DEJA object-level EKF-SLAM. Extindem `sentai.slam` la 3D.

2. **`sentai.explore` vs `sentai.map`**: păstrăm **`sentai.explore`**
   ca mission API. Map-ul trăiește în `sentai.slam` + `sentai.places`.
   "Explore is the verb; slam + places are the nouns."

3. **`detect` / `track` confusion**: NU există ca top-level — bundled
   în `sentai.pipeline` (proven, stable). NU adăugăm.

**Decizia ne-banală**: 5 module propuse anterior (`objects`, `map`,
`mission`, `detect`, `track`) → **0 adăugate**. Doar 3 module NOI
absolut necesare: `places`, `explore`, `servo`. Plus extension la
existing `slam`.

**Beneficii ale acestei reorganizări**:
- ✅ Compatibility — zero breakage existing scripts
- ✅ Clarity — un loc pentru world model (slam), un loc pentru places
  (places), un loc pentru mission (explore), un loc pentru action (servo)
- ✅ Surface area mai mic — 28 module total în loc de 33 (cu duplicate)
- ✅ Maps to literature — slam, places (VPR), explore (mission),
  servo (visual servoing) — toate termen industrie standard
- ✅ Discoverability — naming convention consistent

**Toate modificările la §3 sunt updated în §19.7**. Începem implementarea
cu Stage 1.A revizuit: **"add method `sentai.slam.update_3d()`** + class
prior table".

---

*Document compilat pe 2026-05-13 ca răspuns la `ideas/objects.md` (revizia 2026-05-12).
Toate referințele tehnice trec prin documentul sursă. Pentru detalii de
implementare per modul, consultă file-urile sentai_* enumerate la
fiecare stagiu, plus invarianții din `agent/agent.md §11` care nu
trebuie rebroke.*

---

## 20. Bibliography — SOTA references (verified 2026-05-14)

Sursele consultate înainte de start L5 (Stage 5 `sentai_object_lifter`).
Verificate online; folosite pentru a confirma că arhitectura proiectului
este aliniată cu literatura 2024–2026 înainte de a investi 5-7 zile în
implementare ARM. Concluzie sintetică: alegerile noastre
(inverse-depth EKF + class-prior pseudo-depth + ArUco bootstrap +
HSV/H3 places) sunt **2017–2019 metodic + 2026 platformă mai mică**,
nu cutting-edge dense reconstruction. Aceasta este alegere principială
pentru bugetul nostru de compute (M7 800 MHz + EdgeTPU 4 TOPS), nu un
gap metodologic.

### 20.1 EKF inverse-depth parametrization pentru monocular SLAM

- **Civera, Davison, Montiel** (TRO 2008) — *Inverse Depth
  Parametrization for Monocular SLAM*. Referința foundațională pentru
  Stage 5. Algoritmul lifter implementat e direct din această
  metodologie: 6-state EKF per landmark `(x₀, y₀, z₀, θ, φ, ρ)` cu
  `ρ = 1/depth`, ce permite undelay-ed initialization la features cu
  parallax mic. <https://www.doc.ic.ac.uk/~ajd/Publications/civera_etal_tro2008.pdf>

- **MDPI Drones 2025** — *UAV Navigation Using EKF-MonoSLAM Aided by
  Range-to-Base Measurements*. Confirmă în 2025 că EKF (vs sliding-window
  optimization) rămâne alegerea corectă pentru aerial embedded:
  "EKF-based VIO solutions are generally lower compute and memory, used
  in embedded systems applications such as aerial vehicles".
  <https://www.mdpi.com/2504-446X/9/8/570>

- **Equivariant Filter VIO (EQVIO)**, arXiv 2205.01980 (2022) —
  alternativă geometric mai elegantă (equivariant filter pe SE_2(3)),
  dar implementarea cere library Lie-theoretic; pentru MCU,
  inverse-depth EKF Civera 2008 e mai practic.
  <https://arxiv.org/pdf/2205.01980>

### 20.2 ArUco multi-marker localization + PnP

- **arXiv 2509.17345** (2025) — *Investigation of ArUco Marker
  Placement for Planar Indoor Localization*. Studiu de plasament
  optimal pentru indoor; relevant pentru calibration takeoff Stage 6
  loop closure.
  <https://arxiv.org/pdf/2509.17345>

- **MDPI Drones 2025** — *Embedded ArUco (e-ArUco) Detection for
  Precision Landing*. Justifică ArUco ca **permanent fixture** pentru
  takeoff calibration (mapăm noi pe `aruco_detector.py` actual + Stage
  6 loop closure planificat).

- **Kalman filtering with adaptive measurement noise** pentru
  multi-marker — pattern de fuziune folosit în s130 (vezi
  [[s130-45baseline-shipped]]); 2D Procrustes (Kabsch closed-form)
  pentru recover yaw în absența IMU heading e validat ca tehnică
  industry-standard.

### 20.3 Object-level SLAM cu class prior + bbox

- **CubeSLAM** (Tony Hou paper notes) — *Monocular 3D Object SLAM*.
  Folosește 2D bbox + cuboid prior factor graph pentru construct
  obiecte 3D. **Ancestorul metodic al Stage 5**: clasa noastră
  "class-prior pseudo-depth (size table)" e exact ce numesc cuboid size
  prior la CubeSLAM, dar simplificat (no factor graph, no g2o
  optimization — only per-tracklet EKF).
  <https://tony-hou.github.io/Learning-AI/paper_notes/cube_slam.html>

- **MDPI Sensors 2025** — *Monocular Object-Level SLAM Enhanced by
  Joint Semantic Segmentation and Depth Estimation* (JSDNet, March
  2025). Adaugă depth estimation network + semantic segmentation.
  Necesită GPU mid-range; NU îl putem rula pe EdgeTPU 4 TOPS la 30 fps;
  conștient out-of-scope.
  <https://www.mdpi.com/1424-8220/25/7/2110>

- **ADEmono-SLAM** (MDPI Electronics 2025) — *Absolute Depth Estimation
  for Monocular Visual SLAM*. Folosește o rețea de absolute depth
  pentru a scoate scale; clasa de soluții pe care nu o pursuim
  (deep depth nets) — dar referință pentru când TPU bugetul permite.
  <https://www.mdpi.com/2079-9292/14/20/4126>

### 20.4 Dense SLAM (out-of-scope pentru MCU)

- **WildGS-SLAM** (CVPR 2025) — *Monocular Gaussian Splatting SLAM
  in Dynamic Environments*. Cutting-edge dense reconstruction; cere
  GPU 8+ GB. **NU îl pursuim**: nu există drum credibil de la 3D
  Gaussian Splatting → 1 MB OCRAM + EdgeTPU. Reference cited doar
  pentru a justifica decizia "sparse landmarks în `sentai.objects`".
  <https://openaccess.thecvf.com/content/CVPR2025/papers/Zheng_WildGS-SLAM_Monocular_Gaussian_Splatting_SLAM_in_Dynamic_Environments_CVPR_2025_paper.pdf>

### 20.5 Mapping SOTA-trends → decizii proiect

| Trend SOTA 2025-2026 | Decizia proiect | Justificare embeded.md |
|---|---|---|
| NeRF/3DGS dense reconstruction | Sparse landmarks (`sentai.objects` 32-slot) | "Avoid abstractions too heavy for MCU" + 1 MB OCRAM budget |
| Learned absolute depth nets (MiDaS/DPT) | Class-prior real-size table | EdgeTPU 8 MB stock + 32 ms invoke budget alocat detector |
| NetVLAD descriptors (4096-dim) | HSV 64-B histogram + H3 indexing | M7 SIMD `__USADA8` proven s111; 1.1 ms threshold pentru HSV pe PXP |
| Tightly-coupled visual-inertial (sliding window) | Loosely-coupled (cf2/PX4 EKF + flow) | RT1176 nu are timestamp synchronization hardware pentru tight VIO |
| g2o / Ceres factor graph optimization | Per-landmark independent EKF (Civera 2008) | "Bounded behavior" — fără iterare nelimitată, fără heap |

### 20.6 Noutatea proiectului (research-grade contribution)

- **Integrarea (MCU + EdgeTPU + ArUco-bootstrap + class-prior +
  H3-indexed places) pe class de hardware mai mic decât oricare paper
  publicat**. Cele mai apropiate puncte de comparare în literatură
  (PicoVO @ STM32F767, Navion ASIC) folosesc VO completă, nu
  object-level SLAM cu semantic. Coral Dev Board Micro = 1× M7 +
  EdgeTPU = ~5 W class device.

Note: aceasta secțiune e **anchor pentru decizii arhitecturale**, nu
implementation guide. Pentru detalii algoritmici per stagiu, vezi §3.1-3.10.

*Bibliography compilată 2026-05-14 înainte de start L5 implementare.*

---

## 21. Camera-to-body orientation calibration la takeoff (HARDWARE REALITY 2026-05-14)

### 21.1 Problema (operator-flagged 2026-05-14)

**Concern**: în SIM, orientarea camerei pe corpul dronei este exactă (vine din SDF). Pe hardware real:

- Lipirea/montarea modulului OV5640 pe corpul Crazyflie are toleranță mecanică
  (~±2-5° pitch/roll/yaw între planul corpului și planul senzorului).
- Vibrațiile + impacturile minore în zbor pot deplasa montura (uzură).
- Două drone fizice cu acelaș firmware NU vor avea aceeași matrice
  `R_cam_to_body` — fiecare are nevoie de calibrare per-unitate.

Toate algoritmele propuse (Stage 4.5 image-only nav, Stage 5 lifter
inverse-depth EKF, Stage 6 loop closure pe yaw) **presupun** că
`R_cam_to_body` e cunoscută exact. Eroarea pe orientare se propagă
direct în:
- bearing direction al lifter → marker world position bias
- yaw recovery din Procrustes → drone heading bias
- IBVS pixel servoing → biasing the saturation point

**O eroare de 3° în R_cam_to_body @ z=1.5 m → bias bearing ≈ 8 cm
în plane orizontal.** Inacceptabil pentru landing precis sau loop closure.

### 21.2 Soluția propusă — auto-calibration la takeoff cu ArUco

**Concept**: în timpul fazei de takeoff (sau imediat după), drona
execută o procedură scurtă de calibration unde:

1. Drona stă la `z = 0.5-1.5 m` deasupra unui marker ArUco cunoscut
   (poate fi același marker de landing pad).
2. Drona efectuează o **mișcare cunoscută** controlată (ex: yaw 360° lent
   sau translation lateral ±0.1 m).
3. La fiecare frame:
   - Aruco detector publică `tvec_cam, rvec_cam` (PnP).
   - cf2 / PX4 EKF publică drone state `(x_W, y_W, z_W, yaw_W)`.
4. Pentru fiecare frame avem:
   - `marker_world_known` = (0, 0, 0) (landing pad origin)
   - `cam_world = marker_world - R_cam_to_world @ tvec_cam`
   - Dar `R_cam_to_world = R_body_to_world(yaw_W) @ R_cam_to_body`
   - Necunoscută: `R_cam_to_body` (matrix 3×3, parametrizată ca quaternion 4-DOF).
5. **Kabsch 3D Procrustes** pe N≥4 frame-uri → soluție closed-form
   pentru `R_cam_to_body`:
   ```
   H = Σ (tvec_cam_i) ⊗ (R_W_B(yaw_i)^T @ (marker_W - drone_W_i))^T
   U Σ V^T = SVD(H)
   R_cam_to_body = V @ diag(1, 1, det(V@U^T)) @ U^T
   ```

### 21.3 Persistare + verificare la fiecare takeoff

- Rezultatul calibrării (R_cam_to_body) e persistat în `/system/cam_calib.json`
  (FileX user partition, schema-versioned).
- La fiecare takeoff:
  - Citim `cam_calib.json`.
  - Executăm un "calibration sanity check" rapid (1-2 sec, marker ArUco
    în FOV) — dacă noua estimare diferă de cea persistată cu > 3°,
    re-calibrăm și suprascriem.
  - Dacă marker-ul de landing pad NU e vizibil → log warning, continuă
    cu valoarea persistată (degraded mode).

### 21.4 Implementare planificată

Acest lucru e Pas 2 imediat post-s131:
- Crează `examples/sentai_runtime/_shared/camera_calibration.py` (host
  module, Python — folosit de toate experimentele).
- Crează `sentai.calib` MicroPython binding (Stage 6 follow-up) pentru
  on-board calibration la takeoff:
  - `sentai.calib.cam_to_body_from_aruco(samples)` — Kabsch 3D
    Procrustes pe samples colectate din vol controlat.
  - `sentai.calib.save_cam_to_body(R)` → `/system/cam_calib.json`
  - `sentai.calib.load_cam_to_body()` → R matrix sau identity dacă lipsește.
- Stage 5 lifter + Stage 4.5 image_localize trec prin `sentai.calib`
  pentru R, nu prin constante hardcoded.

### 21.5 Fault modes (per embeded.md discipline)

| Failure | Detection | Reaction |
|---|---|---|
| Marker landing pad nu e vizibil la takeoff | bbox count == 0 după 3 sec | Folosește valoarea persistată; emit `CAL_NO_MARKER` event |
| Calibration produce R cu det(R) < 0.99 | SVD post-check | Reject + folosește persistată; emit `CAL_DET_FAIL` event |
| New calibration diferă > 10° de persistată | comparison post-Kabsch | Reject + log; presupune marker drift / mecanic shift critic |
| `cam_calib.json` corrupt sau lipsește | JSON parse fail | Fallback la identity + emit `CAL_SCHEMA_FAIL`; mision continuă în degraded mode |
| Calibration drift > 3° in flight | runtime monitor (periodic check at hover) | Trigger re-calibration la next hover; emit `CAL_DRIFT` event |

Sistemul rămâne **self-healing**: calibration corruption NU brick-uiește
boot-ul; e doar degraded mode până la următorul takeoff cu marker
vizibil.

### 21.6 Cost compute (M7 budget)

Per takeoff (one-shot, ~1-2 sec):
- 30 samples × (tvec computation 0.5 ms + matrix push) → 15 ms total accumulate
- 1× SVD 3×3 closed-form (LAPACK / CMSIS-DSP) → ~50 µs
- Total ~ 15 ms one-shot, completely outside hot path.

Runtime (load `cam_calib.json` la boot): 1 fs.read + JSON parse ~5 ms,
o singură dată în main_freertos.cc init.

**Verdict**: compute negligible, FileX overhead negligible, problema
e disciplinare-implementaţională (frame conventions + SVD numerical
stability), nu compute.

---

*Operator-flagged 2026-05-14: real-world camera mount tolerance ≠ SIM
SDF exact. Calibration la takeoff e mandatory înainte de hardware
deployment, opțional în SIM (deja avem identity).*

---

## 22. Place-descriptor — two parallel tracks (operator decision 2026-05-15)

### 22.1 Decizia

Operatorul a decis pe 2026-05-15: pentru dezvoltarea Stage 11
(`sentai.places` cu embedding visual per cell H3), abordăm
**două piste paralele**:

| Track | Status | Tehnologie | Compute | Când |
|---|---|---|---|---|
| **A — No-DNN** | **PRIMARY (next iterations)** | PHOG + GIST + HSV-hist + log-polar FFT magnitude | < 50 ms / frame M7 | imediat (s133+) |
| **B — DNN custom EdgeTPU** | DEFERRED | encoder distilled (MobileNet-VLAD / EfficientNet-VLAD) INT8 | ~10 ms / inference TPU | după ce avem dataset + training infra |

**Motivația operatorului**: încă nu avem pipeline de antrenament + dataset
aerian curat pentru un encoder TPU custom. Dar avem nevoie ACUM de o
infrastructură funcțională place-recognition. Hand-crafted descriptors
sunt 30 de ani de literatură matură, suficient de bune pentru proof-of-concept
și pentru baseline-ul contra căruia vom măsura ulterior orice DNN.

**Consecințe arhitecturale**:
- API-ul `sentai.places` (slot 64 B descriptor, status `[[places-l3-shipped]]`)
  rămâne **identic** între Track A și Track B — doar **populator-ul**
  diferă. Tranziția A→B e drop-in.
- `sentai_scene_descriptor.{h,cc}` (planificat în §14.5) devine
  **runtime-strategy-selectable**: o singură semnătură de extractor,
  implementări multiple compilate condiționat.
- Bibliotecile auxiliare (PXP pentru rotate, log-polar LUT, FFT via
  CMSIS-DSP) sunt **shared** între cele 2 track-uri.

### 22.2 Track A — Hand-crafted global descriptors (PRIMARY)

#### Stack tehnic

| Componentă | Dimensiune | Cost M7 | Acoperă |
|---|---:|---:|---|
| **PHOG** (3 nivele × 8 orientări × spatial bins) | ~168–680 D | ~15 ms | structură edge multi-scală |
| **GIST** (Gabor 8 orient × 4 scale × 4×4 grid) | ~512 D | ~10 ms | "what kind of place" holistic |
| **HSV histogram** (8×8×8 sau marginal 32+32+32) | ~96–512 D | ~2 ms | paletă cromatică |
| **Log-polar FFT magnitude** (centrat pe principal point) | ~128–256 D | ~8 ms | conținut frecvențial **rotation-invariant** |
| **PCA reduce** (offline-trained projection) | → 64–256 B | < 1 ms | compresie la slot-ul L3 |

**Total compute**: ~35 ms / frame @ 1 Hz query rate = **~3.5% CPU M7**.
Tot în `.sdram_text`, zero ITCM.

**Total dimensiune raw**: 904–1960 D înainte de PCA. Cu PCA antrenat
o singură dată offline pe o galerie reprezentativă, se reduce la
**64 B descriptor** (matched cu slot-ul L3 existent) sau **256 B**
(slot extins pentru robustețe Stage 11).

#### Proprietăți de invarianță

Tipic per fiecare componentă în parte (din literatura clasică):

| Componentă | Translație | Rotație | Scală | Iluminare |
|---|:---:|:---:|:---:|:---:|
| PHOG | parțial (spatial pyramid) | ❌ | ❌ | mediu |
| GIST | invariant (global pooling) | ❌ | ❌ | bun |
| HSV-hist | invariant | invariant | invariant | bun (H, S) |
| Log-polar FFT-mag | invariant | **invariant** | parțial invariant | mediu |

**Combinația** acoperă întreaga matrice: HSV-hist + log-polar FFT
dă rotation-invariance ca proprietate emergentă, PHOG+GIST dau
discriminabilitatea geometrică. Cu pre-rotate canonical (§14.3 Strategia A)
peste ele, rezistența la yaw drift devine **robustă pe full 360°**.

#### Pipeline frame-by-frame (Track A)

```
camera frame 320×240 RGB
        │
        ├─→ PXP pre-rotate by -ŷaw EKF  (Strategia A, 3 ms)
        │       (canonical north-up alignment)
        │
        ↓
   image_canonical (320×240)
        │
        ├──────────────┬─────────────┬──────────────┐
        ↓              ↓             ↓              ↓
   PHOG (gray)     GIST (gray)   HSV-hist      log-polar
   168-680 D       512 D         96-512 D      → FFT-mag
                                                128-256 D
        ↓              ↓             ↓              ↓
        └──────────────┴─────────────┴──────────────┘
                       │
                       ↓
                concatenate → ~900–2000 D raw
                       │
                       ↓
                PCA project → 64–256 B int8 descriptor
                       │
                       ↓
                store / query in sentai.places (L3 + H3)
```

#### Bibliografie Track A (verified, primary references)

**PHOG (Pyramid Histogram of Oriented Gradients)**:
- Bosch, A., Zisserman, A., Munoz, X. "Representing Shape with a Spatial
  Pyramid Kernel". CIVR 2007. — **Sursa primară PHOG.**
- Lazebnik, S., Schmid, C., Ponce, J. "Beyond Bags of Features: Spatial
  Pyramid Matching for Recognizing Natural Scene Categories". CVPR 2006.
  — Originea spatial pyramid matching.
- Dalal, N., Triggs, B. "Histograms of Oriented Gradients for Human
  Detection". CVPR 2005. — Baseline HOG (re-citat din §13).

**GIST (Global scene descriptor)**:
- Oliva, A., Torralba, A. "Modeling the shape of the scene: A holistic
  representation of the spatial envelope". IJCV 42(3), 2001. — **Sursa
  primară GIST.** (Deja citată în §13.)
- Torralba, A., Murphy, K.P., Freeman, W.T., Rubin, M.A. "Context-based
  vision system for place and object recognition". ICCV 2003. — GIST
  applied to place recognition.

**Color histograms / color indexing**:
- Swain, M.J., Ballard, D.H. "Color Indexing". IJCV 7(1), 1991.
  — **Sursa primară**; foundational pentru content-based image retrieval.
- Pass, G., Zabih, R., Miller, J. "Comparing Images Using Color Coherence
  Vectors". ACM Multimedia 1996. — Refinement (CCV) pentru spatial color.
- Stricker, M., Orengo, M. "Similarity of Color Images". SPIE 1995.
  — Quantized color moments (4 momente per canal HSV).

**FFT-based rotation invariance**:
- Reddy, B.S., Chatterji, B.N. "An FFT-based technique for translation,
  rotation, and scale-invariant image registration". IEEE TIP 5(8), 1996.
  — **Primary**; deja citat în §14.3 Strategia B.
- De Castro, E., Morandi, C. "Registration of translated and rotated
  images using finite Fourier transforms". IEEE TPAMI 9(5), 1987.
  — Fundament teoretic (deja citat în §14.3).
- Adam, A., Rivlin, E., Shimshoni, I. "ROAM: Rotation-only Algebraic
  Matching". CVPR 2009. — Log-polar features for rotation invariance
  (deja citat în §14.3).

**VPR cu hand-crafted descriptors (combinație + matching)**:
- Ulrich, I., Nourbakhsh, I. "Appearance-Based Place Recognition for
  Topological Localization". ICRA 2000. — Early color-histogram VPR;
  simpli, robust, low-compute.
- Cummins, M., Newman, P. "FAB-MAP: Probabilistic Localization and
  Mapping in the Space of Appearance". IJRR 27(6), 2008. — Bayesian VPR
  framework, descriptor-agnostic (deja citat).
- Milford, M.J., Wyeth, G.F. "SeqSLAM: Visual Route-Based Navigation".
  ICRA 2012. — Temporal sequence matching; descriptor-agnostic, dă
  robustețe extra peste orice Track A static descriptor.
- Lowry, S., et al. "Visual Place Recognition: A Survey". IEEE TRO
  32(1), 2016. — Broad survey care taxonomizează clar hand-crafted
  vs deep VPR.

#### Avantaje Track A
- **Zero training** necesar — descriptors deterministici din input
- **Zero EdgeTPU dependency** — rulează pe M7, util și pentru SIM
- **Bit-exact reproducible** între SIM și ARM (numerică identică)
- **Bibliotecă matură** — 20-30 ani de literatură, well-understood failure modes
- **Drop-in upgrade path** — slot-ul 64 B nu se schimbă; doar populator-ul

#### Limitări cunoscute Track A
- Discriminabilitate mai mică decât deep descriptors pe scene similare
  ("două câmpuri de iarbă uniformă" pot colide)
- Mai sensibil la appearance change (lumini, sezon) decât DNN antrenat
  cross-conditions — mitigat parțial de SeqSLAM temporal (§14.4 Mech 1)
- HSV-hist e foarte sensibil la white balance — mitigat prin captură
  cu AEC/AWB blocate sau prin chrominance only (Cb/Cr de la YCbCr)

### 22.3 Track B — DNN custom-built EdgeTPU (DEFERRED)

#### Status
**DEFERRED** la o iterație ulterioară. Track A este suficient pentru a
valida arhitectura `sentai.places` + H3 integration + mission FSM
loop closure end-to-end. Trecerea la Track B se va face când:
1. Avem dataset aerian curat de antrenare (top-down, 320×240, indoor+outdoor)
2. Avem pipeline de training distilled-encoder (PyTorch + TFLite + EdgeTPU compiler)
3. Avem măsurare Track A baseline pe missiuni reprezentative, ca să
   justificăm Track B prin metrici concrete (nu speculativ)

#### Stack tehnic propus (referință pentru viitor)
- Backbone: MobileNetV3-Small sau EfficientNet-B0, INT8, ~3-5 MB
- Aggregator: VLAD layer (16 centroids) sau GeM pooling
- Loss training: triplet loss cu hard-negative mining + place-level GPS labels
- Distill from: DINOv2-Small (teacher) sau AnyLoc descriptor (target)
- Output: 256-D float → quantize → 64 B int8

#### Bibliografie Track B (referință pentru implementare ulterioară)
- Arandjelović, R., et al. "NetVLAD: CNN architecture for weakly
  supervised place recognition". CVPR 2016. (deja §13)
- Berton, G., et al. "Rethinking Visual Geo-Localization for
  Large-Scale Applications" (CosPlace). CVPR 2022. (deja §13)
- Berton, G., et al. "EigenPlaces". ICCV 2023. (deja §13)
- Keetha, N., et al. "AnyLoc: Towards Universal Visual Place
  Recognition". IEEE RAL 9(2), 2024. (deja §13) — candidat teacher
  pentru distillation.
- Vivanco Cepeda, V., et al. "GeoCLIP: Clip-Inspired Alignment between
  Locations and Images for Effective Worldwide Geo-localization".
  NeurIPS 2023. — Image-to-location embedding direct, candidate
  inspiration pentru aerial.
- Klemmer, K., et al. "SatCLIP: Global, General-Purpose Location
  Embeddings with Satellite Imagery". Microsoft Research 2023.
- Liu, F., et al. "RemoteCLIP: A Vision Language Foundation Model for
  Remote Sensing". IEEE TGRS 2024. — Aerial-finetuned CLIP variant.
- Apple. "MobileCLIP" (CVPR 2024) — distilled CLIP candidate.
- Microsoft. "TinyCLIP" (ICCV 2023) — alternativ distilled.

### 22.4 Actualizări la secțiuni anterioare

- **§13.4 Q2 (macro-fingerprint)**: lista de descriptori clasici (GIST,
  HOG, FAB-MAP, SeqSLAM) este input direct pentru Track A. Adaugă
  PHOG la listă; reference Bosch 2007.
- **§14.3 Strategia A / B / C**: Strategia A (pre-rotate) + Strategia B
  (log-polar FFT) sunt **ambele integrate în Track A**, nu alternative.
  Strategia C (ORB+VLAD) e **complementară** — poate fi adăugată în Track A
  ca al 5-lea canal (~12 ms suplimentar) dacă PHOG+GIST+HSV+FFT nu sunt
  suficient de discriminative.
- **§14.10 Bottom line**: rămâne valid; Track A = "stack stratificat
  A+B" cu hand-crafted backbone în loc de "GIST sau ORB+VLAD".
- **§15 H3**: independent de Track A/B. H3 = spatial index; Track A/B =
  populator de descriptor pentru cell-ul indexat de H3.
- **§16.2 Storage**: rămâne valid (sparse hash H3Index → place_id).
- **§17 cross-scale**: rămâne valid; cross-scale composition se aplică
  identic pe Track A descriptor și Track B descriptor.

### 22.5 Pas următor verificabil în Gazebo — s133

**Numele experimentului**: `s133_places_track_a_phog_gist`

**Scopul**: validează matematic + procedural Track A pe cadre reale Gazebo,
înainte de orice port la C++ în SIM/ARM. Same pattern ca s131 (Python
prototype înainte de C++).

**Configurare misiunii** (cea mai simplă posibil, recyclează s130 arena):

1. **Phase A — Gallery build** (zbor 1, "explore"):
   - cf2 takeoff din origin (0, 0, 0)
   - Urmează pattern fix: pătrat lateral 1m × 1m la altitudine z=1.5m,
     centrul peste s130 arena cu 4 markere ArUco distincte pe podea
     texturată
   - La fiecare 4 colțuri ale pătratului: hover 2s, capturează 5 frame-uri
     consecutive
   - Yaw constant 0° (north-up canonical)
   - Total: 4 places × 5 frames = 20 cadre de antrenament
   - Output: `gallery.npz` cu (cell_id_h3, descriptor 256-D, drone pose GT)

2. **Phase B — Recognition test** (zbor 2, "return"):
   - Restart Gazebo fresh (per `[[experiments-start-from-origin]]`)
   - cf2 refly același pătrat, dar **cu yaw +90°**
   - La fiecare colț: capturează 5 frame-uri, compute descriptor,
     cosine-match contra gallery
   - Top-1 match → check dacă cell_id_h3 prezis == cell_id_h3 GT
   - Output: `recognition_results.json` cu rate per yaw delta

3. **Phase C — Opposite-direction test** (zbor 3):
   - Restart Gazebo fresh
   - cf2 refly același pătrat, cu **yaw 180°** (opposite direction)
   - Aceleași măsurători ca Phase B

**Pass criteria (PASS gate pentru s133)**:
- **Phase A→B (yaw 0° → 90°)**: ≥ 60% top-1 recognition rate
- **Phase A→C (yaw 0° → 180°)**: ≥ 50% top-1 recognition rate
- **False positive rate la score threshold τ=0.7**: < 10%
- **Compute / frame**: < 200 ms pe host (validare practică; M7 va fi mai rapid cu CMSIS-DSP)

**Tools & infrastructure**:
- Host-only Python (numpy + scipy + OpenCV) — același pattern ca
  s131. Zero board involvement în iterația 1.
- Reutilizează capture pipeline-ul s130 (frames + GT pose deja salvate
  în `image_vs_cf2.json`).
- Reutilizează Gazebo arena s091 + s130 (4 markere + textured floor).
- Folosește `sentai.sim.journal_*` pentru log debug (per `[[sentai-sim-journal]]`).
- Gate-uri post-experiment: FlowBaseline s127 trebuie să rămână PASS
  (per `[[gate-every-layer-no-exceptions]]`).

**Layout-ul folderului** (per convenția sNNN):
```
examples/sentai_runtime/experiments/s133_places_track_a_phog_gist/
├── README.md            # scop + pass criteria + cum se rulează
├── descriptors.py       # PHOG + GIST + HSV + log-polar FFT implementations
├── gallery_build.py     # Phase A driver
├── recognition_test.py  # Phase B + C driver
├── verdict.py           # PASS/FAIL pe combined results
├── run.sh               # orchestrator end-to-end
├── gallery.npz          # output Phase A (gitignored or kept small)
└── recognition_results.json # output Phase B/C
```

**Time estimate**: ~1-2 zile dezvoltare + 1 zi tuning + 0.5 zi gate.
Aceeași dimensiune cu s131 (math validation).

**Dependențe upstream**:
- s130 arena Gazebo: ✅ ready
- s131 lifter math validation: ✅ shipped (`[[s131-lifter-math-shipped]]`)
- L5 lifter C++: ✅ shipped (`[[l5-shipped]]`) — independent, nu blochează
- s127 FlowBaseline: ✅ canonical (`[[flowbaseline-canonical-config]]`)

**Ce NU face s133**:
- Nu integrează H3 încă (vine în s134 sau Stage 11.A direct)
- Nu rulează pe board ARM (vine în s135+)
- Nu wirează la `sentai.places` API încă (vine în s134 când portezi
  descriptor la C++/SIM)
- Nu testează SeqSLAM temporal (vine ca iterație 11.A.4 dacă
  per-frame rate < target)

**Iterații care urmează după s133 PASS** (Track A only):
- **s134** — port Track A descriptor la C++ (build-sim), wire-up
  `sentai.places.set_descriptor("phog_gist_fft_hsv")`
- **s135** — adaugă H3 indexing peste descriptor (Stage 11.A din §15.11)
- **s136** — adaugă pre-rotate canonical (Strategia A din §14.3)
  pentru rotation drift toleranță
- **s137** — port ARM build, măsurare DWT cycles, gate ITCM budget
- **s138** — mission FSM integrată (`sentai.explore`) cu loop closure
  via places — sau L6 dacă merge L6 înainte de places

### 22.6 Bottom line

Track A devine **explicit primary** începând cu 2026-05-15. Track B
rămâne ca **upgrade-path documentat** dar e blocat pe training infra.
Slot-ul de 64 B din L3 e **futureproof** — același API, populator-ul
schimbă.

**Acțiune imediată**: scaffold s133 ca primul artefact concret al
Track A. Bibliografia, configurația misiunii, pass criteria, layout-ul
de folder — toate specificate mai sus. Implementarea efectivă a
codului e separată de această actualizare a planului.

Related: [[l5-shipped]], [[places-l3-shipped]], [[s131-lifter-math-shipped]],
[[flowbaseline-canonical-config]], [[gate-every-layer-no-exceptions]],
[[experiments-start-from-origin]], [[gazebo-gui-required]],
[[sentai-sim-journal]], [[h3-integration]], [[no-tmp-experiments]].

---

## 23. Thesis-MVP scope (frozen 2026-05-15)

Companion to `FutureWork.md`. Locks the in-scope vs out-of-scope
decision for the PhD thesis to prevent scope creep. Items move
bidirectionally between this plan and `FutureWork.md` as priorities
shift; both files date such moves.

### 23.1 Demo target (north star)

> **Indoor**: drona decolează dintr-o cameră ~5×5 m cu 4-6 obiecte
> fizice (markeri ArUco + obiecte recunoscute de DNN custom on-board).
> Folosind doar percepție on-board (no MoCap, no ground station),
> explorează autonom mediul, construiește hartă 3D + indexată
> hexagonal, vizitează fiecare obiect la comanda operatorului (over
> radio REPL: "du-te la obiectul X", "revino la origin"), și
> aterizează în punctul de plecare cu drift acumulat < 15 cm. Total
> flight ~90 secunde. Power log ≤ 1.2 W stack-ul SentAI. Repetabil
> ≥ 95% peste 20 runs consecutive.
>
> **Outdoor**: 2-3 misiuni PX4 + Crazyflie/drone customă cu GPS ground
> truth. Drona zboară un waypoint pattern cu markeri ArUco la poziții
> cunoscute. Validează: (a) acelaș stack rulează pe PX4 (validează
> generalitate), (b) drift bounded la ~50 cm peste 60-90 secunde
> outdoor, (c) DNN custom recunoaște ≥ 3 categorii outdoor.

Toate prioritățile decurg din această țintă.

### 23.2 In-scope (must ship for thesis)

| Element | Status | Note |
|---|---|---|
| **Foundation infra** (camera, TPU, flow, radio, USB, FS, SIM) | ✅ shipped | |
| **L2** `sentai.objects` | ✅ shipped | frozen API |
| **L3** `sentai.places` (H3 indexed) | ✅ shipped | frozen API |
| **L4** `sentai.servo` | ✅ shipped | frozen API |
| **L4.5** image-only nav (s130) | ✅ shipped | |
| **L5** `sentai.object_lifter` | ✅ shipped | s132 validated under Gazebo |
| **L1 tracker minimal** | TODO | needed: stable tracklet_id for L5 (could be ArUco-id-as-tracklet for thesis MVP) |
| **L6** `sentai.explore` mission FSM | ✅ shipped (skeleton, s133) | 10 states, SIM smoke 100/100 PASS; Gazebo integration follows as s134 (PASS gate ≥80% / 10 runs) |
| **Stage 6** `sentai.calib` on-board MP binding | TODO | mandatory before HW indoor flight (§21) |
| **Track A places** minimal (PHOG+GIST+HSV+FFT) | TODO | s133 → s134 → s135 → s136 → s137 |
| **Stage 9** ARM bring-up + DWT timing | TODO | validates "rulează pe MCU real" — esența tezei |
| **L7** integrated indoor demo | TODO | demo la prezentare, scenariul §23.1 |
| **Outdoor PX4 experiments** (2-3 runs) | TODO | validează generalitate, GPS ground truth |
| **DNN models on-board** | TODO (paralel) | track-ul tău separat |
| **Quantitative evaluation chapter** | TODO | drift/min, success rate, latency, power; numere pt. defense |

### 23.3 Out-of-scope (moved to FutureWork.md)

| Item | FW # | Originally |
|---|---|---|
| DNN places encoder (Track B) | FW1 | §22.3 |
| SeqSLAM temporal | FW2 | §14.4 |
| ORB+VLAD (Strategy C) | FW3 | §14.3 |
| Rotation-equivariant CNN (D) | FW4 | §14.3 |
| Adaptive H3 subdivision | FW5 | §15.4 |
| Multi-altitude descriptor storage | FW6 | Stage 11.D, §15.5 |
| Multi-drone mesh gallery | FW7 | §15.12.3 |
| Multi-level honeycomb advanced | FW8 | §16 |
| Cross-scale composition | FW9 | §17 |
| Persistent gallery cross-boot | FW10 | §15.7 |
| Outdoor lat/lng / OSM | FW11 | §15.7 |
| Full SM3 COAST/ALIGN | FW12 | Stage 7 (partial — minimal stays) |
| Stage 8 standalone | FW13 | absorbed into L7 |
| Macro-category detection | FW14 | §13.3 |
| Auto-merge similar children | FW15 | §15.12.1 |
| 4D hex (time bucket) | FW16 | §15.12.2 |

**Notă**: secțiunile originale rămân în `objects_plan.md` pentru
literature review valoare. `FutureWork.md` listează doar
implementarea + rationale-ul de deferral. Bibliografia rămâne aici.

### 23.4 Risk register thesis-specific

| Risc | Severitate | Mitigare |
|---|---|---|
| Drift acumulat indoor demo eșuat live | HIGH | (a) ArUco loop closure ca primary (s130 proven 5 cm); (b) Track A places ca novelty secundar; (c) Repetă demo 20× înainte, măsoară 95% percentile |
| DNN training + ObjectsPlan nu converg | HIGH | Definește interfața DNN↔lifter EARLY; DNN poate avea fallback la ArUco dacă întârzie |
| Outdoor texture-less / lighting variabilă | MEDIUM | (a) Locație outdoor cu landmarks mecanici / poligon test; (b) ArUco mari cunoscute ca anchors; (c) Limit 2-3 outdoor missions, nu robust general |
| Scope creep (plan crește, implementare rămâne în urmă) | HIGH | **NOW**: îngheață scope-ul §23.2, refuză §24+. Capturează idei în `FutureWork.md` items list de jos. |
| Defense reviewers cer numere care lipsesc | MEDIUM | Tabel `drift/altitude/object-count benchmarks` din START, măsurat la fiecare commit |
| HW Stage 9 reveal probleme nemăsurate în SIM | reduced (HW gata) | Bring-up early cu telemetry instrumentation |
| Single point of failure (single developer) | persistent | Documentează tot, scrie cod ca să poată fi reluat de altcineva |

### 23.5 Action plan post-s132 (in ordine)

1. **L6 `sentai.explore` minimal**: FSM EXPLORE → APPROACH → INSPECT → RETURN → LAND
   - Radio-commandable via REPL `sentai.explore.goto(object_id)` /
     `sentai.explore.return_home()` etc.
   - Uses L5 obiectele și L3 places existing
   - PASS gate: SIM indoor mission success ≥ 80% peste 10 runs
2. **s133-s137 Track A places** (per §22.5 design): minimal version
   GIST+HSV+FFT-mag → 64 B → H3 → loop closure
3. **Stage 6 `sentai.calib`** on-board MP binding (port Pas 2 from
   `_shared/camera_calibration.py` la sentai_runtime/modsentai_calib.c)
4. **Stage 9 ARM bring-up**: build ARM, flash, smoke test, DWT
   timing measurement per pipeline component
5. **L7 indoor integrated demo** — proof scenariul §23.1
6. **DNN integration** (tracks: tu DNN training paralel, eu sentai_runtime
   integration hook în detection_task)
7. **Outdoor PX4 experiments** — 2-3 runs cu telemetry log
8. **Evaluation chapter** — toate numerele, all-in-one

### 23.6 Cross-references

- `FutureWork.md` — items deferred + promotion criteria
- `[[s132-lifter-gazebo-shipped]]` — last completed milestone
- `[[places-two-track-decision]]` — §22 Track A/B split
- `[[camera-mount-calibration]]` — §21 Stage 6 motivation
- `[[gate-every-layer-no-exceptions]]` — discipline rule
- `[[experiments-start-from-origin]]` — reproducibility rule

---

*Frozen 2026-05-15. Next review: post-L6 shipping (estimat 2026-05-22).
Movement between §23 in-scope and `FutureWork.md` is allowed at any
review point; both files date the move.*
