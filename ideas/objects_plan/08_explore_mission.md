<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 2845,3397. -->

# Chapter 08_explore_mission — Mission sentai.explore — autonomous two-flight exploration (§18)

WBS anchors: OP-S3 / ARCH-L5; mission FSM SM2

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

