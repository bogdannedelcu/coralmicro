<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 3398,4027. -->

# Chapter 09_namespace_audit — Namespace audit (§19) — sentai.* policy decisions

WBS anchors: ARCH-L{n} cross-cutting; canonical noun-vs-verb mapping

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

