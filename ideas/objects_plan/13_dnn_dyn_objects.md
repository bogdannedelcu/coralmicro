<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 4695,$. -->
<!-- Translated to English 2026-05-17 per English-only docs rule. -->

# Chapter 13_dnn_dyn_objects — DNN dynamic object detection — separate concern (§24)

WBS anchors: parallel track; does NOT feed flow / EKF; OTHER DNN
models with separate contracts may.

## 24. DNN dynamic object detection — separate concern (added 2026-05-17)

### 24.1 Why a separate chapter

Operator-stated 2026-05-17: once stabilization + SLAM + places + map
are functional, the final goal is to **search for objects using a
top-down DNN (camera pointing down) on top of the infrastructure
already built**.  Those objects may be **moving** (people, vehicles,
synthetic fauna in Gazebo), and that dynamism can **corrupt
orientation + stabilization** if it is coupled naively into the
SLAM/flow/EKF pipeline.

Solution: **separation of concerns** between:

| Track | Role | Reads | Writes |
|---|---|---|---|
| **Stabilization + SLAM** (existing L0–L6) | keeps the drone oriented in the world | flow, IMU, ArUco markers, places (static) | `flow.anchor_pose`, EKF state |
| **Dynamic-object DNN** (THIS chapter) | identifies + tracks dynamic objects + mission targets | top-down camera, TPU model | `objects` map, `tracker` `confirmed_id`, intent target |

**Cardinal rule (specific to this § — does NOT apply to every
on-board DNN)**: detections of DYNAMIC objects (movable classes:
people, vehicles, rolling cubes, etc.) MUST NOT feed back into
flow / EKF / `anchor_pose` / static-places galleries (Track A).
This is a one-way channel: camera → DNN → tracker → lifter → objects
→ mission.

**Important distinction (operator-stated 2026-05-17)**: this does NOT
preclude other DNN models from feeding the EKF.  SentAI will run
several on-board models, possibly per-frame:
- DNN dynamic-object detector (this §) → NOT into the EKF
- DNN scene-level descriptor for places (FutureWork Track B / §22.3)
  → feeds L3 places loop closure → indirectly into `anchor_pose`
- DNN image-feature anchor (FutureWork F24.6) → may be a VPE source
  alongside ArUco PnP, directly into the EKF measurement update

The filter is per-CLASS-of-model, not per-DNN globally.  This §
covers ONLY *dynamic-object detection*; other models have their own
contracts.

For *dynamic-object detection* specifically:

- L1 `sentai.tracker` filters temporal stability
  (TENTATIVE → CONFIRMED → LOST with trash-on-motion) so a moving
  object does not become a "place" in the L3 places gallery.
- L3 `sentai.places` (Track A descriptors PHOG + GIST + HSV + FFT)
  is BUILT ON FRAMES WITHOUT MASKING THE DNN DETECTIONS — places
  sees the scene as a static whole; dynamic objects are cosmic
  noise.  A more aggressive variant (FutureWork) masks the
  DNN-detected regions before computing the descriptor so that the
  same place with different foreground objects produces the same
  fingerprint.
- L5 `sentai.object_lifter` (inverse-depth EKF) accepts ONLY
  CONFIRMED-stable detections from the L1 tracker — never the raw
  per-frame DNN bbox.
- The L6 `sentai.explore` FSM does not receive DNN events directly;
  it consumes L2 `sentai.objects` (which is fed by the L5 lifter,
  which is fed by the L1 tracker filter).

### 24.2 Proposed pipeline (architectural map)

```
   ┌──────────────────────────────────────────────────────────────┐
   │                STABILIZATION + SLAM (Track 1)                │
   │                                                              │
   │  flow ──▶ EKF ──▶ anchor_pose ◀── ArUco PnP                │
   │             ▲                                                │
   │             │                                                │
   │   L3 places (static descriptors) ◀── camera frame (raw)    │
   └──────────────────────────────────────────────────────────────┘
                                  │
                                  │ pose feed (read-only)
                                  ▼
   ┌──────────────────────────────────────────────────────────────┐
   │              DYNAMIC OBJECT DNN (Track 2 — this §)          │
   │                                                              │
   │   camera (down) ──▶ EdgeTPU YOLO (top-down classes)        │
   │                          │                                   │
   │                          ▼                                   │
   │                   L1 tracker (BoT-SORT-lite + IMU CMC)      │
   │                          │ filter motion, age, class         │
   │                          ▼                                   │
   │                   CONFIRMED tracks (with stable id)          │
   │                          │                                   │
   │                          ▼                                   │
   │                   L5 lifter (inv-depth EKF, per-track)       │
   │                          │                                   │
   │                          ▼                                   │
   │                   L2 sentai.objects (map of dynamic objects) │
   └──────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
                       L6 explore consumes for
                       "go to object X" intent
```

### 24.3 DNN model design — top-down outdoor + indoor

**Top-down camera specifics**: the SentAI camera is mounted pointing
down (per `[[flow-body-frame-baseline]]` cam0).  Images are
approximately orthographic when the drone flies at constant altitude.
The DNN classes are **visible from above** — different from ordinary
ego-vehicle datasets.

Relevant classes (initial scope, in order of difficulty):
1. **ArUco-like fiducial tags** (the 4–6 mission markers in §23.1).
2. **Static colored cubes / cylinders** (red cube §12.1 — Stage 2).
3. **Synthetic dynamic objects in Gazebo**: rolling ball, moving
   vehicle, walking pedestrian — simple kinematic models.
4. **Outdoor categories** (Stage 7+): cars, people, dogs, trash bins
   (DOTA-style top-down classes).

**Model candidate**: YOLOv8n, INT8-quantized + EdgeTPU-compiled,
re-trained on a synthetic dataset with domain randomization +
Gazebo rendering.  Stage 2 in §3 already establishes the training
pipeline.

**Changes vs. generic Stage 2**:
- top-down view augmentation (random altitude, random yaw)
- dynamic-object augmentation (motion blur, perspective
  foreshortening)
- mask DNN bounding boxes during PHOG/GIST training (FutureWork; see
  §24.6)

### 24.4 Test plan (separation of concerns enforced)

**T1 — Static-only baseline**: flight without DNN running.  L3
places, flow, EKF, explore all active — should match the current
§23 numbers (5–10 cm closure, drift bounded).  Establishes that the
DNN can be OFF without breaking the infra.

**T2 — DNN running, no dynamic objects in scene**: flight with the
DNN active but all objects STATIC.  Closure + drift identical to T1
(no coupling regression).  Validates that Track 2 does not pollute
Track 1.

**T3 — DNN running, dynamic objects in scene**: flight with a
synthetic moving cube in Gazebo.  Closure + drift identical to T1
(the moving object is filtered by the L1 tracker before any L3/L4
consumer ever sees it).  Hard gate: difference > 2 cm from T1 = fail.

**T4 — DNN running, dynamic object IS the mission target**: drone
commanded to "find moving cube".  L6 explore consumes `L2.objects`
and hunts the target.  Verdict: tracker holds the target stable
≥ 30 s.

**T5 — Adversarial**: a dynamic object PASSES THROUGH the camera
field during a places lookup.  The L3 match score must not degrade
by more than 10 % (descriptor noise budget).  If degraded > 10 %,
FutureWork item: mask the DNN regions before descriptor compute.

### 24.5 In-scope vs. out-of-scope for the §23 thesis MVP

**In §23.2 (must ship)**: nothing from §24 is explicitly in scope.
The existing line "DNN models on-board | TODO (parallel)" stays and
refers to a **separate DNN training track**, not to a formal
integration with SLAM/places.

**FutureWork (after §23.2 ships)**:
- F24.1 — Synthetic dynamic-object dataset generator (Gazebo + DR).
- F24.2 — L1 tracker dynamic-aware filter (motion-prior + class-prior
  in `tracker_decide_state`).
- F24.3 — T1–T5 test-suite execution.
- F24.4 — Places + DNN-mask interaction study (degradation ≤ 10 %
  gate).
- F24.5 — Outdoor dynamic-object classes (PX4 missions with real
  GPS-tracked vehicles as ground truth).
- F24.6 — Image-feature-anchor DNN (separate model): a dense
  per-frame descriptor that enters the EKF as a VPE alternative when
  ArUco is missing.  Separate contract from §24 dynamic; explicitly
  documented as on-board model #2.  Promotion criteria: after the L7
  indoor demo is demonstrated.

Promotion criteria: §24 promotes from FutureWork to in-scope when
§23.2 (L1–L7 + Stage 9 ARM + outdoor) has shipped AND a use-case
requirement specifically asks for moving-object search (e.g., thesis
extension review-board feedback).

### 24.6 Risk register §24

| Risk | Severity | Mitigation |
|---|---|---|
| Added DNN-detection latency (40 ms invoke) pushes tracker tick > 50 ms and breaks the §23 timing budget | MEDIUM | decouple: DNN @ 5 Hz on M7 in parallel with flow at 30 Hz; tracker consumes with a large Δt; lifter compensates via integrated IMU |
| L1 tracker confuses moving + static (ID swap) | HIGH | class-aware Kalman, separate ID pools per class, IoU + class gate |
| Place descriptors degrade when a dynamic object is prominent in frame | MEDIUM | T5 test gate; mitigation = DNN-region mask pre-PHOG |
| Outdoor dynamic object (vehicle) with velocity > drone capacity → drone loses target | LOW (outdoor) | out-of-scope §23; thesis demo uses only slow indoor objects |
| Accidental coupling DNN-dynamic → orientation via unexpected channels (e.g., raw bbox in the flow log) | HIGH | code-review gate: the *dynamic-detector* TPU pipeline output cannot be read by `sentai_flow.cc`.  Other DNN models (scene descriptor, image-feature anchor) have their own separate contracts. |

### 24.7 Cross-references

- `[[places-two-track-decision]]` — Track A static descriptors (this
  § is orthogonal: "Track B" here is the DNN for DYNAMIC OBJECTS,
  not for place descriptors).
- §23.2 line "DNN models on-board | TODO (parallel)" — same track,
  now formalized.
- §3 Stage 2 — DNN training pipeline (static cube); §24 extends it
  to dynamic objects + top-down camera specifics.
- `[[next-steps-2026-05-17]]` — `OP-S10-W7` (L1 tracker minimal) is
  a prerequisite for §24 integration.

---

*§24 added 2026-05-17.  Out of §23.2 in-scope.  Re-review after the
L7 indoor demo ships.*
