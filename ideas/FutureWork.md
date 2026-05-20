# FutureWork — items deferred from ObjectsPlan beyond thesis scope

**Created 2026-05-15** as companion to `objects_plan.md`. Holds
research directions and engineering features deferred from the
critical path for the PhD thesis demo + defense.

## Purpose

Items live here when they are:
- Research-justified but not on the critical path for thesis demo
- Engineering-feasible but bandwidth-prohibitive for single developer
- Dependent on future hardware / dataset / training infra not yet
  available

Movement is **bidirectional and dated**: items can be promoted back
into `objects_plan.md` when a trigger condition is met, or descoped
from `objects_plan.md` into here when scope creeps.

## Promotion criteria (FW → ObjectsPlan)

An item moves OUT of FutureWork back into the active plan when:
1. Thesis is defended (post-defense expansion)
2. A specific need triggers it during thesis work (e.g., outdoor
   texture-less scene reveals Track A insufficient → promote FW1
   Track B)
3. Time / bandwidth opens up before defense

## Descope criteria (ObjectsPlan → FW)

When a planned item is found to be premature, over-engineered, or
out-of-scope for thesis demo, it moves here with rationale captured.

---

## FW1 — DNN-based place descriptor (Track B)

**Source**: `objects_plan.md` §22.3
**Moved**: 2026-05-15
**Scope**: Custom MobileNet/EfficientNet encoder + VLAD head, INT8
quantized, distilled from DINOv2/AnyLoc teacher, ~5 MB model on
EdgeTPU. Produces 256-D embedding → PCA → 64 B descriptor for places
gallery.
**Why deferred**: Requires training pipeline + aerial dataset not yet
available. Track A (PHOG/GIST/FFT-mag/HSV-hist) is sufficient for
thesis demo per §22.1 decision. L3 slot 64 B is populator-agnostic,
so drop-in upgrade later.
**Promotion trigger**: Track A measured insufficient on indoor demo
or outdoor missions; OR aerial dataset + training pipeline becomes
available.
**Bibliography**: see `objects_plan.md` §22.3 (NetVLAD, CosPlace,
EigenPlaces, AnyLoc, GeoCLIP, SatCLIP, RemoteCLIP, MobileCLIP,
TinyCLIP).

---

## FW2 — SeqSLAM temporal accumulation

**Source**: `objects_plan.md` §14.4 Mechanism 1
**Moved**: 2026-05-15
**Scope**: Match K consecutive frames as a sequence instead of single
frame; ratio-aggregate score. Robust to day↔night appearance change.
**Why deferred**: Single-frame Track A descriptors likely sufficient
for indoor demo + 2-3 outdoor missions. SeqSLAM adds K× compute +
ringbuffer storage for marginal robustness gain at thesis scope.
**Promotion trigger**: Single-frame false-positive rate > 10% at
threshold τ in indoor demo; OR outdoor lighting variation kills
recognition.
**Bibliography**: Milford & Wyeth ICRA 2012; Pepperell ICRA 2014.

---

## FW3 — ORB + VLAD local features (Strategy C)

**Source**: `objects_plan.md` §14.3 Strategia C
**Moved**: 2026-05-15
**Scope**: Detect ORB keypoints with dominant orientation, extract
binary descriptors, aggregate via VLAD (K=16 centroids) → 2048-D →
PCA 256-D.
**Why deferred**: Track A global descriptors sufficient. ORB+VLAD
adds ~12 ms/frame plus keypoint storage overhead. Complementary, not
replacement.
**Promotion trigger**: Track A fails on scenes with sparse global
texture but distinctive local features (sparse corners, isolated
objects).
**Bibliography**: Rublee et al. ICCV 2011 (ORB); Jégou et al. CVPR
2010 (VLAD); Mur-Artal et al. TRO 2015 (ORB-SLAM); Galvez-López &
Tardós TRO 2012 (DBoW2).

---

## FW4 — Rotation-equivariant CNN (Strategy D)

**Source**: `objects_plan.md` §14.3 Strategia D
**Moved**: 2026-05-15
**Scope**: G-CNN / Steerable CNN / Spherical CNN with rotation
equivariance built-in. ~50-100 MB models; uncertain EdgeTPU support
for group convolutions.
**Why deferred**: Out of scope for MCU thesis target. Pre-rotate
canonical + log-polar FFT magnitude (Track A) achieves practical
rotation invariance at fraction of compute.
**Promotion trigger**: Post-PhD work expanding to more capable HW
or where Track A rotation tolerance fails.
**Bibliography**: Cohen & Welling ICML 2016; Weiler et al. CVPR 2018;
e2cnn library.

---

## FW5 — Adaptive H3 resolution subdivision (Uber-style)

**Source**: `objects_plan.md` §15.4 + §15.6 (subdivision algorithm)
**Moved**: 2026-05-15
**Scope**: Background task @ 1 Hz that splits H3 cells with visits >
N_HIGH to next resolution; merges cells with visits < N_LOW back to
parent. Density-adaptive honeycomb (sparse → large hex, dense → small
hex).
**Why deferred**: Thesis demo uses fixed res (12 or 13) per mission.
Adaptive subdivision is elegant but adds state-machine complexity not
needed for proving the core architecture.
**Promotion trigger**: Multi-altitude missions where fixed-res leaks
detail (low alt) or wastes slots (high alt); OR long missions where
gallery slot exhaustion becomes a real constraint.
**Bibliography**: Sahr 2011; Kim & Cho ISPRS 2017; Bondaruk et al.
2020 (DGGS survey).

---

## FW6 — Multi-altitude descriptor storage per H3 cell

**Source**: `objects_plan.md` Stage 11.D + §15.5
**Moved**: 2026-05-15
**Scope**: Store 2-3 descriptor slots per H3 cell, indexed by
altitude band. Query at altitude selects appropriate slot.
**Why deferred**: Thesis missions are roughly fixed-altitude per
phase (~1.0-1.5 m indoor, ~5-10 m outdoor). Multi-altitude descriptor
complexity isn't justified for the demo.
**Promotion trigger**: Mixed-altitude exploration missions where
single descriptor fails across altitudes.

---

## FW7 — Multi-drone H3 gallery sharing via Meshtastic

**Source**: `objects_plan.md` §15.12.3
**Moved**: 2026-05-15
**Scope**: Drones flood compact descriptor + H3 cell ID over mesh
radio (Meshtastic-style). Later drones can recognize places they have
never visited via shared gallery.
**Why deferred**: Single-drone thesis. Multi-drone is post-PhD
follow-up work.
**Promotion trigger**: Multi-drone follow-up project / paper.
**Bibliography**: Avizonis et al. ICUAS 2019; Singh et al. 2020.

---

## FW8 — Multi-level honeycomb storage advanced features

**Source**: `objects_plan.md` §16 (entirety)
**Moved**: 2026-05-15
**Scope**: Sparse hash + on-demand hierarchy traversal + EMA
embedding update across visits + 6-bucket hex-aligned orientation
storage + cross-correlation for continuous yaw recovery.
**Why deferred**: More sophisticated than thesis demo requires.
Single-resolution + single-shot descriptor with simple replace-on-
update is sufficient for the demo and presentation.
**Promotion trigger**: Cross-mission persistence + outdoor cross-day
robustness needed; OR explicit yaw recovery from places becomes
critical (today it's done via Stage 6 object-level loop closure).

---

## FW9 — Cross-scale embedding composition (parent ↔ children)

**Source**: `objects_plan.md` §17 (entirety)
**Moved**: 2026-05-15
**Scope**: Parent embedding = function of children embeddings;
enables coarse-to-fine spatial reasoning ("am I in the right
neighborhood?" before "am I in the right house?").
**Why deferred**: Single-resolution operation is sufficient for
thesis. Cross-scale composition is a clever extension but not
demo-critical.
**Promotion trigger**: Multi-resolution missions warranted (e.g.,
explore at 50 m altitude, inspect at 5 m).

---

## FW10 — Persistent gallery across mission boots

**Source**: implied in `objects_plan.md` §15.7, §15.11.F
**Moved**: 2026-05-15
**Scope**: Serialize places gallery + H3 hash to FileX user partition
on graceful shutdown; restore on boot. Drone learns over multiple
flights (revisits known places, prevents re-discovery).
**Why deferred**: Thesis demo is single-flight per mission. Cross-
flight learning is nice-to-have but not story-critical for defense.
**Promotion trigger**: Long-duration mission demonstrations; OR
defense reviewer specifically pushes on persistence story.

---

## FW11 — Outdoor lat/lng / OSM / ArcGIS integration

**Source**: `objects_plan.md` §15.7.4, §15.12
**Moved**: 2026-05-15
**Scope**: Anchor H3 origin to real lat/lng via GPS; integrate
gallery with OpenStreetMap / ArcGIS / Mapbox for known-landmark prior.
**Why deferred**: Thesis outdoor missions use ENU-local + GPS as
pose source, no need for lat/lng-indexed places. OSM integration is
substantial engineering work for marginal thesis value.
**Promotion trigger**: Outdoor mapping product / publication
follow-up; OR explicit defense reviewer feedback that outdoor
section is weak.

---

## FW12 — Full SM3 track health with COAST / ALIGN

**Source**: `objects_plan.md` Stage 7 (partial — minimal SM3 stays)
**Moved**: 2026-05-15
**Scope**: Full track health state machine with COAST mode (no
observations, predict-only) and ALIGN mode (re-aligning after long
COAST).
**Why deferred**: Thesis demo doesn't have long-occlusion periods.
Minimal SM3 (TRACKING + LIFTED + LOST from current L5) suffices.
**Promotion trigger**: Mission with long occluded segments needed
(e.g., flying through a tunnel).

---

## FW13 — Stage 8 standalone end-to-end SITL test

**Source**: `objects_plan.md` Stage 8
**Moved**: 2026-05-15 (absorbed into L7)
**Scope**: Originally a separate stage; absorbed into L7 integrated
demo + thesis evaluation chapter. No standalone work needed.

---

## FW14 — Macro-category aerial scene detection

**Source**: `objects_plan.md` §13.3 Approach A
**Moved**: 2026-05-15
**Scope**: Train YOLO/DETR variants on `INTERSECTION, BRIDGE, LAKE,
HOUSE, ROAD_FORK, FOREST_CLEARING` for landmark-level navigation
outdoors.
**Why deferred**: Operator's separate DNN training work covers the
DNN side. Macro-categories specifically need datasets (NWPU-RESISC45,
EuroSAT, AID) that need fine-tuning effort outside thesis scope.
**Promotion trigger**: Outdoor missions needing large-scale landmark
navigation.
**Bibliography**: Cheng et al. ISPRS 2017 (NWPU-RESISC45); Helber et
al. JSTARS 2019 (EuroSAT).

---

## FW15 — Adaptive auto-merge of similar children cells

**Source**: `objects_plan.md` §15.12.1
**Moved**: 2026-05-15
**Scope**: Pair-wise compare 7 child cells under a parent; if all
descriptor pairs have cosine > 0.95, merge them back to parent +
free child slots. Compaction for textureless regions.
**Why deferred**: Slot exhaustion not a realistic risk at thesis
scope (256 slots, single-flight demos).
**Promotion trigger**: Long missions where slot pressure becomes
real.

---

## FW16 — 4D hex (lat × lng × time) for diurnal variation

**Source**: `objects_plan.md` §15.12.2
**Moved**: 2026-05-15
**Scope**: Store descriptor per (H3 cell, time-bucket) to handle
day/night/season variation of same place.
**Why deferred**: Thesis missions short-duration. Diurnal robustness
addressed by Track A invariance properties + (later) SeqSLAM FW2.
**Promotion trigger**: Multi-day mission demonstrations.

---

## FW17 — Mini-paper "AirREPL: LLM-Commandable Embedded Runtime"

**Source**: conversation 2026-05-15 (post-s132 strategy discussion)
**Moved**: 2026-05-15 (parked for post-thesis write-up)
**Canonical doc**: **`ideas/AirREPL_paper.md`** — full paper plan
including contribution claims (C1–C5), what we have, experiments
E1–E10 with setup/metrics/pass criteria, paper structure,
AirREPL-vs-MCP positioning, venues, reproducibility plan, open
questions, and full bibliography.

**Scope (one line)**: Short paper / arxiv preprint documenting the
AirREPL pattern (Turing-complete Python REPL on Cortex-M7 over
~30 B CRTP radio, designed for LLM-agent consumption) as a
distinct contribution separable from the thesis.

**Why deferred**: thesis claim = drift-bounded autonomous nav on a
wireless sensor; AirREPL is the **mechanism** beneath that claim.
Framing it as a standalone artifact preserves both stories. Needs
benchmark suite + comparative table + reproducibility bundle, none
of which are critical-path for thesis defense.

**Promotion trigger**: thesis defended → write up. Earliest realistic
submission Q2 2027. Effort ≈ 6 weeks post-defense.

---

## FW18 — OP-S10-W11 prep-pipeline observability polish

**Source**: W11 embeded.md audit 2026-05-17 (items M5/M6/M7/m4 from
the evaluation, deferred from `T3.1` per operator scope-cut).
**Canonical doc**: see commit message of `OP-S10-W11-T3.1` for the
finding catalog; this entry is the parking lot.

**Scope** — four discipline upgrades on the prep slot + SlamTask path:

1. **M5 Camera-health gate**: PrepTask checks `sentai_cam_is_initialized()`
   per iteration; on failure increment a counter + disable slot
   publishing so consumers observe a frozen `seq` (escalation to
   mission code, no silent stale matches).
2. **M6 Stack watermark in `slam_stats`**: 1-line
   `uxTaskGetStackHighWaterMark(s_slam_task)` exposure so the
   `kSlamStackWords` guess can be empirically validated.
3. **M7 Split `frames_dropped`** into `drops_slot_get / drops_hsv /
   drops_timeout` for diagnostic-coverage breakdown (§4.3).
4. **m4 Formal safe-state contract** in `sentai_prep.h` +
   `slam_task.h` headers per `embeded.md §1.4`.

**Why deferred**: T3.1 already lands the critical/major fixes
(seqlock, SERR codes, health integration).  These four are polish
that the existing per-frame counters and ARM live tests will
implicitly surface; no autonomy hazard.

**Promotion trigger**: any non-trivial follow-up touch to
`sentai_prep` / `slam_task` (e.g. T4 SIM producer, T5 EXP-s163
live test).  Pack into a `T3.2` discipline commit.

---

## FW19 — Flow vs LightV8nPnP DNN pose ablation

**Source**: operator-stated 2026-05-20 during the WhyCon
optimization sprint, after the per-stage breakdown made the
"classical-vs-learned pose" tradeoff concrete.
**Moved**: 2026-05-20

**Scope** — when the DNN pose stage is wired in (YOLOv8 nano +
PnP solver on detected corners, running through the existing M7 +
EdgeTPU pipeline), run a head-to-head ablation against the
classical optical-flow stack we have today.

Metrics to capture:

1. **Latency on the same scene** — per-stage breakdown like the
   per-stage DWT instrumentation we now have on WhyCon (`Phase A
   / B / W` decomposition).
2. **Accuracy + drift** over a 30-s static-board run (zero-motion
   input → cumsum should be zero; anything else is bias).
3. **Robustness** to lighting variation, motion blur, partial
   occlusion.  Flow degrades gracefully (less contrast → lower
   confidence + deadband zeros it); DNN may hard-fail on
   out-of-distribution inputs.
4. **Slot usage** at 30 / 60 Hz SafetyTask periods.

**Reference numbers (today, build #1399)**:

| Stack | Latency / publish | Bench source |
|---|---:|---|
| Flow SAD (M7, USAD8+LD32U) | 0.94 ms / 2.29 ms full | experiment.md #1139 |
| WhyCon-lite (M7, OCRAM+SIMD+inline) | 6.00 ms @ 4 markers | OP-S10-W17 |
| ArUco rolling (M7, OCRAM) | 24.83 ms @ 4 markers | OP-S10-W16 |

**Gate before merging DNN to production**: must beat Flow's
2.29 ms / WhyCon's 6 ms on the same scene OR have a clear
robustness story that justifies the extra latency.

**Why deferred**: DNN pose stage isn't wired yet; this is a
thesis-data-point item rather than a development gate.  The
WhyCon + Flow numbers are the comparison baseline.

**Promotion trigger**: any DNN-based pose / corner detection
work that touches `sentai_runtime.cc` TPU pipeline.

---

## Items currently NOT in FutureWork (may be added)

When operator notes a new idea during thesis work that's not on the
critical path, capture here. Examples of likely future additions:

- Loop closure ambiguity resolution via geometric consistency
- Active perception / next-best-view planning
- Dynamic obstacle avoidance (currently static-environment only)
- Sound / audio modality
- Vibration-based localization
- Battery-aware mission planning (separate from `sentai.safety`)
- Multi-rotor wind disturbance compensation

(operator: add free-form notes here as ideas surface — keeps
`objects_plan.md` clean)
