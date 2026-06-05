---
name: places-two-track-decision
description: "Places descriptor split decided 2026-05-15: Track A no-DNN (PHOG+GIST+HSV-hist+log-polar FFT-mag) PRIMARY for next iterations; Track B DNN-EdgeTPU DEFERRED until training infra+dataset exist. API slot 64B unchanged — populator-only swap."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Decizia 2026-05-15 (operator)**: pentru implementarea Stage 11
(`sentai.places` cu embedding visual per H3 cell), abordăm **două
piste paralele**:

| Track | Status | Tech | When |
|---|---|---|---|
| **A — No-DNN** | **PRIMARY (next iterations)** | PHOG + GIST + HSV-hist + log-polar FFT-mag | imediat (s133+) |
| **B — DNN EdgeTPU** | DEFERRED | distilled MobileNet/EfficientNet + VLAD INT8 | după dataset + training infra |

**Why**: nu avem încă training pipeline + dataset aerian curat pentru
encoder TPU. Hand-crafted descriptors (30 ani literatură matură) sunt
suficient de bune pentru proof-of-concept și baseline contra căruia
măsurăm Track B ulterior.

**How to apply**: 
- Slot-ul 64 B din L3 (`[[places-l3-shipped]]`) rămâne identic între
  cele 2 track-uri. Doar populator-ul de descriptor diferă.
- Implementarea Track A începe cu s133 (Python host-side prototype,
  Gazebo verification, same pattern ca s131).
- NU începe Track B până nu există măsurare Track A baseline ca să
  justifice. Discuții despre DNN deferred până ne lovim de limita
  Track A.
- Strategiile rotation din §14.3 (A pre-rotate, B log-polar FFT) sunt
  **componente** ale Track A, nu alternative. Strategia A se aplică
  înainte de descriptor; Strategia B (FFT-mag) e canalul nr. 4 al
  descriptor-ului.

**Track A bibliography primary**:
- Bosch, Zisserman, Munoz, CIVR 2007 — **PHOG (sursa primară)**
- Lazebnik, Schmid, Ponce, CVPR 2006 — spatial pyramid origin
- Oliva & Torralba IJCV 2001 — GIST
- Swain & Ballard IJCV 1991 — color indexing (**primary color histogram**)
- Reddy & Chatterji IEEE TIP 1996 — FFT rotation invariance
- Ulrich & Nourbakhsh ICRA 2000 — early appearance-based VPR
- Lowry et al. TRO 2016 — VPR survey (clear taxonomy hand-crafted vs deep)
- Cummins & Newman IJRR 2008 — FAB-MAP (Bayesian framework, descriptor-agnostic)
- Milford & Wyeth ICRA 2012 — SeqSLAM (temporal accumulation, complementary)

**Track B bibliography (deferred reference)**:
- Arandjelović CVPR 2016 — NetVLAD
- Berton CVPR 2022 / ICCV 2023 — CosPlace / EigenPlaces
- Keetha RAL 2024 — AnyLoc (DINOv2 + VLAD, candidate teacher)
- Vivanco Cepeda NeurIPS 2023 — GeoCLIP
- Klemmer MSR 2023 — SatCLIP
- Liu IEEE TGRS 2024 — RemoteCLIP
- MobileCLIP (Apple CVPR 2024), TinyCLIP (Microsoft ICCV 2023)

**Concret detaliat în** [objects_plan.md §22](../../../../work/coralmicro/ideas/objects_plan.md).
Forward-pointer adăugat la §14.10.

**Next step proposat**: s133_places_track_a_phog_gist — Python
host-side prototype, mission simplă Gazebo (cf2 zboară pătrat 1×1 m
peste s130 arena la yaw 0°, refly la yaw 90° și 180°, măsoară
recognition rate). PASS criteria: ≥ 60% top-1 la 90°, ≥ 50% la 180°,
< 10% false-positive la τ=0.7.

Related: [[l5-shipped]], [[places-l3-shipped]], [[s131-lifter-math-shipped]],
[[flowbaseline-canonical-config]], [[gate-every-layer-no-exceptions]],
[[experiments-start-from-origin]], [[h3-integration]],
[[no-tmp-experiments]], [[sentai-sim-journal]].
