<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 1448,1878. -->

# Chapter 04_research_toponav — SOTA: pigeon-style topological nav + global scene fingerprint (§13)

WBS anchors: OP-S10-W{1..6} background; §13.6 Stage 11 sentai.places

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

