<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 1879,2379. -->

# Chapter 05_research_places — SOTA: place fingerprint, rotation invariance, opposite-direction (§14)

WBS anchors: OP-S10-W{4..6}; §14.4 opposite-direction = OP-S10-W6 gate ≥ 80% recall

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

