<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 4249,4574. -->

# Chapter 12_places_two_track — Places — two parallel tracks Track A no-DNN / Track B DNN (§22)

WBS anchors: OP-S10-W{1..6} = Track A; Track B → FW-1

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

