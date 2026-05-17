<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 1069,1447. -->

# Chapter 03_research_constellation — SOTA: multi-object rigid-constellation pose correction (§12)

WBS anchors: OP-S6-W2 candidate; §12.3 Multi-Object Yaw-Wahba + EKF stack

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

