<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 4028,4138. -->

# Chapter 10_bibliography — Bibliography — SOTA references (§20)

WBS anchors: literature citations across §12-§15

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

