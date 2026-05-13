# Drone Autonomous Navigation pe RT1176 — Design Document & SOTA Synthesis

> Document de design pentru un sistem complet de navigație autonomă a unei drone
> quadcopter mici (~250g), fără GPS, în mediu interior, cu detecție de obiecte prin
> rețea neurală pe EdgeTPU. Sintetizează state-of-the-art-ul din literatura
> 2008–2026 pentru localizare, mapping, tracking intermitent și visual servoing
> pe MCU.

---

## 1. Platforma hardware

| Componentă | Detaliu |
|---|---|
| MCU | NXP i.MX RT1176 — Cortex-M7 @ 1 GHz + Cortex-M4 @ 400 MHz, SDRAM externă |
| NN accelerator | EdgeTPU (Coral), ~4 TOPS INT8, prin USB/PCIe |
| IMU | 6-DoF on-board (accelerometru + giroscop, ~1 kHz) |
| Optical flow | On-sensor (PMW3901 / PX4FLOW style), output viteză 2D body-frame |
| Altimetru | Barometru |
| Cameră | RGB monoculară frontală, 320×240 sau 640×480, 30 fps |
| **Lipsă** | GPS, magnetometru, lidar |

**Implicație critică**: fără magnetometru, yaw-ul drifteză liniar în timp. Anti-drift
se face prin re-observarea obiectelor cunoscute (loop closure pe harta de obiecte).
Această buclă trebuie să funcționeze pentru misiuni > 1 minut.

---

## 2. Arhitectura generală (6 straturi)

```
┌──────────────────────────────────────────────────────────────┐
│  Stratul 6: Controler hibrid PBVS → IBVS                     │
├──────────────────────────────────────────────────────────────┤
│  Stratul 5: State machines (Vehicle, Mission, Track Health)  │
├──────────────────────────────────────────────────────────────┤
│  Stratul 4: Lifting 2D→3D + EKF de hartă (inverse-depth)     │
├──────────────────────────────────────────────────────────────┤
│  Stratul 3: Tracker 2D pe imagine (ByteTrack-style)          │
├──────────────────────────────────────────────────────────────┤
│  Stratul 2: Detector NN pe EdgeTPU                           │
├──────────────────────────────────────────────────────────────┤
│  Stratul 1: Ego-motion EKF (IMU + flow + baro)               │
└──────────────────────────────────────────────────────────────┘
```

**Principiu fundamental**: NU se face control direct pe pixeli. Controlul se face
pe **estimarea 3D persistentă** a obiectului în world frame, care există mereu
(predicție EKF) chiar și când detectorul scapă frame-uri.

---

## 3. Frame-uri de coordonate

Trei sisteme distincte care nu se amestecă niciodată:

- **World frame (W)** — inerțial, fixat la decolare. ENU (X=est, Y=nord, Z=sus).
  Originea = punct takeoff. Aici trăiesc obiectele.
- **Body frame (B)** — atașat de dronă. Se mișcă și rotește cu ea.
- **Camera frame (C)** — atașat de senzor. Legat de B prin extrinsic fix (calibrare).

Relația: `T_W_C(t) = T_W_B(t) · T_B_C`

**Obiectele NU se stochează relativ la dronă.** Se stochează ca `p_W = (x, y, z)`
absolut în W. Numai drona "se mișcă" în acest frame; obiectele sunt statice.

Atitudinea se reprezintă intern ca **quaternion** (q_W_B), nu unghiuri Euler —
fără gimbal lock, integrare numerică stabilă.

---

## 4. Structuri de date

### 4.1 Harta de obiecte

**Array static contiguu**, fără heap, fără liste înlănțuite:

```c
#define MAX_OBJECTS 32

typedef struct {
    uint8_t  id;
    uint8_t  status;         // FREE / TENTATIVE / CONFIRMED / COASTING / STALE / LOST
    uint8_t  class_id;
    uint8_t  observations;
    float    p_W[3];          // poziție în world frame [m]
    float    P[6];            // covarianță upper-triangular 3×3 (6 floats)
    uint32_t last_seen_ms;
    uint16_t descriptor[8];   // pentru re-asociere
} object_t;

object_t map[MAX_OBJECTS];   // ~64B × 32 = ~2 KB
```

### 4.2 Lista de obstacole (pentru evitare)

Reprezentare ca **primitive geometrice** (paradigma FASTER/MADER):

```c
typedef struct {
    uint8_t id;
    uint8_t shape;            // BOX / SPHERE / CYLINDER
    float   center_W[3];
    float   half_extents[3];
    float   yaw;
} obstacle_t;

obstacle_t obstacles[MAX_OBSTACLES];
```

Coliziunea drone-vs-box = câteva înmulțiri și un AABB check. ~20 cicluri pe Cortex-M7.

### 4.3 Stare EKF

Vector contiguu `x[N]` și matrice `P[N×N]`, ambele statice. Obiectele indexează
cu offset fix → Jacobianul H rămâne sparse-pe-bloc, iar update-urile EKF se
reduc la operații pe sub-blocuri 3×3.

---

## 5. Stratul 1 — Ego-motion EKF

Indirect EKF (error-state), 16 stări:

```
x = [ p_W (3), v_W (3), q_W_B (4), b_g (3), b_a (3) ]
```

**Predicție** din IMU la 200–500 Hz.

**Update-uri**:
- Optical flow → velocitate body, rotită în W (100 Hz)
- Barometru → z absolut (20–50 Hz, σ medie — drifteză cu temperatura)
- **Re-observare obiect cunoscut** → corecție pose dronă, inclusiv yaw

**Observabilitate**:
- Roll, pitch — observabile din gravitație (accelerometrul "vede" în jos în repaus)
- Yaw — neobservabil fără magnetometru sau ancorare vizuală → necesită loop closure

### Referințe principale

- **Honegger et al., ICRA 2013** — "An Open Source and Open Hardware Embedded Metric Optical Flow CMOS Camera". Reperul PX4FLOW, optical flow on-sensor pe Cortex-M4F la 250 Hz.
- **He et al., ICRA 2021** — "PicoVO: A Lightweight RGB-D Visual Odometry Targeting Resource-Constrained IoT Devices". VO 6-DoF pe STM32F767, 33 fps @ 320×240.
- **Trawny & Roumeliotis, UMN tech report 2005-002** — "Indirect Kalman Filter for 3D Attitude Estimation". Math-ul quaternion EKF complet, standardul de referință.
- **Santamaria et al.** — observare directă a stării de mișcare din flow brut, fără calcul de velocitate liniară (reduce CPU overhead).
- **LEVIO, arXiv 2602.03294 (2026)** — VIO complet pe SoC RISC-V ultra-low-power, 20 fps @ <100 mW.

---

## 6. Stratul 2 — Detector NN pe EdgeTPU

**Model**: YOLOv8n quantizat INT8, compilat cu `edgetpu_compiler`. Alternativ
MobileNet-SSD-Lite v2.

**Pipeline**:
```
capture cameră → resize → quantize INT8 → EdgeTPU inferență
→ post-process (NMS) → listă bbox-uri (u, v, w, h, class_id, confidence)
```

**Performanță țintă**: > 90% recall, < 5% false positive, > 20 fps.

**Antrenament**: dataset cu Domain Randomization (lumini, texturi, distractoare,
unghiuri 0°–90°) generat sintetic din Gazebo.

---

## 7. Stratul 3 — Tracker 2D pe imagine

Implementare **ByteTrack simplificată**:

- Kalman per track, state `(cx, cy, w, h, vcx, vcy, vw, vh)` în spațiul imaginii
- Asociere greedy IoU (≤ 5 tracks simultane, Hungarian e overkill)
- **Camera motion compensation**: înainte de IoU, rotește bbox-urile prezise cu Δattitude între frame-uri (din BoT-SORT)
- Două runde de asociere: high-conf detecții întâi (creează tracks noi), low-conf doar pentru salvare track-uri existente

### Referințe

- **Bewley et al., ICIP 2016** — "Simple Online and Realtime Tracking" (SORT). Baseline-ul: Kalman + Hungarian + IoU.
- **Wojke et al., ICIP 2017** — "Simple Online and Realtime Tracking with a Deep Association Metric" (DeepSORT). Adaugă embedding aparență pentru ocluzie.
- **Zhang et al., ECCV 2022, arXiv 2110.06864** — "ByteTrack: Multi-Object Tracking by Associating Every Detection Box". Folosește toate detecțiile, inclusiv low-conf. **Recomandat ca bază.**
- **Aharon et al., arXiv 2206.14651 (2022)** — "BoT-SORT: Robust Associations Multi-Pedestrian Tracking". Camera motion compensation — esențial pe dronă.
- **Cao et al., CVPR 2023, arXiv 2203.14360** — "Observation-Centric SORT" (OC-SORT). Re-update retroactiv după ocluzie, 700+ fps CPU.

---

## 8. Stratul 4 — Lifting 2D → 3D + EKF de hartă

Pentru fiecare tracklet 2D matur (≥3 observații):

### 8.1 Calcul bearing

```
r_C = K⁻¹ · [cx, cy, 1]ᵀ          // direcție în camera frame
r_C = r_C / ||r_C||                 // normalizează
r_B = R_B_C · r_C                   // în body
r_W = R_W_B(t) · r_B                // în world
o_W = t_W_B(t)                      // originea razei
```

### 8.2 Pseudo-depth din class prior

```
depth_est = f · real_size_class / pixel_size_observed
σ_depth ≈ depth² · σ_pixel / pixel_size
```

`real_size_class` e tabel hardcoded per `class_id` (ex: scaun = 0.45 m, cub = 0.30 m).

### 8.3 Inițializare cu inverse-depth

Stare obiect: `(o_W, θ, φ, ρ)` unde ρ = 1/depth_est, σ_ρ mare.

Pe observații succesive, paralaxa colapsează ρ. După convergență (σ_ρ sub prag) →
conversie la `(x_W, y_W, z_W)` cu covarianță 3×3 și mutare în harta principală.

### 8.4 Loop closure

**Critic în absența magnetometrului**: când un obiect cunoscut e re-observat,
observația acționează ca update pe pose-ul dronei (inclusiv yaw), nu pe poziția
obiectului. Aceasta-i singura ancorare absolută pentru yaw.

### Referințe

- **Civera, Davison, Montiel, TRO 2008** — "Inverse Depth Parametrization for Monocular SLAM". Math complet pentru inițializare landmark fără paralaxă. **Fundamentul.**
- **Solà et al., IJCV 2012** — "Impact of Landmark Parametrization on Monocular EKF-SLAM with Points and Lines". Comparație parametrizări, Framed Inverse Depth (FID) — cost redus.
- **Yang & Shi, MIR 2013** — "Bearing-only Visual SLAM for Small UAVs in GPS-denied Environments". EKF ierarhic (atitudine+velocitate separat de poziție+map). Blueprint aproape direct aplicabil.
- **Davison, ICCV 2003 / TPAMI 2007** — "Real-Time Simultaneous Localisation and Mapping with a Single Camera" (MonoSLAM). Reperul istoric EKF-SLAM monocular.
- **Salem & Chand, arXiv 2410.10409 (2024)** — "SMART-TRACK: Kalman Filter-Guided Sensor Fusion for Robust UAV Object Tracking". Tratează explicit detecții intermitente.
- **Hoiem et al., IJCV 2008** — "Putting Objects in Perspective". Class-specific size priors pentru localizare 3D monoculară.

### 8.5 Modele alternative pentru reprezentarea obiectelor

| Reprezentare | Cost | Recomandare |
|---|---|---|
| Point landmark + class lookup | 3 floats + 6 cov | **Recomandat pentru MCU** |
| 3D cuboid (CubeSLAM) | 9 floats | Doar CPU |
| Quadric / ellipsoid (QuadricSLAM) | 10 floats | Doar CPU |
| Mesh / SDF / Gaussian splats | MB | Doar GPU |

- **Yang & Scherer, TRO 2019** — "CubeSLAM: Monocular 3-D Object SLAM". Cuboizi din vanishing points + 2D bbox.
- **Nicholson et al., RAL 2019** — "QuadricSLAM: Dual Quadrics from Object Detections as Landmarks".

### 8.6 Reprezentări obstacole

| Reprezentare | Memorie | Note |
|---|---|---|
| Lista primitive (FASTER/MADER) | ~30 B/obj | **Recomandat pentru MCU** |
| 2D occupancy grid + altitudine fixă | KB | Fallback ieftin |
| OctoMap (Hornung 2013) | MB | Standard CPU, prea greu MCU |
| ESDF (Voxblox, FIESTA) | MB | Gold standard CPU |
| Polytope corridor (EGO-Planner) | KB | SOTA agile drones |

- **Hornung et al., Autonomous Robots 2013** — "OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees".
- **Oleynikova et al., IROS 2017** — "Voxblox: Incremental 3D Euclidean Signed Distance Fields".
- **Tordesillas & How, RAL 2020** — "FASTER: Fast and Safe Trajectory Planner". Obstacole ca polytope-uri.
- **Zhou et al., RAL 2022** — "EGO-Planner-v2". ESDF local + corridor convex.

---

## 9. Stratul 5 — State machines

Trei FSM-uri ierarhice rulează în paralel, nu unul singur. Confuzia clasică e
încercarea de a comprima totul într-un graf — devine ingestibilă rapid.

### 9.1 SM1 — Vehicle Lifecycle

```
BOOT → READY_ON_GROUND → ARMING → TAKEOFF → HOVER → MISSION
                                                       ↓
                                                       RTL → LANDING → DISARMED
                              EMERGENCY (din orice stare)
```

Aliniat cu PX4 nativ. **Kill switches**:
- EKF covariance trace > prag → EMERGENCY
- Baterie < critic → RTL
- Loss of optical flow > 2s la altitudine joasă → EMERGENCY
- Tilt > 60° → EMERGENCY

### 9.2 SM2 — Mission / Behavior

```
IDLE → MISSION_START → SEARCH → INITIALIZE → APPROACH ⇄ COAST
                          ↑                       ↓
                          └──────── ALIGN ────────┘
                                       ↓
                                     FINAL → HOLD → DONE → RETURN
```

| Stare | Output intenție | Tranziție-out cheie |
|---|---|---|
| **SEARCH** | spirală pătrată expansivă, yaw +20°/s | ≥ 5 detecții consecutive @ conf > 0.7 → INITIALIZE |
| **INITIALIZE** | zbor lateral oscilant ±1.5m, yaw lock pe obiect | trace(P_obj) < 0.5 m² → APPROACH |
| **APPROACH** | PBVS velocity = K_p·(p_obj − p_drone), saturat 1 m/s | distanță < 2.0m → ALIGN; det_age > 1s → COAST |
| **COAST** | velocity redusă spre p_obj prezis, urcare +0.3m | re-detecție → APPROACH; > 5s → SEARCH |
| **ALIGN** | PBVS cu z descrescând + bbox centering | distanță < 1.5m ȘI pixel err < 50 → FINAL |
| **FINAL** | IBVS: roll/pitch din pixel error, z↓ @ 0.05 m/s | pixel err < 20 px susținut 1s → HOLD |
| **HOLD** | position hold pe (p_obj.xy, 0.4m) | susținut 2s → DONE |
| **DONE** | log + marchează visited + pop next target | next target → APPROACH; lipsă → RETURN |

**Reguli critice**:

1. **Guard conditions, nu evenimente** — verificate la fiecare tick: `if det_age > 1000ms goto COAST`. Nu `on_detection_lost`. Asta evită race conditions.
2. **Hysteresis pe tranziții** — praguri asimetrice:
   - APPROACH → COAST: det_age > 1000 ms
   - COAST → APPROACH: det_age < 200 ms ȘI ≥ 3 frame-uri OK
3. **Timeout per stare** — fără care rămâi blocat:
   ```
   SEARCH: 60s, INITIALIZE: 10s, APPROACH: 30s, COAST: 5s,
   ALIGN: 8s, FINAL: 15s, HOLD: 3s
   ```

### 9.3 SM3 — Per-Object Track Health

```
NO_TRACK → TENTATIVE → CONFIRMED ⇄ COASTING → LOST
                            ↓
                         STALE (vechi dar still-valid)
                            ↓ (on re-see)
                         CONFIRMED
```

- **TENTATIVE**: 1–2 observații, low evidence. Nu se folosește pentru control.
- **CONFIRMED**: ≥ 3 observații, trace(P) < 1.0. Track stabil.
- **COASTING**: nu observat, age < 5s. Predicție-only, încă "de încredere".
- **STALE**: nu observat, age > 5s, dar plauzibil. Util pentru misiuni secvențiale lungi.
- **LOST**: eliberează slot-ul.

### 9.4 Interacțiunea SM-urilor

```
SM1 ──supervises──→ SM2 ──reads──→ SM3 (per obiect)
                     ↓
                  intent {type, target_pos, speed_cap, ...}
                     ↓
              Action Layer (PBVS/IBVS/Search pattern)
                     ↓
              velocity_setpoint → PX4 inner loop
```

**Decuplare critică**: SM2 emite _intenții_, nu comenzi. Action Layer convertește
intent → velocity. Asta permite înlocuirea controlerului fără modificare în logica
de misiune; permite testarea SM2 cu action layer mock.

### Referințe state machines & visual servoing

- **Chaumette & Hutchinson, IEEE RAM 2006-2007** — "Visual Servo Control. I. Basic Approaches" și "II. Advanced Approaches". Fundația IBVS/PBVS.
- **Wynn & McLain, ICUAS 2019** — "Visual Servoing for Multirotor Precision Landing in Daylight and After-Dark Conditions". VDOF IBVS, drone precision landing.
- **Springer et al., arXiv 2403.03806 (2024)** — "A Precision Drone Landing System using Visual and IR Fiducial Markers and a Multi-Payload Camera". Multi-scale camera switching, aterizare 0.19m eroare de la 168m.
- **Wang et al., arXiv 2411.08144 (2024)** — "Visual Tracking with Intermittent Visibility: Switched Control Design and Implementation" (SVT). Modelare formală TRACK/RECOVER cu garanție Lyapunov. **Direct aplicabil pentru tranziția APPROACH ⇄ COAST.**
- **Maravall et al., Front. Neurorobotics 2017** — "Navigation and Self-Semantic Location of Drones in Indoor Environments by Combining the Visual Bug Algorithm and Entropy-Based Vision".
- **Kortenkamp & Simmons, Springer Handbook of Robotics 2008, cap. 8** — "Robotic Systems Architectures and Programming". Pattern-uri hierarchical FSM.
- **Colledanchise & Ögren, CRC 2018** — "Behavior Trees in Robotics and AI". Alternativă BT pentru 50+ stări (FSM rămâne mai clar pentru < 15 stări).
- **Marder-Eppstein et al., ICRA 2010** — ROS Navigation Stack. Pattern decision/action layer separation.
- **Macenski et al., 2020** — Nav2. Versiunea modernă.

---

## 10. Stratul 6 — Controler hibrid PBVS → IBVS

### 10.1 PBVS (faza APPROACH)

Eroare în world frame:
```
e_W = p_obj_W − p_drone_W
v_cmd_W = K_p · e_W       // saturat la v_max
yaw_cmd = atan2(e_W.y, e_W.x)
```

Velocity comand transmis către PX4 inner loop. Yaw smooth ramping pentru evitare
saltului.

### 10.2 IBVS (faza FINAL)

Eroare în pixeli:
```
e_pixel = (cx_bbox, cy_bbox) − (cx_image, cy_image)
roll_cmd  = -K_p · e_pixel.x − K_d · de_pixel.x/dt
pitch_cmd = -K_p · e_pixel.y − K_d · de_pixel.y/dt
z_rate_cmd = -0.05  // m/s, descindere lentă
yaw_freeze = true   // nu rotești, pierzi obiectul
```

### 10.3 Tranziția hibridă

Smooth blending într-o zonă de overlap (ex: 1.3m–1.7m distanță) pentru a evita
salt brusc:
```
α = clamp((d - 1.3) / 0.4, 0, 1)
v_cmd = α · v_pbvs + (1-α) · v_ibvs
```

### 10.4 Evitare obstacole

APF (Artificial Potential Field) cu obstacolele cunoscute:
```
v_repulse = Σᵢ K_rep · (d_safe − d_i) · n̂_i   pentru d_i < d_safe
v_cmd_final = v_cmd + v_repulse
```

Sau VFH (Vector Field Histogram) dacă obstacolele sunt detectate dinamic.

---

## 11. Pași de implementare — ordine obligatorie

1. **Toolchain & simulator**: MCUXpresso SDK pentru RT1176; EdgeTPU SDK; Gazebo Garden + PX4 SITL pe Linux. Quadcopter `x500` virtual.
2. **Ego-motion EKF**: Eigen (PC), apoi CMSIS-DSP (target). Test izolat: după 60s hover, drift poziție < 0.5m, yaw < 5°.
3. **Detector EdgeTPU**: antrenament YOLOv8n cu Domain Randomization. Target: > 90% recall, > 20 fps.
4. **Tracker 2D**: test cu drop-out artificial 30% — track-ul supraviețuiește gaps de 1s.
5. **EKF de hartă**: după 5s zbor lateral, ||p_obj_est − p_GT|| < 0.3m, trace(P) < 0.5.
6. **Loop closure pe yaw**: test 2 minute zbor în pătrat, drift yaw < 2° cu re-observații (vs 10-30° fără).
7. **State machines izolat**: test cu evenimente mock, determinism, fără dead-locks.
8. **Controlere PBVS/IBVS izolat**: PBVS eroare < 0.2m; IBVS centrare pe marker static.
9. **Integrare end-to-end** în SITL — scenariul de test (secțiunea 12).
10. **Portare pe RT1176**: înlocuire Eigen → CMSIS-DSP; profiling per loop (buget 20ms @ 50Hz); test HIL.

**Recomandare implementare incrementală pentru SM-uri**:

Prima iterație, simplificat:
- SM1: HOVER → MISSION → LANDING + EMERGENCY
- SM2: SEARCH → APPROACH → FINAL → DONE (fără COAST, ALIGN)
- SM3: doar CONFIRMED / LOST

Apoi adaugi pe rând când vezi simptomul:
1. COAST (când vezi flickering detecție)
2. ALIGN (când tranzițiile PBVS→IBVS sunt urâte)
3. INITIALIZE (când EKF de obiect converge lent)
4. STALE/COASTING (când extinzi la multi-obiect secvențial)

---

## 12. Scenariul de test în simulator

### 12.1 Lumea Gazebo (`test_arena.sdf`)

- **Cameră interior**: 20m × 20m × 5m. Podea texturată (parchet/marmură), pereți cărămidă/lemn. Ambient 0.4 + două surse direcționale soft.
- **Punct decolare**: origine (0,0,0), marker alb 50cm diametru.
- **Obiect-țintă**: cub roșu 30×30×30 cm la (8.0, 3.0, 0.15). HSV H=0±10, S>0.7, V>0.5. class_id=1, real_size=0.30m.
- **Distractoare**: dreptunghi maro la (5.0, -2.0, 0.20) 40×40×40 cm; cilindru verde la (12.0, 1.0, 0.15) 25cm diam.
- **Obstacole**: doi pilastri la (4.0, 0.0) și (10.0, 4.0), 30cm diam, 2m înălțime.
- **Randomizare** între rulări: poziție cub x∈[7,10], y∈[-1,4]; iluminare ±20%; bias IMU drift 0.01°/s.

### 12.2 Misiunea — secvență

| Faza | Stare SM2 | Acțiune | Tranziție-out |
|---|---|---|---|
| T=0 | BOOT | Hover @ 1.5m, EKF se stabilizează | După 3s |
| T=3s | SEARCH | Spirală pătrată, yaw 20°/s | 5 detecții consecutive |
| T_detect | INITIALIZE | Lateral oscilant ±1.5m, 3s, yaw fix | trace(P_obj) < 0.5 |
| — | APPROACH | PBVS spre obiect, max 1 m/s, alt coboară la 1.0m | d < 2.0m |
| (opțional) | COAST | Predict EKF, viteză /2, +0.3m alt | re-detecție |
| — | ALIGN | Centering bbox + descindere | d < 1.5m ȘI pixel err < 50 |
| — | FINAL | IBVS, descindere 0.05 m/s | pixel err < 20px susținut 1s |
| — | HOLD | Hover deasupra @ 0.4m | susținut 2s |
| — | DONE | Log eroare finală | — |

### 12.3 Criterii de succes

Pe 10 rulări randomizate:

- Detectare inițială: < 30s
- Timp total misiune: < 90s
- Eroare orizontală finală: < 0.15m în ≥ 8/10 rulări
- Zero coliziuni
- Drift yaw final: < 5° (vs ground truth Gazebo)

### 12.4 Logging obligatoriu

CSV @ 50ms cu: timestamp, p_drone (est & GT), yaw (est & GT), p_obj (est, trace(P)),
SM2 state, detection (conf, bbox center, age), velocity command.

Plot-uri post-run: traiectorie 2D, evoluție covarianță obiect, stări SM în timp.

---

## 13. Reguli stricte

- **Fără heap dinamic** după boot. Tot ce e runtime e static.
- **Fără senzori suplimentari** (GPS dezactivat în Gazebo).
- **Validare izolată** per pas înainte de integrare.
- **Quaternioni**, nu Euler, intern.
- **Compensare camera motion** în tracker 2D — non-negociabil pe dronă.
- **Yaw loop closure** funcțional înainte de misiuni > 1 minut.

---

## 14. Bibliografie completă

### 14.1 Ego-motion / VIO / VO pe MCU

1. Honegger, D., Meier, L., Tanskanen, P., Pollefeys, M. "An Open Source and Open Hardware Embedded Metric Optical Flow CMOS Camera for Indoor and Outdoor Applications". ICRA 2013.
2. He, Y., Wang, Y., Liu, C., Zhang, L. "PicoVO: A Lightweight RGB-D Visual Odometry Targeting Resource-Constrained IoT Devices". ICRA 2021.
3. Trawny, N., Roumeliotis, S.I. "Indirect Kalman Filter for 3D Attitude Estimation". University of Minnesota tech report 2005-002.
4. Kühne, J., Magno, M., Benini, L. "Low Latency Visual Inertial Odometry with On-Sensor Accelerated Optical Flow for Resource-Constrained UAVs". IEEE Sensors Journal, 2024.
5. *LEVIO: Lightweight Embedded Visual Inertial Odometry for Resource-Constrained Devices*. arXiv 2602.03294, 2026.
6. *Efficient and Accurate Downfacing Visual Inertial Odometry*. arXiv 2509.10021, 2025.
7. Suleiman, A., Zhang, Z., Carlone, L., Karaman, S., Sze, V. "Navion: A 2-mW Fully Integrated Real-Time Visual-Inertial Odometry Accelerator". IEEE JSSC 54(4), 2019.

### 14.2 EKF-SLAM monocular & inverse-depth

8. Davison, A.J. "Real-Time Simultaneous Localisation and Mapping with a Single Camera". ICCV 2003; TPAMI 2007 ("MonoSLAM").
9. Civera, J., Davison, A.J., Montiel, J.M.M. "Inverse Depth Parametrization for Monocular SLAM". IEEE TRO 24(5), 2008.
10. Solà, J., Vidal-Calleja, T., Civera, J., Montiel, J.M.M. "Impact of Landmark Parametrization on Monocular EKF-SLAM with Points and Lines". IJCV 97(3), 2012.
11. Yang, P., Shi, W. "Bearing-only Visual SLAM for Small Unmanned Aerial Vehicles in GPS-denied Environments". Machine Intelligence Research 10(5), 2013.
12. Mur-Artal, R., Tardós, J.D. "ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras". IEEE TRO 33(5), 2017.
13. Campos, C., Elvira, R., Gómez Rodríguez, J.J., Montiel, J.M.M., Tardós, J.D. "ORB-SLAM3". IEEE TRO 37(6), 2021.

### 14.3 Object-level SLAM

14. Yang, S., Scherer, S. "CubeSLAM: Monocular 3-D Object SLAM". IEEE TRO 35(4), 2019.
15. Nicholson, L., Milford, M., Sünderhauf, N. "QuadricSLAM: Dual Quadrics from Object Detections as Landmarks". IEEE RAL 4(1), 2019.
16. Hoiem, D., Efros, A.A., Hebert, M. "Putting Objects in Perspective". IJCV 80(1), 2008.

### 14.4 Multi-object tracking

17. Bewley, A., Ge, Z., Ott, L., Ramos, F., Upcroft, B. "Simple Online and Realtime Tracking" (SORT). ICIP 2016.
18. Wojke, N., Bewley, A., Paulus, D. "Simple Online and Realtime Tracking with a Deep Association Metric" (DeepSORT). ICIP 2017.
19. Zhang, Y. et al. "ByteTrack: Multi-Object Tracking by Associating Every Detection Box". ECCV 2022, arXiv 2110.06864.
20. Aharon, N., Orfaig, R., Bobrovsky, B.-Z. "BoT-SORT: Robust Associations Multi-Pedestrian Tracking". arXiv 2206.14651, 2022.
21. Cao, J., Pang, J., Weng, X., Khirodkar, R., Kitani, K. "Observation-Centric SORT: Rethinking SORT for Robust Multi-Object Tracking". CVPR 2023, arXiv 2203.14360.
22. Du, Y. et al. "StrongSORT: Make DeepSORT Great Again". IEEE TMM, 2023.

### 14.5 Visual servoing & precision landing

23. Chaumette, F., Hutchinson, S. "Visual Servo Control. I. Basic Approaches". IEEE Robotics & Automation Magazine 13(4), 2006.
24. Chaumette, F., Hutchinson, S. "Visual Servo Control. II. Advanced Approaches". IEEE RAM 14(1), 2007.
25. Wynn, J.S., McLain, T.W. "Visual Servoing for Multirotor Precision Landing in Daylight and After-Dark Conditions". ICUAS 2019.
26. Springer, J. et al. "A Precision Drone Landing System using Visual and IR Fiducial Markers and a Multi-Payload Camera". arXiv 2403.03806, 2024.
27. Wang, H. et al. "Visual Tracking with Intermittent Visibility: Switched Control Design and Implementation". arXiv 2411.08144, 2024.
28. Salem, M., Chand, A. "SMART-TRACK: A Novel Kalman Filter-Guided Sensor Fusion For Robust UAV Object Tracking in Dynamic Environments". arXiv 2410.10409, 2024.

### 14.6 Topological mapping & navigation

29. Maravall, D., de Lope, J., Fuentes, J.P. "Navigation and Self-Semantic Location of Drones in Indoor Environments by Combining the Visual Bug Algorithm and Entropy-Based Vision". Frontiers in Neurorobotics 11, 2017.
30. Teymouri, M.S., Bhattacharya, S. "Landmark-based Distributed Topological Mapping and Navigation in GPS-denied Urban Environments Using Teams of Low-cost Robots". arXiv 2103.03741, 2021.
31. Hornung, A., Wurm, K.M., Bennewitz, M., Stachniss, C., Burgard, W. "OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees". Autonomous Robots 34(3), 2013.
32. Oleynikova, H., Taylor, Z., Fehr, M., Siegwart, R., Nieto, J. "Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning". IROS 2017.
33. Tordesillas, J., How, J.P. "FASTER: Fast and Safe Trajectory Planner for Navigation in Unknown Environments". IEEE RAL, 2020.
34. Zhou, X. et al. "EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors". IEEE RAL 6(2), 2021. Și EGO-Planner-v2, 2022.

### 14.7 Software architecture

35. Kortenkamp, D., Simmons, R. "Robotic Systems Architectures and Programming". Springer Handbook of Robotics, cap. 8, 2008.
36. Colledanchise, M., Ögren, P. "Behavior Trees in Robotics and AI: An Introduction". CRC Press, 2018.
37. Marder-Eppstein, E. et al. "The Office Marathon: Robust Navigation in an Indoor Office Environment". ICRA 2010 (ROS Navigation Stack).
38. Macenski, S., Martín, F., White, R., Ginés Clavero, J. "The Marathon 2: A Navigation System". IROS 2020 (Nav2).

### 14.8 Survey-uri utile

39. *Vision-based Localization Methods Under GPS-Denied Conditions*. arXiv 2211.11988.
40. *Vision-Based Learning for Drones: A Survey*. arXiv 2312.05019, 2024.
41. Cadena, C. et al. "Past, Present, and Future of Simultaneous Localization and Mapping: Toward the Robust-Perception Age". IEEE TRO 32(6), 2016.

---

*Document compilat ca sinteză a stării artei (2008–2026) pentru implementare
target pe NXP i.MX RT1176 + EdgeTPU. Toate referințele sunt verificabile prin
DOI, arXiv ID sau venue + an.*