<!-- Chapter file extracted from objects_plan.md on 2026-05-17. -->
<!-- §N.M section anchors below are PRESERVED globally — do not renumber. -->
<!-- Master index: ../objects_plan.md (executive summary + §23 thesis MVP scope + ToC). -->
<!-- Source line range: 855,1068. -->

# Chapter 02_mcu_feasibility — MCU/ARM feasibility (§11)

WBS anchors: cycle/memory budgets across ARCH-L{1..7}

## 11. Feasibility detaliat pe MCU/ARM (RT1176 Cortex-M7 @ 800 MHz)

> Această secțiune răspunde la întrebarea critică: **ce parte din planul
> de mai sus chiar rulează pe i.MX RT1176, și cu ce buget?** Bazat pe
> măsurători reale din s111 (PXP+SIMD bench), characteristicile siliciului
> (256 KB ITCM, 512 KB DTCM, 1 MB OCRAM, 32 MB SDRAM extern), și
> capabilitățile FPU-ului hard single-precision al M7.

### 11.1 Resurse hardware disponibile

| Resursă | Cantitate | Note |
|---|---|---|
| M7 @ 800 MHz | 800 cicluri/µs | hard-float FPU single-precision (FPv5-d16); SIMD DSP-extension (`__USAD8`, `__SEL`, `__QADD8`, etc.) |
| Buget per frame @ 30 fps | **33.3 ms = 26.6M cicluri** | împărțit între ISR-uri, MicroPython, flow_task, detection_task, mission, action, lifter, anchor_forward |
| ITCM (m_text) | 252 KB | ~32 KB liber post-relocation; **single-cycle**, hard-wired la CPU |
| DTCM (m_data) | 256 KB | ~50 KB liber; single-cycle, dedicated bus |
| OCRAM | 1 MB | 916 KB = `.tpu_input` (load-bearing); ~50 KB liber pentru sectiuni numite |
| SDRAM (cached) | 16 MB | ~50-100× mai lent decât ITCM la cold fetch; ICache 16 KB absoarbe loop-uri |
| EdgeTPU @ 4 TOPS INT8 | ~32 ms/invoke yolo_1 (512×512) | 41 fps proven pipeline |
| PXP HW (2D pipe) | scale/rotate/CSC `<` tens of µs | 1.1 ms threshold proven s111 |
| eDMA 32 channels | ~24 free | unused; pot face prefetch tile staging |

### 11.2 Bugetul de cicluri per layer

Cifre din s111 + estimări conservative cu CMSIS-DSP `arm_mat_*_f32`:

| Layer | Operație critică | Cicluri estimate | µs @ 800 MHz | Rată | % CPU |
|---|---|---:|---:|---:|---:|
| **L1 EKF predict** (16×16 mat-mul, 2×) | `arm_mat_mult_f32(16×16)` | ~25K × 2 = 50K | 62 | 200 Hz | **1.2%** |
| **L1 EKF flow update** (3×3 inversion + Kalman gain) | small mat ops | ~40K | 50 | 100 Hz | **0.5%** |
| **L1 EKF baro update** | scalar | ~5K | 6 | 50 Hz | **0.03%** |
| **L1 EKF anchor update** (loop closure, rare) | 3×3 ops | ~80K | 100 | 1-2 Hz | **0.02%** |
| **L2 Detector** | EdgeTPU USB invoke + M7 NMS | proven | **~32 ms/frame** | 30 fps | **40%** (TPU offloaded; M7 doar coord) |
| **L3 Tracker (ByteTrack + IMU CMC)** | already in production | proven | ~2-3 ms | 30 fps | **8%** |
| **L4 Object lifter** (per tracklet, 3-state EKF) | bearing rot + inverse-depth update | ~12K | 15 | 30 tr/s | **0.05%** |
| **L4 Object map maintain** (32 sloturi × 6 KF/sec) | 6 KF updates/s × 10 µs | — | — | — | **0.01%** |
| **L5 Mission SM tick** | switch + guard checks (no math) | ~500 | 0.6 | 20 Hz | **0.001%** |
| **L5 Track health SM3** (32 obiecte × tick) | 32 × O(1) | ~3K | 4 | 20 Hz | **0.008%** |
| **L6 PBVS** | vec3 sub + mul + saturate | ~400 | 0.5 | 50 Hz | **0.003%** |
| **L6 IBVS** | 2 PD controllers | ~800 | 1 | 50 Hz | **0.006%** |
| **L6 APF obstacles** | 10 obstacole × distance | ~3K | 4 | 50 Hz | **0.02%** |
| **L6 send_velocity** (MAVLink encode + UART/UDP) | per anchor_forward | ~5K | 6 | 50 Hz | **0.03%** |

**Total estimat budget M7 pentru layers L1+L4+L5+L6**: **<2.5% CPU** la
30-50 Hz cadence. Restul (Layer 2 = 40% offloaded to TPU+USB, Layer 3
= 8% already proven) e deja parte din pipeline-ul existent.

**Marja libera M7 pentru toate cele 4 layers noi: ~95%.** Compute NU
este bottleneck.

### 11.3 Bugetul de memorie

| Modul | ITCM (.text) | SDRAM (.sdram_text+.sdram_bss) | OCRAM | Note |
|---|---:|---:|---:|---|
| `sentai_objects.cc` (Stage 1) | 0 (default sdram) | ~6 KB cod + 2 KB date | 0 | static `object_t map[32]` |
| `sentai_object_lifter.cc` (Stage 5) | 0 | ~8 KB cod + 3 KB date (per-obj EKF state) | 0 | calls CMSIS-DSP `arm_mat_*` (already linked) |
| `sentai_mission_sm.cc` (Stage 3) | 0 | ~4 KB cod + <1 KB date | 0 | enum + guards |
| `sentai_action_layer.cc` (Stage 4) | 0 | ~5 KB cod + <1 KB date | 0 | PBVS + IBVS + APF |
| MP bindings (objects, mission, servo) | landed în `liblibmicropython.a` → SDRAM | — | — | per `.ld` rule existing |
| **Total nou** | **0 KB** | **~30 KB SDRAM** | **0 KB** | Margine ITCM rămâne ~32 KB |

**Conclusie memorie**: Niciun layer nou nu atinge ITCM, OCRAM rămâne
neatins (TPU pipeline neaffectat), SDRAM are 16 MB → 30 KB e zgomot.

### 11.4 Punctele tari ale RT1176 pentru această sarcină

1. **FPU hard single-precision (FPv5-d16)** — toate operațiile float
   sunt single-cycle dispatched. Quaternion math, EKF Kalman gain,
   inverse-depth conversion — toate în hardware. Nu există overhead
   de soft-float.

2. **CMSIS-DSP deja linkat** (3.6 KB) — funcții `arm_mat_mult_f32`,
   `arm_mat_inverse_f32`, `arm_mat_cholesky_f32` disponibile.
   Implementate cu SIMD intrinsics (`__SMLAD`, `__SXTB16`). Stage 5
   poate folosi direct.

3. **PXP HW pentru orice 2D pixel transform** — scale, rotate (90°
   step), CSC, alpha blend. Tens of µs per operație. Stage 5 lifter
   poate folosi PXP pentru rectificare ROI bbox dacă apare nevoia.

4. **eDMA cu 24 canale libere** — Stage 5 poate face prefetch tile
   staging din SDRAM în DTCM pentru a face inner loops single-cycle
   (s111 a măsurat -386 µs prin DTCM staging).

5. **OCRAM `.tpu_input` deja alocat** — TPU pipeline are buffer
   dedicat, NU împart cu noi → zero contenție SEMC.

6. **`sentai.flow.anchor_forward` arhitectură deja probată** —
   pattern-ul fault-gated FreeRTOS task @ 10 Hz cu NaN/OOB/stale
   gates e direct refolosibil pentru mission SM + action layer.

### 11.5 Puncte critice / riscuri reale pe MCU

**R1 — Stabilitate numerică inverse-depth EKF (Stage 5)**:
Single-precision FP poate diverge dacă ρ (inverse depth) ajunge
foarte mic. Mitigare (deja în plan):
- Joseph form for covariance update (cost: extra 3×3 mul, ~5 µs)
- Clamp ρ_min ≥ 0.05 (depth_max = 20 m)
- Skip update dacă `trace(P)` crește post-update
- Reinitialize landmark când ρ devine negativ

Verdict: **gestionabil cu disciplina deja prevăzută** în fault model
Stage 5 F5/F6.

**R2 — SDRAM I-cache thrashing cu cod nou în `.sdram_text`**:
s111 a măsurat: `-O3 -funroll-loops` poate face naïve 7×7 box filter
**MAI ÎNCET** pe SDRAM cod (+28% time) din cauza unrolled loop
nepăsător la ICache 16 KB.

Mitigare:
- Default `-Os` pentru cod cold (mission_sm, objects, action_layer)
- Selectiv `-O3` doar pe inner loops verificate cu `objdump` + bench
- Layer 5 lifter EKF inner loop: măsurate cu DWT cycle counter prima
  oară, optimizat second pass

Verdict: **important, dar avem precedent**.

**R3 — Tight FreeRTOS scheduling când multe task-uri rulează simultan**:
Acum avem: REPL, camera, flow, anchor_forward, fs_task, http,
watchdog, dmesg. Adding mission_sm @ 20 Hz + action_layer @ 50 Hz +
lifter @ 30 Hz adaugă **3 task-uri noi**.

Mitigare:
- Toate la `tskIDLE_PRIORITY + 2` (matches anchor_forward fix din s113 P2)
- Stack `configMINIMAL_STACK_SIZE * 2` per task (= 16 KB POSIX, 2 KB ARM)
- Liveness check 500 ms în `_start()` exact ca anchor_forward
- Total task count rămâne sub 16 (FreeRTOS porter limit comfortable)

Verdict: **disciplină probată, fără surprize**.

**R4 — Loop closure rate spike când drona zboară prin "muzeu de markeri"**:
Dacă apar 10 obiecte cunoscute în câmp simultan, lifter poate
publica 10 LC events/frame × 30 fps = 300 events/sec → suprasolicită
anchor_forward queue.

Mitigare (deja în plan Stage 6 F3):
- Rate-limit LC publish la 2 Hz în lifter (drop majoritatea)
- LC events de prioritate înaltă vs anchor regulat
- Anchor_forward fault gate `dropped_stale` deja există

Verdict: **rate-limit la sursă, nu la consumer**.

**R5 — Camera pose timestamp jitter** (flow.anchor_pose age):
Lifter are nevoie de drone pose `T_W_B(t_observation)`. Dacă camera
+ flow update + IMU integrare introduce > 50 ms latență, bearing
math devine inacurat.

Mitigare:
- Stage 5 F1: skip frame dacă `src_ts_ms` > 200 ms vechi
- Timestamp camera frame chiar din CSI ISR (deja proven în
  detection_task.cc cu `frame_seq`)
- Latency budget total camera→pose ≤ 70 ms (33 ms frame + 25 ms TPU
  + 12 ms tracker) — sub pragul de 200 ms cu margine

Verdict: **margine de 3× — sigur**.

**R6 — Action layer velocity send via cf2 CRTP MTU 30 B**:
CRTP packet pentru velocity setpoint = port 7 (commander_generic),
12-13 octeți payload. Ușor sub MTU. Latență < 10 ms.

Verdict: **non-issue**, deja făcut în `sentai.crazy.send_flow`.

### 11.6 Verdictul final per layer

| Layer | Verdict feasibility M7 | Justificare |
|---|---|---|
| **L1** Ego-motion EKF | ⚠️ Feasible dar **DEFER** — folosim PX4 EKF2 / cf2 KF | Are sens doar dacă vrem firmware standalone fără PX4/cf2 — out of scope |
| **L2** Detector TPU | ✅ **Already proven** | 41 fps yolo_1 în producție |
| **L3** Tracker 2D | ✅ **Already proven** | ByteTrack + IMU CMC running |
| **L4** Map EKF (object slots) | ✅ Feasible | ~3 KB date, <0.1% CPU per tick, CMSIS-DSP disponibil |
| **L5** State machines | ✅ Trivial | ~0.01% CPU, < 1 KB date |
| **L6** Controller PBVS/IBVS | ✅ Trivial | ~0.05% CPU, math elementar |

**În ansamblu**:
- **Compute budget**: < 3% M7 nou folosit, 95% rămâne liber
- **Memory budget**: 0 KB ITCM (all `.sdram_text`), ~30 KB SDRAM
- **Toate pattern-urile necesare au precedent** în `sentai.flow.anchor_forward` / `sentai_tracker` / `sentai_objects` proposed
- **Cele 4 module noi** sunt foarte modeste comparativ cu ce există deja (TPU pipeline = 600+ KB SDRAM cod)
- **Risc rămas critic**: **stabilitate numerică inverse-depth EKF** (R1) — singura piesă cu istoric de divergence; mitigarea deja prevăzută în plan

### 11.7 Comparație cu literatura citată

Cifre raportate în papers pentru hardware similar:

| Sistem | Hardware | Sarcină comparabilă | Buget raportat |
|---|---|---|---|
| Honegger ICRA 2013 (PX4FLOW) | Cortex-M4F @ 168 MHz | Optical flow 250 Hz | Folosea ~60% CPU |
| He ICRA 2021 (PicoVO) | STM32F767 @ 216 MHz | VO 6-DoF 33 fps @ 320×240 | Folosea ~75% CPU |
| LEVIO arXiv 2602.03294 (2026) | RISC-V ultra-low-power | VIO complet 20 fps | < 100 mW |
| Navion JSSC 2019 | ASIC custom 65nm | VIO real-time | 2 mW |

RT1176 Cortex-M7 @ 800 MHz are **4-5× mai multă putere de calcul**
decât Cortex-M4 @ 216 MHz. Sarcina noastră concretă (L4-L6) e mai
ușoară decât VO completă (nu calculăm feature matches, nu rulăm
optimizare ne-liniară). Conclusion **conservativă**: avem margine
foarte mare.

### 11.8 Recomandare ordine prioritate validare buget

În stagiul Stage 9 (ARM bring-up + timing validation), valida în
ordinea:

1. **Stage 5 lifter EKF tick cost** — măsurat cu DWT, target < 200 µs/tick @ 30 Hz. Dacă depășește 500 µs → optimizare cu CMSIS-DSP Joseph form / inline matrix ops.
2. **Stage 3 mission SM tick cost** — < 50 µs target (e doar guard checks). Dacă depășește 100 µs → suspect timing bug, investigate.
3. **Stage 4 action layer tick cost** — < 100 µs @ 50 Hz. APF cu 10 obstacole = ~30 µs. PBVS = ~5 µs.
4. **Stage 1 object map operations** — `add/get/list` < 10 µs (e doar memcpy + bounds check).
5. **Total task heartbeat** — toate task-urile la dwell time per tick < 10% din period. Vizibil în `sentai.<task>.stats() → tick_us`.

Dacă oricare depășește 2× budget → fallback: reduce rate (50 Hz → 30
Hz pentru action_layer), tile-stage cu DTCM (precedent s111 Phase 5
DTCM staging −26%).

---

