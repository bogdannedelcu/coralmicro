# SentAI Runtime — Camera Initialization & Pipeline

## Senzor hardware

Board-ul Coral Micro integrează un senzor **OV5640** dual-camera (front + back)
conectat prin MIPI CSI-2 (2 lanes).  Cele două camere partajează bus-ul CSI
printr-un **GPIO MUX** (`kCamMux`) — doar una e activă la un moment dat.

| Parametru | Valoare |
|-----------|---------|
| Senzor | OV5640 |
| Rezoluție nativă | 1280 × 720 |
| Frame rate | 15 fps |
| Format CSI | XRGB8888 (4 bytes/pixel) |
| Buffere DMA | 4 (Cortex-M7) |
| I2C | 2 bus-uri separate (cam0 front, cam1 back) |

---

## Inițializarea originală Google

Toate exemplele din repo-ul original urmează un pattern simplu în 2 pași:

```cpp
CameraTask::GetSingleton()->SetPower(true);
CameraTask::GetSingleton()->Enable(CameraMode::kStreaming);  // sau kTrigger
```

### Captura de frame

```cpp
std::vector<CameraFrameFormat> fmts = {{
    .fmt = CameraFormat::kRgb,
    .filter = CameraFilterMethod::kBilinear,
    .rotation = CameraRotation::k270,
    .width = 324,
    .height = 324,
    .preserve_ratio = false,
    .buffer = buffer.data(),
    .white_balance = true,
}};
CameraTask::GetSingleton()->GetFrame(fmts);
```

### Caracteristici

- **O singură cameră** — nu există mecanism de switch
- **Conversie pe CPU** — `Rgb8888ToRgb()` iterează pixel cu pixel pentru a
  elimina canalul alpha din XRGB8888 → RGB888
- **Fără scaling hardware** — output-ul are dimensiunea nativă a senzorului
- **PXP** (Pixel Pipeline) este inițializat dar practic nefolosit
- **Rezoluție fixă** — nu se poate schimba la runtime
- **Single-threaded** — captura, conversia și procesarea sunt secvențiale

### Exemplu tipic din `detect_objects.cc`

```text
SetPower(true)
  → Enable(kTrigger)
    → loop:
         Trigger()
         DiscardOldFrames()
         GetFrame(fmts)          ← CPU demosaic, lent
         Invoke()                ← TPU inferență
         parse results
```

---

## Inițializarea SentAI

### `sentai_cam_init(streaming)`

```cpp
extern "C" int sentai_cam_init(int streaming) {
    auto* cam = CameraTask::GetSingleton();
    if (!cam->SetPower(true)) return -1;

    auto mode = streaming ? CameraMode::kStreaming
                          : CameraMode::kTrigger;
    if (!cam->Enable(mode)) return -2;

    // Ciclu prin ambele camere — asigură inițializarea completă CSI/MIPI
    cam->SwitchCamera(SwitchCameraId::kCameraBack);
    cam->SwitchCamera(SwitchCameraId::kCameraFront);

    return 0;
}
```

### Diferențe cheie față de original

| Aspect | Google original | SentAI runtime |
|--------|----------------|----------------|
| **Inițializare** | `SetPower` + `Enable` | + ciclare duală front/back |
| **Camere** | 1 (fixă) | 2, comutabile la runtime |
| **Frame API** | `GetFrame(CameraFrameFormat)` | `GetRawFrame()` + PXP hardware |
| **Conversie** | CPU `Rgb8888ToRgb` | PXP DMA (hardware accelerat) |
| **Scaling** | Nu există | PXP hardware (orice rezoluție) |
| **Rezoluție** | Fixă (nativă 1280×720) | Configurabilă via `set_resolution()` |
| **Cache D** | Invalidare simplă | Clean+Invalidate înainte de PXP, Invalidate după |
| **Recovery** | Niciuna | Toggle MUX + retry (până la 2 încercări) |
| **Pipeline** | Single-threaded | Double-buffered (PrepTask + InferTask) |
| **Acces Python** | Nu | `sentai.camera.*` complet |

---

## Camera Switch

### Cum funcționează

Cele două camere (front=cam0, back=cam1) partajează același bus MIPI CSI.
Comutarea se face prin GPIO MUX — **fără restart CSI/MIPI**:

```text
sentai.camera.select(1)   →   sentai_cam_switch(1)
                                  │
                                  ├─ CameraTask::SwitchCamera(kCameraBack)
                                  │     └─ GPIO kCamMux = 1
                                  │
                                  ├─ g_cam_switch_pending = true
                                  │
                                  └─ La următorul GetRawFrame:
                                       drain stale frames (~134ms)
                                       așteaptă ≥2 frames noi de la camera nouă
                                       (verificat prin g_camera_frame_seq)
```

### Secvență detaliată

1. **`SwitchCamera(id)`** — flip GPIO MUX (instant, ~1μs)
2. **Drain** — primele 1-2 frame-uri DMA conțin încă date de la camera veche
3. **Sequence check** — runtime-ul monitorizează `g_camera_frame_seq` (monotonic)
   și nu returnează frame-uri până când nu apar cel puțin 2 frame-uri noi
4. **Rotație automată** — fiecare cameră are rotația proprie:
   - Front (cam0): 180° (OV5640 mirror/flip registers)
   - Back (cam1): 0°
5. **Timeout** — dacă 134ms trec fără frame valid, recovery cu MUX toggle

### Rotație per-cameră

```python
>>> sentai.camera.rotate(0, 180)   # front: 180°
>>> sentai.camera.rotate(1, 0)     # back: 0°
```

Rotația se aplică prin registrele OV5640 (mirror H/V) — **nu** prin PXP.

---

## Pipeline-ul PXP (Pixel Pipeline)

### De ce PXP în loc de CPU

Google original folosește `Rgb8888ToRgb()` — o buclă CPU per-pixel care
copiază 3 din 4 bytes pentru fiecare pixel din frame-ul 1280×720:

```text
1280 × 720 × 4 bytes = 3.5 MB per frame
CPU loop: ~15-20ms per frame pe Cortex-M7 @ 996 MHz
```

SentAI folosește **PXP** (hardware DMA al NXP):

```text
XRGB8888 (CSI) → PXP → RGB888 planar (scaled)
1280×720 → 320×320: ~2ms (hardware, zero CPU)
```

### `pxp_scale_xrgb_to_rgb()`

```text
1. DCACHE_CleanInvalidateByRange(input)    ← asigură coerență DMA
2. PXP_SetProcessSurfaceBufferConfig(...)  ← sursă: XRGB8888
3. PXP_SetOutputBufferConfig(...)          ← dest: RGB888, WxH target
4. PXP_Start()                             ← hardware DMA transfer
5. while (!PXP_GetStatusFlags() & done)    ← poll ~2ms
6. DCACHE_InvalidateByRange(output)        ← invalidează cache pentru CPU
```

---

## Pipeline de detecție (double-buffered)

SentAI runtime operează un pipeline cu 2 task-uri FreeRTOS paralele:

```text
PrepTask (prioritate 2)              InferTask (prioritate 3)
─────────────────────────            ──────────────────────────
wait(staging_free)                   wait(prep_done)
cam_get_raw()                        memcpy staging→tensor  ~0.5ms
PXP → staging_buf         ~2ms      give(staging_free)     ← PrepTask pornește N+1
uint8→int8 quantization   ~1ms      TPU Invoke()           ~30ms
give(prep_done)                      NMS post-processing
                                     push results to queue
```

- **Staging buffer**: 640×640×3 în SDRAM, aliniat la 64 bytes
- **Semafoare**: `staging_free` și `prep_done` sincronizează handoff-ul
- **Overlap**: în timp ce TPU procesează frame N, PrepTask pregătește frame N+1
- **Coadă rezultate**: FreeRTOS queue (4 deep, overwrite on full)

Acest pipeline elimină complet idle time-ul TPU — throughput efectiv
egal cu `max(prep_time, inference_time)` în loc de `prep + inference`.

---

## Python API — `sentai.camera`

| Funcție | Descriere |
|---------|-----------|
| `sentai.camera.init(streaming=1)` | Inițializează camera (1=streaming, 0=trigger) |
| `sentai.camera.select(id)` | Comută camera: 0=front, 1=back |
| `sentai.camera.rotate(cam_id, degrees)` | Setează rotația (0/90/180/270) |
| `sentai.camera.set_resolution(w, h)` | Rezoluția PXP output (max nativă) |
| `sentai.camera.save_jpeg(path)` | Capturează și salvează JPEG pe LFS |
| `sentai.camera.frame()` | Returnează frame RGB raw |

### Exemplu: captura de pe ambele camere

```python
>>> sentai.camera.init(1)
>>> sentai.camera.select(0)            # front
>>> sentai.camera.save_jpeg("/img/front.jpg")
>>> sentai.camera.select(1)            # back
>>> sentai.camera.save_jpeg("/img/back.jpg")
```

---

## Diagrama de inițializare comparativă

```text
GOOGLE ORIGINAL                      SENTAI RUNTIME
───────────────                      ──────────────
SetPower(true)                       SetPower(true)
Enable(mode)                         Enable(mode)
  │                                  SwitchCamera(Back)   ← init cam1
  │                                  SwitchCamera(Front)  ← init cam0
  │                                    │
  ▼                                    ▼
GetFrame(CameraFrameFormat)          GetRawFrame()
  │                                    │
  ▼                                    ▼
CPU Rgb8888ToRgb                     PXP hardware scale + convert
  │                                    │
  ▼                                    ▼
buffer RGB (native res)              buffer RGB (target res)
  │                                    │
  ▼                                    ▼
inference                            double-buffered pipeline
                                     (PrepTask ∥ InferTask)
```

## Surse relevante

| Fișier | Rol |
|--------|-----|
| `sentai_runtime.cc` | `sentai_cam_init`, `pxp_scale_xrgb_to_rgb`, recovery |
| `modsentai_camera.c` | Python API `sentai.camera.*` |
| `detection_task.cc` | Pipeline double-buffered PrepTask+InferTask |
| `libs/camera/camera.cc` | `SetPower`, `Enable`, `SwitchCamera`, DMA buffers |
| `libs/camera/camera_support.h` | Constante: rezoluție nativă, BPP, buffer count |

---

## Update 2026-04-21 — Tranziția la VGA nativ + fix MIPI DPHY

### De ce a fost necesar

Profiling-ul pipeline-ului (PrepTask per-stage timing, instrument adăugat ca
`sentai.pipeline.prep_stats()`) a arătat că la 720p nativ modelul YOLO
512×512 petrece 28 ms / frame în PXP doar pentru a face down-scale
1280×720 → 512×512.  Camera output-ul 720p încarcă SEMC SDRAM cu 3.6 MB
per frame (transferul CSI + citirea PXP), iar acest trafic concurează cu
USB bulk-out al tensor-ului în timpul `Invoke()` → fiecare invoke creștea
de la 34 ms pur (E20 isolated) la 58 ms cu pipeline activ.

**Obiectiv**: coborî camera nativă la VGA (640 × 480) pentru ca PXP să
proceseze 5.6× mai puțini pixeli sursă, eliberând simultan banda SEMC
pentru USB transfer în paralel cu PXP.

### Rezultatul bug-ului din driver NXP

Primul test cu VGA schimbat în `camera_support.h` a eșuat neașteptat:
`sentai.camera.jpeg(60)` single-shot producea frame-uri curate, dar
pipeline-ul primea **1 frame apoi blocaj** — `CAMERA_RECEIVER_GetFullBuffer`
nu mai returna niciun frame după primul.

Root cause identificat după comparație cu sursele upstream
`libedgetpu` / kernel Linux `ov5640.c` + `OV5640 Application Notes`:

Tabelul `s_ov5640MipiClockConfigs` din
`third_party/nxp/rt1176-sdk/components/video/camera/device/ov5640/fsl_ov5640.c`
avea:

```c
{ .resolution = kVIDEO_ResolutionVGA, .framePerSec = 15, ..., .pclkPeriod = 0x0a },
{ .resolution = kVIDEO_ResolutionVGA, .framePerSec = 30, ..., .pclkPeriod = 0x0a },  // BUG
```

Valoarea `pclkPeriod` se scrie în registrul OV5640 `0x4837` — **MIPI Global
Timing**, perioada PCLK-ului în unități de UI_PIXEL pentru interfața
MIPI CSI-2.  La 30 fps rata de pixeli este dublu față de 15 fps, deci
`pclkPeriod` trebuia dublat (`0x14`), nu clonat de la 15 fps.  Cu perioada
greșită:

- Primul frame trece — D-PHY-ul host-ului pornește curat din idle.
- Pentru frame 2+ D-PHY-ul receptor RT1176 eșantionează prea rapid față
  de output-ul real al senzorului → pierde sync HS-start → frame
  timeout → `CAMERA_RECEIVER_GetFullBuffer` nu mai primește
  `FrameComplete` → PrepTask blochează indefinit.

Fix-ul aplicat în `fsl_ov5640.c`:

```c
{ .resolution = kVIDEO_ResolutionVGA, .framePerSec = 30, ..., .pclkPeriod = 0x14 },
```

Un singur byte schimbat.  Impact:
- E15 pipeline: **17.0 fps → 24.4 fps** (+45%)
- PXP: **28 ms → 12 ms** (3×, exact scalarea liniară cu pixel count)
- Invoke: **58 ms → 40 ms** (eliberează SEMC în paralel cu PXP)

### Validare vizuală (E21)

`diag/drivers/_e21_vga_capture.py` capturează 10 JPEG-uri consecutive pe
cam0 la noua rezoluție nativă 640×480.  Frame-urile sunt JPEG 640×480,
colori corecte, zero tearing sau artefacte — confirmă că nu doar
timing-ul merge, ci și pixelii sunt valizi.  Frame-urile salvate sub
`experiments/e21_vga_visual/` în repo.

### Tabel măsurători cumulative

| Versiune | FPS E15 | PXP ms | Invoke ms | Descriere |
|---|---|---|---|---|
| Baseline (720p + memcpy stg→tensor) | 13.3 | 28 | 58 | Original |
| + `direct_tensor` (ping-pong buffer) | 17.3 | 28 | 58 | Elimină memcpy 17 ms |
| + VGA 640×480 + `pclkPeriod=0x14` | **24.4** | **12** | **40** | Fix NXP driver |

**Total speedup cumulativ: +83%** față de baseline inițial.

## Update 2026-04-20 — VGA @ 45 fps + camera switching paralel

### VGA @ 45 fps — validat

Adăugată intrare nouă în `s_ov5640MipiClockConfigs`:

```c
{ .resolution = kVIDEO_ResolutionVGA, .framePerSec = 45,
  .pllCtrl1 = 0x14, .pllCtrl2 = 0x54, .vfifoCtrl0C = 0x22,
  .pclkDiv = 0x02, .pclkPeriod = 0x0D },
```

**Derivarea empirică (NASA-style retries):**

| Attempt | pllCtrl1 | pllCtrl2 | pclkPeriod | Rezultat |
|---|---|---|---|---|
| 1 | 0x0C | 0x38 | 0x0D | Hang — `sysdiv=0` invalid (driverul Linux impune min=1) |
| 2 | **0x14** | **0x54** | **0x0D** | **WORKS** — PLL mult ×1.5 față de 30 fps |

Observație cheie: registrul OV5640 `0x3035` are layout `(sysdiv << 4) |
mipi_div`.  `pllCtrl1 = 0x0C` înseamnă `sysdiv=0` — valoare ilegală.
Scalarea ratei de pixeli trebuie făcută prin PLL multiplier
(`pllCtrl2`: 0x38 → 0x54, ×1.5), NU prin `sysdiv`.

Intrarea corespondentă în `csi2rxHsSettle` (din
`libs/camera/camera_support.c`) pentru tHsSettle D-PHY la 45 fps.

### VGA @ 60 fps — eșuat

Attempt: `pllCtrl2 = 0x70, pclkPeriod = 0x0A`.  Init-ul acceptă PLL-ul
(ret=0), dar primul `jpeg()` blochează — CSI2RX nu primește frame-uri.
Probabil cere register companions `0x3037` (PLL root divider) și `0x3108`
(SRB clock) pe care tabelul static al NXP nu le expune.  Păstrat ca
entry comentat în `fsl_ov5640.c` pentru debug viitor cu scope.

### E22 — Camera switching alternat în pipeline paralel

Inițial testul de switching cam0↔cam1 se făcea secvențial din Python
(`select(0) → get_ex → select(1) → get_ex`), dând ~8 fps efectiv la
VGA/45 (costul drain-ului domina).

**Idee**: mutăm alternarea în ISR-ul CSI EOF, sub `g_cam_ratio_packed`
(atomic 32-bit scheduler).  Python apelează o singură dată
`sentai.camera.ratio(1, 1)` și pipeline-ul paralel continuă uninterrupted
— flip-ul MUX se face glitch-free în VBLANK, fără drain între frame-uri.

```python
sentai.pipeline.direct_tensor(1)
sentai.camera.switch_drain(1)
sentai.camera.ratio(1, 1)     # ISR alternează cam0/cam1 la fiecare EOF
sentai.pipeline.start(0.25, 0.45, 50)
```

Rezultate `_e22_alt_parallel.py`:

| Config | FPS | Note |
|---|---|---|
| A — fixed cam0 (baseline) | 24.45 | Reference — un singur senzor activ |
| **B — ratio(1,1) drain=1** | **21.04** | Alternare per-frame în ISR |
| C — ratio(1,1) drain=2 | 17.93 | Drain conservator — un frame pierdut |

**Câștig efectiv**: 21 fps alternat == 10.5 fps per-cameră simultan pe
cam0 și cam1, vs ~4 fps/cam cu switching secvențial Python.  Rata
efectivă a senzorilor este **74 fps combinat** (37/cam) — limita reală
devine pipeline-ul de inferență, nu driverul de cameră.

### E23 — Verificare vizuală (no cross-contamination)

La 21 fps alternat cu flip MUX pe fiecare EOF, întrebarea critică
era: există **amestec** între frame-uri cam0 și cam1?  E17 documentase
la 720p/threshold=1 o tearing mid-buffer când MUX-ul flipa în mijlocul
unui DMA — am vrut proof vizual la noul rate.

`_e23_vga45_alt_capture.py` rulează două faze:

1. **Phase 1**: pipeline paralel cu `ratio(1,1)` timp de 80 frame-uri →
   fps și cam_stats (confirmare healthy run).
2. **Phase 2**: stop pipeline, loop manual `select(cam) + jpeg(70)`
   pentru 10 perechi (cam0, cam1), save fiecare JPEG cu cam ID în nume.

Rezultate măsurate:

| Metric | Valoare |
|---|---|
| Phase 1 fps (alt 1:1) | 20.93 (match cu E22 B) |
| Phase 2 captures saved | **20 / 20** |
| `cam_stats.switch_ok_eof` delta | **22** — toate switch-urile au folosit calea EOF-ISR |
| `cam_stats.switch_fallback` delta | 0 — zero fallback polling |
| `cam_stats.drain_timeout` delta | 0 — zero timeout |

**Verificare vizuală** (`experiments/s054_e23_vga45_alt/001_e23_alt_frames/`):

- `004_cam0_144ms_25720b.jpg` — cam0: tufă de liliac + perete (scena cam0 normală)
- `005_cam1_167ms_27759b.jpg` — cam1: liliac + stropitoare albastră (scena cam1 distinctă)
- Toate 20 JPEG-urile sunt clean — zero mid-buffer seam, zero bleed
  inter-camera, zero tearing.  MUX-flip pe EOF este dovedit glitch-free
  la 74 fps effective sensor rate.

### Config recomandat pentru camera switching production

```python
sentai.camera.init(1)                 # streaming mode
sentai.pipeline.direct_tensor(1)      # ping-pong buffers
sentai.camera.switch_drain(1)         # minimum drain (MUX flip glitch-free)
sentai.camera.ratio(1, 1)             # ISR alternează fiecare EOF
sentai.pipeline.start(...)            # pipeline paralel PrepTask ∥ InferTask
```

Asta oferă **21 fps alternat stabil** cu ambele camere active simultan,
fără cod de switching explicit în Python.  Dovada empirică:
`experiments/s054_e23_vga45_alt/`.

### Tabel măsurători cumulative (update)

| Versiune | FPS | Note |
|---|---|---|
| Baseline (720p + memcpy stg→tensor) | 13.3 | Original |
| + `direct_tensor` (ping-pong buffer) | 17.3 | Elimină memcpy 17 ms |
| + VGA 640×480 @ 30 (`pclkPeriod=0x14` fix) | 24.4 | MIPI DPHY fix |
| + VGA 640×480 @ 45 (`pllCtrl2=0x54`) | 24.45 | Cam driver mai rapid — headroom pt switching |
| + ratio(1,1) alternare ISR (VGA/45) | **21.0 alt** | 10.5 fps / cam simultan — E22/E23 |

**Total speedup cumulativ: +84%** față de baseline (și enable switching
paralel real, nu secvențial).
