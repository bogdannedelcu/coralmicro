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
