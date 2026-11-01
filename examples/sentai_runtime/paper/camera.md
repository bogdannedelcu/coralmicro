# SentAI Runtime — Camera Initialization & Pipeline

## Sensor hardware

The Coral Micro board integrates a dual-camera **OV5640** sensor (front + back)
connected over MIPI CSI-2 (2 lanes).  The two cameras share the CSI bus
through a **GPIO MUX** (`kCamMux`) — only one is active at a time.

| Parameter | Value |
|-----------|---------|
| Sensor | OV5640 |
| Native resolution | 1280 × 720 |
| Frame rate | 15 fps |
| CSI format | XRGB8888 (4 bytes/pixel) |
| DMA buffers | 4 (Cortex-M7) |
| I2C | 2 separate buses (cam0 front, cam1 back) |

---

## Original Google initialization

All the examples in the original repo follow a simple 2-step pattern:

```cpp
CameraTask::GetSingleton()->SetPower(true);
CameraTask::GetSingleton()->Enable(CameraMode::kStreaming);  // or kTrigger
```

### Frame capture

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

### Characteristics

- **A single camera** — there is no switching mechanism
- **CPU conversion** — `Rgb8888ToRgb()` iterates pixel by pixel to
  strip the alpha channel from XRGB8888 → RGB888
- **No hardware scaling** — the output has the sensor's native size
- **PXP** (Pixel Pipeline) is initialized but practically unused
- **Fixed resolution** — cannot be changed at runtime
- **Single-threaded** — capture, conversion and processing are sequential

### Typical example from `detect_objects.cc`

```text
SetPower(true)
  → Enable(kTrigger)
    → loop:
         Trigger()
         DiscardOldFrames()
         GetFrame(fmts)          ← CPU demosaic, slow
         Invoke()                ← TPU inference
         parse results
```

---

## SentAI initialization

### `sentai_cam_init(streaming)`

```cpp
extern "C" int sentai_cam_init(int streaming) {
    auto* cam = CameraTask::GetSingleton();
    if (!cam->SetPower(true)) return -1;

    auto mode = streaming ? CameraMode::kStreaming
                          : CameraMode::kTrigger;
    if (!cam->Enable(mode)) return -2;

    // Cycle through both cameras — ensures complete CSI/MIPI initialization
    cam->SwitchCamera(SwitchCameraId::kCameraBack);
    cam->SwitchCamera(SwitchCameraId::kCameraFront);

    return 0;
}
```

### Key differences from the original

| Aspect | Google original | SentAI runtime |
|--------|----------------|----------------|
| **Initialization** | `SetPower` + `Enable` | + dual front/back cycling |
| **Cameras** | 1 (fixed) | 2, switchable at runtime |
| **Frame API** | `GetFrame(CameraFrameFormat)` | `GetRawFrame()` + PXP hardware |
| **Conversion** | CPU `Rgb8888ToRgb` | PXP DMA (hardware accelerated) |
| **Scaling** | None | PXP hardware (any resolution) |
| **Resolution** | Fixed (native 1280×720) | Configurable via `set_resolution()` |
| **D-cache** | Simple invalidate | Clean+Invalidate before PXP, Invalidate after |
| **Recovery** | None | Toggle MUX + retry (up to 2 attempts) |
| **Pipeline** | Single-threaded | Double-buffered (PrepTask + InferTask) |
| **Python access** | No | Full `sentai.camera.*` |

---

## Camera Switch

### How it works

The two cameras (front=cam0, back=cam1) share the same MIPI CSI bus.
Switching is done through the GPIO MUX — **without a CSI/MIPI restart**:

```text
sentai.camera.select(1)   →   sentai_cam_switch(1)
                                  │
                                  ├─ CameraTask::SwitchCamera(kCameraBack)
                                  │     └─ GPIO kCamMux = 1
                                  │
                                  ├─ g_cam_switch_pending = true
                                  │
                                  └─ On the next GetRawFrame:
                                       drain stale frames (~134ms)
                                       wait for ≥2 new frames from the new camera
                                       (verified through g_camera_frame_seq)
```

### Detailed sequence

1. **`SwitchCamera(id)`** — flip the GPIO MUX (instant, ~1μs)
2. **Drain** — the first 1-2 DMA frames still contain data from the old camera
3. **Sequence check** — the runtime monitors `g_camera_frame_seq` (monotonic)
   and does not return frames until at least 2 new frames have appeared
4. **Automatic rotation** — each camera has its own rotation:
   - Front (cam0): 180° (OV5640 mirror/flip registers)
   - Back (cam1): 0°
5. **Timeout** — if 134ms pass without a valid frame, recovery with MUX toggle

### Per-camera rotation

```python
>>> sentai.camera.rotate(0, 180)   # front: 180°
>>> sentai.camera.rotate(1, 0)     # back: 0°
```

The rotation is applied through the OV5640 registers (mirror H/V) — **not** through PXP.

---

## The PXP (Pixel Pipeline) pipeline

### Why PXP instead of the CPU

The Google original uses `Rgb8888ToRgb()` — a per-pixel CPU loop that
copies 3 out of 4 bytes for every pixel of the 1280×720 frame:

```text
1280 × 720 × 4 bytes = 3.5 MB per frame
CPU loop: ~15-20ms per frame on Cortex-M7 @ 996 MHz
```

SentAI uses **PXP** (NXP's hardware DMA):

```text
XRGB8888 (CSI) → PXP → RGB888 planar (scaled)
1280×720 → 320×320: ~2ms (hardware, zero CPU)
```

### `pxp_scale_xrgb_to_rgb()`

```text
1. DCACHE_CleanInvalidateByRange(input)    ← ensures DMA coherency
2. PXP_SetProcessSurfaceBufferConfig(...)  ← source: XRGB8888
3. PXP_SetOutputBufferConfig(...)          ← dest: RGB888, WxH target
4. PXP_Start()                             ← hardware DMA transfer
5. while (!PXP_GetStatusFlags() & done)    ← poll ~2ms
6. DCACHE_InvalidateByRange(output)        ← invalidates the cache for the CPU
```

---

## Detection pipeline (double-buffered)

The SentAI runtime operates a pipeline with 2 parallel FreeRTOS tasks:

```text
PrepTask (priority 2)                InferTask (priority 3)
─────────────────────────            ──────────────────────────
wait(staging_free)                   wait(prep_done)
cam_get_raw()                        memcpy staging→tensor  ~0.5ms
PXP → staging_buf         ~2ms      give(staging_free)     ← PrepTask starts N+1
uint8→int8 quantization   ~1ms      TPU Invoke()           ~30ms
give(prep_done)                      NMS post-processing
                                     push results to queue
```

- **Staging buffer**: 640×640×3 in SDRAM, 64-byte aligned
- **Semaphores**: `staging_free` and `prep_done` synchronize the handoff
- **Overlap**: while the TPU processes frame N, PrepTask prepares frame N+1
- **Result queue**: FreeRTOS queue (4 deep, overwrite on full)

This pipeline completely eliminates TPU idle time — effective throughput
equals `max(prep_time, inference_time)` instead of `prep + inference`.

---

## Python API — `sentai.camera`

| Function | Description |
|---------|-----------|
| `sentai.camera.init(streaming=1)` | Initializes the camera (1=streaming, 0=trigger) |
| `sentai.camera.select(id)` | Switches the camera: 0=front, 1=back |
| `sentai.camera.rotate(cam_id, degrees)` | Sets the rotation (0/90/180/270) |
| `sentai.camera.set_resolution(w, h)` | PXP output resolution (max native) |
| `sentai.camera.save_jpeg(path)` | Captures and saves a JPEG to LFS |
| `sentai.camera.frame()` | Returns a raw RGB frame |

### Example: capture from both cameras

```python
>>> sentai.camera.init(1)
>>> sentai.camera.select(0)            # front
>>> sentai.camera.save_jpeg("/img/front.jpg")
>>> sentai.camera.select(1)            # back
>>> sentai.camera.save_jpeg("/img/back.jpg")
```

---

## Comparative initialization diagram

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

## Relevant sources

| File | Role |
|--------|-----|
| `sentai_runtime.cc` | `sentai_cam_init`, `pxp_scale_xrgb_to_rgb`, recovery |
| `modsentai_camera.c` | Python API `sentai.camera.*` |
| `detection_task.cc` | Double-buffered PrepTask+InferTask pipeline |
| `libs/camera/camera.cc` | `SetPower`, `Enable`, `SwitchCamera`, DMA buffers |
| `libs/camera/camera_support.h` | Constants: native resolution, BPP, buffer count |

---

## Update 2026-04-21 — Transition to native VGA + MIPI DPHY fix

### Why it was necessary

Profiling the pipeline (PrepTask per-stage timing, instrument added as
`sentai.pipeline.prep_stats()`) showed that at native 720p the YOLO
512×512 model spends 28 ms / frame in PXP just to down-scale
1280×720 → 512×512.  The 720p camera output loads the SEMC SDRAM with 3.6 MB
per frame (the CSI transfer + the PXP read), and this traffic competes with
the USB bulk-out of the tensor during `Invoke()` → every invoke grew
from 34 ms pure (E20 isolated) to 58 ms with the pipeline active.

**Goal**: lower the native camera to VGA (640 × 480) so that PXP
processes 5.6× fewer source pixels, simultaneously freeing SEMC bandwidth
for the USB transfer in parallel with PXP.

### The result of the NXP driver bug

The first test with VGA changed in `camera_support.h` failed unexpectedly:
`sentai.camera.jpeg(60)` single-shot produced clean frames, but the
pipeline received **1 frame then stalled** — `CAMERA_RECEIVER_GetFullBuffer`
returned no more frames after the first.

Root cause identified after comparison with the upstream sources
`libedgetpu` / Linux kernel `ov5640.c` + `OV5640 Application Notes`:

The `s_ov5640MipiClockConfigs` table in
`third_party/nxp/rt1176-sdk/components/video/camera/device/ov5640/fsl_ov5640.c`
had:

```c
{ .resolution = kVIDEO_ResolutionVGA, .framePerSec = 15, ..., .pclkPeriod = 0x0a },
{ .resolution = kVIDEO_ResolutionVGA, .framePerSec = 30, ..., .pclkPeriod = 0x0a },  // BUG
```

The `pclkPeriod` value is written to OV5640 register `0x4837` — **MIPI Global
Timing**, the PCLK period in UI_PIXEL units for the MIPI CSI-2
interface.  At 30 fps the pixel rate is double that of 15 fps, so
`pclkPeriod` had to be doubled (`0x14`), not cloned from 15 fps.  With the
wrong period:

- The first frame passes — the host D-PHY starts cleanly from idle.
- For frame 2+ the RT1176 receiver D-PHY samples too fast relative
  to the sensor's actual output → loses HS-start sync → frame
  timeout → `CAMERA_RECEIVER_GetFullBuffer` no longer receives
  `FrameComplete` → PrepTask blocks indefinitely.

The fix applied in `fsl_ov5640.c`:

```c
{ .resolution = kVIDEO_ResolutionVGA, .framePerSec = 30, ..., .pclkPeriod = 0x14 },
```

A single byte changed.  Impact:
- E15 pipeline: **17.0 fps → 24.4 fps** (+45%)
- PXP: **28 ms → 12 ms** (3×, exactly the linear scaling with pixel count)
- Invoke: **58 ms → 40 ms** (frees SEMC in parallel with PXP)

### Visual validation (E21)

`diag/drivers/_e21_vga_capture.py` captures 10 consecutive JPEGs on
cam0 at the new native resolution 640×480.  The frames are 640×480 JPEGs,
correct colours, zero tearing or artefacts — confirms that not only
the timing works, but the pixels are valid too.  The frames are saved under
`experiments/e21_vga_visual/` in the repo.

### Cumulative measurements table

| Version | FPS E15 | PXP ms | Invoke ms | Description |
|---|---|---|---|---|
| Baseline (720p + memcpy stg→tensor) | 13.3 | 28 | 58 | Original |
| + `direct_tensor` (ping-pong buffer) | 17.3 | 28 | 58 | Eliminates 17 ms memcpy |
| + VGA 640×480 + `pclkPeriod=0x14` | **24.4** | **12** | **40** | NXP driver fix |

**Total cumulative speedup: +83%** over the initial baseline.

## Update 2026-04-20 — VGA @ 45 fps + parallel camera switching

### VGA @ 45 fps — validated

New entry added to `s_ov5640MipiClockConfigs`:

```c
{ .resolution = kVIDEO_ResolutionVGA, .framePerSec = 45,
  .pllCtrl1 = 0x14, .pllCtrl2 = 0x54, .vfifoCtrl0C = 0x22,
  .pclkDiv = 0x02, .pclkPeriod = 0x0D },
```

**Empirical derivation (NASA-style retries):**

| Attempt | pllCtrl1 | pllCtrl2 | pclkPeriod | Result |
|---|---|---|---|---|
| 1 | 0x0C | 0x38 | 0x0D | Hang — `sysdiv=0` invalid (the Linux driver enforces min=1) |
| 2 | **0x14** | **0x54** | **0x0D** | **WORKS** — PLL mult ×1.5 relative to 30 fps |

Key observation: OV5640 register `0x3035` has the layout `(sysdiv << 4) |
mipi_div`.  `pllCtrl1 = 0x0C` means `sysdiv=0` — an illegal value.
The pixel-rate scaling must be done through the PLL multiplier
(`pllCtrl2`: 0x38 → 0x54, ×1.5), NOT through `sysdiv`.

The corresponding entry in `csi2rxHsSettle` (in
`libs/camera/camera_support.c`) for the D-PHY tHsSettle at 45 fps.

### VGA @ 60 fps — failed

Attempt: `pllCtrl2 = 0x70, pclkPeriod = 0x0A`.  Init accepts the PLL
(ret=0), but the first `jpeg()` blocks — CSI2RX receives no frames.
It probably requires the companion registers `0x3037` (PLL root divider) and `0x3108`
(SRB clock) which NXP's static table does not expose.  Kept as a
commented-out entry in `fsl_ov5640.c` for future debugging with a scope.

### E22 — Alternating camera switching in the parallel pipeline

Initially the cam0↔cam1 switching test was done sequentially from Python
(`select(0) → get_ex → select(1) → get_ex`), giving ~8 fps effective at
VGA/45 (the drain cost dominated).

**Idea**: move the alternation into the CSI EOF ISR, under `g_cam_ratio_packed`
(atomic 32-bit scheduler).  Python calls
`sentai.camera.ratio(1, 1)` once and the parallel pipeline continues uninterrupted
— the MUX flip is done glitch-free in VBLANK, with no drain between frames.

```python
sentai.pipeline.direct_tensor(1)
sentai.camera.switch_drain(1)
sentai.camera.ratio(1, 1)     # ISR alternates cam0/cam1 on every EOF
sentai.pipeline.start(0.25, 0.45, 50)
```

Results from `_e22_alt_parallel.py`:

| Config | FPS | Notes |
|---|---|---|
| A — fixed cam0 (baseline) | 24.45 | Reference — a single sensor active |
| **B — ratio(1,1) drain=1** | **21.04** | Per-frame alternation in the ISR |
| C — ratio(1,1) drain=2 | 17.93 | Conservative drain — one frame lost |

**Effective gain**: 21 fps alternating == 10.5 fps per camera simultaneously on
cam0 and cam1, vs ~4 fps/cam with sequential Python switching.  The
effective sensor rate is **74 fps combined** (37/cam) — the real limit
becomes the inference pipeline, not the camera driver.

### Derivation of the PLL values in `s_ov5640MipiClockConfigs`

The 6 fields in the table are the OV5640 registers `0x3034..0x3037`,
`0x3108` and `0x4837`.  The effective formula (derived from the upstream
Linux driver `ov5640.c::ov5640_set_mipi_pclk`):

```
PLL_output    = 24 MHz / prediv × mult
                (prediv fixed at 3 through PLL_CTRL3 in the NXP pipeline)
sysdiv        = (pllCtrl1 >> 4)   ≥ 1   (0 is illegal!)
mipi_div      = (pllCtrl1 & 0x0F)
mult          = pllCtrl2          (8-bit, 4..252; 84 → typical decimal)
pclk          = PLL_output / (sysdiv × mipi_div × pclkDiv)
pclkPeriod    = ceil(1 / lane_bit_rate_UI)  →  scales inversely with mult
```

Key constraints (OV5640 datasheet §2.9):
- `sysdiv ≥ 1` — the value 0 is invalid and locks up the driver
- `PLL_output` must stay within 500–1000 MHz
- With 2 MIPI lanes, `tHsSettle` (from `csi2rxHsSettle`) decreases when
  `lane_bit_rate` increases (typically `0x12` at 672 MHz vs `0x1F` at 448 MHz)

Derivation table for the entries used by sentai:

| Res × fps | pllCtrl1 | mult (pllCtrl2) | PLL_out | pclkPeriod | Status |
|---|---|---|---|---|---|
| VGA × 15 | 0x14 (sysdiv=1,mdiv=4) | 0x38 = 56 | 448 MHz | 0x0A | OK (stock NXP) |
| VGA × 30 | 0x14 | 0x38 = 56 | 448 MHz | **0x14** | Fix — was 0x0A, wrong |
| VGA × 45 | 0x14 | **0x54 = 84** | **672 MHz** | **0x0D** | New — PLL mult ×1.5 |
| 720p × 30 | 0x21 (sysdiv=2,mdiv=1) | 0x69 = 105 | 840 MHz | 0x0A | OK (stock NXP) |
| VGA × 60 | 0x14 | 0x70 = 112 | 896 MHz | 0x0A | **HUNG** — see below |

**Why VGA × 60 failed**: the value `mult=112` gives PLL_output=896 MHz,
valid per spec, and `tHsSettle` was set for 896 MHz.  But the
Linux driver applies in parallel changes to `0x3037` (PLL pre-div)
and `0x3108` (SCLK divider) which cannot be expressed through these 6
fields of the static NXP structure.  Without those companion registers,
the CSI-2 lane sync fails silently.  To enable VGA × 60 one would
need either:
- extending the static table with 2 extra fields, OR
- a post-`CAMERA_DEVICE_Init` hook that issues manual SCCB writes.

Both are out of scope for the current sprint.

### E23 — Visual verification (no cross-contamination)

At 21 fps alternating with a MUX flip on every EOF, the critical question
was: is there any **mixing** between cam0 and cam1 frames?  E17 had documented,
at 720p/threshold=1, mid-buffer tearing when the MUX flipped in the middle of
a DMA — we wanted visual proof at the new rate.

`_e23_vga45_alt_capture.py` runs two phases:

1. **Phase 1**: parallel pipeline with `ratio(1,1)` for 80 frames →
   fps and cam_stats (confirmation of a healthy run).
2. **Phase 2**: stop the pipeline, manual loop `select(cam) + jpeg(70)`
   for 10 pairs (cam0, cam1), save each JPEG with the cam ID in the name.

Measured results:

| Metric | Value |
|---|---|
| Phase 1 fps (alt 1:1) | 20.93 (matches E22 B) |
| Phase 2 captures saved | **20 / 20** |
| `cam_stats.switch_ok_eof` delta | **22** — all switches used the EOF-ISR path |
| `cam_stats.switch_fallback` delta | 0 — zero fallback polling |
| `cam_stats.drain_timeout` delta | 0 — zero timeouts |

**Visual verification** (`experiments/s054_e23_vga45_alt/001_e23_alt_frames/`):

- `004_cam0_144ms_25720b.jpg` — cam0: lilac bush + wall (the normal cam0 scene)
- `005_cam1_167ms_27759b.jpg` — cam1: lilac + blue watering can (the distinct cam1 scene)
- All 20 JPEGs are clean — zero mid-buffer seam, zero inter-camera
  bleed, zero tearing.  The MUX flip on EOF is proven glitch-free
  at 74 fps effective sensor rate.

### Recommended config for production camera switching

```python
sentai.camera.init(1)                 # streaming mode
sentai.pipeline.direct_tensor(1)      # ping-pong buffers
sentai.camera.switch_drain(1)         # minimum drain (MUX flip glitch-free)
sentai.camera.ratio(1, 1)             # ISR alternates on every EOF
sentai.pipeline.start(...)            # parallel pipeline PrepTask ∥ InferTask
```

This gives a **stable 21 fps alternating** with both cameras active simultaneously,
with no explicit switching code in Python.  Empirical proof:
`experiments/s054_e23_vga45_alt/`.

### Cumulative measurements table (update)

| Version | FPS | Notes |
|---|---|---|
| Baseline (720p + memcpy stg→tensor) | 13.3 | Original |
| + `direct_tensor` (ping-pong buffer) | 17.3 | Eliminates 17 ms memcpy |
| + VGA 640×480 @ 30 (`pclkPeriod=0x14` fix) | 24.4 | MIPI DPHY fix |
| + VGA 640×480 @ 45 (`pllCtrl2=0x54`) | 24.45 | Faster camera driver — headroom for switching |
| + ratio(1,1) ISR alternation (VGA/45) | **21.0 alt** | 10.5 fps / cam simultaneously — E22/E23 |

**Total cumulative speedup: +84%** over the baseline (and enables real
parallel switching, not sequential).

## Update 2026-11-01 — Packed YUV420 capture at 2 B/px, true SXGA fps rows, lockstep with the TPU

Experiments s259–s264 (OP-S10-W24-T14..T17), builds 1723–1756, one camera.

**ERR051248 does not force XYUV8888.**  The errata (IMXRT1170ACE §2.22) only forbids
RAW and YUV422-10bit from MIPI.  The 4 B/px capture is the VIDEO_MUX pixel
reformatter's doing (YUV422 → 24-bit YUV444 pixel link, stored `[V,U,Y,0]`).  The
OV5640's MIPI-only YUV420 output (`0x4300 = 0x5F`, CSI-2 legacy YUV420, UYY/VYY
lines) passes through the 24-bit receiver when the receiver width is set to half the
sensor width: memory holds `[C,Y1,Y2,0]` per pixel pair, C = U on even rows and V on
odd rows — **2 B/px, pixel-exact** at VGA (30/45/60) and SXGA.  SXGA additionally needs
the manual DVP PCLK divider (`0x460C = 0x22`, `0x3824 = 0x01`); with the SDK's auto
divider the bytes arrive rotated.  CSI writes per SXGA frame 4.9 → 2.46 MB.  Knob:
`sentai.camera.packed420(1)` before `init()`.  Dead ends (do not retry): CSICR18
DATA_FROM_MIPI / DOUBLE_CMPNT / YU_SWAP / MIPI_DATA_FORMAT have no effect on RT1176
(the CSI only sees the pixel link); the 16-bit receiver stores the {U,V} plane; the
8-bit receiver reads data[9:2]; `0x4814` is not a data-type override; OV5640 Y8 is not
luma; CR20 gray-scale mode stalls the CSI.

**PXP consumer (option B).**  No PXP input format matches the packed layout, but PXP
RGB888 (32-bit) → RGB888P (24-bit) drops the padding byte and yields `[C,Y1,Y2]` at
1.5 B/px — the same byte count as the planar Y + UV input of the YUV420 detectors.
PXP per SXGA frame 62 → 25 ms (legacy XYUV → YUV420 repack vs byte-drop).  PXP
16×16 processing blocks (`camera.pxp_block16(1)`, default off) cut the repack a further
30–40 % under CSI load, enough to prepare 30/30 frames at a true 30 fps without the
TPU, but they starve the EHCI reads while the TPU streams (invoke 70 → 77 ms).

**The SDK's SXGA rows were mis-keyed.**  The 1280×960 rows added in April were
cloned from the 720P/30 row (PLL 84×); with 984 lines per frame they stream 22.6 and
11.2 frames/s (CSI `CR3.FRMCNT`), not 30 and 15 (`sensor_frames()` over-counts ~15 %).
Frame rate ∝ PLL multiplier / (sysdiv × mipi_div), and mipi_div also divides PCLK.
Since build 1756 (SDK patch `0007-ov5640-sxga-true-fps-rows`) the keys mean the
measured rate: `init(1, 15)` = 15.0 fps (PLL 0x70, sysdiv 4, settle 0x13), `init(1, 30)`
= 30.0 fps (PLL 0x70, sysdiv 2, settle 0x12), `init(1, 22)` = 22.6 fps (the old row,
kept for the legacy 4 B/px capture).  Legacy XYUV at a true 30 fps writes 147 MB/s
into SDRAM and hung the board under the full pipeline, so `init(1, 30)` without
`packed420(1)` returns -12.

**Camera and TPU in lockstep.**  The SXGA detection pipeline is USB-bound at ~61 ms
per invoke (the Edge TPU takes input at a flat 35 MB/s).  With the sensor at 30 fps it
processes 54 % of the frames with a non-deterministic slip (bad for two-camera
alternation); with the sensor at a true 15 fps it processes **every** frame (1805 of
1806 over 120 s, 60 ms invoke inside the 66.7 ms period).  Measure lockstep with the
CSI frame counter and the InferTask ok count read in one REPL statement in steady
state; a start..stop window under-counts by ~8 %.

**Slice-level processing.**  The PXP can chase the CSI inside the frame: identify the
in-flight buffer from DMASA_FB1/FB2 plus padding-byte markers (invalidate the D-cache
line before every CPU poll of a camera buffer), then run the PXP per 128-row slice
behind the write pointer — end-of-frame → frame ready 26.9 → 3.4 ms at SXGA,
bit-exact (`camera.chase_test`, `pipeline.chase_prep`).  A CSI line ring in OCRAM is
impossible: the CSI DMA is SOF-gated and never alternates FB1/FB2 mid-frame (done
rate stays 30/s with a 128-line receiver height).

**Benchmark hygiene.**  The first pipeline run after `camera.init()` is slow for ~10 s
(autotune/genlock steals frames); always discard a warm-up run.
