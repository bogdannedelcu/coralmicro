# EXP-s258 — Edge TPU at 1280×960 YUV420: model, camera, pipeline, and the SDRAM contention fix

> **Correction (2026-11-01, EXP-s260):** the sensor row used here as "SXGA 30 fps"
> actually delivers 22.6 frames/s (CSI `CR3.FRMCNT`; the row was cloned from 720P/30
> and the 1280x960 frame has 984 lines), and "SXGA 15 fps" ~11.3.  `sensor_frames()`
> over-counts by ~15 %.  Read every "camera 30 fps" below as 22.6 fps.  A true 30 fps
> row exists since build 1742 (`camera.init(1, 36)`, SDK patch 0007); the legacy XYUV
> path hangs the board on it, option B (s260) runs.

WBS: **OP-S10-W24-T11** (SXGA inference end to end) and **OP-S10-W24-T12** (SDRAM
contention on the TPU input path).  Board 58eb (rev 1.1), builds 1711–1722,
2026-10-31.  Operator asks: "how many fps can we get at 1280×960 YUV420?", then
"the SDRAM contention is a real problem, find a solution; profile everything".

## Summary

| configuration (build 1722 in flash) | pipeline fps | invoke ms | PXP ms | invoke failures |
|---|---|---|---|---|
| camera 1280×960 YUV @ 30 fps, SXGA model | **13.0** | 76.0 | 61.6 | 0 |
| camera 1280×960 YUV @ 15 fps, SXGA model | 11.2 (camera-phase bound) | 68.6 | 44.7 | 0 |
| camera 640×480 YUV @ 45 fps, trained c2f (production path) | 42.5 (unchanged) | 20.0 | 7.0 | 0 |
| camera 640×480 YUV @ 45 fps, trained c2f_thick | 40.6 (unchanged) | 22.8 | 7.1 | 0 |

Before the fixes the SXGA pipeline ran at 9.5 fps with the camera at 15 fps and
**collapsed at 30 fps** (every invoke failed, 32 EHCI data-buffer errors per
12 s, auto-recover gave up).  The isolated SXGA invoke is 65 ms and is 99 %
USB transfer: the Edge TPU accepts bulk OUT at a flat **35 MB/s** on USB 2.0,
measured identically for instructions and inputs, at 36 KB and 128 KB chunks,
from OCRAM or SDRAM, with the camera on or off, and matching the ~33 MB/s the
operator measured on Google's Coral Dev Board Mini (USB 2.0 host, ch569w
exp024).  That is the device's own USB2 intake limit, not the RT1176.

## 1. The model (host side, `build_model.py`)

The trained 640×480 dual-input YOLOv5n YUV420 detectors exist locally only as
compiled `*_edgetpu.tflite`; the sources are on MLflow (TLS-unreachable).  The
architecture was rebuilt in Keras from `yolov5n_p3p4_c2f_yuv420.yaml` with
random weights (speed on the Edge TPU depends on graph and shapes, not weight
values; detections are irrelevant here), post-training-quantized to int8 with
uint8 I/O and compiled with `edgetpu_compiler` 16.0.  Every op maps to the TPU
(0 CPU ops) once Keras 3's `UpSampling2D` (lowers to `TILE` v3, which the
compiler cannot parse) is replaced by a fixed-size nearest `tf.image.resize`.

The replica had to be validated against the trained model, and it failed the
first time:

| VGA replica variant | invoke ms (board, isolated) | compiled size |
|---|---|---|
| trained c2f (reference) | **19** | 271 KB |
| replica, SiLU activations | 87 | 402 KB |
| replica, channel widths ×8 | 108 | 410 KB |
| replica, C2f split without STRIDED_SLICE | 87 | 410 KB |
| replica, **ReLU6** | **19–20** | 324 KB |
| replica, 3×3 stride-2 stems instead of space-to-depth | 87 | 398 KB |

SiLU (`LOGISTIC` + `MUL` on every layer) costs 4.5× on the Edge TPU at these
feature-map sizes; the trained exports use ReLU (operator confirmed: "the ReLU
and YUV models are the fastest").  The ReLU replica reproduces the trained model
to within 1 ms isolated and 0.7 fps in the pipeline, so it is a valid speed
proxy.  SXGA variants (`--res sxga`: Y 960×1280×1 and UV 480×640×2 inputs with a
3×3 stride-2 "downsampling stem" conv on each plane in front of the unchanged
VGA network; `sxga_s2d`: space-to-depth + 1×1 stem) both invoke in **65 ms**:
297 KB of instructions (vs 90 KB at VGA; the stem over 1.2 MP unrolls large) +
1.87 MB of inputs + 48 KB outputs.

Per-tag URB accounting of one SXGA invoke (`tpu.urb_stats()`, camera off):

| tag | URBs | bytes | wait | rate |
|---|---|---|---|---|
| instructions | 11 | 297 520 | 8.7 ms | 34.3 MB/s |
| inputs (Y then UV) | 57 | 1 866 272 | 53.6 ms | 34.8 MB/s |
| outputs | 3 | 48 000 | 2.0 ms | 24 MB/s |
| event | 1 | 16 | 0.05 ms | |

Sum 64.4 of 65 ms: compute is fully overlapped; **the invoke is the USB
transfer**.  Chunk size 128 KB: 21 input URBs, 64 ms — no gain (same finding as
s257 at VGA).

## 2. Firmware changes for SXGA (builds 1711–1716)

- `sentai.camera.set_hw(w, h)`: selects the sensor geometry before `init()`;
  VGA 640×480 or SXGA 1280×960 (15/30 fps rows exist in the OV5640 driver and
  the D-PHY settle table).  `libs/camera/camera_support.c` keys the receiver,
  sensor and HS-settle configuration on runtime `g_runtime_cam_w/h`; frame
  buffers are sized for the maximum (4 × 4.9 MB).
- The YUV420 dual-input prep/infer path in `detection_task.cc` uses the runtime
  geometry (Y = w·h, UV = w/2·h/2·2) and checks the model's input tensors
  against it (SERR 0x0411 TENSOR_SIZE otherwise).  SXGA Y+UV (1.84 MB) does not
  fit OCRAM, so it is staged in two SDRAM buffers in the non-cacheable camera
  region; VGA keeps the OCRAM buffer.
- `m_ncamera` grows 24 → 32 MB (SDRAM is 64 MB, 16-bit SEMC port, 166.7 MHz;
  8 MB above 0x83800000 were unused).  The virtual camera's 7.4 MB of frame
  storage leaves the board build (`SENTAI_VIRTUAL_CAMERA=OFF` links a stub;
  the SIM keeps the real one) — operator: not useful on the physical board.
- PXP completion deadline scales with the frame (50 ms VGA → 200 ms SXGA; the
  SXGA repack measured 42–62 ms).
- Two frames in flight for the SXGA pair: the counting semaphores
  `s_sem_bufs_free` / `s_sem_prep_done_c` are sized to the cap at `start()`
  (the old code re-primed a single token at every start, which silently
  capped the SXGA pipeline at one frame in flight).

## 3. Profiling (build 1718–1719): who waits for whom

`sentai.pipeline.trace()` (128-event GPT1-µs ring: prep wait/got buffer,
grab, PXP start/end, prep-done; infer wait/got prep, invoke start, driver's
input-done release, invoke end) replaced guesswork.  Findings:

- The early buffer release (byte target reached after the UV plane) fires at
  **61 ms** into a 68 ms invoke under camera load, not at the 46 ms the VGA
  ratio suggested — the input transfer is 90 % of the invoke.
- With one frame in flight the period is 61 (release) + 42 (PXP) ≈ 104 ms =
  9.6 fps, exactly the measured 9.5.  Yielding in the PXP busy-wait changed
  nothing (first hypothesis, wrong); the single-token re-prime was the cause.
- With two frames in flight at a 15 fps camera, PrepTask waits ~45 ms in the
  grab for the next sensor frame (camera-phase bound, 11.2 fps); at a 30 fps
  camera the pipeline is invoke-bound.

## 4. The SDRAM contention (W24-T12) and the fix

With two frames in flight the PXP (4.9 MB read + 1.84 MB write per frame) and
the CSI (74 MB/s at SXGA 15, 147 MB/s at 30) run concurrently with the EHCI
fetching the TPU input from SDRAM.  SDK patch 0006 exposes the qTD error bits
(`tpu.urb_sync_stats()['ehci']`):

| camera | EHCI qTD errors in 12 s | result |
|---|---|---|
| SXGA 15 fps, 2 in flight | 3 buffer errors + 3 transaction errors | 15 failed invokes, auto-recover |
| SXGA 30 fps, 2 in flight | **32 Data Buffer Errors, 0 transaction errors** | every invoke fails, give-up |

A Data Buffer Error is the EHCI TX FIFO running dry mid-packet because the
AHB/SEMC did not deliver the next burst in time — a host-memory underrun, the
exact mechanism behind the historical "SDRAM-sourced TPU input wedges the TPU
under camera load" rule (agent.md, 2026-04).  The EHCI has two knobs for it
that the SDK leaves at defaults (`USB_OTG2`, base 0x4042C000):

| SBUSCFG.AHBBRST | TXFILLTUNING.TXFIFOTHRES | cam 30 fps | cam 15 fps |
|---|---|---|---|
| 2 = INCR8 (default) | 10 (default) | fails (15–32 buffer errors) | 11.1 fps, 0 errors |
| 2 | 8 | fails even at 15 fps | fails |
| 2 | 12 | 4.5 fps, errors | — |
| 2 | 16 | 10.7 fps, **0 errors**, invoke 91 ms | 11.1 fps, invoke 77 ms |
| **3 = INCR16** | **16** | **12.2 fps, 0 errors, invoke 80.8 ms** | 11.1 fps, invoke 69.8 ms |
| 7 = INCR16 unspecified | 16 | 11.0 fps, invoke 90 ms | — |
| 3 | 24 / 31 | 12.2 fps (no further gain) | — |
| 0 + BURSTSIZE 16 or 32 | 16 | 4.1 fps, invoke 237 ms (fixed bursts are 3× slower) | — |

TXFIFOTHRES = 16 × 64 B means a whole 512 B packet is prefetched before the
controller starts transmitting, so a late burst can no longer abort a packet;
INCR16 halves the number of AHB arbitration rounds per packet.  Both are now set
in `libs/usb/usb_host_task.cc` after `USB_HostInit()`.

SEMC bus-master arbitration (`SEMC->BMCR1`, the non-core AXI queue) was swept
as well: max QoS + age weight and max page-hit weight shave another 5 ms per
invoke (81 → 76 ms, 12.1 → 13.0 fps); bank-rotation and read/write-switch
weights made it worse.  Set in `libs/nxp/rt1176-sdk/board_hardware.c`.  Neither
tuning changes the VGA45 production path (42.5 / 40.6 fps, 0 errors, 1 040
frames checked after the change).

## 5. What limits 1280×960 on this board (operator question)

- **USB 2.0 to the Edge TPU, 35 MB/s intake:** 1.84 MB input + ≥ 90 KB
  instructions per frame → 55–62 ms → **16–18 fps absolute ceiling** with
  nothing else on the bus; this SXGA model (297 KB instructions) → 65 ms →
  15.4 fps.
- **SDRAM, 16-bit at 166.7 MHz (~333 MB/s peak, ~200 MB/s practical):** every
  SXGA frame costs 4.9 MB CSI write (the MIPI bridge stores 4 bytes/pixel,
  ERR051248) + 4.9 MB PXP read + 1.84 MB PXP write + 1.84 MB EHCI read =
  13.5 MB.  At 15 processed fps that is 200 MB/s = the practical ceiling; with
  the camera at 30 fps the CSI alone adds 147 MB/s.  Measured cost: the invoke
  grows from 65 (quiet) to 76 ms (camera 30 fps + PXP), the PXP from 42 to
  62 ms.  Even with a VGA-sized tensor (PXP doing the stride-2 downscale, 19 ms
  invokes), the 9.8 MB/frame of CSI write + PXP read bounds SXGA processing at
  about **20 fps**.
- **Camera rows:** SXGA 15 and 30 fps exist and both stream (init 0, genlock
  converges); 45 fps (datasheet maximum for 1280×960, ~442 Mb/s per MIPI lane)
  would need the 1080p30 PLL row (0x3035 = 0x11, 0x3036 = 0x54) and is pointless
  while the processing ceiling is 13–20 fps.

So: **13 fps today, ~15 fps with a tighter instruction stream, ~17–18 fps is the
USB ceiling, ~20 fps is the SDRAM ceiling**.  Halving the input (PXP downscale
to the VGA model) trades the USB bound for the SDRAM bound and would land at
15–18 fps; it is the only path above 15.

## Files

- `build_model.py` — Keras replica of the YUV420 YOLOv5n (flags `--relu`,
  `--div8`, `--nosplit`, `--nospd`; `--res vga,sxga,sxga_s2d`), int8 PTQ, Edge
  TPU compile with op-mapping check.  `models/` holds the compiled variants and
  compiler logs.
- `probe_variants.py` — upload + isolated invoke timing of a model list
  (`probe_b1711_variants.stdout`: the SiLU-vs-ReLU table).
- `run_sxga.py` — phase A (VGA replica check + SXGA isolated) / phase B (camera
  SXGA + pipeline); `run_b17xx_*.stdout` per build.
- `contention_b1719.txt` — camera on/off and VGA45/SXGA15 isolated invokes +
  per-tag URB rates.
- `trace_b1718_sxga15.txt` — the first pipeline timeline.
- `txfifothres_b1720.txt`, `semc_arbitration_b1721.txt` — the EHCI and SEMC
  sweeps; `final_b1722.txt` — the table at the top.
