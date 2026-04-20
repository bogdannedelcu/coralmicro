# Glossary

Terms, acronyms and platform-specific identifiers used in the paper
and its appendices.  Grouped by domain.

---

## Embedded-systems / RTOS

| Term | Expansion / meaning |
|---|---|
| **DMA** | Direct Memory Access.  Hardware engine that moves bytes without CPU involvement. |
| **eDMA** | "enhanced" DMA — the NXP i.MX RT1176 DMA0 peripheral used for the SDRAM→tensor memcpy. |
| **ISR** | Interrupt Service Routine.  Runs in interrupt context with bounded, minimal work per NASA/JPL discipline ([embeded.md](../agent/embeded.md) §C). |
| **MCU** | Microcontroller Unit.  Here: NXP i.MX RT1176. |
| **RTOS** | Real-Time Operating System.  Here: FreeRTOS 10.x (CMSIS M7 build). |
| **SDRAM** | Synchronous DRAM.  On-module, 16 MB on the SEMC bus at 166 MHz. |
| **SEMC** | Smart External Memory Controller.  The RT1176 peripheral that drives the external SDRAM. |
| **SDP** | Serial Download Protocol.  NXP's ROM bootloader protocol; the board falls back to SDP if firmware fails to boot. |
| **WDOG** | Hardware watchdog (WDOG1).  30 s timeout on the 32 kHz clock, independent of CPU. |
| **VBLANK** | Vertical blanking interval.  Period between the last line of one video frame and the first line of the next, during which no pixel data flows on the MIPI-CSI lane. |
| **VSYNC / HSYNC** | Vertical / horizontal sync pulses marking frame / line boundaries.  In MIPI-CSI2, replaced by FS / LS short packets. |

---

## Camera / imaging

| Term | Meaning |
|---|---|
| **CSI** | Camera Serial Interface.  The RT1176 on-chip peripheral that reads pixel data. |
| **CSI-2** | MIPI Camera Serial Interface, version 2.  The protocol spoken between the OV5640 and the RT1176 on our platform. |
| **D-PHY** | MIPI physical-layer specification that CSI-2 runs over. |
| **FS / LS** | Frame Start / Line Start — short packets in MIPI-CSI2 marking frame and line boundaries. |
| **EOF** | End-of-Frame — the interrupt fired when a DMA buffer has finished filling.  Our flip-on-EOF fix runs inside this ISR. |
| **MUX** | Multiplexer.  On this board, an analogue switch on the shared MIPI-CSI2 lane, GPIO-controlled; selects which of the two OV5640 sensors the CSI receiver is talking to. |
| **PXP** | Pixel Pipeline.  The RT1176 2-D graphics accelerator used for colour conversion and resizing between the raw DMA buffer and the TPU input tensor. |
| **OV5640** | OmniVision 5-megapixel image sensor used as both cam0 and cam1 on this board.  Dual instance, shared MIPI lane via MUX. |
| **FSIN** | Frame Sync Input pin on the OV5640, used for master/slave synchronisation between two sensors.  Not wired on the SentAI board. |
| **AEC / AGC** | Auto Exposure Control / Auto Gain Control — the sensor's internal adaptive exposure loops.  Take several frames to converge after a stream resume. |
| **QSXGA / 1080P / 720P / VGA / QVGA** | Standard resolutions: 2592×1944, 1920×1080, 1280×720, 640×480, 320×240. |

---

## AI inference

| Term | Meaning |
|---|---|
| **EdgeTPU** | Google-designed systolic-array AI accelerator, on-die on the Coral Dev Board Micro.  Runs 8-bit quantised TFLite models at fixed clock; execution time scales with input pixel count and FLOPs. |
| **TFLite** | TensorFlow Lite.  The on-device inference framework.  "TFLite Micro" is the embedded variant we use via `sentai.tpu.*`. |
| **YOLOv5 / YOLOv5-enhanced** | Single-stage object-detection architecture family.  The "enhanced" 1-class 512×512 model used throughout E15-E18 has a single upsample at P5/32 and a 1-class head producing `[1, 1344, 6]`. |
| **NMS** | Non-Maximum Suppression.  Post-processing step that deduplicates overlapping detections.  Runs on the CPU (MicroPython-callable) after Invoke. |
| **Anchor** | A pre-defined bounding-box prior used by YOLO-style detectors.  1344 anchors in the 512×512 single-upsample model. |
| **Mode 3 (`kMax`)** | EdgeTPU performance mode requested by `sentai.tpu.load()`. |
| **Invoke** | Single forward pass of the TFLite model.  Reported as `invoke_ms` in every CSV. |
| **Quantisation (scale, zero_point)** | 8-bit integer representation of activations; `real = (int − zero_point) × scale`. |

---

## Firmware platform (SentAI-specific)

| Term | Meaning / where defined |
|---|---|
| **`sentai` module** | Root MicroPython namespace that exposes the hardware surface.  Submodules: `camera`, `tpu`, `pipeline`, `fs`, `rtos`, `diag`, `io`, `imu`, `mic`, `usb`, `link`, `mesh`, `crazy`, `tfl`, `aifes`, `kmeans`, `pca`, `anomaly`, `dtw`, `hmm`, `rl`, `slam`. |
| **PrepTask / InferTask** | The two tasks inside the firmware parallel pipeline (see [camera.md](camera.md) §PXP).  PrepTask does grab + PXP + quant into a staging buffer; InferTask does memcpy-to-tensor + Invoke + NMS. |
| **Staging buffer / tensor buffer** | The two 512×512×3 buffers between the two pipeline tasks.  The staging buffer is written by PrepTask; the tensor buffer is written by InferTask's memcpy (eDMA since the fix) and read by the TPU. |
| **`sentai.camera.select(id)`** | Arm a glitch-free flip to camera `id`; the CSI EOF ISR consumes the arm in VBLANK.  [cam_switch.md](cam_switch.md) §"Fix B". |
| **`sentai.camera.switch_drain(n)`** | Number of fresh ISR frames required after a MUX flip before the next captured frame is returned.  Default 2; 1 is an experimentation hook. |
| **`sentai.camera.ratio(a, b)`** | Stateless auto-alternate scheduler: over any `(a+b)`-frame cycle, cam0 gets `a` frames and cam1 gets `b`.  `(0, 0)` disables. |
| **`sentai.diag.cam_stats()`** | Returns a dict of persistent fault counters: `switch_ok_eof`, `switch_fallback`, `drain_timeout`, `grab_retry`, `grab_fatal`. |
| **Session** | A `/diags/sNNN_<name>/` folder on the LittleFS user partition, opened by `diag.begin(...)` and closed by `diag.end()`.  Self-contained: CSV + descriptor text + manifest + summary + scene snapshots. |
| **Manifest** | `manifest.csv` inside a session: one row per experiment completed in that session. |
| **Diag experiment** | A function `e<N>_xxx(...)` inside the `diag` package; the numbered experiment classes referenced throughout this paper (E13, E14, E15, E16, E17, E18). |

---

## Storage / USB

| Term | Meaning |
|---|---|
| **LFS / LittleFS** | Flash-friendly file system by ARM; used for the NAND "user" partition.  All `/diags/` data lives here. |
| **MSC** | USB Mass Storage Class.  Mode entered by `sentai.usb.drive(1)`; exposes the raw LittleFS block device as `/dev/sda` on the host. |
| **CDC-ACM** | USB Communications Device Class — Abstract Control Model.  The REPL transport (`/dev/ttyACM0`). |
| **CDC-NCM** | USB Communications Device Class — Network Control Model.  The IP transport (`enxXXXX` on the host). |
| **HTTP fast path vs slow path** | See [lfs.md](lfs.md) §4.2.  `/api/raw` reads go through a mutex-protected fast path; `/api/ls` always queues to `lfs_task` (slow path) to avoid long `tcpip_thread` blocking. |
| **"Bricked"** | Board visible on USB as Google Coral ID `18d1:9307`.  Means firmware never started; requires a manual button press to enter SDP mode.  The camera-switch work was careful to keep USB CDC up before any risky code so the board is never in this state (see [embeded.md](../agent/embeded.md) §M). |

---

## Measurement methodology

| Term | Meaning |
|---|---|
| **Warm-up** | Pre-measurement iteration(s) that exercise every stage on the path but are not recorded.  See [experiments/methodology.md](../experiments/methodology.md) §3.1. |
| **Drop-first-sample** | Convention that the first recorded iteration is discarded from statistics, because it straddles warm-up and steady state.  See [statistical_notes.md](statistical_notes.md) §2. |
| **Sweep** | One block of N iterations with a fixed alternation pattern.  E18 runs three sweeps per session (A fixed cam0, B fixed cam1, C alternating). |
| **Per-switch overhead** | `C_total_mean − max(A_total_mean, B_total_mean)` in an E18 head-to-tail session.  Table 3 of [evaluation.md](evaluation.md). |
| **Directional asymmetry** | `cam1_total_mean − cam0_total_mean` within the alternating sweep of an E16/E18 session.  Non-zero before Fix B; within noise after Fix B. |
| **Scene snapshot** | JPEG captured from each camera at the start and end of a session, archived next to the CSVs for offline scene-drift verification. |
| **Fault counter** | Persistent-since-boot counter of a degraded-path event (ISR arm not consumed, drain timeout, grab retry, grab fatal).  Zero-valued post-run counter is a measurement-integrity check. |

---

## Build / tooling

| Term | Meaning |
|---|---|
| **`flashtool.py`** | `scripts/flashtool.py` — NXP-provided tool; `-e sentai_runtime` flashes the example in persistent mode. |
| **`_host_upload_repl.py`** | `diag/_host_upload_repl.py` — our REPL-chunked uploader.  Used instead of HTTP `/api/write/` because the latter hangs on this firmware. |
| **`repl_run.py`** | Simple REPL driver.  Has a stale-prompt bug on long-running commands; for robust driving we use the inline `_send_line` pattern from `_host_upload_repl.py`. |
| **QSTR** | MicroPython's interned-string pool.  Must be regenerated (with the `micropython-embed-package` make target) whenever a `MP_QSTR_*` symbol is added, renamed, or removed.  See [agent/agent.md](../agent/agent.md) §6. |
| **`ticks_ms()`** | `sentai.rtos.ticks_ms()` — 32-bit FreeRTOS tick counter exposed to MicroPython.  Wraps every 49.7 days; all deltas are computed with unsigned subtraction so the wrap is harmless. |
| **`_ticks()`** | Module-local alias for `sentai.rtos.ticks_ms()` used throughout `diag/`. |
