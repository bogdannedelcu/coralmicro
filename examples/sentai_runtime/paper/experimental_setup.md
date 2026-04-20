# Experimental setup

This chapter describes once, canonically, the hardware, firmware,
software and measurement tooling used by every experiment reported in
this paper.  Chapters that present a specific optimisation — the
SDRAM-to-tensor eDMA memcpy ([memcpy.md](memcpy.md)) and the
glitch-free dual-camera switch ([cam_switch.md](cam_switch.md)) — refer
back to this description rather than repeating it.  The same
configuration is used end-to-end across all reported sessions;
departures are flagged explicitly in the individual result tables.

---

## 1. Hardware platform

The device under test is a Coral Dev Board Micro carrying the
custom-designed **SentAI** daughter-board revision v1.0.  The relevant
subsystems of this stack are summarised below.

| Subsystem | Component | Parameters |
|---|---|---|
| MCU | NXP i.MX RT1176 crossover | Cortex-M7 @ 800 MHz + Cortex-M4 co-processor (M4 is idle throughout our measurements) |
| On-chip AI accelerator | Google EdgeTPU | internal USB 2.0 bus, Mode 3 (`kMax`) requested by `sentai.tpu.load()` |
| External SDRAM | On-module SEMC SDR-SDRAM | 16 MB @ 166 MHz, accessible via the SEMC controller |
| On-chip memory | OCRAM / DTCM / ITCM | 1.25 MB / 256 KB / 256 KB |
| External flash | Octal-SPI NAND + NOR | NAND hosts the LittleFS user partition (firmware + `/diags/` data + `/lib/diag/` package) |
| Camera 0 (front) | OV5640-based coralmicro module | 1280×720 native, 30 fps streaming (see §3) |
| Camera 1 (back)  | OV5640-based coralmicro module | 1280×720 native, 30 fps streaming |
| Camera MUX | Analogue switch on the shared MIPI-CSI2 lane | GPIO-controlled (`kCamMux`); GPIO polarity canonicalised in [cam_mux.h](../../../libs/camera/cam_mux.h) |
| Host link | USB-C to Linux workstation | CDC-ACM (REPL) + CDC-NCM (IP 10.0.0.1) simultaneously; see [usb.md](usb.md) |
| Power | USB-C bus-powered | no separate supply; thermal envelope within the 3-second experiment windows |

The only dual-sensor hardware constraint that matters for
interpretation: **the two OV5640 sensors share a single MIPI-CSI2
lane through an analogue MUX**.  There is no hardware sync pin wired
between them (no FSIN master/slave), so their internal frame clocks
drift independently.  Every measurement of "switch cost" in this work
is a measurement against *that* topology, not against a
two-receiver SoC.

---

## 2. Firmware stack

| Component | Version / parameters |
|---|---|
| Firmware project | `examples/sentai_runtime/`, build **#640+** at the time of the final post-refactor session (`s045_e18_post_refactor`) |
| Linker script | `MIMXRT1176xxxxx_cm7_ram_mp.ld` (text and rodata in flash, working set in SDRAM) |
| RTOS | FreeRTOS 10.x (CMSIS M7 build, 1 ms tick) |
| USB stack | NXP USB device driver, CDC-ACM + CDC-NCM simultaneously (MSC only in "storage mode"; see [usb.md](usb.md)) |
| HTTP server | lwIP httpd with custom `FsOpenCustom` / `FsCloseCustom` in [sentai_httpd.cc](../sentai_httpd.cc); LS requests route through a dedicated lfs_task — see [lfs.md](lfs.md) §4.2 build #633 for the rationale |
| MicroPython runtime | stable embed port under `third_party/micropython/ports/embed`; GC heap **512 KB** in `.sdram_bss` |
| EdgeTPU driver | Coral `EdgeTpuManager` with per-model package cache |
| Camera driver | NXP `fsl_ov5640.c` + `camera.cc`, with the custom additions documented in [cam_switch.md](cam_switch.md) (flip-on-EOF ISR, ratio scheduler) |
| Watchdog | WDOG1 hardware @ 30 s; software activity watchdog with 60 s warn / 120 s dead thresholds — [watchdog.md](watchdog.md) |
| Persistent diagnostic store | LittleFS user partition, `/diags/` and `/lib/diag/` subtrees |

The firmware image for a reported result is identified by
`build_version.h` and reflected in every session's `summary.txt`
(implicit via build date) and in the session name suffix we chose at
the time (e.g. `_post_refactor`).  Major build transitions called out
in the results tables:

| Transition | Build | Effect |
|---|---|---|
| Pre-eDMA CPU memcpy | #584 or earlier | baseline used as the left-hand side of [memcpy.md](memcpy.md) tables |
| eDMA memcpy merged | ≈#622 | `sentai.pipeline.dma_memcpy(0/1)` A/B flag available at runtime |
| Fix A (atomic snapshot) | #632 | [cam_switch.md](cam_switch.md) §"Fix A" |
| Fix B (flip-on-EOF, 30 fps, ratio scheduler) | #635 | [cam_switch.md](cam_switch.md) §"Fix B" |
| LS slow-path (reset-loop fix) | #633 | [lfs.md](lfs.md) §4.2 |
| NASA/JPL review refactor (A1-A7, B1-B4) | #640 | [cam_switch.md](cam_switch.md) §"Fix B post-review" |

---

## 3. Sensor configuration

### 3.1 Native mode

Both OV5640s are driven at **1280×720 @ 30 fps** in MIPI-CSI2 mode,
with the PLL lookup at `s_ov5640MipiClockConfigs[resolution=720P,
framePerSec=30]`:

```
pllCtrl1 = 0x21   // SYSTEM_CLK_DIV = 2
pllCtrl2 = 0x54   // PLL multiplier = 84
vfifoCtrl0C = 0x20
pclkDiv = 0x04
pclkPeriod = 0x0a
```

`tHsSettle` in the CSI2RX is set to `0x12` — the NXP-recommended value
for this (resolution, fps) pair.  Attempts to push to 45 fps (not in
the NXP driver lookup) or 60 fps (PLL accepted, CSI2RX did not lock)
were reverted and are documented in the comment block of
`libs/camera/camera_support.h`.

### 3.2 Logical resolution

The host application requests a logical resolution via
`sentai.camera.set_resolution(w, h) + sentai.camera.init(1)`.  The
firmware reconfigures the CSI receiver accordingly.  Supported pairs
are the union of the NXP driver lookup and the sentai wrapper: 720p,
1080p (limited mode), 640×480 (VGA), 320×240 (QVGA), plus the custom
512×512 target used by the TPU model.  Every experiment's `.txt`
descriptor records the resolution active during that run.

### 3.3 Rotation

`cam0` is configured with a 180° rotation (`sentai.camera.rotate(0,
180)`) via OV5640 MIRROR H/V registers at sensor-init time.  `cam1` is
unrotated.  Rotation does not change per-frame cost.

---

## 4. Vision model under test

The single-class 512×512 model used for every E15-E18 session is:

| Property | Value |
|---|---|
| File | `/yolo_1_class_512_1_upsample_512_inloc_de_1024_la_P5_32.tflite` |
| On-flash size | 5 591 680 B (5.33 MB) |
| Input tensor | `uint8[1, 512, 512, 3]` — **786 432 B**, the buffer at the centre of the eDMA memcpy optimisation |
| Input quantisation | scale = 1/255, zero_point = 0 |
| Output tensor | `uint8[1, 1344, 6]` — 8 064 B — YOLOv5-enhanced anchor format |
| Output quantisation | scale = 1/255, zero_point = 0 |
| Output row layout | `[cx, cy, w, h, obj_conf, class_conf]` (normalised 0..1) |
| Architecture | YOLOv5 enhanced, single-upsample at P5/32, 1-class head |
| TFLite arena | 799 KB used / 8 192 KB available |
| EdgeTPU mode | 3 (`kMax`), fully compiled for the accelerator (no CPU fallback) |

The earlier pipeline-bring-up sessions (E1-E14 groundwork, listed in
Appendix A of [experiments/README.md](../experiments/README.md)) used
`yolo26n.edgetpu_1.tflite` — an 80-class model at 320×320 that
partially falls back to CPU.  Those sessions are baseline evidence for
the `~6.4 FPS` floor; they are not presented as final performance
numbers.

---

## 5. Scene

All camera-related measurements were captured with a static scene:
two lilac flower arrangements against a pale wall, with a teal ceramic
mug in the upper-right of `cam0`'s field of view.  The scene did not
contain any object class present in the 1-class YOLOv5-enhanced model,
so every frame's `num_detections` is zero.  That is deliberate: NMS
runs its candidate scan over all 1344 anchors in both the zero-
detection and the non-zero case; reporting a zero-detection run
isolates the timing contribution of the post-processing pipeline from
the confound of variable detection counts.

Every session captured before/after JPEGs from BOTH cameras via
`diag.snapshot_both_cameras(when)` (see
[experiments/methodology.md](../experiments/methodology.md) §5.1), so
a reviewer can verify offline that the scene did not drift across a
multi-sweep session.  Figure~\ref{fig:scene} shows the canonical
scene captured at the start of the final post-refactor session.

\begin{figure}[H]
\centering
\begin{subfigure}[t]{0.44\linewidth}
\centering
\includegraphics[width=\linewidth]{figures/experiment_frames/fig_scene_cam0.jpg}
\caption{cam0 (front, rotated 180°)}
\label{fig:scene:cam0}
\end{subfigure}\hfill
\begin{subfigure}[t]{0.44\linewidth}
\centering
\includegraphics[width=\linewidth]{figures/experiment_frames/fig_scene_cam1.jpg}
\caption{cam1 (back, unrotated)}
\label{fig:scene:cam1}
\end{subfigure}
\caption{Canonical scene used throughout every E15–E18 measurement.
Two lilac arrangements against a pale wall, with a teal ceramic mug
in the upper right of cam0's field of view.  Both images are 512×512
JPEG quality-75 captured by the on-board PXP pipeline at the start
of session \texttt{s045\_e18\_post\_refactor} / \texttt{s044\_e18\_headtail\_drain2}.
The scene contains no instances of the model's target class, so every
measured frame has \texttt{num\_detections == 0} — an explicit design
choice that isolates timing from detection-count variance.}
\label{fig:scene}
\end{figure}

---

## 6. Measurement tooling

### 6.1 On-device: the `diag` package

Experiments are implemented as Python functions in
`examples/sentai_runtime/diag/`, invoked from the REPL as
`diag.eN_xxx(...)`.  Each function:

- Opens (or inherits) a session
- Sets a known firmware state (`verbose(0)`, `ratio(0,0)`,
  `switch_drain(2)`, pipeline stopped, `gc.collect()`)
- Runs a warm-up block
- Executes N timed iterations, dropping the first
- Writes `NNN_<name>.csv` + `NNN_<name>.txt` + records in
  `manifest.csv`
- Returns a summary dict

A list of experiment functions relevant to the performance story:

| ID | Function | What it measures |
|---|---|---|
| E13 | `e13_pipeline_full` | Sequential pipeline at 320×320, per-stage timing |
| E14 | `e14_pipeline_parallel` | `sentai.pipeline.start/get/stop` throughput (parallel `PrepTask + InferTask`) |
| E15 | `e15_pipeline_parallel_512` | E14 wrapper for the 512×512 1-class model |
| E16 | `e16_camera_switch_512` | Alternating cam0↔cam1 sequential, per-stage timing |
| E17 | `e17_switch_drain_visual` | Per-switch JPEG capture (in-RAM buffer; LFS write deferred so timing is clean) |
| E18 | `e18_camera_switch_headtail` | Three-sweep benchmark in one session: fixed cam0, fixed cam1, alternating |

### 6.2 Time source

`sentai.rtos.ticks_ms()` — FreeRTOS tick counter exposed to
MicroPython.  Resolution 1 ms, wrap at 49.7 days (handled by
delta subtraction).  All stage timings are either this counter (host-
visible) or the firmware's own wrapper around its invoke duration
(returned as the integer ms value of `sentai.tpu.invoke()`).

### 6.3 Statistical tools

Means, stdev (Bessel-corrected), min/max, median are computed offline
from the raw CSVs by the appendix generator
[`experiments/_build_appendix.py`](../experiments/_build_appendix.py).
The device never reports a summary statistic except the short
`"mean=X fps=Y"` string in each manifest row, and that is informational
only — the numbers that appear in paper tables are always re-derived
from the raw per-iteration CSVs.

See [experiments/methodology.md](../experiments/methodology.md) for
the full protocol (warm-up drop, repetition count rationale,
reproducibility criteria, noise budget).  The short form is: every
reported number is the mean of n ≥ 19 independent iterations with the
first sample dropped; sessions are self-contained; the same scene,
model, and firmware build underlie any cross-session comparison.

---

## 7. Host environment

| Component | Value |
|---|---|
| OS | Linux 6.17 (kernel), Ubuntu-derived userland |
| USB stack | `cdc_acm` + `cdc_ncm` kernel modules, MSC via `littlefs-fuse` |
| IP | USB-NCM: host `10.0.0.N/24`, device `10.0.0.1` |
| Python | 3.12, `pyserial` for REPL-driven uploads |
| Build | CMake 3.x + Ninja/Make, arm-none-eabi-gcc 10+ |
| Firmware flash | `python3 scripts/flashtool.py -e sentai_runtime` |
| Result fetch | HTTP GET from `http://10.0.0.1/api/raw/diags/…` with `lfs_busy` retry |

The exact host details matter only insofar as they influence
measurement access (e.g. the REPL-chunked uploader at
[`diag/_host_upload_repl.py`](../diag/_host_upload_repl.py) exists
because HTTP POST to `/api/write/` hangs on this firmware).  None of
the performance numbers reported in this paper depend on host
scheduling or host I/O; they are all measured on-device and fetched
post-hoc.

---

## 8. Summary of what changes between experiments, and what does not

A reader comparing two sessions should know that the following are
**held constant** across every E15-E18 result in this paper unless a
section explicitly notes a departure:

- Hardware (same SentAI board unit, no swap of camera modules, no
  physical rework)
- OV5640 PLL and sensor-init register sequences (same NXP driver
  lookup)
- Scene (static lilac arrangement, no lighting changes)
- Model (`yolo_1_class_512_…_P5_32.tflite`, compiled for EdgeTPU Mode
  3)
- Logical resolution requested (512×512 unless the section says VGA
  or QVGA)
- Warm-up protocol (one full cycle per camera before the measured
  loop)
- Warm-up sample drop (first measurement sample discarded from stats)
- Fault-counter check (post-run `sentai.diag.cam_stats()` values are
  recorded; a non-zero degraded-path counter invalidates a session)

What *does* change between the sessions the paper compares:

- The firmware build (documented per-section in the cross-session
  comparison tables — e.g. [cam_switch.md](cam_switch.md) §"Cross-
  session comparison")
- The A/B runtime flag under test (`dma_memcpy(0/1)`,
  `switch_drain(1/2)`, `ratio(0,0) / (3,1) / (9,1)`)
- The chosen alternation pattern (fixed cam0, fixed cam1, or
  alternating — the three sweeps of E18)

A reviewer verifying a specific claim can thus navigate directly: the
numeric claim → the section that presents it → the session name → the
folder under [`../experiments/`](../experiments/) with the raw CSV.
The claim-to-file mapping is tabulated in [artifact.md](artifact.md).
