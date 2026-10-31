# EXP-s257 — Edge TPU inference on the YUV420 dual-input detectors

**WBS:** OP-S10-W24-T3.  **Board:** rev 1.1 unit 58eb, build 1694 (RAM-loaded,
same image persisted in flash), both OV5640 at VGA 45 fps in native YUV
(`camera.yuv_mode(1)` before `init`).  **Date:** 2026-10-31.

## Question

Do the YUV420 Edge TPU models (two inputs: UV 320×240×2 + Y 640×480×1, the s240
lineage) run on the current firmware straight from the camera's native YUV
frames, and how does the invoke time compare with the RGB (921 KB tensor) models
measured in s236?

## Models

Taken from disk (`/home/bogdan/work/ch569w/experiments/exp021_yolov5_640_board/models/`,
the Aug-20 / Sep-16 exports; the MLflow server copies were not fetched — its TLS
certificate does not verify from this host, operator to list/download if newer
ones exist):

| model | bytes | inputs |
|---|--:|---|
| msblock_yuv420_int8_edgetpu | 213 856 | in[0] uint8[1,240,320,2] UV, in[1] uint8[1,480,640,1] Y |
| c3_yuv420_int8_edgetpu | 258 912 | same |
| c2f_yuv420_int8_edgetpu | 271 200 | same |
| c2f_pan2_yuv420_int8_edgetpu | 344 928 | same |
| gelan_yuv420_int8_edgetpu | 361 312 | same |
| c2f_thick_yuv420_int8_edgetpu | 951 136 | same |

Pushed once over HTTP (`/api/write/models/<name>`, 1 MB cap — all fit).  The
loader prints `Inputs: 2` and sets the dual-input path automatically; arena
485 KB / 8 MB for every model.

## Method

`run_yuv420.py [--no-upload] --seconds 12 --fps 45`: one boot, `yuv_mode(1)`,
`camera.init(1, 45)`, then per model `tpu.load` → `pipeline.infer_reset` →
`pipeline.start(0.5)` → 12 s → `pipeline.stats()` + `pipeline.infer_stats()` →
`pipeline.stop`.  Pass criteria: every model loads, `fail == 0`, pipeline rate
within 15 % of the camera rate, no CSI / phase events in dmesg.

## Results (pass 2, `results_b1694_r2.csv`, `run_b1694_r2.log`)

| model | invoke ms (avg) | pipeline fps | frames / 12 s | fail | s236 RGB invoke ms |
|---|--:|--:|--:|--:|--:|
| msblock_yuv420 | 20.0 | 41.5 | 536 | 0 | – |
| c3_yuv420 | 20.0 | 41.4 | 534 | 0 | – |
| c2f_yuv420 | 20.5 | 41.2 | 532 | 0 | 38.1 |
| c2f_pan2_yuv420 | 21.5 | 39.9 | 515 | 0 | 40.2 |
| gelan_yuv420 | 22.0 | 39.6 | 511 | 0 | – |
| **c2f_thick_yuv420** | **23.0** | **39.4** | 509 | 0 | **FAIL (0B62)** |

dmesg after the sweep: only the camera init events (0A06, 0A12 pre-trim, 0A0C
genlock 28 µs).  No stalls, no CSI faults, no phase drift.

- Invoke time is 20–23 ms for every architecture: the pipeline is bound by the
  USB transfer of the 460 KB YUV420 pair, not by the network (s236 found the same
  for RGB at 921 KB: 25.6 ms of input transfer alone).  Halving the tensor halves
  the invoke: c2f 38.1 → 20.5 ms, c2f_pan2 40.2 → 21.5 ms.
- The thesis champion `c2f_thick` **runs** in YUV420 (23 ms, 39 fps) — on RGB it
  fails to invoke on the single-EP Edge TPU firmware (s236, 0B62).  The YUV420
  export is therefore the deployable form of the champion.
- Pipeline rate 39–41.5 fps against a 45 fps camera: the remaining gap is the
  PXP repack + prep hand-off, consistent with s241 (YUV 40 fps).
- `detections_total` = 0 in all runs: the lab scene has no training-class
  objects in view; this experiment measures the data path, not accuracy
  (accuracy lives in s241 / the paper's mAP tables).

## Pass 1 anomaly (`run_b1694.log`, open item OP-S10-W24-T6)

Pass 1 uploaded the six models over HTTP/NCM in the same boot right before the
sweep.  msblock and c3 ran at 43 fps, the third model (c2f) ran at only 12 fps
(186 frames / 15 s, invoke still 20.6 ms, `cycle_ms_sum` 1.7 s), and after the
fourth `tpu.load` (c2f_pan2, "Model loaded" printed) the MicroPython REPL went
silent while HTTP kept answering (`/api/ls` fine).  Recovery: `flashtool --ram`
(the running firmware still answered the HID reset).  Previous-boot dmesg showed
nothing beyond the normal boot events.  Pass 2 on a fresh boot without the
upload step ran all six models cleanly, so the trigger involves the 2.4 MB NCM
upload + camera + repeated model loads in one boot.  Pattern matches the
[[tpu-usb-wedge-mechanism]] note (USB-host stall starves the CDC-ACM task) but is
not reproduced yet; one deliberate repro (upload, then sweep) is the next step.

## Files

- `run_yuv420.py` — driver (upload + sweep; `--no-upload` to reuse the SD copies).
- `results_b1694_r2.csv`, `run_b1694_r2.log` — pass 2 (the result above).
- `run_b1694.log` — pass 1 (REPL silence after the 4th load).
