# s269 — Detector recall on reference frames shown on the display (both cameras, YUV420 Edge TPU model)

**WBS**: OP-S10-W24-T23 (detector-score metric for colour / camera tuning), uses the T22 production register sets
**Started**: 2026-11-02
**Status**: 🟡 iter1–4 done: 0 recall through the display on every frame; TWO firmware bugs found on the way (headless-model decoder → FIXED build 1769; plain tpu.invoke input path → still broken), camera-independent baseline still missing
**Board**: 58eb, build 1766, VGA 30 fps, `camera.yuv_mode(1)`, production register sets, Kamvas 16 at ~45 cm

## Claim

The end-to-end chain display → camera → ISP → YUV420 → Edge TPU detector recovers the people the
dataset ground truth says are there.  Reference frames (with YOLO box labels) come from the
EDAPA synthetic aerial datasets served by the LAN Data Viewer (barn_10k_v4, bologna_10k_v1,
cairo_10k_v1, quarry_10k: 1280x960, people 28–73 px tall in the frame, 3–15 per frame).  The
frame is scaled into the screen rect both cameras see (1440x1080 of the 1920x1080 pattern
space, i.e. ≈ 0.45x in the 640x480 camera image, close to the model's own 2x downscale of the
training images), the ground truth is projected into each camera image through the s265
homography, and the on-board detections (`pipeline.get_ex`, boxes in model-input pixels) are
matched greedily at IoU ≥ 0.3.

Model: `c2f_thick_yuv420_int8_edgetpu.tflite` (MLflow `sentai_yuv420_v1`, the champion of
s241: mAP@.5 0.823 float; already on the board's SD card in `/models/`).  The MLflow server
needs credentials this session does not have, so the on-board copy is used.

## Pass criteria

- `cam_id` of every detection frame equals the selected camera; ≥ 5 frames read per image.
- Recall at conf ≥ 0.25 and IoU ≥ 0.3 reported per dataset and camera; the two cameras within
  0.1 of each other on the same frames (otherwise the colour / tone difference matters to the
  detector and the T23 metric has something to optimise).
- FP/frame reported; overlays `overlay_<frame>_cam<N>.jpg` (GT green, detections red cam0 /
  orange cam1 back-projected onto the reference image).

## How to run

```bash
python3 run_display_eval.py --out iter1          # ~3 min: 17 frames x 2 cameras x 6 detection frames
```

## Files

- `refs/` — 17 reference frames + labels (`<dataset>__<frame>.jpg/.txt`), `selected_frames.json`
  (frames with 3–15 people, median height ≥ 28 px, chosen from 40 random labels per dataset).
- `run_display_eval.py`; `iter1/`: `canvas/`, `overlay_*.jpg`, `results.json`, `repl.log`.

## Result (2026-11-02, builds 1766 → 1770)

**Recall through the display: 0.00 on every frame, both cameras, every variant** (iter1 full
frames at 45 cm; iter2 crop 67 % + lower AEC target; iter3 crop 50 % + grey canvas, 8 highest-
contrast frames; iter4 = iter3 on build 1769 with the fixed decoder; RGB `iarna_c2f` and YUV
`c2f_thick_yuv420`; conf down to 0.02).  The camera view (`iter*/view_cam0*.jpg`) shows why the
geometry is marginal: at 45 cm the 1440x1080 canvas region covers only ~400x300 px of the
640x480 frame; with the 2x crop the people are 16–40 px tall and visible, yet no candidate at
conf ≥ 0.02 overlaps any projected GT box (iter4 probe: 50 candidates, max conf 0.25, none on a
person).  The only stable detection is a false positive on the Kamvas power LED (cam0, conf 0.38).

**Firmware bug 1 (fixed, build 1769):** `sentai_tpu_detect` only decoded single-tensor exports
([1,N,5+C] / [1,4+C,N]).  Every model on the card (iarna_*, *_yuv420_*) is a *headless*
YOLOv5 p3p4 export with two raw grid heads [1,30,40,6] + [1,60,80,6], so the function returned
-2 and the detection pipeline has been publishing **zero detections for every frame** since
those models were adopted (speed sweeps s236–s264 never looked at detections).  Port of the
reference host decoder (`s236/host_decode.py`: stride = in_h/gh, anchor = 2·stride,
x = (σ·2−0.5+g)·stride, w = (σ·2)²·anchor, conf = σ(obj)·σ(cls), 256-entry sigmoid LUT per
head) into `yolo_decode_heads()`; `sentai.tpu.detect(conf, iou, max)` binding added so a loaded
image can be scored without the camera.

**Firmware bug 2 (open):** `tpu.load_image()` + `tpu.invoke()` does not feed the Edge TPU: the
output fingerprint is identical for a black, a white and a natural image (`hash(tpu.output(0))`
= 28933/40453, out0 channel-constant 239/243/242/243/0/255), with zero-copy input on or off, and
also after publishing the input tensor pointers through `g_sentai_tpu_input_override` in
`sentai_tpu_invoke_internal` (build 1770).  The camera pipeline path (direct tensor,
`sentai_tpu_invoke_with_input`) *does* feed the TPU: with the sensor test patterns the
fingerprints differ per pattern (T16 procedure).  s238 (July) had board/host parity with
`load_image("/test.bin")`, so this is a W24 regression of the arena path.  The virtual camera
(BMP playback) is compiled out since W24-T11 (`SENTAI_VIRTUAL_CAMERA` OFF, `select(-1)` → -1), so
no camera-independent baseline exists on the board today; 18 BMPs are on the card under `/vb/`
and `virtual_feed_eval.py` is ready for a build with the option ON.

**What this means for the question asked:** the chain display → camera → YUV → TPU → decode is
now complete and live (decoder fixed, pipeline feeds the TPU), but it recognises none of the
EDAPA people in this setup.  Before blaming the display (scale ~0.4x of the training scale, panel
contrast, haze) the model's own recall on the raw 640x480 frames must be measured on the board —
blocked by bug 2 (or by a virtual-camera build).  Scripts: `run_display_eval.py` (display),
`direct_feed_eval.py` (load_image path), `virtual_feed_eval.py` (virtual camera).


## Host baseline (CPU int8, `host_eval.py`, venv-coral312 / ai_edge_litert, conf 0.25, IoU 0.3)

The same architectures as on the board (s238 `host_int8_models/`), fed the raw 640x480 frames
directly — no display, no camera, reference decoder `s236/host_decode.py`:

| model | barn_10k_v4 (43 GT) | bologna (55) | cairo (44) | quarry (28) | FP total |
|---|---|---|---|---|---|
| c2f | 0.26 | 0.09 | 0.27 | 0.11 | 9 |
| c2f_thick | 0.28 | 0.09 | 0.25 | 0.25 | 4 |

(The YUV420 models in `s241/models_yuv/` are Edge-TPU-compiled and cannot run on the CPU; the
s238 control image gives 5 detections at conf 0.25 — the July table used a lower threshold.)

**Reading:** even on perfect input the current detectors find only 10–28 % of the EDAPA people
(they were trained on the cover_v1 / iarna synthetic set, not on EDAPA).  The 0 % through the
display is therefore the product of a weak model on this domain and the ~0.4x display scale,
not a sign that the camera chain is broken — it is now live (decoder fixed, pipeline feeds the
TPU).  For a detector-based tuning metric (T23) the model must first be one that detects these
frames (an EDAPA-trained export, or frames from the models' own training domain).
