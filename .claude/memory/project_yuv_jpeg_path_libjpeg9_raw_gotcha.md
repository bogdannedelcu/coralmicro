---
name: project_yuv_jpeg_path_libjpeg9_raw_gotcha
description: OP-S10-W24-T2 native-YUV JPEG path shipped (build 1693/1694) — IJG libjpeg 9 raw_data_in needs do_fancy_downsampling=FALSE; byte order verified vs RGB path; cam1 AWB stall seen once after warm reset
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-10-31T14:14:40.371Z
---

# Native-YUV JPEG path (EXP-s256, OP-S10-W24-T2), 2026-10-31

**Chain:** `camera.yuv_mode(1)` before init (OV5640 YUYV, 0x4300=0x3F; CSI
stores XYUV8888, byte2 = Y, byte1 = U, byte0 = V) → `sentai_pxp_xyuv_to_yuv420()`
(Y plane + interleaved UV 4:2:0, UV byte0 = Cb) → `cbcr_deinterleave` →
`coralmicro::JpegCompressYuv420()` (libjpeg `raw_data_in`, JDCT_IFAST).
`sentai.camera.jpeg(q, yuv=1)`.  Total 69–76 ms vs 108 ms RGB path at
640×480 q75 (RGB path: PXP XRGB→RGB888 + libjpeg colour convert + downsample).

**Gotcha (cost ~2 h, reproduced on host):** the vendored NXP libjpeg is IJG
**9b**.  With `do_fancy_downsampling` left TRUE (default) the encoder performs
the 2×2 chroma subsampling *inside the DCT* (chroma `DCT_h/v_scaled_size` = 16,
expects full-resolution chroma rows).  In `raw_data_in` mode we already supply
subsampled Cb/Cr, so it read 16 rows from an 8-row buffer → 16-row-period
green/black stripes.  Fix: `cinfo.do_fancy_downsampling = FALSE` before
`jpeg_start_compress`.  Host replica: compile the SDK libjpeg sources with
gcc + `template/jconfig.h` and feed the planes dumped by
`sentai.camera.yuv420_test()` — same bytes, same bug, 10 s turnaround.

**Byte order verified:** same scene RGB path vs YUV path per camera: cam0
YCbCr 114/139/129 vs 112/141/128, cam1 118/148/130 vs 118/147/132
(`s256/ab_rgb_vs_yuv_2x2.jpg`).  Cb = UV byte0 (NV12 order) is correct.

**Observed once, open (W24-T5):** camera 1 AWB gains 0x3400–0x3405 stuck at
unity (yellow-green frames) after a warm `sys.reset()` out of a YUV-mode
session into an RGB-mode init; camera 0 fine; toggling 0x3406 1→0 restarted
it.  Clean cold init: both AWB engines run from the first second.  Sensor state
survives the SoC reset (no sensor power cycle) — suspect, not proven.

**Also fixed in 1694:** `sentai_cam_phase_from_log()` judged the alternation
regime by the count of same-camera intervals; a consumer blocking > T (JPEG
encode) leaves 2T same-camera entries → T doubled → spurious PHASE_DRIFT
(phi = T − 4 µs).  Now flip-pair estimate first, then folded/sanity-checked
against the VTS-derived period.

**W24-T3 (EXP-s257, build 1694):** all six on-disk YUV420 dual-input models run
from native camera YUV: 20–23 ms invoke, 39–41.5 fps, 0 fails; c2f 38.1 → 20.5 ms
vs its RGB export; **c2f_thick runs (23 ms) where the RGB export fails (0B62)**
— the YUV420 export is the deployable champion.  Open W24-T6: REPL went silent
after the 4th tpu.load in a boot that had just taken a 2.4 MB NCM upload (HTTP
still alive; flashtool --ram recovered); not reproduced without the upload.

**Why:** the YUV path is the production direction (Edge TPU YUV420 models,
W24-T3) and halves the JPEG CPU cost on the recorder.
**How to apply:** any new libjpeg raw-data producer must set
`do_fancy_downsampling = FALSE`; when a JPEG looks striped with a 16-row
period, suspect DCT scaling before suspecting the planes.  Debug image paths
on the host with the dumped planes first.  See [[project_fs_moves_to_sdcard_decision]],
[[project_csi_mux_flip_deferred_f16]].
