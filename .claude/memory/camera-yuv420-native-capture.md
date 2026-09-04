---
name: camera-yuv420-native-capture
description: "Native YUV camera capture (OV5640 YUYV -> CSI XYUV8888) WORKS on this HW, unlike the RGB565 dead-end; runtime-selectable via sentai.camera.yuv_mode"
metadata: 
  node_type: memory
  type: project
  originSessionId: 218d6445-6c73-4c0f-bb51-4d13943aaeb1
---

**Native YUV camera capture works** on the coralmicro OV5640/MIPI-CSI (build 1578,
2026-07-02). This is for the YUV420-input detection models (MLflow experiment
`sentai_yuv420_v1`, e.g. `c2f_thick_yuv420_relu`, int8 mAP50 0.844) — feeding the
camera's native YUV avoids an RGB->YUV step AND matches the model's training color
space (OV5640 CMX coefficients).

**Config (runtime-selectable, default OFF = legacy XRGB, zero regression):**
`sentai.camera.yuv_mode(1)` BEFORE `camera.init()` sets `g_sentai_camera_yuv_mode`
(camera_support.c). BOARD_InitCamera then branches:
- sensor: `kVIDEO_PixelFormatYUYV` (YUV422; OV5640 CMX does RGB->YUV on-chip) vs RGB565.
- CSI receiver: `kVIDEO_PixelFormatXYUV` (XYUV8888) vs XRGB8888. BOTH 4 BPP.

**Key HW fact:** the MIPI CSI2RX->CSI bridge UPCONVERTS the sensor's 16-bit stream
to 32-bit either way (XYUV8888 / XRGB8888) — the CSI always captures 4 BPP
(CR18.PARALLEL24_EN needs bytesPerPixel=4). So the camera DMA stays 1.2 MB/frame
for 640x480 — **YUV does NOT shrink the camera DMA** (the RGB565 dead-end and the
NXP csi_mipi_yuv example both confirm 4 BPP MIPI capture). CSI2RX is stream-driven
(parses the MIPI dataType itself; no dataType config needed) — which is why YUYV
works where RGB565 failed: RGB565 changed only the CSI BPP (16-bit) while MIPI kept
delivering 24-bit -> mismatch -> 2x2-tile garbage. XYUV8888 keeps CSI at 4 BPP, so
no mismatch.

**Verified clean** (build 1578): captured a JPEG in yuv_mode(1) — the image is
structurally IDENTICAL to the XRGB reference (same room/chair/desk), just false-
colored (teal cast) because the JPEG encoder reads XYUV as XRGB. NOT the 2x2-tile
garbage the RGB565 attempt produced. In XYUV8888 the per-pixel bytes are X,Y,U,V
(Y at offset 1). Camera inits fine in YUV mode (init=0, no D-PHY deadlock).

**Wins from YUV capture:** (1) matches training color space; (2) no M7 RGB->YUV;
(3) enables a YUV420 TPU input (Y 640x480 + UV 320x240x2 = 460800 B vs RGB 921600 =
-50% on the TPU input transfer, the ~26 ms invoke bottleneck per s238). NOT a
camera-DMA win (HW forces 4 BPP).

**Phase 3 DONE + verified (build 1579).** `sentai_pxp_xyuv_to_yuv420(src, w, h,
y_dst, uv_dst)` in sentai_runtime.cc (.sdram_text) does ONE PXP pass: PS input
kPXP_PsPixelFormatYUV1P444 (XYUV) -> output kPXP_OutputPixelFormatYUV2P420 =>
buffer0 = Y plane (640x480), buffer1 = UV plane (320x240 interleaved, 2x2
subsampled). No CSC (data already YUV). Verified on board: dumped both planes,
reconstructed on host — Y is a clean full-res grayscale of the scene, and Y+UV->
RGB reconstructs with CORRECT colors (green plant / brown wood / white wall), so
both the Y extraction AND the UV order/subsampling are right. Diagnostic binding
`sentai.camera.yuv420_test(y_path, uv_path)`.

**The model (c2f_thick_yuv420_edgetpu.tflite, MLflow run 0c016b63...): DUAL-INPUT,
input order matters:** input[0] = tensor#0 UV [1,240,320,2] uint8
(`serving_default_uv:0`); input[1] = tensor#1 Y [1,480,640,1] uint8
(`serving_default_y:0`). **uint8** (not int8) -> the camera's Y/U/V bytes feed
DIRECTLY, no quant conversion. Outputs [1,30,40,6]+[1,60,80,6] uint8 (P4+P3 heads).

**Architecture insight (operator):** WhyCon markers + optical Flow use ONLY the Y
(luma) channel. In YUV mode the Y plane IS the native grayscale -> extract Y ONCE
and it is the single grayscale source for detection input[1] AND flow (scaled 80x60)
AND markers/aruco (scaled) — zero RGB->Y conversion anywhere (which is what the s240
PXP-Y8 path did). UV goes only to detection input[0]. All extraction/scaling stays
in C++ (PrepTask); MP only orchestrates (repo rule: no heavy ops in MicroPython).

**Phase 4 (next):** wire the dual-input model into the direct_tensor prep/infer
path (stage UV->input[0], Y->input[1]); the current path stages ONE input tensor,
so multi-input staging is the main new work. Model host copy:
experiments/s240.../models/c2f_thick_yuv420_edgetpu.tflite.
