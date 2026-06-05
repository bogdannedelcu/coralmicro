---
name: arm-hw-primitives-first
description: "HARD RULE 2026-05-17. Before writing ANY pixel-processing / matrix / signal-conversion code on ARM, recall the HW primitive that already covers it: PXP for RGB↔YUV↔Y8 conversions + scale + rotate, CMSIS-DSP for matrix/FFT/luma, SIMD (USADA8/UQADD8/SEL) for hot inner loops, SDK driver libs (fsl_*.h) for everything peripheral.  NEVER write scalar `for (i) { y = (66*r + 129*g + 25*b)>>8; ... }` style conversions when PXP can do it in HW.  Reason: 30-100× faster, freed CPU, less ITCM pressure, and consistent with thesis claim 'compute lives in C with HW acceleration'."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Operator-stated 2026-05-17** after I wrote a scalar RGB→Y8 conversion
loop in `bindings/modsentai_camera.c` when `kPXP_OutputPixelFormatY8`
was right there in `fsl_pxp.h`.

## Rule

Before writing ANY new C/C++ pixel-processing, matrix, or
signal-conversion routine for ARM, run this checklist:

1. **Does PXP do it?** PXP supports: XRGB8888, RGB888, RGB565, YUV422,
   YUV420, Y8, Y4 (in + out), scale up/down, 90/180/270 rotate, alpha
   blend, CSC1 (RGB↔YCbCr), CSC2.  See `fsl_pxp.h` enums:
   `kPXP_PsPixelFormat*`, `kPXP_OutputPixelFormat*`.
2. **Does CMSIS-DSP do it?** Matrix ops, FFT, filters, statistics,
   complex math.  See `arm_math.h`.
3. **Is there a SIMD intrinsic?** `__USADA8`, `__UQADD8`, `__USUB8`,
   `__SEL`, `__PKHBT`, `__SXTB16`.  Cortex-M7 has 16+ DSP instructions.
4. **Is there an SDK driver?** `fsl_*.h` covers every peripheral.

If yes to any: USE IT.  Write scalar fallback only for SIM (where
PXP/CMSIS-DSP don't exist) and gate via `sentai_*_shim.h` pattern
(see `sentai_pxp_shim.h`, `sentai_fft_shim.h`).

## Why this matters

- **Speed**: PXP RGB→Y8 320×240 is ~50 µs vs ~5 ms scalar.  100×.
- **CPU freed**: no busy loop on M7; PXP DMAs in the background.
- **ITCM budget**: scalar inner loops compile to `.text` (= ITCM).
  Hot scalar inner loops can blow the m_text overflow gate
  (see `[[itcm-budget]]`).
- **Thesis claim**: "compute lives in C with HW acceleration".
  Scalar loops contradict this and look amateurish in the eval chapter.
- **Technical debt**: duplicating what's already in the SDK creates
  parallel implementations that drift over time.

## How to recall this rule proactively

(Self-instruction to future-me — operator asked how to encode this
better.)

- Before any `for (i = 0; i < W*H; ++i) { ... }` involving pixel
  channels, STOP and `grep -nE 'kPXP_|arm_|__USADA' include` first.
- Look for existing primitives in `[[m7-aruco-bench-findings]]`,
  `[[runtime-camera-hw]]`, `[[csi-rgb565-silicon-dead-end]]`,
  `[[tpu-pipeline-aggressor]]`, `[[ov5640-aec-dominance]]`, and
  paper/implementation_03_vision_pipeline.md.
- Memory references existing fns: `pxp_scale_xrgb_to_rgb` already
  exists in `sentai_runtime.cc` for the XRGB→RGB888 case — extend
  it (sibling fn) rather than duplicate-with-scalar.

## Related

- `[[m7-aruco-bench-findings]]` — 11× CPU saved via PXP+SIMD+DTCM
- `[[runtime-camera-hw]]` — CSI delivers XRGB8888 (HW format)
- `[[pxp-init-required]]` — must call PXP_Init() if used before camera
- `[[csi-rgb565-silicon-dead-end]]` — why we live with XRGB8888 input
- `[[itcm-budget]]` — new ARM code defaults to .sdram_text; scalar
  hot loops violate this
