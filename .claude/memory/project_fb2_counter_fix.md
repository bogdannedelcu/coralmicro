---
name: CSI ISR is timing-sensitive — DO NOT modify without cross-ratio test
description: 2026-04-23. Adding XOR+store in CSI ISR (for a "better" frame counter) broke alt camera switch. Camera ratio modes require CSI ISR to stay lean.
type: feedback
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
## The attempt

`g_camera_frame_seq` is gated on FB2-done in CSI ISR → ticks at half
sensor rate (~22 Hz for 45 FPS sensor).  MP binding doc falsely
claimed "one tick = one sensor frame".  I tried adding a dual
counter `g_cam_any_fb_seq` that ticks on FB1 XOR FB2 (per sensor
frame), plus a snapshot at MUX-flip, for use by switch-drain logic.

## What broke

Camera alternation (ratio(1,1), ratio(2,1)) — which previously worked
cleanly — started failing 90%+ of TPU invokes with status=1.  Even
with the drain logic reverted to the original FB2-gated counter,
**just having the extra read + XOR + conditional store in the ISR**
was enough to cause alt-mode failures.

Full revert restored alt-mode to working state:
- cam0 baseline: 41.7 FPS, 0 fails
- alt 1:1: 21.4 FPS, 0 fails
- biased 2:1: 25.6 FPS, 0 fails

## Why: The CSI ISR is timing-sensitive

BASEADDR_SWITCH mode alternates FB1/FB2 per sensor frame.  Each IRQ
has a narrow VBLANK window in which MUX flips must happen before the
next line starts.  Adding instructions — even cheap ones — can push
the ISR past this deadline at high sensor rates (45 Hz, ~5 ms VBLANK
window).

## How to apply

**Rule:** Treat CSI ISR body as sacred.  Do NOT add:
- New counter increments
- New conditional branches
- New global stores
- Anything beyond what's already there

If you believe a measurement is wrong (e.g., frame counter reports
half the real FPS), **fix it OUTSIDE the ISR**:
- Change the MP binding doc to note "ticks per 2 sensor frames"
- Expose a helper that multiplies by 2
- Derive from the existing counter in task context

Before touching CSI ISR at all, **test at least**:
- `_t_early_long.py` (baseline cam0 pipeline)
- `_t_camalt.py` (ratio(0,0), (1,1), (2,1))

If any of those regresses, revert immediately.

## Related truth

FPS measurement via `sentai.camera.frame_count()` is **half real
sensor rate** (FB2-gated).  For ~42 Hz real sensor → counter shows
~21 Hz.  For switch_drain, threshold=1 waits ~44 ms (1 counter tick
= 2 sensor frames).  This is INTENTIONAL for stable MUX post-flip
drain.  Don't "fix" it to 22 ms — tested, breaks alt mode.

## Shipped state

Counter gate unchanged.  Switch drain behaves as before github
baseline.  43 FPS baseline preserved, alt 1:1 21 FPS preserved,
0 fails across all modes.

## Related ceiling: alt 1:1 is 21 FPS by construction

With 1 TPU + 2 cameras + ratio(1,1):
- Each frame alternates cam0/cam1
- TPU invokes serialize (22 ms each) → period 44 ms → ~22 FPS ceiling
- We measure 21.4 FPS — at ceiling

Attempted mitigations, ALL unsuccessful:
- `DEMO_CAMERA_BUFFER_COUNT` 4→6: CSI driver queue hardcoded to 4,
  extras useless, caused boot instability.  Reverted.
- `drain_threshold=0` (bypass wait): wrong-camera buffer returned,
  TPU status=1.  Reverted.
- Per-camera dedicated buffers: would save 2-5 ms drain overhead,
  NOT the 22 ms TPU serialization.  Not worth the refactor.

To break the alt-1:1 ceiling requires either:
- Faster invoke (<22 ms model)
- Batched dual-camera model (one invoke per 2 frames)
- Dual TPU (hardware unavailable)
