---
name: flow-convention-audit
description: Validate that any code touching optical flow, camera frames, or body-frame coordinates respects the load-bearing conventions (FOV, image-to-body mapping, camera mount offset, sign of dx/dy). Use when adding a new flow consumer, modifying PXP/FFT shims, or before declaring a flow-related drift number final.
---

# /flow-convention-audit

The flow pipeline depends on a chain of coordinate conventions that EACH must hold for cf2 EKF to accept samples without rejecting or amplifying drift.  A silent mismatch in any one = 30 cm+ drift that looks "the algorithm is bad" but is actually a sign-flip.  These rules are authoritative per `Sim.md` §10b + `paper/flow_body_frame.md`.

## The conventions (authoritative)

### A. Lens FOV (calibrated, not register-derived)

| Axis | Pixels (raw) | FOV |
|---|---|---|
| H (long, 640) | 640 | **58°** — body FW–BACK |
| V (short, 480) | 480 | **45°** — body L–R |

Aspect 4:3.  Focal length per axis: `f_grid = (grid_w / 2) / tan(FOV_h / 2) ≈ 72.3` for `grid_w = 80`.

### B. Image → body mapping (cam0 with `vflip=1`)

| Image position | Body direction |
|---|---|
| LEFT   | FORWARD  (+x_body) |
| RIGHT  | BACKWARD (-x_body) |
| TOP    | RIGHT    (-y_body) |
| BOTTOM | LEFT     (+y_body) |

Encoded as `body_xform = {0: (-1.0, 0.0, 0.0, +1.0)}` (cam0 verified empirically 2026-05-07 with LED-cued translation).  Cam1 is still placeholder.

### C. Sign convention (verified)

- Drone moves FORWARD → flow `dx` is **negative**.
- Drone moves BACKWARD → flow `dx` is **positive**.

### D. Camera mount offset on cf2 (body frame)

| Axis | Offset | Sign |
|---|---|---|
| body X (FW) | **−4 cm** | camera is BEHIND CoM |
| body Y (L)  | 0 | centred |
| body Z (UP) | **−2 cm** | camera is BELOW CoM |

This must be mirrored in:
1. The Gazebo SDF (camera link relative to drone link).
2. The cf2 EKF lever-arm config (`motion_pos.x/y/z` log params).
3. Any host-side ground-truth comparator (`gt_recorder.py` etc.).
4. `sentai_markers_set_cam_extrinsics(tx, ty, tz, roll, pitch, yaw)` invocations.

### E. PXP shim parity (SIM ↔ ARM)

ARM: `PXP DMA` XRGB8888 640×480 → RGB888P 80×60, ~1.15 ms.  
SIM: `sim/sentai_pxp_scale.c` reimplements area-average on CPU (`#ifdef SENTAI_PLATFORM_SIM`).  
Outputs must match within ±1 LSB rounding.

### F. Flow input chain (identical ARM ↔ SIM after PXP shim)

```
RGB 80×60  →  grayscale Y plane  →  centre-crop 64×60  →  zero-pad 64×64
→  FFT phase-correlation  (CMSIS arm_cfft_f32 on ARM, FFTW3 on SIM)
→  parabolic sub-pixel fit  →  (dx, dy, conf) in milli-grid-pixels
→  deadband + conf-floor  →  publish flow_shared_t
```

Each stage must produce identical output across SIM and ARM modulo rounding.  Diverge in any → flow drift looks "platform-dependent".

## Audit checklist

When touching flow / camera / coords code:

1. **FOV** — code that converts pixel-rate → angular-rate uses 58° / 45° (NOT 90° / 90°, NOT the register-derived 60°×60°).
2. **body_xform** — calls into `sentai_flow_xform` use `(-1.0, 0.0, 0.0, +1.0)` for cam0.  No other tuple.
3. **Sign** — when commanding FORWARD, the test must verify `dx < 0`.  If a new test asserts `dx > 0`, the sign is wrong somewhere.
4. **Mount offset** — Gazebo SDF camera pose AND `sentai_markers_set_cam_extrinsics()` arguments AND `motion_pos.*` log params all carry `(-0.04, 0, -0.02)`.  Grep for `-0.04` and `-0.02` and confirm they appear together in matched places.
5. **PXP shim** — if you added a SIM-only shortcut to the flow pre-process, verify A/B against ARM build on the same input frame (`./build/.../sentai_runtime_test_flow_input <jpeg>` and compare).
6. **FFT precision** — CMSIS arm_cfft_f32 is single-precision; FFTW3 default is double.  If using FFTW3 in SIM, configure it for `float` (`fftwf_*`).  Mixed precision = SIM-only sub-pixel-fit drift of ~0.1 px.

## Grep checks

```bash
# A — wrong FOV constants
grep -rn -E '\b(90\.0?|60\.0?)\s*(\*|/)?\s*(deg|tan|pi)' \
    /home/bogdan/work/coralmicro/libs/sentai/sentai_flow*.cc \
    /home/bogdan/work/coralmicro/sim/sentai_*shim*.c

# B — body_xform tuples that aren't the canonical one
grep -rn 'body_xform\|sentai_flow_xform' /home/bogdan/work/coralmicro/

# D — mount offset
grep -rn -- '-0.04' /home/bogdan/work/coralmicro/sim/gazebo/ \
    /home/bogdan/work/coralmicro/examples/sentai_runtime/
# Expect to find pairs of (-0.04, ..., -0.02)
```

## When this audit fires after a bug report

If someone reports "drift in X but not Y" or "drift only in SIM not ARM":
1. Walk every step A–F.
2. Especially compare the FFT shim outputs across SIM ↔ ARM (`fftwf` vs CMSIS may differ in scaling).
3. Re-run `[[flowbaseline-canonical-config]]` after any fix to confirm 7.4 cm dist_mean recovers.

## Reject patterns

- New flow consumer that hardcodes its own coord transform "to be safe".  Use `sentai_flow_xform` only.
- SIM-side change that doesn't have the matching ARM-side change.
- Test that PASSES on SIM but never runs on ARM (the bug usually lives on the platform you didn't test).

## See also

- `Sim.md` §10b "Camera + flow conventions (load-bearing for Phase 4)"
- `examples/sentai_runtime/paper/flow_body_frame.md` (authoritative on body-frame)
- `examples/sentai_runtime/paper/flow_altitude.md`
- `[[flowbaseline-canonical-config]]` auto-memory — the regression gate
- `[[sentai_pxp_shim]]` pattern doc
