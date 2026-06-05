---
name: EMA smoothing breaks per-frame integration
description: Don't EMA-smooth values that downstream code integrates. Use deadband + conf-floor + parabolic-shallow rejection instead.
type: feedback
originSessionId: 23ce703b-532f-42c3-ac8e-f35fa410241e
---
**Rule:** never apply EMA / IIR smoothing at the SOURCE of values
that downstream code integrates (cumulative sum, etc.).  Use it
only for INSTANTANEOUS DISPLAY.

**Why:** An EMA with alpha=0.5 (50/50 mix) reports each real
event N times in the tail (1.0, 0.5, 0.25, 0.125, ...) → the
cumulative sum picks up roughly 2x the real motion.  Combined
with per-frame sub-pixel noise that integrates over 25 fps,
firmware cumsum was 6x larger than offline cumsum on the same
gray frames during the 2026-05-05 flow rework.

**The right tools at the source:**
- **Deadband**: zero out output when |value| < threshold
- **Confidence floor**: zero out when match quality is poor
- **Parabolic-fit shallow-surface rejection**: don't extrapolate
  on flat surfaces

These are CONDITIONAL gates -- they output exactly what was
detected, or nothing.  No smearing, no inflation.

**How to apply:** When designing a per-frame measurement that
will be integrated downstream (motion, drift, inertia counters,
trajectory cumsum), default to NO smoothing in firmware.  If
display jitter is annoying, smooth at the consumer (Python /
host) which doesn't propagate the bias.
