---
name: op-s8-w1-mission-safety-triggers
description: "OP-S8-W1 FlowBaseline mission needs robust abort triggers for safety + clean failure modes.  Operator-noted 2026-05-18 after s167 iter #3 trial saw drone crash mid-F2-iter without graceful detection.  Plan: derive min-safe-altitude during calibration phase + abort on (a) marker FOV loss for N consecutive frames, (b) altitude below 0.2 m floor.  Apply in iter #4 of s167 (or a clean s168)."
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

## Why this exists

s167 iter #3 (2026-05-18) ran with a crash detector keyed on EKF z (abort
if EKF z < CRASH_Z_M=0.20 for >0.6 s).  Problem: EKF z can drift
arbitrarily AWAY from ground truth — the crash detector never tripped
even though GT showed the drone hit the ground at t=53 s.  Mission
continued running impulses against a grounded drone, then F3 ran 10 s
of "hover" with drone stationary on the floor.  EKF z post-crash
integrated to 60 m fictively.

Operator note: "ar trebui sa identificam si altitudinea minima de la
care sa declansam landing.  Adica sa oprim misiunea daca pierdem cei
4 markeri sau daca suntem sub o anumita altitudine, 0.2m."

## Abort trigger spec (to apply in iter #4 / s168)

Three orthogonal abort conditions, ALL active during F2 and F3:

1. **Altitude floor (PnP-z based, not EKF)**:
   - If last valid PnP-z < 0.20 m for ≥ 0.5 s → ABORT, land immediately.
   - Why PnP-z not EKF: PnP-z is independent of cf2 EKF drift (it's a
     direct ArUco geometry measurement).  Reliable in SIM as long as
     markers are visible.
   - Fall-back if PnP-z unavailable: use EKF z but warn that it may lie.

2. **Marker FOV loss**:
   - If 0 markers detected for ≥ 30 consecutive frames (~5 s at 6.6 Hz
     dump rate) → ABORT, land in place.
   - Rationale: without markers, no VPE, no PnP-z, no calibration
     ground truth.  Continuing to fly blind is reckless.
   - Distinguish from transient occlusion: require sustained loss.

3. **Hard altitude ceiling** (already not in spec but trivially safe):
   - If EKF z > 3 × Z_HIGH (e.g. 2.4 m for Z_HIGH=0.8) → ABORT.
   - Catches the EKF-runaway case (post-crash integration to 60 m).

## Min-safe-altitude derivation during calibration

Add a sub-phase in F2-iter that computes:
- `z_floor_m = max(0.20, marker_pad_height + 0.10)`
  (marker pad is at ~0.005 m in SDF, so z_floor=0.20 is plenty)
- `z_loss_margin = (camera_fov_horiz_half * marker_pad_width / 2)` —
  smallest altitude at which all 4 markers still fit in FOV given the
  current marker layout (~0.3 m for the A4 pad).
- `z_min_safe = max(z_floor_m, z_loss_margin)`

Use `z_min_safe` as the lower bound of any z command in subsequent
phases.  If drone PnP-z drops below z_min_safe, command an upward
correction OR trigger abort if persistent.

## Implementation sketch (iter #4)

```python
class SafetyMonitor:
    def __init__(self, z_min_safe, marker_loss_max_frames):
        self.z_min_safe = z_min_safe
        self.last_below_floor_t = None
        self.no_marker_streak = 0
        self.aborted = False
        self.reason = None
    def update(self, pnp_z, n_dets):
        now = time.monotonic()
        if pnp_z is None or pnp_z < self.z_min_safe:
            if self.last_below_floor_t is None:
                self.last_below_floor_t = now
            elif now - self.last_below_floor_t > 0.5:
                self.aborted = True
                self.reason = f"PnP-z {pnp_z} < {self.z_min_safe} for >0.5s"
                raise SafetyAbort(self.reason)
        else:
            self.last_below_floor_t = None
        if n_dets == 0:
            self.no_marker_streak += 1
            if self.no_marker_streak >= 30:
                self.aborted = True
                self.reason = "no markers for >=30 frames"
                raise SafetyAbort(self.reason)
        else:
            self.no_marker_streak = 0
```

Hook the monitor into every phase loop's PnP poll.  On `SafetyAbort`,
fall through to emergency land same as `CrashAbort`.

## Distinction from `sentai.safety` namespace

This is HOST-SCRIPT-LEVEL safety for ad-hoc calibration missions.  It
is NOT the place for a generic drone safety system —
`[[no-safety-logic-in-explore]]` reserves the latter for a proper
`sentai.safety` namespace.  Here we just need the calibration mission
to fail loudly and quickly when things go wrong, so we don't waste
trial time on broken runs.

## Cross-references

- `[[cf2-sitl-cheat-odom-gt]]` — why we need PnP-z not EKF-z.
- `[[op-s8-w1-cf2-sim-honest]]` — parent WP.
- `[[no-safety-logic-in-explore]]` — distinction from drone-firmware safety.
