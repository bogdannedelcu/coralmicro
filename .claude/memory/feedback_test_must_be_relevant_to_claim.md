---
name: test-must-be-relevant-to-claim
description: "Every test must be RELEVANT to what it claims to prove. If a test claims \"drone explores\" but the drone stays still, the test is invalid regardless of PASS verdict. Before writing any test, name the behavior under test and verify the setup forces that behavior to actually occur — don't rely on the verdict's shape checks alone."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Operator-stated 2026-05-15** after seeing s135's verdict-PASS while
the drone barely moved.

## Rule

A test is valid only if it actually exercises the behavior it claims
to prove.  PASS from the verdict means **nothing** if the behavior
under test never occurred.

Examples of invalid tests:
- Test claims "drone navigates to marker" but drone stays in place
  because target_xy is already inside drone's current position (within
  L6's stop_dist tolerance).
- Test claims "FSM goes through RETURNING" but RETURNING is silently
  skipped because target was within HOME_RADIUS — and verdict only
  has a soft WARN, not a hard FAIL.
- Test claims "tracker handles >10 tracklets" but only 2 tracklets
  ever appear in the run.
- Test claims "EKF converges under wind" but wind disturbance was set
  to 0 m/s.
- Test claims "TPU pipeline at 30 FPS" but only 5 frames processed
  before timeout.

In all of these, the verdict can show PASS because the *shape* of the
expected output is present.  But the *substance* — the actual
behavior — never happened.  These tests will mislead future readers
into believing the system works in a regime where it has never been
exercised.

## Before writing a test

1. **Name the behavior under test in one sentence.**  Examples:
   - "L6 sends drone from A to B, drone arrives within X cm of B."
   - "Lifter converges σ_ρ below ε·ρ² after ≥3 lateral observations."
   - "Tracker maintains stable IDs across 30 frames with 2 occlusions."

2. **Identify the geometric / physical preconditions that force the
   behavior to occur.**  If goto needs visible drone motion, target
   must be >> stop_dist away.  If RETURNING needs the transition to
   fire, distance must be >> HOME_RADIUS.

3. **Add the precondition as a HARD assertion in the verdict.**  Not
   a WARN, not an info log — a FAIL gate.  Examples:
   - `assert drone_xy_displacement_during_goto > 0.15`
   - `assert 'RETURNING' in transitions_seen`
   - `assert wind_speed_during_run > 0.05`

4. **Verify the actual behavior in the log after the run.**  Don't
   trust the verdict alone — look at telemetry, look at intermediate
   metrics, check that the thing actually happened.

## What this changes operationally

- Every new test folder under `experiments/sNNN_*/` MUST include in
  its README a "**Behavior under test**" section with the
  one-sentence claim and the precondition assertions in the verdict.
- Existing tests should be audited.  s135 is the first known
  offender: verdict PASSes but the drone moves < 8 cm — does NOT
  prove exploration.  s136 fixes this with real parallax + visible
  goto + RETURNING-required gate.
- Future audit prompt: "If the system under test were quietly broken
  in the worst plausible way, would this test catch it?"

## Related

[[sim-test-must-return-home]] — complementary rule: closure precision
[[gate-every-layer-no-exceptions]] — discipline complement
[[experiment-run-cadence]] — verbose(1) single-trial before multi-run
