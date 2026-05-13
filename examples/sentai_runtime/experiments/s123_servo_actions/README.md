# s123 — sentai.servo action layer skeleton (Stage 4)

**Goal.** Verify the new `sentai.servo` namespace exposes a
backend-agnostic action API the mission state machine (sentai.explore,
Stage 3.B) will call into.

This is **Stage 4** of `ideas/objects_plan.md` namespace audit. Without it,
sentai.explore would need to branch between `sentai.crazy.*` (cf2/CRTP) and
`sentai.link.command(MAV_CMD_*)` (PX4/MAVLink) at every state edge — units
differ (cm vs m), frames differ (body NED vs ENU), and command vocab is
entirely different. `sentai.servo` is the single dispatch point.

This commit only ships the **skeleton** — actions are recorded to an
internal 16-entry ring buffer (`trace()` returns the log) so the FSM
can be verified end-to-end before wiring each action to the actual
transport in a follow-up stage.

## API surface

| Function | Returns |
|----------|---------|
| `servo.init(backend)`           | 0 ok / -1 invalid backend |
| `servo.arm()`                    | 0 ok / -1 no-backend / -2 already-armed |
| `servo.disarm()`                 | 0 ok / -1 / -2 not-armed |
| `servo.takeoff(alt_m)`           | 0 ok / -1 / -2 / -3 alt out-of-range (0,30] |
| `servo.land()`                   | 0 ok / -1 / -2 |
| `servo.hover()`                  | 0 ok / -1 / -2 |
| `servo.move(dx, dy, dz [, dyaw])`| 0 ok / -1 / -2 / -3 per-axis safety bound (5 m / ±π/2) |
| `servo.status()`                 | dict: backend / armed / trace_count / seq / last_action |
| `servo.trace()`                  | `[(name, ok, a, b, c, d, seq), ...]` oldest-first |
| `servo.clear_trace()`            | None |

Backends accepted: `"sim"` (no-op record), `"cf2"`, `"px4"` (latter two are
placeholders this stage; actual transport binding TBD).

## Run

```bash
cmake --build build-sim --target sentai_sim
python3 examples/sentai_runtime/experiments/s123_servo_actions/test_servo_basic.py
```

## Verified 2026-05-13

22/22 checks PASS. Verified:

  - state machine: pre-init / post-init backend reporting
  - arm/disarm fault-gates (duplicate arm = -2, disarm-without-arm = -2)
  - takeoff / move bounded-input fault gates (alt > 30 m, step > 5 m)
  - happy-path sequence ARM → TAKEOFF → MOVE → HOVER → LAND → DISARM
    captured in trace ring in order
  - clear_trace() and re-init across backends reset state cleanly

## What's deferred to Stage 4.A (next)

  - **Backend dispatch.** `"cf2"` action paths route into
    `sentai_crazy.cc` (CRTP packets); `"px4"` into `sentai_link.cc`
    (MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_LAND, SET_POSITION_TARGET_LOCAL_NED).
    Today the actions only push trace entries — no wire effect.
  - **Async vs sync.** Today every call returns immediately. A future
    `servo.wait_done(timeout_s)` can poll the transport's ack/state
    if the FSM needs to gate on completion.
