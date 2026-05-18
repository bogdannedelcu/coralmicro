# sentai.safety — firmware-side mission safety service

**WBS**: OP-S10-W12 (opened 2026-05-18, operator-approved 2026-05-18)
**Status**: T1 header ✅, T2 state machine ✅, T3/T4/T5 in progress
**Replaces**: host-side `SafetyMonitor` Python class in
`examples/sentai_runtime/experiments/s167_flowbaseline_calibrated/mission_flowbaseline2.py`
(retained as interim debug artefact; do NOT extend).

## 1. Why this exists

Per [[no-safety-logic-in-explore]] + [[missions-run-in-sentai-only]] +
the CLAUDE.md core principle ("Compute-intensive work lives in C/C++;
MicroPython is for logic, control, and simple glue"), continuous
per-frame mission-safety monitoring belongs in firmware C/C++, NEVER
in host-side Python.

The trigger for this WP was OP-S8-W1 (cf2 SITL cheat removal): with
the GT-publisher plugin gone, the FlowBaseline mission needs robust
abort triggers (no markers visible → emergency land) AND those triggers
must run at camera FPS regardless of which mission is flying.  A
host-side `SafetyMonitor` was built as iter #4-#6 stopgap in
`mission_flowbaseline2.py`; it has two structural problems:

1. **Wrong location** — violates [[missions-run-in-sentai-only]] which
   says any mission code (including its safety monitoring) runs only
   on the firmware, never on the host.
2. **Wrong cadence** — Python polling cadence was bottlenecked by the
   mission's phase loops; safety only saw a fraction of camera frames
   so the 30-frame streak counter under-counted by ~70 %.

This module is the proper firmware replacement.

## 2. Architectural principle — REUSE, DO NOT DUPLICATE

`sentai.safety` is a **state machine only**.  It does not:

- call `detect_in_ppm` / `sentai_aruco_detect` itself
- read camera frame buffers
- run PnP, FFT, or any per-pixel compute

It consumes results from upstream subsystems via push endpoints:

```
sentai.camera (existing)
   └── sentai_camera_grab_gray_zerocopy()
            ▲
            │
sentai_aruco_detect()  (existing — sentai.aruco)
   ├── caches result via sentai_aruco_get_latest()
   │      (shared with mission MP — single detection per frame)
   │
   └── pushed via sentai_safety_on_aruco_result(n_dets, seq, ts)
                                                       │
                                                       ▼
                                           sentai_safety state machine
                                            ├── per-check streak / dwell
                                            ├── sticky abort flag
                                            └── 32-entry event ring

                                           ┌─── MP read (any context) ───┐
                                           │   sentai.safety.aborted()    │
                                           │   sentai.safety.reason()     │
                                           │   sentai.safety.snapshot()   │
                                           └──────────────────────────────┘
```

The **only new continuous loop** is `sentai_safety_task.cc`: it
polls camera shared memory at camera FPS, runs ArUco via the
EXISTING detector, and pushes the result into the state machine.
All other subsystems (aruco, camera, calib, pipeline) are reused
unchanged.

Future checks plug into the same pattern:
- `alt_floor` consumes `sentai.calib` PnP-z via `on_alt_pose`
- `ekf_ceil` consumes `sentai.crazy` EKF telemetry via `on_ekf_state`
- `battery` consumes `sentai.crazy` battery LOG via `on_battery`
- `link` consumes `sentai.crazy` keepalive ticks via `on_link_keepalive`

In every case, **no detection / sensor pipeline is duplicated** —
safety is a pure downstream consumer.

## 3. C API (`examples/sentai_runtime/sentai_safety.h`)

```c
// Lifecycle
int  sentai_safety_init(void);          // idempotent, clears state
int  sentai_safety_clear(void);         // re-arm boundary only

// Per-check enable/disable
int  sentai_safety_enable(sentai_safety_check_t, const void* params);
int  sentai_safety_disable(sentai_safety_check_t);

// PUSH endpoints (called by feeders; single-writer per check)
int  sentai_safety_on_aruco_result(int n_dets, uint32_t seq, uint32_t ts_ms);
int  sentai_safety_on_alt_pose(float z_pnp_m, uint32_t ts_ms);          // stub
int  sentai_safety_on_ekf_state(float ekf_z_m, uint32_t ts_ms);         // stub
int  sentai_safety_on_battery(float v_batt_v, uint32_t ts_ms);          // stub
int  sentai_safety_on_link_keepalive(uint32_t ts_ms);                   // stub

// Stale-feed watchdog (any periodic context can drive)
int  sentai_safety_tick(uint32_t now_ms);

// Read-side (lock-free atomic load on flag; mutex-protected snapshot)
int          sentai_safety_snapshot(sentai_safety_snapshot_t* out);
int          sentai_safety_is_aborted(void);
const char*  sentai_safety_reason(void);
int          sentai_safety_get_events(sentai_safety_event_t* out, int cap);
```

### Check registry

| Enum | Status | Feeder | Triggers |
|---|---|---|---|
| `SENTAI_SAFETY_CHK_ARUCO`     | T2 SHIPPED | `sentai_safety_task` | n_dets < n_min for ≥ max_loss_s |
| `SENTAI_SAFETY_CHK_ALT_FLOOR` | T-future   | mission via `on_alt_pose` | PnP-z < z_floor_m for ≥ dwell_s |
| `SENTAI_SAFETY_CHK_EKF_CEIL`  | T-future   | mission via `on_ekf_state` | EKF z > z_ceil_m |
| `SENTAI_SAFETY_CHK_BATTERY`   | T-future   | sentai.crazy LOG | v_batt < v_min for ≥ dwell_s |
| `SENTAI_SAFETY_CHK_LINK`      | T-future   | sentai.crazy keepalive | last_ts older than max_silent_s |

Plus an **orthogonal stale-feed trigger** (any check, any feeder):
if `now - last_push_t > STALE_TIMEOUT_S` (default 2.0 s), the check
abort latches with reason "feeder silent ...".  Catches silent
SafetyTask / sentai.crazy death.

## 4. MP API (binding in `bindings/modsentai_safety.c`, T4)

```python
sentai.safety.init()                                       # idempotent
sentai.safety.enable("aruco", n_min=4, max_loss_s=1.0)     # arm
sentai.safety.disable("aruco")
sentai.safety.tick()                                       # MP-driven watchdog (opt)

sentai.safety.aborted() -> bool                            # sticky
sentai.safety.reason() -> str                              # "" if not aborted
sentai.safety.snapshot() -> dict                           # full state
sentai.safety.events() -> list[dict]                       # ring of recent

sentai.safety.clear()                                      # only at re-arm
```

Snapshot dict shape (T4):
```python
{
  "aborted": bool,
  "active_mask": int,            # bitmask over check enum
  "abort_kind": str | None,      # "aruco" | "alt_floor" | …
  "abort_t_ms": int,
  "reason": str,
  "aruco": {
    "last_n_dets": int,
    "streak_ms": int,
    "n_frames_processed": int,
    "last_push_t_ms": int,
  },
}
```

## 5. Implementation files

| File | Lines | Purpose |
|---|---:|---|
| `examples/sentai_runtime/sentai_safety.h`             | ~210 | API contract + SYSTEM MODEL |
| `examples/sentai_runtime/sentai_safety.cc`            | ~350 | State machine (pure compute) |
| `examples/sentai_runtime/sentai_safety_task.cc`       | ~80  | Camera FPS worker (T3, pending) |
| `examples/sentai_runtime/bindings/modsentai_safety.c` | ~250 | MP binding (T4, pending) |

The .cc detects platform via `__ARM_ARCH` (or explicit
`SENTAI_HAVE_FREERTOS`) for mutex impl: `xSemaphoreCreateMutexStatic`
on ARM/RTOS, `pthread_mutex_t` on POSIX SIM.

## 6. Anti-cheat invariants (codified, audit-friendly)

1. **ArUco feed comes only from the real camera pipeline**
   (camera_bridge_recv on SIM, detection_task / CSI ISR on ARM).
   Gazebo `/dynamic_pose` is NEVER consumed.  See
   [[sentai-sim-air-gapped-from-truth]] + [[cf2-sitl-cheat-odom-gt]].

2. **The abort flag is read-only from MP** — there is no
   `ignore_safety` API.  Only `clear()` resets, and only at known
   re-arm boundaries.

3. **Resetting `clear()` emits a `SENTAI_SAFETY_EV_CLEAR` event**
   into the audit ring (preserved across reset) so silent re-arming
   can be detected post-mortem in the events log.

4. **The state machine is single-writer per check.**  Multi-mission
   re-use is via `clear()` between trials — never via concurrent
   writes from MP + feeder.

## 7. Operational notes

### How a mission uses it

```python
# Mission MP file under sentai_fs_root/
import sentai

def run_mission():
    sentai.safety.init()
    sentai.safety.enable("aruco", n_min=4, max_loss_s=1.0)

    sentai.servo.takeoff(z=0.6)
    try:
        while sentai.servo.is_airborne():
            if sentai.safety.aborted():
                print("safety abort:", sentai.safety.reason())
                break
            run_calibration_step()
    finally:
        sentai.servo.land()
        sentai.safety.disable("aruco")
        # Post-mortem: dump events for the verdict.
        for ev in sentai.safety.events():
            print(ev)
        sentai.safety.clear()
```

### Failure handling

- Mission MUST poll `aborted()` at a useful cadence (recommended ≥10 Hz).
  No automatic "force-land" from sentai.safety — it only flags; the
  mission FSM is responsible for the recovery action.  This keeps
  safety free of side effects on actuator state.

- If the mission ignores the flag, the drone keeps flying with
  whatever blind controller was active — the flag is a recommendation,
  not an autopilot override.  Future WP could add an "auto-land on
  safety abort" hook in `sentai.servo` if desired.

### Stale-feed watchdog

If `sentai_safety_task` dies silently (POSIX thread crash, etc.) the
ArUco check stops being fed.  Two seconds after the last push, the
stale watchdog flips the abort flag with reason `"aruco: feeder silent
for X ms"`.  This way a broken pipeline cannot fail-OPEN.

The watchdog runs whenever `sentai_safety_tick(now_ms)` is called.
For ARM, `sentai_safety_task` calls it on its own; for SIM it's
called from the same task or from `sentai.safety.tick()` in MP.

## 8. WBS — work breakdown for OP-S10-W12

| T# | Task | Status | Files |
|---|---|---|---|
| T1 | API design doc / sentai_safety.h | ✅ | sentai_safety.h |
| T2 | SafetyManager state machine (ArUco check, stubs, event log, stale watchdog) | ✅ | sentai_safety.cc |
| T3 | SafetyTask camera FPS worker (POSIX + FreeRTOS) | ⬜ | sentai_safety_task.cc |
| T4 | MP bindings `bindings/modsentai_safety.c` + QSTR regen | ⬜ | modsentai_safety.c |
| T5 | SIM CMakeLists.txt entries + sentai_sim dispatch | ⬜ | sim/CMakeLists.txt, sim/modsentai_sim.c |
| T6 | s170 SafetyTask smoke test (no markers → abort flag → recover) | ⬜ | experiments/s170_safety_smoke/ |
| T7 | Migrate FlowBaseline2 to use sentai.safety + retire host-side SafetyMonitor | ⬜ | mission_flowbaseline2.py (delete or port to MP) |
| T8 | Memory + agent.md section + this Safety.md update | ⬜ (this file is start) | Safety.md, agent/agent.md |
| T9 | ARM build + ITCM budget check + FlowBaseline gate (s127) | ⬜ | n/a |

Estimated total: ~16 h focused work.

## 9. Cross-references

- [[no-safety-logic-in-explore]] — origin principle
- [[missions-run-in-sentai-only]] — HARD RULE that forced the firmware move
- [[op-s8-w1-mission-safety-triggers]] — interim host-side spec being replaced
- [[op-s8-w1-cf2-sim-honest]] — parent crisis WP
- [[flowbaseline2-4markers-abort]] — operator HARD RULE (`n_dets<4` for 30 frames)
- [[op-s6-w3-aruco-shipped]] — sentai.aruco already on ARM (reused)
- [[op-s6-w1-calib-shipped]] — sentai.calib (future alt_floor feeder)
- [[itcm-budget]] — new C code routed to `.sdram_text`
- [[sentai-sim-air-gapped-from-truth]] — anti-cheat
- `CLAUDE.md` — "compute in C/C++, MP for glue"
- `Sim.md` — companion arch doc for SIM build
