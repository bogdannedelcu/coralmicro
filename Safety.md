# sentai.safety — firmware-side mission safety service

**WBS**: OP-S10-W12
**Status**: **SHIPPED** 2026-05-18 (commit `6303b95d`).  T1-T6 done;
T7-T9 (FlowBaseline migration, ARM build, future checks) pending.
**Replaces**: host-side `SafetyMonitor` Python class in
`examples/sentai_runtime/experiments/s167_flowbaseline_calibrated/mission_flowbaseline2.py`
(retained as interim debug artefact; do NOT extend).
**Validated**: `s171_safety_unit` (state machine) 8/8 PASS;
`s170_security_aruco_baseline` (end-to-end SIM, drift + abort + land)
PASS, abort fired at `n_dets=2 < 4 for 1031 ms`, landed 5.2 cm from
origin (≤10 cm rule [[sim-test-must-return-home]]).

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

## 4. MP API (binding in `bindings/modsentai_safety.c`)

Shipped surface is **minimal** — only ints / bools / str, no MP dicts.
Operator-mandated 2026-05-18 ("in MP tinem doar lucruri simple").

```python
# Lifecycle
sentai.safety.init()              -> int   # idempotent reset
sentai.safety.clear()             -> int   # re-arm boundary only

# Per-check arming
sentai.safety.enable_aruco(n_min, max_loss_s)   -> int
sentai.safety.disable_aruco()                   -> int

# Worker control (auto-inits sentai.aruco internally)
sentai.safety.task_start()        -> int
sentai.safety.task_stop()         -> int

# Read-side
sentai.safety.aborted()           -> bool    # sticky
sentai.safety.reason()            -> str     # "" if not aborted

# Test injection (unit tests; NEVER from real missions)
sentai.safety._test_push_aruco(n_dets, seq, ts_ms)   -> int
```

**Why no `snapshot()` / `events()` in MP?** Snapshots over MP would
mean allocating MP dicts every poll — heap churn in the hot loop.  If
post-mortem detail is needed, push it to `sentai.fr` events instead
(text CSV, drained on a separate task, ms-cheap on the push side).

## 5. Implementation files

| File | LoC | Purpose |
|---|---:|---|
| `examples/sentai_runtime/sentai_safety.h`             | 248 | API contract + SYSTEM MODEL |
| `examples/sentai_runtime/sentai_safety.cc`            | 425 | State machine (pure compute) |
| `examples/sentai_runtime/sentai_safety_task.h`        | 161 | Worker API + system model |
| `examples/sentai_runtime/sentai_safety_task.cc`       | 318 | Camera FPS worker (FreeRTOS task) |
| `examples/sentai_runtime/bindings/modsentai_safety.c` | 119 | MP binding (9 fns, scalars only) |

**Threading**: shipped uses FreeRTOS on both targets (ARM + the SIM's
libfreertos_posix).  `xTaskCreate` dynamic at `tskIDLE_PRIORITY + 2`
(same priority class as `crazy_rx` and other SIM tasks — proven to
schedule reliably).  Stack = `configMINIMAL_STACK_SIZE * 4` (POSIX
pthread frames are larger than ARM; aruco's contour finder + Jacobi
PnP also consume frames).

**Memory**: 100 % static.  Stats struct, markers buffer
(`SENTAI_ARUCO_MAX_MARKERS = 16`), and the safety state's event ring
(32 entries) are all file-level statics.  Zero heap, per
`agent/embeded.md` §2 rule 3.

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

### How a mission uses it (canonical pattern from s170)

The reference mission is `examples/sentai_runtime/experiments/
s170_security_aruco_baseline/mission_security_aruco.py`.  Skeleton:

```python
import sentai

def run():
    # ── Init ─────────────────────────────────────────────────────
    sentai.camera.init()
    sentai.safety.init()                  # idempotent reset

    # ── Crazy connect + takeoff (cf2 EKF is stable post-settle) ─
    sentai.crazy.init()
    sentai.crazy.arm()
    sentai.crazy.takeoff(0.60, 2.0)
    sentai.rtos.sleep_ms(2500)            # let takeoff transient finish

    # ── ARM safety AFTER takeoff (avoid pre-takeoff transient) ──
    sentai.safety.enable_aruco(4, 1.0)    # n_min=4, max_loss_s=1.0s
    sentai.safety.task_start()            # auto-inits sentai.aruco

    # ── Mission body — poll abort flag at 10 Hz ─────────────────
    for i in range(int(20 * 10)):         # 20 s deadline
        if sentai.safety.aborted():
            print("ABORT:", sentai.safety.reason())
            break
        # ... mission step (e.g. go_to, hover, …)
        sentai.rtos.sleep_ms(100)

    # ── Land + teardown ─────────────────────────────────────────
    sentai.crazy.land(0.0, 2.5)
    sentai.rtos.sleep_ms(3000)
    sentai.safety.task_stop()
    sentai.safety.disable_aruco()
    sentai.safety.clear()                 # re-arm boundary
```

Key orderings (HARD-LEARNED 2026-05-18):

1. **Open `sentai.fr` BEFORE `sentai.safety.task_start()`** if you
   want SafetyTask's per-frame PGM push to be recorded.  SafetyTask
   calls `sentai_fr_push_frame()` whether or not the channel is open;
   a closed channel is a silent no-op (zero cost).
2. **Arm safety AFTER `takeoff_settled`**, not before — the
   takeoff transient (~2 s) sees the camera FOV swing through
   marker-poor regions and would trip the abort early.
3. **`task_stop()` BEFORE `disable_aruco()`** so the worker exits
   cleanly before the state machine drops the check.

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
| T1 | API design doc / sentai_safety.h | ✅ SHIPPED | sentai_safety.h |
| T2 | SafetyManager state machine | ✅ SHIPPED | sentai_safety.cc |
| T3 | SafetyTask camera FPS worker | ✅ SHIPPED | sentai_safety_task.{h,cc} |
| T4 | MP bindings + QSTR regen | ✅ SHIPPED | bindings/modsentai_safety.c |
| T5 | SIM CMakeLists.txt + dispatch | ✅ SHIPPED | sim/CMakeLists.txt, sim/modsentai_sim.c |
| T6 | s170 SafetyArucoBaseline smoke test (drift → abort → land) | ✅ SHIPPED | experiments/s170_security_aruco_baseline/ |
| T7 | Migrate FlowBaseline2 (s167) host SafetyMonitor → firmware | ⬜ | mission_flowbaseline2.py port |
| T8 | Memory + agent.md section + Safety.md update | ✅ | Safety.md (this), `memory/project_op_s10_w12_w13_shipped.md` |
| T9 | ARM build + ITCM budget check + FlowBaseline gate (s127) | ⬜ | n/a |
| T10 | s171 state-machine unit test (8/8 PASS) | ✅ SHIPPED | experiments/s171_safety_unit/ |
| T11 | sentai_aruco frame_seq memoisation (cache) — eliminate duplicate detect across mission + safety | ⬜ | sentai_aruco.cc |

Total shipped 2026-05-18 commit `6303b95d`: T1-T6 + T8 + T10.
Remaining (T7, T9, T11): non-blocking; tracked.

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
