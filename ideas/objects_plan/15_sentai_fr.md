# §26 — `sentai.fr` — Flight Recorder subsystem

**WBS**: `OP-S10-W13`
**Opened**: 2026-05-18 (operator-decided 2026-05-18 after T3 review:
"hai sa avem un FR bun pentru a depana misiunea SafetyArucoBaseline...
ca la avionics sa avem flight recorder").
**Status**: **SHIPPED** 2026-05-18 (commit `6303b95d`).  T1-T5 + T7
done; T6 (kernel channel from sentai_dmesg), T8 (ARM port), T9
(migrate `sim/modsentai_sim_journal.c`) pending.
**Authoritative C API**: `examples/sentai_runtime/sentai_fr.h` +
`examples/sentai_runtime/sentai_fr_task.h`.
**Validated**: `s170_security_aruco_baseline` last trial (post-split):
frames `pushes=135 / writes_ok=135 / drops_full=0 / worst_q=1` and
events `pushes=14 / writes_ok=14` — no drops, drain keeps up at
camera FPS.

## Purpose

Independent NASA/JPL-style flight data recorder (FDR analogue) per
`agent/embeded.md` §3.1 (strict layer separation) + §7.2 (structured
event log: "Persistent breadcrumbs for post-mortem ... Bounded size +
atomic writes (ring buffer)").

Producers (SafetyTask, mission MP, flow_task, future health task,
sentai_dmesg) push items into channel queues and continue without
blocking.  A single recorder task drains queues to disk asynchronously.
Producers NEVER block on I/O.

## Why it's its own work package

Recorder ≠ safety ≠ mission.  Aviation analogy: the FDR/CVR are
dedicated subsystems that NEVER live inside the flight control loop.
Splitting `sentai.fr` out of `sentai.safety` keeps both modules pure
and lets the recorder serve every future subsystem with the same
push-and-forget contract.

## Architecture summary

```
              ┌──────────────────────────┐
              │  Producers (any task)     │
              │  - SafetyTask             │
              │  - flow_task              │
              │  - mission MP (sentai.fs) │
              │  - sentai.health          │
              └──────────┬───────────────┘
                         │ push (non-blocking, bounded queue, O(1))
                         ▼
   ┌──────────────────────────────────────────────────┐
   │  sentai_fr — per-channel bounded queues          │
   │                                                  │
   │  ch: frames    pool 16×640×480 gray = 4.7 MB ★   │
   │  ch: events    pool 256 × {ts,type,text}         │
   │  ch: scalars   pool 1024 × {ts,label,double}     │
   │  ch: kernel    mirror of sentai_dmesg (future)   │
   └─────────────────────┬────────────────────────────┘
                         │
                         ▼
              ┌─────────────────────────┐
              │  recorder_task (single) │
              │  drains queues → disk    │
              │  per-channel sinks:      │
              │   frames : PGM file/item │
              │   events : CSV append    │
              │   scalars: CSV append    │
              └─────────────────────────┘
                         │
                         ▼
                   `SENTAI_FR_DIR=/tmp/...`
                   (FxUser on ARM later)
```

★ Capacities tunable via compile-time `-D`; the SIM defaults size
the frames pool for 640×480 gray to support full-resolution snapshots
when needed.  Per-experiment caller chooses a smaller resolution
upstream (e.g. resize before pushing) to lower memory pressure.

## Format (PX4-spirit minimal, no JSON in MP path)

Operator-mandated 2026-05-18 ("din MP nu vom publica jsoane, e prea
complicat... event type si TEXT, poate coma separated... nu vreau
mai complicat de atat, vezi cum face PX4"):

```
# frames
<dir>/t<ts_ms>_n<n_aruco>_f<frame_seq>.pgm     # PGM gray, name carries metadata

# events
# sentai.fr events  ts_ms,type,text
12345,safety_arm,n_min=4 max_loss_s=1.0
12459,n_dets_change,4 -> 2
13002,abort,markers_lost_streak=1015ms

# scalars
# sentai.fr scalars  ts_ms,label,value
12500,ekf_z,0.601
12500,pnp_z,0.587
12500,n_dets,4
```

Commas / newlines / control chars in input are sanitised to spaces by
the recorder before write so the line-oriented CSV stays robust on
weird input.

## MP API (shipped surface — minimal)

Operator-mandated 2026-05-18 ("din MP nu vom publica jsoane, e prea
complicat... in MP tinem doar lucruri simple"): no MP dicts, no JSON.
Return values are ints / bools / str or 7-tuple of ints (`stats`).

```python
# Lifecycle
sentai.fr.init()                                 -> int  # idempotent
sentai.fr.open(channel_str, path)                -> int  # 0 / err
sentai.fr.close(channel_str)                     -> int

# Worker
sentai.fr.task_start()                           -> int
sentai.fr.task_stop()                            -> int

# Producer endpoints from MP (text only — frames pushed from C side)
sentai.fr.push_event(type_str, text_str)         -> int
sentai.fr.push_scalar(label_str, value, ts_ms=0) -> int

# Stats — 7-tuple of ints (pushes_total, accepted, drops_full,
#                          writes_ok, writes_fail, queue_depth,
#                          worst_queue_depth)
sentai.fr.stats(channel_str) -> tuple[int×7]
```

`channel_str` ∈ {"frames", "events", "scalars", "kernel"}.

**Frame recording is C-side only** — there is intentionally no
`sentai.fs.record_image()` MP helper.  SafetyTask (and any future
camera-FPS consumer) pushes via `sentai_fr_push_frame()` so the raw
pixel bytes never cross the MP heap boundary — preserves the
[[no-heavy-data-through-mp]] hard rule.  If a closed channel is hit,
the push is a silent no-op (zero cost).

## Implementation files (state ↔ worker split)

Mirrors `sentai.safety`'s `state machine | task` split.  Each half is
independently testable.

| File | LoC | Purpose |
|---|---:|---|
| `sentai_fr.h`       |  ~210 | Public API — channels, push, stats, drain primitive |
| `sentai_fr.cc`      |  ~440 | State + pools + locking + drain functions |
| `sentai_fr_task.h`  |   ~35 | Worker lifecycle (start / stop) |
| `sentai_fr_task.cc` |  ~130 | xTaskCreate + drain loop at 20 ms cadence |
| `bindings/modsentai_fr.c` | ~115 | MP binding (9 fns, scalars / tuples only) |

## Threading + concurrency (load-bearing decisions)

- **FreeRTOS task @ `tskIDLE_PRIORITY + 2`** (NOT `+1`).  Empirically
  `+1` (one above idle) gets starved by the MP main thread + safety
  task + crazy_rx on FreeRTOS POSIX SIM — writes_ok stayed at 0.
  `+2` is the proven priority class (crazy_rx, sentai_safety_task).
- **Producer/drain lock = binary semaphore**, NOT
  `xSemaphoreCreateMutex`.  A FreeRTOS mutex carries priority
  inheritance; contention between SafetyTask and the drain task
  tripped the `pxTCB == pxCurrentTCB` assertion in
  `xTaskPriorityDisinherit`.  PI also entangles task priorities,
  which contradicts the operator-mandated principle "FR trebuie sa
  fie independent de alte task-uri".  Binary semaphore = non-PI
  mutex; held only briefly per slot, no priority inversion in
  practice.
- **Drain-side scratch is static BSS, NOT on the task stack.**  A
  `FrameSlot` at 320×240 gray is ~76 KB; the FreeRTOS POSIX task
  stack is ~16 KB (`configMINIMAL_STACK_SIZE × sizeof(StackType_t) ×
  4`).  Copying a frame to the stack overflows instantly.  Lifting
  `s_drain_snap_frame` to static BSS fixes this; only the drain task
  reads/writes it, so no extra synchronisation needed.
- **20 ms poll, not condition-variable wake**.  Producers do NOT
  signal the worker; the worker polls via `vTaskDelay`.  Trades a
  little CPU for one less synchronisation primitive (and avoids
  another counting-semaphore-wake-pattern bug seen during bring-up).

## SIM-only quirks bundled in same commit

- `sim/modsentai_sim_camera.c`: `sentai_now_ms` is a weak symbol.
  When no other TU provided it on SIM, every grabbed frame's
  `ts_ms = 0` and FR's time-stamped filenames collided.  Now the
  SIM camera shim falls back to `clock_gettime(CLOCK_MONOTONIC)`
  via a static inline `sim_local_now_ms_`.

## Task list — actual status

| T# | Task | Status |
|---|---|---|
| T1 | sentai_fr.h header (channels, slot pools, status codes, capacities) | ✅ SHIPPED |
| T2 | sentai_fr.cc state + push API + drain functions (4 channels) | ✅ SHIPPED |
| T3 | bindings/modsentai_fr.c MP binding (9 fns, no dicts) | ✅ SHIPPED |
| T4 | SIM CMakeLists.txt + sim/modsentai_sim.c dispatch + QSTR regen | ✅ SHIPPED |
| T5 | SafetyTask integration: `sentai_fr_push_frame` from worker | ✅ SHIPPED |
| T6 | Kernel channel → mirror sentai_dmesg ring (stub today) | ⬜ |
| T7 | SIM bring-up smoke (covered by s170 SafetyArucoBaseline indirectly) | ✅ SHIPPED |
| T8 | ARM port: FxUser sinks; disable `frames` channel by default per operator note ("salvarea frames probabil nu va merge pe ARM pentru ca consuma f mult procesor") | ⬜ |
| T9 | Migrate `sim/modsentai_sim_journal.c` → events channel | ⬜ |

## Cross-references

- `examples/sentai_runtime/sentai_fr.{h,cc}`
- `examples/sentai_runtime/sentai_fr_task.{h,cc}`
- `examples/sentai_runtime/bindings/modsentai_fr.c`
- `examples/sentai_runtime/experiments/s170_security_aruco_baseline/`
- §25 — `sentai.safety` (primary frame producer)
- `agent/embeded.md` §3.1 layer separation, §7.2 event log, §4.1 memory
- `wbs.md` `OP-S10-W13` full task list
