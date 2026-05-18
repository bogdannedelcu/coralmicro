# §26 — `sentai.fr` — Flight Recorder subsystem

**WBS**: `OP-S10-W13`
**Opened**: 2026-05-18 (operator-decided 2026-05-18 after T3 review:
"hai sa avem un FR bun pentru a depana misiunea SafetyArucoBaseline...
ca la avionics sa avem flight recorder").
**Authoritative C API**: `examples/sentai_runtime/sentai_fr.h`.

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

## MP API (minimal)

```python
sentai.fr.open("frames",  "/tmp/run/frames")    # dir
sentai.fr.open("events",  "/tmp/run/events.csv")
sentai.fr.open("scalars", "/tmp/run/scalars.csv")
sentai.fr.task_start()
# ... mission ...
sentai.fr.task_stop()
sentai.fr.close("frames")
sentai.fr.stats("frames") -> (writes_ok, drops, queue_depth)  # tuple of ints
```

For frame recording the friendlier helper is:

```python
sentai.fs.record_image(source="gray", n_dets=4)    # source ∈ {"gray", "rgb", "resized"}
```

(operator spec 2026-05-18: caller picks what to record — recorder
fetches the image from the requested source via the existing camera
APIs, then pushes through `sentai_fr_push_frame`.)

## Task list (mirrors `wbs.md` OP-S10-W13)

T1 header • T2 state + recorder task • T3 MP binding +
`sentai.fs.record_image` helper • T4 SIM CMake + dispatch + QSTR •
T5 migrate sentai_safety_task in-place PGM dump → push_frame •
T6 migrate sentai_dmesg → kernel channel (T-future) •
T7 EXP-s171 FlightRecorder smoke • T8 ARM port (FxUser sinks,
T-future) • T9 migrate `sim/modsentai_sim_journal.c` → events
channel (T-future).

## Cross-references

- `examples/sentai_runtime/sentai_fr.{h,cc}`
- `examples/sentai_runtime/bindings/modsentai_fr.c` (T3)
- `examples/sentai_runtime/experiments/s171_fr_smoke/` (T7)
- §25 — `sentai.safety` (primary consumer)
- `agent/embeded.md` §3.1 layer separation, §7.2 event log, §4.1 memory
- `wbs.md` `OP-S10-W13` full task list
