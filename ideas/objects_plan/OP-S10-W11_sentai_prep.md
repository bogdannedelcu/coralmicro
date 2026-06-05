# OP-S10-W11 — `sentai_prep` frame-slot pipeline

WBS code: `OP-S10-W11`.  Status: 🟡 IN PROGRESS — T1+T2 SHIPPED,
T3–T5 TODO.  Source files: `examples/sentai_runtime/sentai_prep.{h,cc}`
+ producer hooks in `examples/sentai_runtime/detection_task.cc`
(ARM) and `sim/camera_bridge_recv.c` (SIM).

## What it is

A small, self-contained pre-processing subsystem sitting between
the camera and downstream consumers.  PrepTask wakes once per
camera frame and does TWO things in sequence:

**Path A — TPU staging (predates the slot system)**: PrepTask
owns the entire camera→TPU pre-processing chain; InferTask never
touches the camera ring or PXP (per detection_task.cc:15-18
*"staging_buf: only PrepTask writes; PXP hardware: only used by
PrepTask"*).  The sequence:

1. `sentai_cam_grab_latest(&raw)` returns a pointer (no memcpy)
   into the OV5640 CSI ring buffer — `m_ncamera` region, 24 MB
   **uncached SDRAM** (`NonCacheableCamera` section, `0x82000000`).
2. `sentai_pxp_scale(raw, ..., dst_buf, ...)` kicks PXP DMA to
   copy + scale from that SDRAM ring into `dst_buf` in OCRAM
   (`.tpu_input` section).  PrepTask blocks on PXP-done; **CPU
   does NOT do the memcpy** — PXP hardware moves the bytes via a
   bus path independent of SEMC, which is the whole point.
3. Output of PXP is RGB888 `uint8_t` (0..255).  Then — **only if
   the active model's input tensor is `kTfLiteInt8`** (TFLite
   type 9) — `sentai_quant_uint8_to_int8(dst_buf, total, zp)`
   applies an in-place `uint8 → int8` reinterpretation
   (`x_int8 = (int)x_uint8 + zp`, with `zp ≈ -128`).  For legacy
   `kTfLiteUInt8` models (type 3) the quant step is skipped and
   the buffer stays raw `uint8_t`.  The C container is
   **always `uint8_t[]`** (`static uint8_t
   s_tpu_input_buf_single[...]`); the signed interpretation lives
   in TFLite's tensor metadata, not in the storage type.
4. `xSemaphoreGive(s_sem_prep_done)` signals InferTask.
   InferTask then either `memcpy`s OCRAM staging → TFLite arena
   (legacy path, OCRAM → OCRAM) or pointer-swaps in direct mode.
   Either way, the SDRAM ring is now free for the next CSI DMA.

Critical placement: the OCRAM destination is **OCRAM**
(`.tpu_input` section), NOT SDRAM —
critical lesson from 2026-04-22 ([[arena-in-ocram-done]] +
[[tpu-pipeline-ocram-tensor]]): SDRAM staging wedged the TPU
under CSI DMA load because the SEMC bus is shared.  OCRAM has a
separate bus path to the USB/eDMA crossbar, so TPU input read +
camera DMA write never collide.  Bandwidth result: 1.8 → 41.4 FPS
(23×) for yolo_1.

**Path B — aux slot fan-out (W11 addition)**: for each ENABLED
slot, PXP scales the same source frame into a "slot" buffer at a
fixed format + resolution, atomically publishes it (seqlock).
Consumer tasks (SlamTask, future PHOG/GIST/FFT) read the slot
zerocopy.  Slot buffers live in **SDRAM** `.sdram_bss` — cold-path
consumers (~1 Hz) don't have the contention issue Path A does.
Path B keeps heavy data in C and never crosses the MicroPython
heap (per the `[[no-heavy-data-through-mp]]` hard rule).

Together: one camera frame → one PXP-driven prep cycle → one
TPU staging write (OCRAM) + N slot writes (SDRAM, where N is the
count of enabled slots).

Path A — TPU staging buffer (NOT a slot, special-case, OCRAM):

| Storage                  | Container | Content                          | Size                        | Consumer  | Region                     |
|--------------------------|-----------|----------------------------------|-----------------------------|-----------|----------------------------|
| `s_tpu_input_buf_single` | `uint8_t` | RGB888 raw (uint8); cast to int8 in-place iff model = `kTfLiteInt8` | model-input (640×480×3 max) | InferTask | **`.tpu_input` → m_ocram** |

Path B — aux fan-out slots (SDRAM):

| ID                             | Format  | Size    | Consumer                       | Region          |
|--------------------------------|---------|---------|--------------------------------|-----------------|
| `SENTAI_PREP_SLOT_GRAY_NATIVE` | Y8      | 320×240 | `sentai.aruco` (geometry-grade) | `.sdram_bss`    |
| `SENTAI_PREP_SLOT_RGB_64`      | RGB888  | 64×64   | HSV / PHOG / GIST descriptors   | `.sdram_bss`    |
| `SENTAI_PREP_SLOT_GRAY_64`     | Y8      | 64×64   | FFT-mag log-polar (W5, planned) | `.sdram_bss`    |
| `SENTAI_PREP_SLOT_FLOW_GRAY_80x60` | Y8  | 80×60   | `sentai.flow` / FlowTask        | `.sdram_bss`    |

Slot IDs are **append-only** — never renumbered, consumers refer
by enum.  New slot adds a row; existing rows are immutable.

## Cadence vs camera

Camera ingress on the physical board and Gazebo provider is VGA-class
`640×480` unless explicitly reconfigured.  PrepTask consumes every frame off
the camera task's output queue and owns the downscales into consumer slots:
`SLOT_GRAY_NATIVE` is `320×240` for marker geometry, while
`SLOT_FLOW_GRAY_80x60` is `80×60` for optical flow.

- **ARM** (`detection_task.cc:prep_task_fn`): wakes on
  `s_sem_prep_done` / camera buffer-available, runs until camera
  returns no frame, sleeps via `xSemaphoreTake` with bounded
  timeout.  Per-frame cost dominated by PXP HW (~2 ms for the
  staging path + ~50 µs per active aux slot).
- **SIM** (`sim/camera_bridge_recv.c`): per UDS packet from
  Gazebo (~30 Hz natural, may slip under host load).  Same slot
  fan-out via `sentai_prep_publish_slot_rgb_64(...)`; SIM uses a
  scalar PXP shim (`sentai_pxp_scale`) since no HW exists.

**Frame divider** (future): each slot has a `frame_div = N` field
in `sentai_prep_stats_t`.  Setting `N=4` for HSV would fire that
slot every 4th camera frame (~7.5 Hz).  Wired in `_set_div()` API
but the producer ignores it today — all enabled slots fire every
frame.  Promote when an empirical workload demands rate decoupling.

## How it's enabled

**Refcount-based**, consumer-owned.  Slot is populated by the
producer **iff** refcount > 0:

```c
sentai_prep_init();                              // app_main, once
sentai_prep_slot_enable(SENTAI_PREP_SLOT_RGB_64); // SlamTask start
// ... consumer runs ...
sentai_prep_slot_disable(SENTAI_PREP_SLOT_RGB_64); // SlamTask stop
```

Multiple consumers can `enable()` the same slot without
coordinating — refcount accumulates, producer fires as long as any
consumer holds it open.  No central registry needed.

The producer dispatch is mask-based:

```c
uint32_t fire_mask = sentai_prep_tick_frame();
if (fire_mask & (1u << SENTAI_PREP_SLOT_GRAY_NATIVE)) { ... }
if (fire_mask & (1u << SENTAI_PREP_SLOT_RGB_64))      { ... }
if (fire_mask & (1u << SENTAI_PREP_SLOT_GRAY_64))     { ... }  // T4
```

`tick_frame()` consults each slot's refcount + frame divider and
returns the mask of slots due THIS frame.

## MicroPython usage — typical init sequence

Slot enable/disable is **not exposed directly** to MP (consumers
are C tasks owning their own refcount).  From the script side you
start/stop the consumer task; the C code handles slot enable
internally.  Example: SlamTask (place recognition):

```python
import sentai

# 1. Set up the camera + intrinsics (must run before PrepTask makes
#    sense — calibration feeds both PnP and the slot scale path).
sentai.camera.init()                  # OV5640 @ 320×240, 30 FPS
sentai.calib.load_or_default()        # /system/cam_calib.json

# 2. Inspect PrepTask cadence (read-only — actual rate is driven
#    by the camera task, not throttled here unless explicitly capped).
print(sentai.pipeline.prep_stats())
# {'frames_total': 0, 'prep_buf_timeout': 0, 'prep_to': 0, ...}

# 3. Start SlamTask — internally calls sentai_slam_start() which
#    runs sentai_prep_slot_enable(SLOT_RGB_64) under the hood
#    (refcount 0 → 1, producer now fires that slot every frame).
sentai.places.start_slam()

# 4. Tight loop: poll the latest match result.  Only small scalars
#    cross the MP boundary — slot buffers stay in C.
import time
for _ in range(60):
    r = sentai.places.slam_current()
    # {'match_id': 3, 'score_pct': 87, 'l1_dist': 14,
    #  'frame_seq': 1142, 'result_seq': 39, 't_compute_us': 7320}
    if r['match_id'] >= 0:
        print("match", r['match_id'], "score", r['score_pct'], "%")
    time.sleep(0.1)

# 5. Pipeline-level diagnostics (PrepTask producer side, not
#    consumer-side).
print(sentai.pipeline.prep_stats())
# {'frames_total': 1850, 'prep_to': 0, ...}

# 6. Stop SlamTask — disables SLOT_RGB_64 refcount (1 → 0).
#    Producer stops firing that slot on next frame.
sentai.places.stop_slam()
```

The same pattern applies to future descriptor tasks: a C task owns
the slot refcount; MP only starts/stops the task and polls the
small result struct.  Slot enable/disable from MP would violate
the `[[no-heavy-data-through-mp]]` principle because MP would
need a way to know when to call `_disable` — easier to delegate
lifetime to the task that consumes it.

The `sentai.pipeline.prep_fps(n)` knob caps the rate at which
PrepTask runs (legacy throttle from TPU days, still useful for
power-budget experiments).  Set 0 / omit for full camera rate.

## How consumers read

Two protocols, depending on tolerance for tearing:

**Tear-tolerant (cold-path, ≤ 1 Hz):**

```c
const uint8_t* buf; int w, h; uint32_t seq;
if (sentai_prep_slot_get(SLOT_RGB_64, &buf, &w, &h, &seq) == 0) {
    compute_on(buf, w, h);  // may see torn data ~0.5% at 30 Hz
}
```

**Tear-detecting seqlock (preferred for non-trivial compute):**

```c
const uint8_t* buf; int w, h; uint32_t ticket;
if (sentai_prep_slot_begin_read(SLOT_RGB_64, &buf, &w, &h, &ticket) == 0) {
    compute_on(buf, w, h);                       // any duration OK
    if (!sentai_prep_slot_end_read(SLOT_RGB_64, ticket)) {
        // torn — producer wrote during compute.  Caller may retry
        // once (bounded) or accept best-effort.  s_producer_overruns
        // bumped automatically.
    }
}
```

Seqlock is the right default for any consumer that touches the
buffer for more than a few µs.  `_begin_read` and `_end_read` each
emit `__DMB()` so the snapshot happens-before-and-after compute.

## Producer-side contract

Producers (and **only** producers — PrepTask on ARM, camera_bridge_
recv on SIM) use the matching write API:

```c
uint8_t* buf = sentai_prep_slot_begin_write(SLOT_RGB_64, &w, &h);
if (buf) {
    sentai_pxp_scale(src, src_w, src_h, buf, w, h);  // PXP HW or shim
    sentai_prep_slot_commit(SLOT_RGB_64);             // __DMB + seq++
}
```

`_commit()` is the atomic publish: `__DMB()` then `slot.seq++`.
Single-writer / multi-reader by construction — no producer mutex.

## ARM vs SIM differences

| Aspect             | ARM                          | SIM                                |
|--------------------|------------------------------|------------------------------------|
| Producer task      | `prep_task_fn` (prio 2)      | `camera_bridge_recv` UDS loop      |
| Source frame       | OV5640 → CSI DMA → camera_task queue | UDS socket from Gazebo plugin    |
| PXP                | Real HW (`fsl_pxp.h`)        | Scalar shim `sentai_pxp_scale`     |
| RGB→Y8 cast        | PXP `kPXP_OutputPixelFormatY8` | scalar BT.601                    |
| Slot storage       | `.sdram_bss` (32 MB SDRAM)   | default `.bss` (host RAM)          |
| `__DMB()`          | ARM `dmb` instruction        | no-op (x86 single-core)            |

The `.h` API and `.cc` body are **identical**.  Storage attribute
swaps via `SENTAI_PREP_BSS` macro; barriers swap via `SLAM_DMB()`.

## Memory footprint (ARM, current settings)

Path A (TPU staging — OCRAM, NOT counted in W11 W14 budgets):

```
s_tpu_input_buf_single   640×480×3 = 921 600 B
                                   = 900 KB in .tpu_input → m_ocram
```

Path B (aux slots — SDRAM):

```
SLOT_GRAY_NATIVE   320×240×1 =  75 KB
SLOT_RGB_64         64×64 ×3 =  12 KB
SLOT_GRAY_64        64×64 ×1 =   4 KB
SLOT_FLOW_GRAY      80×60 ×1 = 4.8 KB
                              ───────
Total slots                   ~96 KB in .sdram_bss
```

Plus ~1 KB of subsystem state (refcount table, seq counters,
stats).  Both within the post-T22 budget.  OCRAM after `.tpu_input`
has ~120 KB headroom (1 MB OCRAM − 900 KB staging − ~32 KB other).

## Open tasks

- **T3** — `slam_task.cc` perception loop (Phase 1c).  InferTask-
  style consumer wired against SLOT_RGB_64 → HSV → places query.
  **Currently SHIPPED on ARM** per commit `cc13484e` (see
  [[op-s10-w11-prep-pipeline]] memory); SIM wiring follows in T4.
- **T4** — SIM mirror in `camera_bridge_recv.c` (Phase 1d).
  SLOT_RGB_64 producer already shipped (line 614-624);
  SLOT_GRAY_NATIVE + SLOT_GRAY_64 not yet wired.  Design decision
  2026-05-19: **GRAY_64 is luma-cast from RGB_64** (NEON ~7 µs),
  not a separate PXP pass — see WBS T4 entry for the full
  rationale (pixel-perfect consistency + one PXP setup per
  frame).  Producer commits RGB_64 first, then luma-casts into
  GRAY_64 if its refcount > 0.
- **T5** — `EXP-s162` live scene-discrimination experiment
  (Phase 1e).  Drone flies over 2-3 distinct scenes in Gazebo,
  SlamTask publishes `current_match` per scene, gate ≥ 80 %
  recall.  Closes W11 and unblocks W6 (opposite-direction
  recall validation across the four Track A descriptors).

## Design invariants (don't break these)

1. **Single-writer per slot** — only PrepTask/camera_bridge_recv
   call `_begin_write`/`_commit`.  Consumers calling these is
   undefined behaviour.
2. **Slot IDs are append-only** — never renumber.  Consumers refer
   by enum at compile time; reordering breaks ABI silently.
3. **No MP heap exposure** — slot buffers must not leak into
   `mp_obj_new_bytes` or similar.  Only small scalar results
   (match id, score, l1_dist) cross to MicroPython.
4. **Producer never blocks on consumers** — slot writes are
   non-blocking, last-frame-wins.  Slow consumers see tearing,
   not back-pressure.
5. **`__DMB()` brackets every seq update** — seqlock correctness
   on ARM Cortex-M7 requires barriers on producer commit AND
   consumer begin/end_read.  Both already in code; do not optimise
   away under "but x86 doesn't need it" pressure (the shared
   header compiles for both).
6. **GRAY_64 ⊆ RGB_64** (post-T4 invariant) — both 64×64 slots
   refer to the SAME source frame.  GRAY_64 producer is a luma-
   downcast of RGB_64, not an independent capture.  Document in
   `sentai_prep.h` when T4 lands.

## Why a dedicated subsystem (not just "PXP at the call site")

- **Reuse**: HSV, PHOG, GIST, FFT-log-polar, future DNN — all
  want the same downscaled view.  Without slots they'd each
  re-invoke PXP at slightly different params and times.
- **Bandwidth**: SEMC SDRAM has finite throughput; one PXP pass
  per format-resolution serves N consumers, vs N PXP passes.
- **Consistency**: same source frame for all consumers makes
  fusion (e.g. HSV ∪ FFT-log-polar) statistically meaningful —
  no inter-consumer drift.
- **Testability**: stub the producer, drive slots from canned
  frames, exercise consumers in isolation (`s162` design).
- **Cross-platform**: ARM and SIM see one API; the heavy lifting
  in PXP vs scalar is hidden behind `sentai_pxp_scale`.

## References

- `examples/sentai_runtime/sentai_prep.h` — API contract.
- `examples/sentai_runtime/sentai_prep.cc` — implementation.
- `examples/sentai_runtime/detection_task.cc:prep_task_fn` — ARM
  producer.
- `sim/camera_bridge_recv.c` — SIM producer.
- `examples/sentai_runtime/slam_task.cc` — consumer reference
  implementation (SLOT_RGB_64 → HSV → places match).
- Memory: `[[op-s10-w11-prep-pipeline]]` — T1-T4 shipping log.
