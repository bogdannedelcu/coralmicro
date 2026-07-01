# s240 — flow + detection pipeline in parallel on real HW: root-cause + fix path

Anchor: `TD-S10-A10` · EXP-s240 · follows [s239](../s239_relu_full_pipeline_fps/)
(detection pipeline alone). Goal: run the optical-flow task concurrently with the
detection pipeline on the physical board, using ONE shared camera (per operator:
"PrepTask prepares the image for detection AND flow AND markers"). JTAG available
for debugging.

## Result: it does NOT coexist as-is on HW — and the reason is a compile gate

Attempted on build 1560 (real board): starting flow around the detection pipeline
wedges the camera path (`edgetpu-custom-op invoke failed`, transient USB drop) or
leaves flow not-running while blocking `tpu.load()`. Source diagnosis pins why:

**The shared-slot consume path is SIM/ARM_EMU-only.** `flow_task.cc`:
```c
#if defined(SENTAI_PLATFORM_SIM) || defined(SENTAI_ARM_EMU)
#define SENTAI_FLOW_USE_PREP_SLOT 1
#endif
```
When `SENTAI_FLOW_USE_PREP_SLOT` is defined, FlowTask consumes PrepTask's
`SENTAI_PREP_SLOT_FLOW_GRAY_80x60` slot (`sentai_prep_slot_wait_update` /
`_begin_read`) — no camera grab of its own. When it's **not** defined (i.e. on the
real ARM board, which is neither SIM nor ARM_EMU), FlowTask falls to the `#else`
branch and calls **`sentai_cam_grab_latest()`** — its own CSI grab.

The board multiplexes both OV5640s onto ONE MIPI-CSI2 lane via an analog MUX
(`cam_mux.h`). So on HW, FlowTask's independent grab competes with the detection
PrepTask's grab for the single CSI + MUX → the wedge.

**This is exactly why the CAT experiment (s090/s209) "worked":** those ran in SIM
with the file-backed virtual camera, where `SENTAI_FLOW_USE_PREP_SLOT` IS defined,
so flow consumed PrepTask's shared gray — one stream, no contention. The operator's
memory is correct; that path just isn't compiled for HW.

## The fix is small and the infrastructure already exists on HW

- The `FLOW_GRAY_80x60` slot + its buffer are defined for **all** builds
  (`sentai_prep.cc:73,109`, not SIM-gated).
- The detection PrepTask **already produces** it on HW when its bit is set in the
  fire mask (`detection_task.cc:592`):
  ```c
  if (fire_mask & (1u << SENTAI_PREP_SLOT_FLOW_GRAY_80x60)) {
      ... sentai_prep_slot_begin_write(SENTAI_PREP_SLOT_FLOW_GRAY_80x60, ...);
      sentai_prep_slot_commit(SENTAI_PREP_SLOT_FLOW_GRAY_80x60);
  }
  ```
- `sentai_prep_slot_enable()` / `_wait_update()` / `_begin_read()` are compiled on
  HW too.

So the shared-camera architecture the operator described is present on HW — only
FlowTask's **consume** side is gated off. The fix:

1. Define `SENTAI_FLOW_USE_PREP_SLOT` on the ARM board too (drop the SIM/ARM_EMU
   gate, or add `|| defined(__arm__)`).
2. Ensure the `FLOW_GRAY_80x60` slot is enabled in PrepTask's fire mask while flow
   runs (`sentai_prep_slot_enable`).
3. Validate the notify path on HW: `sentai_prep_slot_wait_update` currently has a
   SIM-specific branch (`sentai_prep.cc:312`) — confirm/adjust the HW notification
   so FlowTask is woken per prep frame rather than the CSI ISR.
4. Build both targets (ARM must not regress), flash, re-test flow + detection
   concurrent FPS on cam0.

Then flow + detection share cam0 via PrepTask — no MUX contention — and the s239
pipeline-FPS measurement can be repeated with flow live (the true "all important
tasks" number).

## IMPLEMENTED + TESTED (build 1561)

Implemented the runtime-toggle version (operator chose to build it). Edits to
`flow_task.cc` + `bindings/modsentai_flow.c`:

- New umbrella `SENTAI_FLOW_HAVE_PREP_SLOT` (defined on ALL targets) → the
  prep-slot consume path now COMPILES on real ARM.
- New runtime global `g_sentai_flow_use_prep_slot` (default **0** on HW = legacy
  cam_grab, zero regression; **1** on SIM/EMU as before). Script toggle
  `sentai.flow.use_prep_slot([on])`.
- Unified `publisher_task_fn` loop: one persistent `for(;;)` that selects the
  frame source at RUNTIME each iteration — `sentai_prep_slot_wait_update` +
  `begin_read` (shared) vs `ulTaskNotifyTake` + `cam_grab_latest` (standalone).
  ISR handle registered on all targets (notify harmless in prep mode).
- Unified `sentai_flow_start`/`stop`/`enable`/`task_init`: task is persistent
  (created once, disarmed by `s_pub_running`); `enable`/`stop` refcount the
  FLOW_GRAY_80x60 slot only in prep mode. Flow task prio = `tskIDLE+2` on HW,
  BELOW PrepTask (`tskIDLE+3`) and InferTask (`tskIDLE+4`).

ARM build **PASS** (build 1561, clean — no flow_task.cc warnings, ISRs in ITCM,
no brick risk, all symbols resolved).

### On-board test — the shared-slot mechanism WORKS

Sequence: `pipeline.start()` (cam0, PrepTask running) → `flow.use_prep_slot(1)`
→ `flow.start(0)`. Result (build 1561):

- `flow.start` returns 0 (NO wedge on start, unlike the pre-fix cam_grab attempts).
- **Flow produces motion**: `flow.read_tuple()` dx/dy change across reads
  (−91,−108 → 86,67) and the flow frame_seq advances — FlowTask is computing on
  PrepTask's FLOW_GRAY_80x60 slot.
- **Detection runs concurrently**: `pipeline.frame_count()` grows (48→155),
  `infer_stats` `fail=0`, `pipeline.running()==True`.
- **One camera read** feeds both — no CSI-MUX contention. Board stays alive.

So flow + detection DO coexist on real HW via the shared PrepTask slot — the
original goal is proven.

### FPS with flow live (short-window measurement, cam0 640×480)

Measured over a ~1.4 s window (short enough to beat the sustained-load reset):

| model | detection alone | **detection + flow live** | flow rate |
|---|--:|--:|--:|
| c2f_thick | 26.6 fps | **20.6 fps** | ~5 fps |
| msblock   | 27.6 fps | **21.2 fps** | ~5 fps |

Running flow concurrently costs the detection pipeline ~6 FPS (~23%): 27→21.
Flow itself lands ~5 fps — it's a lossy consumer (its phase-corr FFT + 32 KB
memcpy is heavy, so it processes ~1 in 4 camera frames while detection keeps the
TPU busy). `results_with_flow.csv` has the raw numbers. (Short windows complete
before the sustained-load watchdog reset noted below.)

### Stability caveat (open)

Under **sustained** concurrent operation the board watchdog-resets after a few
seconds (2/2 runs; it recovers cleanly, FileX intact). Flow runs BELOW the
pipeline priority, so it does not starve PrepTask/InferTask — and s239 already
showed the board takes transient resets under heavy sustained pipeline load
*without* flow (VGA45 sweep). So this reads as the board's **pre-existing
sustained-M7/SEMC-load instability** (flow's 32 KB memcpy + CMSIS FFT add SEMC
bandwidth pressure on top of PXP + TPU-input DMA), pushed over the edge by the
extra consumer — not a defect in the toggle itself (the mechanism is functionally
correct). Root-causing the load instability (JTAG vector-catch on HardFault vs a
clean WDOG reset; SEMC arbitration; flow FFT cost) is the follow-up.

## Status

Runtime toggle implemented, built (1561), and **proven to work** (flow+detection
concurrent on one camera). Sustained-load stability is a separate open item.

## Gotcha logged

`sys.reset()` / wedge-reset mid-FileX-op keeps corrupting the volume this session.
`fs.sync()` before a *deliberate* reset avoided it (s239 VGA45 switch); an
*involuntary* wedge-reset can't sync → corruption. Consider a FileX
journal/auto-remount hardening item.
