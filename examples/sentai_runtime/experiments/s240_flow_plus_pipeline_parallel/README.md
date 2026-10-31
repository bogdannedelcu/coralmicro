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

### Instability ROOT CAUSE — FOUND: PXP 8:1 downscale hangs PrepTask

Under sustained concurrent operation the board watchdog-resets (frame_count
157→1 confirms a real core reset; ~3–11 s, intermittent). Root-caused to a
specific HW-only code path:

`publish_aux_slots_from_raw` (`detection_task.cc:592`) produces the
`FLOW_GRAY_80x60` slot with `sentai_pxp_xrgb_to_y8` — a **PXP 640×480→80×60
(8:1) downscale**. The real-HW impl (`sentai_runtime.cc:2526`) does:
```c
PXP_SetProcessSurfaceScaler(DEMO_PXP, 640,480, 80,60);   // 8:1
PXP_Start(DEMO_PXP);
while (!(kPXP_CompleteFlag & PXP_GetStatusFlags(DEMO_PXP))) taskYIELD();  // UNBOUNDED
```
The **Y8-output** PXP variant at this 8:1 ratio never raises the complete flag →
the **unbounded busy-wait hangs PrepTask** → the watchdog fires → reset. Note the
subtlety: 8:1 PXP itself is fine — standalone flow's `sentai_flow_publish_frame`
does the SAME 640×480→80×60 8:1 via `sentai_pxp_scale` (XRGB→**RGB888**) and works.
It's specifically the **`sentai_pxp_xrgb_to_y8` (Y8 output) at 8:1** that hangs.
Evidence fits: HW-only (SIM/EMU use software shims); `slot_div` throttling didn't
help (not load — even one Y8 8:1 hangs); intermittent; `GRAY_NATIVE` 2:1 works.
The FLOW_GRAY slot had never been enabled on HW before flow consumed it, so this
Y8-8:1 path had never run on real hardware.

**Fix (implemented, build 1563):** in `publish_aux_slots_from_raw`, produce
FLOW_GRAY via the proven path — `sentai_pxp_scale` (XRGB→RGB888, works at 8:1) into
a scratch, then a software RGB→Y loop — exactly mirroring standalone flow. Avoids
the hanging Y8-output PXP variant entirely.

**Fix result (build 1563, on board): improved but not fully resolved.** With the
fix, concurrent flow+detection runs noticeably LONGER (reached fc≈355 at ~13 s,
vs the reliable 3–11 s reset before) — so the Y8-8:1 PXP hang WAS a real cause and
the fix removes it. But a residual intermittent instability remains: a pipeline
frame_count reset (fc→1 with uptime CONTINUOUS ≈ pipeline-only restart, ~t=20 s)
followed by an eventual real core reset (uptime dropped). So there is at least one
MORE cause beyond the Y8 PXP hang. Candidates for the residual: PXP contention
between PrepTask's now-3 serial PXP ops/frame (tensor + GRAY_NATIVE + FLOW_GRAY)
and any other PXP user; or the board's pre-existing sustained-pipeline-load
instability (s239 VGA45 took transient resets WITHOUT flow). Needs a follow-up
(detection-alone sustained control test to isolate flow's contribution; JTAG
fault-catch when the env allows). The strategic fix that sidesteps all of it:
move flow compute to the idle M4 core (true parallelism — the s240 architectural
recommendation), which the operator's original intuition pointed at.

Also added `sentai.flow.slot_div(n)` (runtime FLOW_GRAY production divisor,
default 2) — a load knob; useful but does NOT fix the PXP hang (build 1562).

### (superseded) earlier stability note

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

### Residual instability RESOLVED — SEMC-bandwidth contention (build 1565 + throttle)

After the Y8-8:1 PXP hang was fixed (1563), the residual reset was isolated by
control tests: detection-alone STABLE, detection + FLOW_GRAY-production-only
STABLE, detection + **FlowTask** RESETS → FlowTask's compute is the trigger. The
operator's diagnostic question ("is the flow output in OCRAM? if the buffers
aren't in OCRAM the algorithms will run very slowly") pointed straight at the
cause: **every flow working buffer lived in SDRAM (`.sdram_bss`, SEMC bus).**

Root cause = **SEMC-bandwidth contention**. Flow's phase-corr does a 2D 64×64 FFT
per frame over `s_curr_fft`/`s_cross`/`s_cross_spec` (3×32 KB) + a 32 KB gray
memcpy + an 8 KB Tukey window — all in SDRAM. At full production rate those
per-frame FFT bursts saturate the SEMC bus concurrently with the TPU-input DMA
(786 KB `.tpu_input` in OCRAM is fine, but the pipeline's SDRAM traffic + PXP +
flow FFT together over-subscribe SEMC) → the M7 stalls long enough to trip the
watchdog → core reset.

**Hard constraint discovered:** the fast on-chip RAM is **full**. OCRAM
(0x20240000, 1 MB) holds `.tpu_input` (786 KB) + RPMSG → ~13 KB free. DTCM
(`m_data` 0x20008000, 0x38000 = 224 KB) holds the firmware `.bss` (~215 KB) →
~9 KB free. Moving flow's 96 KB of FFT buffers into fast RAM **does not fit**:
attempting it (build 1566) overflowed `m_data` by 105 016 bytes. So the "proper"
fix (buffers off SEMC into TCM) is blocked by the memory budget on the M7.

**What DID fit (build 1565):** the two hottest small buffers moved to DTCM —
FlowTask stack (`s_pub_stack`, aligned(8)) and the double-buffered gray
(`s_gray[2][80*60]`, aligned(4)). This alone was **not enough** (still reset at
t≈14 s) because the 96 KB FFT buffers remain on SEMC.

**Resolution (mitigation that attacks the cause, not the symptom):** throttle
flow production so the FFT bursts fire less often, dropping SEMC occupancy below
the contention threshold. `sentai.flow.slot_div(n)` divides the FLOW_GRAY
production rate. Sweep result on build 1565, cam0 640×480, `c2f_thick` +
flow concurrent:

| slot_div | flow rate | result over 40–48 s |
|---|--:|---|
| 2 (default) | ~13 fps | resets (SEMC saturated) |
| 6 | ~4 fps | **USB CDC unstable** (8/8 windows dropped) |
| **12** | **~2 fps** | **STABLE — 0 core resets, 0 USB blips, uptime continuous 44→66 s, detection ~26 fps sustained (fc 220→1152)** |

So the shippable concurrent operating point is **build 1565 + `slot_div(12)`**:
flow at ~2 fps (adequate for slow drift estimation on a hovering drone) alongside
detection at the full ~26 fps, sustained with no resets. Set it from the mission
script right after `flow.start`:
```python
sentai.pipeline.start(...); sentai.flow.use_prep_slot(1)
sentai.flow.slot_div(12); sentai.flow.start(0)
```

**Proper long-term fix (recovers the throttled FPS too): move flow compute to the
idle M4 core.** M4 has its own TCM (separate from the M7 DTCM that's full) for the
FFT buffers, off the M7's SEMC path, and runs truly in parallel — this is the
architectural recommendation the operator's original intuition ("it runs in
parallel, the CPU has time") pointed at. Tracked as the s240 follow-up; the
throttle is the correct interim resolution given the M7 memory ceiling.

### OCRAM placement experiment (build 1568) — NEGATIVE RESULT, reverted

Operator proposed freeing OCRAM by demoting the now-unused ArUco buffers (ArUco is
superseded by WhyCon markers) and moving flow's 3 hot FFT scratch buffers
(`s_curr_fft`/`s_cross`/`s_cross_spec`, 96 KB) there, off the SEMC bus. Budget
worked: ArUco's `.ocram_bss` is ~93 KB; demoting it + the ~9 KB tail gave ~103 KB,
and the 96 KB fit (build 1568, 7 KB headroom, no ASSERT). **But on-board it made
stability WORSE**: with the FFT in OCRAM, `slot_div=12` reset the board at
t≈13 s (USB dropped), whereas the same throttle with the FFT in **SDRAM** had run
clean.

**Why OCRAM is the wrong place — the key correction to the root-cause model:** the
TPU's input tensor `.tpu_input` **lives in OCRAM**, and `c2f_thick`'s input is a
**full 640×480×3 = 921 600-byte** image that the TPU **re-DMAs every invoke
(~26 ms of input transfer per s238)**. Putting flow's per-frame FFT in OCRAM
(directly after `.tpu_input`) collides flow against the TPU's single largest
memory operation on the OCRAM/AXI path — worse than the SEMC contention it was
meant to relieve. So the earlier "OCRAM is fine, only 786 KB" note above is
misleading: OCRAM isn't free, it's the *busiest* bus during inference.

**The real conclusion: on the M7, neither memory bus is free.** OCRAM is owned by
the 900 KB TPU-input DMA; SEMC/SDRAM is owned by the camera DMA + PXP. Detection
**alone** already saturates the M7's memory bandwidth; flow's FFT tips it over no
matter which bus it sits on. Reverted to the SDRAM layout (build 1569 = 1565
semantics: FFT in SDRAM, ArUco back in OCRAM, stack+`s_gray` in DTCM).

### Honest status on the throttle: reduces but does NOT reliably eliminate resets

Re-testing the reverted 1569 exposed that `slot_div(12)` is **not a reliable
fix** — the reset is **intermittent** (as the original characterization above
always noted: 3–11 s, sometimes reaching fc≈355 at 13 s). One early 1565 run at
div=12 went clean for 48 s (fc 220→1152), but a later 1569 run at the same div=12
reset right at `flow.start`. Throttling **lowers the reset probability** (higher
div = fewer FFT bursts = less bandwidth pressure) but the memory-bandwidth
saturation is fundamental, so no M7-side throttle removes it entirely.

### THE fix: move flow compute to the M4 core

This is no longer just "recovers FPS" — it is the **only** way to make flow +
detection reliably coexist, because it is the only way to get flow's FFT working
set off **both** contended M7 buses:
- M4 has its **own TCM** (ITCM/DTCM), physically separate from the M7 DTCM (full)
  and not on the OCRAM or SEMC path — the FFT scratch + prev-frame caches live
  there with zero contention against TPU-input DMA or camera/PXP.
- M4 runs truly in parallel (the operator's original intuition: "it runs in
  parallel, the CPU has time" — correct; it's the *memory bus*, not the CPU, that
  was the bottleneck, and M4's TCM sidesteps it).
- Flow input (the 80×60 gray slot) is a tiny once-per-frame cross-core handoff via
  RPMSG/shared OCRAM — negligible bandwidth vs the FFT itself.

Tracked as the s240 follow-up WP. Until then, flow + detection concurrent on the
M7 is **best-effort** (throttle high, expect occasional resets under sustained
load); detection-alone and flow-alone are both stable.

## Status

Runtime toggle implemented, built, and **proven to functionally work**
(flow+detection concurrent on one shared camera — motion + detection both produce
output). Root cause of the instability is understood and is **fundamental to the
M7**: detection alone saturates the M7 memory bandwidth (OCRAM = 900 KB TPU-input
DMA/invoke; SEMC = camera+PXP), so flow's FFT tips it over on whichever bus it
uses. The OCRAM-placement attempt (1568) was a NEGATIVE result — worse, reverted
to 1569. `slot_div` throttle lowers reset probability but does not reliably
eliminate it. **Definitive fix = move flow to the M4 core (its own TCM, off both
contended buses).** That is the recommended next WP.

## Gotcha logged

`sys.reset()` / wedge-reset mid-FileX-op keeps corrupting the volume this session.
`fs.sync()` before a *deliberate* reset avoided it (s239 VGA45 switch); an
*involuntary* wedge-reset can't sync → corruption. Consider a FileX
journal/auto-remount hardening item.
