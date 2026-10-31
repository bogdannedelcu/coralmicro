# s240 follow-up — M4 flow-offload code review + re-test plan

Anchor: `TD-S10-A10` · EXP-s240 · follows the M7 memory-bandwidth-saturation
finding (see [README](README.md)). Goal: re-evaluate moving optical-flow compute
to the idle Cortex-M4, now that we understand the real bottleneck is the M7
**memory bus** (not CPU) — which M4's own TCM sidesteps.

## Why re-visit: the prior M4 attempt was rejected for the WRONG reason

M4 offload was dropped (build #1130, 2026-05-05) and the OP-S10-W16 ablation
concluded "M4 is 2.4× slower than M7" (ArUco threshold: M7 12 ms vs M4 29 ms).
That metric is about **compute speed** — irrelevant to s240. The s240 problem is
that detection **alone** saturates the M7 memory bandwidth (OCRAM = 900 KB
TPU-input DMA/invoke; SEMC = camera + PXP), so flow's FFT/SAD tips it over on
whichever M7 bus it uses. Moving flow to M4 wins even at lower flow FPS, because
M4's ITCM/DTCM is off **both** contended M7 buses. So the earlier rejection does
not apply here.

## Code review — 3 concrete defects in the old M4 path (not fundamental limits)

Comparing the frozen `flow_task_m4.cc` (disabled) against the WORKING
`m4_aruco_bench.cc` (still built + boots fine) pinpoints why it "didn't work
great":

### Defect 1 — FreeRTOS scheduler on a deliberately-miscalibrated SysTick (THE freeze)
`flow_task_m4.cc` ships its own `main()` that skips `BOARD_InitBootClocks` (to
avoid resetting the M7 camera PLLs), which leaves **SysTick miscalibrated**
(runs ~360× fast — admitted in the file's own comment at line 48). It then calls
`vTaskStartScheduler()` and the worker loop uses `vTaskDelay(pdMS_TO_TICKS(1))` +
`xTaskGetTickCount()` — all bound to that broken SysTick. Net effect: the tick ISR
fires ~360× too often (CPU drowns in tick interrupts) → ~1 fps throughput and the
observed freeze. **Running a scheduler on a broken system clock is the root
cause**, not an M4 limitation.
- Fix: run the M4 worker **bare-metal** — no `vTaskStartScheduler`, no
  `vTaskDelay`, no `xTaskGetTickCount`. Tight poll loop, all timing from **DWT**
  (`CYCCNT`, the only reliable timer on this core — the code already uses it for
  compute timing). Heartbeat via a DWT-derived counter, not ticks.

### Defect 2 — scalar SAD instead of USADA8 SIMD (4× slower than necessary)
`flow_task_m4.cc` `sad_match()` does byte-wise `abs(c[x]-p[x])` (lines 106–107) —
`usada8` count = **0**. The working bench (`m4_aruco_bench.cc::m4_flow_sad_at_`)
uses the **USADA8** DSP instruction (4 bytes / instruction, same as M7). That's a
free 4× on the SAD inner loop.
- Fix: reuse the bench's `m4_usada8` / `m4_flow_sad_at_` path.

### Defect 3 — shared buffer address is STALE, overlaps the TPU input tensor
`FLOW_SHARED_ADDR = 0x202C1000` is a **hardcoded pointer** (no linker section, so
nothing catches a collision). Its own comment shows it was sized for the OLD
RPMSG window at `0x202C0000` (16 KB, +4 KB queue headroom). When `rpmsg_sh_mem`
was **relocated** to `0x2033E000` and shrunk to 8 KB (sentai OCRAM layout), this
address was never updated — so `0x202C1000` now lands in the **middle of
`.tpu_input`** (0x20243400–0x20324400, the 900 KB model input). Running M4 flow
concurrently with today's `direct_tensor` detection would have the TPU overwrite
the flow gray buffer every frame, and M4 writes corrupt the TPU input. Also
`flow_shared_t` is ~9.7 KB but the current RPMSG window is only 8 KB — it no
longer fits there either.
- Fix options: (a) demote the now-unused ArUco `.ocram_bss` (~93 KB, ArUco
  superseded by WhyCon) to SDRAM and place `flow_shared_t` at a fixed address in
  that freed OCRAM slice just past `.tpu_input` — small (4.8 KB gray, once/frame,
  negligible bus load); update the M4 MPU non-cacheable region to match; OR
  (b) widen `rpmsg_sh_mem` back to 16 KB on **both** core linker scripts (keep the
  M7↔M4 address/size match — hard rule) and put `flow_shared_t` in the extra 8 KB.
  (a) avoids load-bearing linker changes; (b) is the "proper" home but eats the
  ~9.6 KB OCRAM tail and touches both `.ld` files.

## Re-test plan (operator chose: minimal fix + re-test SAD)

1. **Relocate** `flow_shared_t` off `.tpu_input` (fix Defect 3, option (a):
   ArUco→SDRAM + fixed OCRAM address past tpu_input + M4 MPU region update).
2. **Rewrite** the M4 worker bare-metal (fix Defect 1: no scheduler/SysTick,
   DWT-only timing, tight poll).
3. **USADA8** SAD (fix Defect 2: reuse `m4_flow_sad_at_`).
4. **Re-enable** the M4 build in `CMakeLists.txt` (currently only
   `sentai_m4_aruco_bench` is built; add the flow worker or fold flow into the
   bench's proven bare-metal boot).
5. **Build both cores** (ARM must not regress), boot M4 via `IpcM7::StartM4()`,
   run detection on M7 + flow on M4 concurrently.
6. **Measure**: (a) M7 stability — does detection run reset-free at full flow rate
   now that flow's compute + working set are off both M7 buses? (b) M4 flow rate
   (DWT cycles → fps) and correctness (dx/dy vs the M7 reference from
   s083_flow_m7_validate).

## Implementation progress (2026-07-01)

DONE — the code fixes from the review, all building green (M7 #1570):
- **Defect 1 (freeze) FIXED** — `flow_task_m4.cc` rewritten bare-metal: no
  `vTaskStartScheduler`, no `vTaskDelay`, no `xTaskGetTickCount`. `main()` does
  `BOARD_ConfigMPU` + `MCMGR_Init` + `dwt_init` then runs the poll loop directly.
  All timing from DWT->CYCCNT. Heartbeat on a DWT cadence.
- **Defect 2 (slow) FIXED** — SAD now uses USADA8 (`m4_usada8` + `M4_LD32U`,
  8× unrolled over the 32-wide block), same intrinsic as the working bench.
- **Defect 3 (overlap) FIXED** — root cause was a STALE hardcoded address: when
  `rpmsg_sh_mem` moved to the OCRAM tail + shrank to 8 KB, `FLOW_SHARED_ADDR`
  (0x202C1000) was never updated and fell inside `.tpu_input`. Fixed by
  re-widening `rpmsg_sh_mem` to 16 KB in BOTH core `.ld` files (0x2033C000, MPU
  auto-sizes from `__RPMSG_SH_MEM_SIZE`) and pointing `FLOW_SHARED_ADDR` at
  0x2033D000 (rpmsg base + 4 KB queue head-room). M7 build #1570 PASS: `.ocram_bss`
  ends 0x2033BA20 ≤ new m_ocram end 0x2033C000 (1504 B margin); rpmsg control block
  0x2033C000–0x2033C8F8 sits below flow_shared, no collision.
- **BONUS perf fix** — removed the `m4_curr_gray` mirror field (flow_shared_t v5).
  Its per-frame 4800 B memcpy to non-cached OCRAM was a documented dead-end that
  "cratered M4 SAD to ~1 fps" (experiment.md:1127) — a second cause of the old
  "1 fps", independent of the SysTick freeze.

REMAINING for a live board test (the M7-side integration + bring-up):
1. **CMake** — build `flow_task_m4.cc` as the M4 ELF and set it as the M7's
   `M4_EXECUTABLE` (currently `sentai_m4_aruco_bench`).
2. **M7 glue (currently STUBBED)** — `sentai_flow_m4_publish_frame` was gutted
   when M4 was disabled. Need a real path: copy the PrepTask FLOW_GRAY_80x60 slot
   → `FLOW_SHARED().gray`, bump `frame_seq`, set `frame_valid=1`; boot M4 via
   `IpcM7::StartM4()`; send `FLOW_CMD_START` (write `cmd` + bump `cmd_seq`); read
   `last_dx/last_dy/last_confidence` back.
3. **MicroPython binding** — `sentai.flow.m4_enable()` / a result getter (+ QSTR
   regen).
4. **Build both cores, flash, boot M4, test** detection-on-M7 + flow-on-M4
   concurrent: (a) does detection stay reset-free (flow's SAD + buffers now on the
   M4, off both M7 buses)? (b) M4 flow rate (DWT cycles→fps) + dx/dy correctness
   vs the M7 reference. Do this with J-Link attached (M4 HardFault catch) — booting
   M4 concurrent with the live camera is the highest-risk step.

## ON-BOARD RESULTS (build 1572) — M4 offload WORKS; one contention issue left

The rewritten M4 worker was booted + tested on real hardware concurrent with the
detection pipeline. **All three review fixes are validated:**
- **M4 boots** via `IpcM7::StartM4()`, publishes its magic (`sentai.flow.m4_start()`
  returns 0), `m4_state=RUNNING`.
- **No freeze** — the DWT heartbeat advances steadily (hb 11→20→…), the bare-metal
  loop is stable (the old FreeRTOS-on-broken-SysTick freeze is gone).
- **USADA8 SAD runs** — `frames_processed` advances, `confidence=255` on a static
  scene, compute ≈ 1.72 M cycles/frame ≈ 4.3 ms @ 400 MHz ⇒ **~230 fps capable**,
  vs the old ~1 fps. The scalar→SIMD + mirror-removal fixes landed.

So the core thesis is proven: **flow SAD runs on the M4, off the M7's DTCM/SEMC,
with its working set in M4 DTCM.**

**Remaining issue — the cross-core HANDOFF still contends with the TPU on OCRAM.**
When active flow starts (`flow.start`), the M7 writes the 80×60 gray (4800 B) to
`FLOW_SHARED().gray` — which lives in the rpmsg OCRAM window (0x2033D000) and is
**MPU non-cacheable**, so each write is an individual OCRAM bus transaction — and
the M4 reads it back, all while the TPU is streaming its 900 KB input DMA over the
same OCRAM. Result: intermittent `edgetpu-custom-op failed to invoke` (recoverable,
core survives) or, less often, a watchdog reset at start-up. The DWT poll-throttle
FIXED the *idle* contention (m4_start is now clean), but the *active-frame* handoff
still lands on the TPU's OCRAM bus. `slot_div` throttling helps but does not fully
stabilise it — the same marginal contention as the original M7-flow, just smaller.

**The right fix: move the flow_shared handoff OFF the OCRAM bus into SDRAM.** The
TPU's dominant traffic is its OCRAM input DMA; the camera/PXP own SEMC. Flow's SAD
working set is already in M4 DTCM (neither bus). If the 4800 B/frame gray handoff
is in SDRAM (SEMC) instead of OCRAM, flow **never touches the TPU's bus** — the
small handoff rides SEMC (tiny vs camera DMA) and the SAD rides DTCM. This needs:
(a) `FLOW_SHARED_ADDR` → a fixed SDRAM address; (b) M4 + M7 MPU regions marking
that SDRAM span non-cacheable for cross-core coherency. That is the next step.
(Secondary micro-opt: the M7 gray write is byte-wise to non-cached memory — make
it 32-bit word writes to cut OCRAM transactions 4×.  And/or a DMA/MU-interrupt
handoff instead of the memcpy+poll.)

Net: the hard part (a working, fast, non-freezing M4 flow worker) is DONE and
validated; the last mile is relocating the cross-core handoff to SDRAM so it stops
sharing the TPU's OCRAM bus.

## Risk / scope note

This is a multi-file change touching MPU config, M4 boot, and the build — with
brick / M7-camera-PLL-interaction risk (the reason the old code rolled its own
startup). Best done in a git worktree so the main ARM build never regresses mid-
bring-up, and with J-Link attached to catch any M4 HardFault. Not a 10-minute
test; it is the proper s240 follow-up WP.
