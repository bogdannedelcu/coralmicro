# OP-S10-W16 — Multi-core Investigation Plan

Companion to `OP-S10-W16_multicore.md`.  This is the *execution
plan* for resolving the M4 viability question definitively.
Operator request 2026-05-19 "fa un plan".

## 0. Evidence inventory (what we already know)

### 0.1 The board WILL run heavy compute on M4

`examples/tflm_person_detection_m4/` is shipped + working in
tree.  It does:

- Full FreeRTOS scheduler on M4
- TFLite Micro 300 KB model inference on M4 (no TPU)
- **Camera streaming controlled from M4** (`CameraTask::GetSingleton()`)
- M7-side is one line: `StartM4()` + `M4IsAlive(500)` + `vTaskSuspend(nullptr)`
- Per-frame latency reported via `printf` from M4

The `flow_task_m4` retreat at build #1130 was therefore a
**flow-specific failure**, not an M4-platform failure.  Likely
causes (per the diary 2026-05-19 entry):

1. flow_task_m4 was doing per-camera-tick (30 Hz) M7→M4 memcpy of
   a full frame via a busy-wait protocol — high SEMC pressure,
   possibly held interrupts disabled.
2. The handoff used a hand-rolled shared-memory mailbox (not the
   official IpcM4 MessageBuffer), which on RT1176 had subtle
   cache-coherency issues.
3. The 30 Hz handoff might have starved the M4 IpcM4 RX task —
   exactly the failure mode we're seeing right now in T3-v2 (M4
   IsAlive works, but kBenchDone doesn't return).

### 0.2 What multi_core_ipc proves

- `IpcM7::SendMessage` + `IpcM4::RegisterAppMessageHandler` IS the
  correct pattern.
- `IpcM7::M4IsAlive(timeout_ms)` provides a real handshake — it
  returns true only after M4's IpcM4 framework signals ready.
- Handlers run in IpcM7/IpcM4 RX task context (FreeRTOS scheduler
  active, SysTick alive).

### 0.3 What our T3-v2 measurement says (build #1359)

- M7 calls `IpcM7::StartM4()` → M4 boots → `M4IsAlive(500)` returns
  TRUE on the second sentai.diag.m4_aruco_bench() call.
- M7 SendMessage(kBenchGo) is queued; M4's `handle_m7_message_` is
  presumably called.
- BUT M7-side `s_result_sem` never gets given → timeout, ok=0.
- At block=101 (heavier work) the USB serial disconnects → either
  M4 enters a livelock that holds resources M7 needs, or a
  cross-core deadlock fires the WDOG.

**Working hypothesis (high-confidence)**: doing ~1 ms of compute
inside `handle_m7_message_` starves the IpcM4 RX-task message-
buffer service.  The reply SendMessage is queued but the
underlying `xMessageBufferSend` call from RX context may need a
context switch that never happens before timeout.

## 1. Plan, ordered by leverage

### 1.1 T2 — JTAG hello-world harness (½ day)

Goal: prove M4 SysTick is healthy in isolation, *before* loading
ArUco onto it.

- Drop a tiny `m4_jtag_hello.cc` that:
  - Initialises a GPIO output
  - Runs `vTaskDelay(pdMS_TO_TICKS(100))` in a loop
  - Toggles the GPIO each iteration
- Build as separate target, embed into a stripped-down M7 firmware
  that ONLY does `StartM4` + suspend.
- Run with J-Link attached.
- Verify via OpenOCD / PyOCD:
  - SysTick ISR fires every 1 ms (read `xTickCount` from M4 RAM)
  - `vTaskDelay` returns on time
  - GPIO toggles at 10 Hz
- Stress test: simultaneously load M7 (TPU pipeline running,
  camera at 30 fps) and check whether M4 SysTick *jitter* increases.

**Output**: T2 verdict — "M4 SysTick is healthy on this silicon
under our typical M7 load" OR "M4 SysTick has X ms jitter under Y
condition".  Either answer kills hypotheses cleanly.

### 1.2 T3.5 — Move bench compute off RX context (1-2 hours)

Goal: fix the kBenchDone-not-arriving issue.  Standard
producer-consumer pattern.

**Operator note 2026-05-19**: M4 also supports doing work in ISR
context (not just FreeRTOS tasks).  This widens the design space:

| Pattern | When appropriate | Trade |
|---|---|---|
| Work in RX-callback (current T3 attempt) | only for trivial replies (~<10 µs) | starves IpcM4 RX task if heavier — what we hit |
| Work in dedicated FreeRTOS task | jobs ≥ 100 µs; cooperative pre-emption | adds ~5 µs context switch each side |
| Work in custom M4 ISR | sub-100 µs, hardware-driven (e.g. timer- or GPIO-triggered) | must stay reentrant-friendly, no FreeRTOS API calls except `…FromISR` variants |
| Hybrid ISR + task notify | typical drone-control pattern — ISR collects fresh sample, task does the math | clean separation, requires `vTaskNotifyGiveFromISR` glue |

For ArUco-on-M4 specifically, **dedicated task is the right
pick** (per-frame compute is ~tens of ms, way above ISR scope).
The hybrid ISR-then-task pattern becomes the right pick later if
we move *flow_task* per-frame work to M4 — Flow SAD on a 4.8 KB
post-PXP frame is sub-ms and naturally fits "ISR-on-camera-tick
fires + task runs SAD" pattern.

Architecture:
- M4-side:
  - `app_main`: register handler; create a "work" task at
    priority IDLE+3; `vTaskSuspend(nullptr)`.
  - Handler: just copies the kBenchGo block size into a shared
    field + gives a binary semaphore.  Returns immediately.
  - Work task: blocks on the semaphore; on take, runs the
    threshold + DWT timing + `IpcM4::SendMessage(kBenchDone)`.
- M7-side: unchanged.

This matches the canonical "RX handler is fast, work is in a
task" embedded-FreeRTOS pattern.  Almost certainly fixes the T3
failure.

### 1.3 T3.6 — Measure cycle count + scaling (30 min)

After T3.5, run the bench across block ∈ {3, 7, 23, 51, 101, 201,
255}.  Capture:

- Cycles per block size on 80×60 frame, M4 @ 400 MHz
- Sigma over 30 iterations
- Cross-check vs M7 measurement (already shipped: 22 ms on 320×240)
- Compute the M4-vs-M7 ratio for "same algorithm, different cores"

### 1.4 T4 — Decision matrix (1 hour)

Given T3.6 data, fill in:

| Workload | M7 cost | M4 cost (est.) | Cross-core overhead | Verdict |
|---|---|---|---|---|
| ArUco threshold (Bradley) | 22 ms (320×240) | ? ms (80×60), ? ms (160×120 extrapolated) | ~150 µs (IpcMessage round-trip @ 30 Hz × 2) | TBD |
| Flow USAD8 SAD inner loop | ~1 ms / call | ? ms | ~50 µs (4.8 KB post-PXP handoff) | TBD |
| TPU pre-process (RGB→Y8 + quant) | ~0.7 ms | ? ms | requires PXP on M7 first | likely keep on M7 |

Recommendation lands in `paper/multi_core_final.md`.

### 1.5 T5 — Architecture freeze (½ day)

Write up the final split + cache rules + IPC contract in a
single design doc the thesis chapter cites.  Includes the
DO-NOT-REDISCOVER list from W16-spec section 9.

### 1.6 T3b (stretch, after T5 freeze) — Flow hot path on M4

If ArUco-on-M4 lands cleanly, extend the same harness to host
the Flow USAD8 SAD inner loop on M4.  PXP stays on M7; the
post-PXP 80×60 Y8 (4.8 KB) handoff per camera tick uses the
same IpcMessage + shared OCRAM-buffer pattern.

## 2. Documentation sources to consult next session

### 2.1 In-tree references (already located today)

- `examples/multi_core_hello/{hello_world_cm7.cc,hello_world_cm4.cc}`
  — minimal "M4 boots + prints" example.  Read full for any
  startup nuance.
- `examples/multi_core_ipc/{multi_core_ipc.cc,multi_core_ipc_m4.cc,
  example_message.h}` — canonical IPC pattern (already studied).
- `examples/tflm_person_detection_m4/` — proof M4 can run TFLM
  inference + camera autonomously.  Showcases:
  - M4-side `CameraTask::GetSingleton()->Init(I2C5Handle())`
    + `Enable(kStreaming)` — camera runs from M4
  - `TimerMicros()` for M4-side timing
  - `printf` from M4 (goes where? — confirm if it surfaces in M7's
    serial output via RPMSG, or is M4-only)
- `examples/multi_core_blink_led/` — even-more-minimal smoke
  reference.
- `libs/base/ipc_m4.{h,cc}`, `libs/base/ipc_m7.{h,cc}` — framework
  internals; read once to understand the queue plumbing.
- `libs/base/main_freertos_m4.cc` — M4 startup + how user
  `app_main` is called.

### 2.2 NXP / ARM official docs to fetch

- **NXP RT1176 Reference Manual** (IMXRT1170RM.pdf, multi-thousand
  pages): chapters on:
  - 8.3 OCRAM Memory Map — aliasing between cores
  - 11 Multicore Communication (Messaging Unit MU)
  - 12.4 MPU configuration on M7 vs M4
  - Cache management (D-cache invalidate / clean ranges)
- **NXP MCMGR library docs**: how `MCMGR_StartCore` actually works.
  Component path: `third_party/nxp/rt1176-sdk/components/multicore/mcmgr/`.
- **RPMsg-Lite documentation**: used internally by IpcM7/IpcM4,
  understand the message-buffer guarantees.
- ARM Cortex-M4 / M7 Technical Reference Manuals — for the
  per-core SysTick + Cache + MPU specs.
- AN12822 or similar NXP application note on RT1170 dual-core
  development.

### 2.3 Web research targets (for next session)

- `site:community.nxp.com RT1176 multicore SysTick freeze`
- `imxrt1170 ocram_m4 alias address map`
- `freertos m4 m7 mcmgr deadlock`

## 3. Risks + mitigations

| Risk | Likelihood | Mitigation |
|---|---|---|
| build-#1130 freeze reproduces under heavy M4 load | medium | T2 stress test isolates root cause before adding ArUco |
| ArUco on M4 isn't fast enough (>30 ms) for our 30 Hz SafetyTask | medium-low | run T3.6 *with realistic frame size* (need 160×120 for production accuracy; bench at multiple sizes) |
| Cross-core latency > useful for VPE @ 30 Hz | low | IpcMessage round-trip is reported as <1 ms in our other RPMSG-using subsystems |
| Cache coherency bites us on a hot path | medium | rely on linker-attributed `.noinit.$rpmsg_sh_mem` instead of hand-rolled MPU, follow the DCACHE clean/invalidate rules from the WP spec |
| FPU usage on M4 stalls IPC | very low | M4F has FPU; we'll use only single-precision |

## 4. Decision gates (operator approval points)

- **After T2**: go / no-go on M4 viability.  If SysTick is broken,
  pivot to "stay on M7, optimise harder OR descope ArUco rate."
- **After T3.6**: go / no-go on shipping ArUco-on-M4.  If M4
  ArUco at 160×120 is faster than M7 ArUco at 160×120 plus the
  cross-core overhead, move.  Otherwise keep on M7 + investigate
  PXP threshold (algorithm shift, last resort).
- **After T4**: pick the canonical architecture for the thesis.
- **After T5**: paper-ready freeze; T3b and future moves are
  separate WPs based on this freeze.

## 5. Estimated total effort

- T2 JTAG hello-world: ½ day
- T3.5 work-task fix: 1-2 h
- T3.6 measure + scale: 30 min
- T4 decision matrix: 1 h
- T5 paper/multi_core_final.md: ½ day
- **Total: 1.5-2 days** of focused work + JTAG access.

T3b is +1-2 days on top *if* the verdict is favourable.

## 6. Where this fits the thesis

Multi-core offload turns into a strong **embedded-engineering
chapter** for the defense:

- "Single-core M7 saturated at X % budget"
- "Dual-core RT1176 silicon was present from day one but unused"
- "Investigated, root-caused the original retreat, validated under
  JTAG, moved <subsystem> to M4 freeing Y % M7"
- "Final architecture: M7 = perception + control + I/O, M4 =
  dedicated vision worker, communicating via Z Hz IpcMessage"

That's a clean narrative arc, independent of any ArUco-specific
choices.  Even if the verdict is "stay on M7" the analysis itself
is a defensible chapter.
