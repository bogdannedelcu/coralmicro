---
name: op-s10-w16-multicore-2026-05-19
description: "OP-S10-W16 — multi-core (M7+M4) architecture investigation WP filed 2026-05-19.  Operator-noted: build #1130+ removed M4 build (CMakeLists.txt:253) citing 'unreliable SysTick + freeze under load' for flow_task_m4; investigation predated the J-Link/SWD debugger now attached to the board, so root-cause was guess-driven.  With proper JTAG access, re-investigate, identify root-cause (SysTick config? RPMSG deadlock? cache? clock root?), and produce final core-allocation decision.  Motivator: 22 ms ArUco × 30 Hz SafetyTask = 66% M7 budget on a single core; M4 is the only no-cost headroom on RT1176."
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

Open ToDo, build-infrastructure step DONE 2026-05-19 (T2 + T3
groundwork).  Spec in `ideas/wbs.md` OP-S10-W16 entry with five
tasks T1..T5.

## Done 2026-05-19 (T2/T3 groundwork)

- `examples/sentai_runtime/m4_aruco_bench.cc` — bare-metal M4
  ArUco threshold bench (no FreeRTOS scheduler, no SysTick =
  sidesteps build-#1130 freeze mode).  Polls a shared mailbox
  at 0x202C0000, runs `aruco_adaptive_threshold` on 160×120
  frame, writes DWT cycle delta back.
- CMake `add_executable_m4(sentai_m4_aruco_bench ...)` plus
  `add_executable_m7(sentai_runtime ... M4_EXECUTABLE
  sentai_m4_aruco_bench DATA)` — M4 ELF builds clean AND
  embeds into the M7 firmware via `.core1_code` at SDRAM
  0x80e00000.  `m4_binary_start` / `_end` / `_size` symbols
  resolve.
- All `libs_*-m4_*` dependencies still build clean — the
  retreat at build #1130 only removed the sentai_runtime ELF
  target, not the underlying library infrastructure.
- Frame-size rationale: 160×120 (not 320×240) chosen for M4
  bench so all buffers fit in M4-local OCRAM/m_data.  Math:
  309 KB integral @ 320×240 vs 78 KB @ 160×120; the 384 KB
  m_ocram region needs to also hold s_binary/s_labels and
  some margin.  Detection-rate impact on the s174 frame set
  is a follow-up question, not blocking the cycle-count
  measurement.
- CMake gotcha worth remembering: `DATA` is a multi-value
  keyword in `add_executable_m7`; `M4_EXECUTABLE + DATA`
  block MUST go AFTER the source list, not before.

## Ahead (resumes next session)

1. M7-side wiring: call `IpcM7::GetSingleton()->StartM4()`
   on boot (or behind an MP binding for on-demand kick).
2. M7 task that writes a synth gray frame at
   `0x202C0000 + 16`, kicks the go flag, polls for done,
   reads the cycle count.
3. Expose result via `sentai.diag.m4_aruco_bench(block)`.
4. Flash + run + capture cycles.

Paper-estimate expectation: M4 cycle count for 160×120
threshold = **~5-6 ms** vs current M7 22 ms for 320×240.
4× pixel reduction is dominant; clock penalty (½) is roughly
cancelled.  If measured number lands close, W16-T3 verdict
is "M4 viable" → proceed T4 (decision matrix) + T3b (Flow
hot-path offload).

## Current core allocation (2026-05-19, code review)

- **Cortex-M7 @ 800 MHz** (single FPU, D-cache, I-cache, ITCM,
  DTCM): runs EVERYTHING.
  - Camera CSI ISR (.ramfunc → ITCM)
  - PrepTask (PXP DMA orchestration)
  - InferTask (TPU via USB-EHCI, async)
  - Flow SAD (flow_task.cc — single core, all M7 per file header
    line 14; was on M4 until build #1130, see W16 history)
  - ArUco detect (sentai_aruco_detect via SafetyTask, 22 ms / call)
  - SafetyTask state machine (period 33 ms = 30 Hz nominal)
  - FR drain (FxUser writes)
  - Crazy radio bridge (UART2)
  - REPL + MicroPython
  - USB CDC-ACM + CDC-NCM stack
  - lwIP HTTP
  - Health / dmesg / calib / places / servo bindings
- **Cortex-M4 @ 400 MHz** (single-precision FPU per NXP datasheet
  — M4F, NOT no-FPU as operator initially recalled): **idle**.
  `flow_task_m4.cc` kept on disk as historical reference, NOT
  compiled.  See `CMakeLists.txt:253` comment for the build-#1130
  removal rationale.

## Why this matters

Single-core M7 budget pressure already real:
- ArUco detect: 22 ms (post T18-T+U CMSIS-DSP, math-identical)
- SafetyTask period: 33 ms (30 Hz nominal)
- Utilisation just for safety: 22/33 ≈ 66 %
- Add TPU orchestration, REPL, USB stack, FR drain, future
  descriptors / L1 tracker / outdoor PX4 work → over budget.

M4 is the obvious release valve, but the build-#1130 freeze was
never root-caused — investigation done without a JTAG debugger.
Operator (2026-05-19): *"eu am impresia ca atunci cand s-a
dezvoltat pe M4 nu aveam swagger atasat iar jtag ne-ar fi dat
concluzii mai bune"*.

## ArUco consumer audit (collected during the operator
question that spawned W16)

Only SafetyTask calls `sentai_aruco_detect()` live in C.
Autotune + VPE forwarder + mission scripts piggyback on
`sentai_aruco_get_latest()` cached results
(see `sentai_calib_task.cc:6-7` header comment — explicit
"REUSE ONLY (no duplicate compute)" contract).

So the M7 budget squeeze comes from **one consumer**
(SafetyTask) — and lowering its period from 33 ms → 100 ms
(10 Hz, plenty for drone abort detection) drops the budget to
22 % M7 instantly.  That's the cheap pre-W16 mitigation; W16
remains the long-term play.

## Cross-refs

- `[[op-s10-w14-t18-simd-threshold-2026-05-19]]` — 22 ms detect
  number this WP is reacting to.
- `[[op-s10-w15-arm-memory-budget]]` — sister design WP for
  the memory side of the same architecture conversation.
- `ideas/wbs.md` OP-S10-W16 — full task breakdown.
