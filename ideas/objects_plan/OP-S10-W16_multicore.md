# OP-S10-W16 — Multi-core (M7 + M4) Architecture

**Status**: T3 + T3 ablation DONE; **PRIMARY VERDICT REVERSED**
post-DCE-fix at commit 576fc1d5.  ArUco threshold on M4 is NOT
viable as a latency optimization — M7 SDRAM (12 ms) beats M4
OCRAM (29 ms) by 2.4×.  Rest of WP (T2/T4/T5 + Flow-on-M4 T3b)
re-scoped accordingly.  See §7.2 + §8.5 below.

## 1. Why this WP exists

Operator-instituted 2026-05-19: when the M4 build was retired at
build #1130 (CMakeLists.txt:253 comment cites "unreliable SysTick +
freeze under load"), the investigation predated the J-Link / SWD
debugger now wired to the board.  Root-cause was guess-driven.

Now that ArUco-detect on M7 measured at **22 ms × 30 Hz SafetyTask =
66 % M7 budget** (post OP-S10-W14-T18-T+U SIMD work), single-core
M7 is genuinely tight.  Camera+IMU-only architecture
([[no-flow-deck-camera-imu-only-2026-05-19]]) means ArUco rate
≡ altitude correction rate, so we can't simply throttle SafetyTask
without degrading Z hold.  **M4 is the only no-cost compute
headroom on this silicon**, and it's currently 100 % idle.

## 2. Silicon reality

NXP i.MX RT1176 = dual-core asymmetric (AMP):

| Core | Clock | Cache | TCM | FPU | SIMD-DSP |
|------|------:|-------|-----|-----|----------|
| Cortex-M7 | 800 MHz | 32 KB I + 32 KB D | FlexRAM 256 KB | yes | yes (USUB8/SEL/UQADD8/…) |
| Cortex-M4F | 400 MHz | 16 KB I, NO D-cache | 64 KB combined | yes (single-prec) | yes (same DSP instr set) |

Operator note 2026-05-19: M4F **does** have FPU (single-precision).
The earlier recall that "M4 has no FPU" was wrong; the real reason
flow_task_m4 was retired was scheduler reliability, not FP support.

## 3. Current core allocation (2026-05-19, code-walk)

- **M7 (800 MHz)** — runs EVERYTHING:
  - Camera CSI ISR (`.ramfunc` ITCM)
  - PrepTask (PXP DMA orchestration)
  - InferTask (TPU via USB-EHCI, async)
  - Flow SAD (`flow_task.cc` header line 14: *"single core, all C++
    on M7"* — was on M4 until build #1130)
  - ArUco detect (`sentai_aruco_detect` via SafetyTask, 22 ms / call
    post T18-T+U)
  - SafetyTask state machine (period 33 ms ≈ 30 Hz nominal)
  - FR drain (FxUser writes)
  - Crazy radio bridge (UART2)
  - REPL + MicroPython
  - USB CDC-ACM + CDC-NCM + lwIP HTTP
  - Health / dmesg / calib / places / servo / explore helpers
- **M4 (400 MHz)** — idle; `flow_task_m4.cc` kept as historical
  reference but not compiled.

## 4. Cross-core memory model (the lesson learned the hard way)

### 4.1 Address aliasing — DO NOT hardcode

RT1176 OCRAM has different aliases from each core.  Per the linker
scripts in tree:

| Core | rpmsg_sh_mem ORIGIN | LENGTH |
|------|--------------------:|-------:|
| M7 (`MIMXRT1176xxxxx_cm7_ram_mp.ld`) | `0x2033E000` | `0x2000` (8 KB) |
| M4 (`libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm4_ram.ld`) | `0x202C0000` | `0x4000` (16 KB) |

These two addresses **alias to the same physical OCRAM region**
via the i.MX RT1170/1176 OCRAM_M7 / OCRAM_M4 bus paths.  But:

- The M7 view of the M4's address (`0x202C0000`) is **CACHEABLE**
  (inside M7's `m_ocram` 0x20240000-0x2033E000), so M7's D-cache
  serves stale data.
- The M4 view of the M7's address (`0x2033E000`) is outside the
  M4's linker map and unreliable.

**Hard rule**: NEVER hardcode an absolute cross-core mailbox
address.  Use the `.noinit.$rpmsg_sh_mem` linker section — both
linkers route that section into their *respective* aliases.  The
SDK + IpcM7/IpcM4 framework configures the MPU to mark this region
non-cacheable on both cores.

This was the architectural mistake in the first iteration of
`m4_aruco_bench.cc` (commit 04b1390e, immediately reverted).

### 4.2 The right pattern — IpcM7 / IpcM4 SendMessage

Canonical reference: `examples/multi_core_ipc/`.  Pattern:

```cpp
// Shared header (both cores include it):
enum class MyMessageType : uint8_t { kFoo, kAck };
struct MyAppMessage { MyMessageType type; /* fields */ } __attribute__((packed));
static_assert(sizeof(MyAppMessage) <= coralmicro::kIpcMessageBufferDataSize);

// M7 side:
auto* ipc = coralmicro::IpcM7::GetSingleton();
ipc->RegisterAppMessageHandler(HandleM4Message);
ipc->StartM4();
CHECK(ipc->M4IsAlive(500));
ipc->SendMessage(my_message);

// M4 side (in app_main):
coralmicro::IpcM4::GetSingleton()->RegisterAppMessageHandler(HandleM7Message);
vTaskSuspend(nullptr);  // RX runs in IpcM4 task context
```

Both `RegisterAppMessageHandler` callbacks run in the respective
core's IpcM4/IpcM7 RX task — i.e. with the FreeRTOS scheduler
active, full SysTick, normal task context.

Per-IpcMessage payload limit: `kIpcMessageBufferDataSize` ≈
80 bytes.  For larger transfers (e.g. camera frames), publish via
a separate non-cacheable buffer + send a "ready" message with the
offset/length.

### 4.3 Cache barriers

The RPMSG buffer storage IS non-cacheable on both sides (linker
takes care of MPU placement).  But any **user buffer outside
rpmsg_sh_mem** that both cores access needs explicit cache
management:

- M7 → M4: `DCACHE_CleanByRange(addr, len)` before signal.
- M4 → M7: `SCB_InvalidateDCache_by_Addr(addr, len)` before read
  on M7 side.

(M4 has no D-cache, so M4-side cache ops are no-ops there.)

## 5. M4 startup conventions (matters a lot)

The SDK's `libs_base-m4_freertos` provides a weak `main()` that:

1. Runs `BOARD_ConfigMPU()` (per-core MPU including the rpmsg
   region as non-cacheable + device-memory)
2. Initialises FreeRTOS data structures
3. Creates a task that calls user-provided `app_main(void*)`
4. Starts the scheduler (vTaskStartScheduler)

**Therefore the M4 user-side entry is `app_main(void* param)`, NOT
`main`.**  Overriding `main` with a custom bare-metal version
(the v1 mistake) bypasses BOARD_ConfigMPU + FreeRTOS init, which
is why my first attempt didn't even get to register an IPC
handler.

The build-#1130 freeze concern about M4 SysTick: I now suspect it
was specific to `flow_task_m4.cc`'s busy-wait + cross-core memcpy
pattern, NOT the SDK FreeRTOS scheduler itself.  The
`multi_core_ipc` example works with the same scheduler — proof
the basic infrastructure is sound.

## 6. M4 memory budget — what fits where

Per `libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm4_ram.ld`:

| Region   | Address                    | Length   | Use |
|----------|---------------------------:|---------:|-----|
| m_interrupts | 0x20200000             | 1 KB     | vector table |
| m_text   | 0x20200400                 | ~127 KB  | code (.text) |
| m_data   | 0x20220000 + NCACHE        | ~128-NC KB | `.bss` + `.data` |
| m_ocram  | 0x202C4000                 | 384 KB   | big buffers (default `.bss`) |
| m_heap   | 0x83000000                 | 8 MB     | FreeRTOS heap (SDRAM) |
| m_sdram  | 0x83000000 + HEAP          | rest 16 MB | SDRAM cold path |

**Implication**: a 320×240 ArUco threshold won't fit M4 fast
memory cleanly — integral image alone is 309 KB.  Options:

- **160×120 (current bench)** — integral 78 KB, total ≤ 120 KB,
  fits m_data + m_ocram comfortably.  Detection-rate impact on
  the s174 frame set is a follow-up validation, not blocking.
- **80×60** — integral 20 KB, fits inside m_data alone.  Used
  by the IPC-style bench (M4 generates synth frame internally,
  no cross-core image transfer needed).
- **320×240** — would need part of integral in M4-accessible
  SDRAM region (m_heap or a dedicated allocation), slower due
  to SEMC.  Only worth doing if a real production scenario
  requires native resolution AND M4-side ArUco.

## 7. Task layout strategies under consideration

### 7.1 Conservative (M7 stays the same)

Don't touch the M7 task layout.  M4 becomes a co-processor for
non-time-critical compute (offline analysis, descriptor pre-
compute, …).  Modest win.

### 7.2 ArUco-on-M4 (OP-S10-W16 primary target) — **ABANDONED**

Original plan: move `sentai_aruco_detect` to M4 (threshold +
connected-components + contour walk + bit decode + PnP).

**Verdict after the corrected ablation (576fc1d5):** NOT viable.
M4 OCRAM @ 320×240 = **29 ms** for just the Bradley threshold
stage, versus M7 SDRAM = **12 ms** for the same stage.  Full
ArUco pipeline (8 stages) on M7 currently runs in 22 ms; an
M4 port would likely exceed 30 ms on threshold alone before any
other stage runs.  That **busts the 33 ms SafetyTask budget**
and offers no headroom for the remaining 7 pipeline stages.

Root cause (honest): even though M7 pays the SDRAM-access tax,
its 800 MHz clock + 32 KB D-cache + superscalar pipeline beat
M4F's 400 MHz single-issue access to OCRAM by a clear factor
of 2.4×.

The earlier conclusion ("M4 wins by ~20%") was caused by a
compiler dead-store elimination artefact in the M4 bench
harness — see §8.5 Round 4 below.

Path forward instead:
- Keep ArUco on M7.
- Look at lower SafetyTask period (33 ms → 100 ms) to drop M7
  budget 66% → 22%.
- Re-evaluate after a smaller TPU input tensor model lands
  (would free OCRAM for M7's integral image; estimate ~0.85 ms
  per threshold call at M7 OCRAM).
- Reserve M4 for low-latency interrupt-driven work where M7's
  cache/SDRAM jitter is unacceptable (sensor fusion, motor
  controllers).  Not bulk compute.

### 7.3 Flow-on-M4 hot path (OP-S10-W16-T3b, stretch)

After ArUco-on-M4 lands, also move the Flow USAD8 SAD inner loop
to M4.  PXP downscale stays on M7 (PXP only reliable from M7);
M7 hands off the post-PXP 80×60 Y8 (4.8 KB) per camera tick to
M4 for SAD search.  Frees ~7 % M7 budget.

### 7.4 Future-future (OP-S10-W16-FW)

If Track A descriptors land + L1 tracker grows, the same M4
offload pattern is the natural release valve.

## 8. Investigation roadmap (T1..T5)

- **T1** — *Document current allocation*: this file partially
  covers it; needs FreeRTOS task list dump from on-board diag
  (uxTaskGetSystemState) to add per-task period/priority/stack.
- **T2** — *J-Link / SWD harness*: tiny M4 hello-world with
  `vTaskDelay(100)` + GPIO toggle; trace via OpenOCD / PyOCD.
  Confirm SysTick fires reliably under no load, M7-heavy load,
  SEMC pressure.
- **T3** — *ArUco bench on M4* — **DONE** 2026-05-19.
  - Build infrastructure: commits 1869594c, 4c4dee42.
  - IPC handshake `M4IsAlive`: works.
  - **kBenchDone reply path: FIXED at 692f8b95** — root cause
    was rpmsg_sh_mem linker mismatch (M7 had it at 0x2033E000/8 KB;
    M4 SDK default at 0x202C0000/16 KB).  IpcM4/IpcM7
    MessageBuffer framework reconstructs handles as
    `__RPMSG_SH_MEM_START | eventData` — different physical
    regions silently lose replies.  Fixed by aligning M4's
    linker to the M7 layout.  See
    `[[rpmsg-cross-core-alignment-hard-rule-2026-05-19]]`.
  - Bench results: **3.9 M cycles @ 320×240 = 9.75 ms** scalar
    Bradley, plain C, no SIMD (commit 862b5c91).
- **T3 ablation** — *measurement triangulation* — **DONE**.
  Three rounds: 862b5c91, e8d8a584, 940611e2.  See §8.5 below.
- **T3b** — *Flow hot path on M4* (stretch, task #79).
- **T4** — *Decision matrix*: with T2 + T3 data, ranked
  recommendation for which workloads move.
- **T5** — *Architecture freeze*: paper/multi_core_final.md
  with the validated split.

## 8.5 Ablation measurements (2026-05-19, FOUR rounds)

| Setup                              | Wall-clock | Cycles  | Cyc/px | Honest? |
|------------------------------------|-----------:|--------:|-------:|--------:|
| M7 SDRAM, cache ON @ 320×240       |    12 ms   |  9.6 M  |  125   |   YES   |
| M7 SDRAM, cache OFF @ 320×240      |   148 ms   | 118.6 M | 1544   |   YES   |
| **M4 OCRAM @ 320×240 (DCE-fixed)** | **29 ms**  | 11.7 M  |  152   | **YES** |
| ~~M4 OCRAM @ 320×240 (DCE)~~       |   9.75 ms  |  3.9 M  |   51   |   NO    |
| ~~M4 OCRAM @ 80×60 (DCE)~~         |   250 µs   |   100 k |   21   |   NO    |
| ~~M7 OCRAM @ 160×120~~             |   212 µs   |   170 k |  8.8   |  M7 yes |
| ~~M4 OCRAM @ 160×120 (DCE)~~       |   2422 µs  |   970 k |   50   |   NO    |
| (M7 OCRAM @ 320×240, extrapolated) |  ~850 µs   |  ~680 k |  ~8.8  | (if free) |

Round 4 (576fc1d5) — **DCE artefact uncovered, RESULTS REVERSED**:

`arm-none-eabi-nm` on the M4 ELF showed `s_binary` symbol
completely absent.  The bench wrote into `s_binary` but the
worker never read it back, so M4 LTO+`-O3` eliminated the entire
Phase-2 compare+store loop.  Previous M4 numbers were Phase-1-
only (integral image build).

Fix: XOR-fold `s_binary` into a `volatile s_bench_sink` after
the timed region (keeps stores load-bearing without forcing
per-byte volatile stores, which would have been pessimistic).

Honest re-measurement on build #1380:
- M4 OCRAM @ 320×240 = **29 ms** (was claimed "9.75 ms")
- 152 cyc/px at 400 MHz — 3× more than DCE'd version

**Verdict reversed**: M7 SDRAM 12 ms < M4 OCRAM 29 ms.  M4
offload is NOT a win.  See §7.2 above for full path forward.

Methodology hard-rule for future bench harnesses: ALWAYS verify
output symbols survive linking via `nm` before trusting cycle
counts.  Any kernel that "produces a side-effect buffer" must
include a load-bearing consumer of that buffer.

Round 1 (862b5c91) — naive "M4 1.6× faster than M7":
**WRONG** + DCE-affected.  Compared M4-OCRAM (Phase-1-only,
mismeasured as 9.75 ms) against M7's `aruco_bench` kernel
(16 ms) — a different implementation, not sentai_aruco.cc's
production threshold.  Apples to oranges, AND apples were
actually 3× larger than reported.

Round 2 (e8d8a584) — M7 cache-disabled + M4 SIMD attempt:
- M7 cache OFF: 12× slower (148 ms).  Cache is **essential** to
  hide SDRAM 50 ns access penalty, not a thrash liability.
- M4 SIMD: 2.5× **slower** than M4 scalar.  Pack-overhead
  doesn't pay off on M4F single-issue without dual-issue
  pipeline to absorb the setup cost.  Reverted at e51915bd.

Round 3 (940611e2) — apples-to-apples both cores in OCRAM at
160×120 (also DCE-affected on M4 side):
- M7 OCRAM 212 µs (honest), M4 OCRAM 2422 µs (DCE'd, real ~6 ms)
- Conclusion "M7 wins 11×" was already pointing the right way
  but the magnitude was inflated by the DCE on the M4 side.

Architectural takeaway (which Round 4 confirms):
1. **800 MHz clock** vs M4's 400 MHz (2× advantage).
2. **D-cache + working-set locality** — even when the integral
   image is in SDRAM, the row-stride scan keeps the relevant
   cache lines warm, so most reads hit L1.
3. **Superscalar dual-issue** on M7 vs M4F single-issue (~1.5×
   per-cycle advantage on memory-bound loops).

Combined: M7 ~2.4× faster than M4 even when M4 has OCRAM and
M7 has SDRAM.  The "M4 has unallocated OCRAM" advantage is
real but insufficient to overcome the core-architecture gap
for compute-bound kernels at production frame sizes.

If a future TPU model shrinks `.tpu_input` enough to free
≥309 KB of OCRAM for ArUco, M7 would run threshold at ~0.85 ms
— a 14× improvement over current production.  Documented as a
deferred lever in `[[op-s10-w15-arm-memory-budget]]`.

## 9. Hard rules / DO NOT REDISCOVER

1. **Never hardcode cross-core addresses**.  Use
   `.noinit.$rpmsg_sh_mem` linker section + IpcM7/IpcM4 API.
2. **M4 user entry is `app_main(void*)`, not `main`**.  The SDK
   weak `main` runs BOARD_ConfigMPU + FreeRTOS — overriding it
   skips critical init.
3. **IpcMessage payload ≤ ~80 bytes**.  Larger data goes via a
   separate buffer + a "ready" message.
4. **Cache barriers on user buffers**: clean before signal
   (M7 side), invalidate before read (M7 side).  M4 has no
   D-cache, so M4 side is a no-op.
5. **PXP is M7-only**.  M4 cannot reliably orchestrate PXP
   (cross-core arbitration unimplemented in this SDK rev).
6. **Per-frame handoff M7 ↔ M4 was the build-#1130 freeze
   trigger** for `flow_task_m4`.  Keep handoffs as rare as
   possible (≤ 10 Hz preferred), use the IpcMessage queue with
   bounded length.

## 10. Cross-refs

- `[[op-s10-w14-t18-simd-threshold-2026-05-19]]` — the 22 ms M7
  number motivating this WP.
- `[[no-flow-deck-camera-imu-only-2026-05-19]]` — why ArUco
  rate ≡ altitude correction rate.
- `[[op-s10-w15-arm-memory-budget]]` — sister design WP for the
  memory side.
- `examples/multi_core_ipc/` — canonical SDK example for IPC.
- `examples/multi_core_hello/` — minimal M4-boot reference.
- `libs/base/ipc_m7.{h,cc}`, `libs/base/ipc_m4.{h,cc}` — the
  framework.
- `examples/sentai_runtime/m4_bench_message.h` +
  `m4_aruco_bench.cc` + `m4_bench_host.cc` — current WIP.
- `examples/sentai_runtime/flow_task.cc:14` — header note that
  Flow runs on M7 since build #1130.
- `examples/sentai_runtime/CMakeLists.txt:253-256` — build-#1130
  M4-retreat comment.
