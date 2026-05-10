# Sim.md — SentAI x86/Linux SIL plan

Companion to [CLAUDE.md](CLAUDE.md). Scope: bring up a SIL (software-in-the-loop) build of the SentAI firmware on x86_64 Linux **without modifying core SentAI source code**, using the FreeRTOS POSIX/Linux port that is already vendored in this repo.

This file is the durable plan for the porting effort. Update it as we add layers.

---

## 1. Goal

**Same source code, two build targets.**

| Build | Toolchain | Where | Status |
|---|---|---|---|
| **ARM (current, prod)** | `arm-none-eabi-gcc` (in `third_party/toolchain-linux/`) | `build/` | ✓ Working — must NEVER regress |
| **SIM (new)** | host `gcc` / `clang` | `build-sim/` | ⏳ To build |

**Phase 1 deliverable**: `./build-sim/sentai_sim` boots on Linux, FreeRTOS scheduler runs, MicroPython REPL prompts on stdin/stdout, `>>> 1+1` returns `2`. No board hardware involved.

**Future phases** (see §7): NAND→file, UART→socket→CrazySim, camera→socket→Gazebo, TPU→libedgetpu Linux. Each phase is independent and can be skipped if not needed.

---

## 2. Hard rules (NEVER violate)

1. **ARM build must always pass.** Any change that breaks `bash build.sh` is rejected. CI must build ARM on every commit.
2. **No `#ifdef SENTAI_SIM` in core SentAI source.** Conditional compilation lives in HAL/shim layers (new `sim/` folder), or in headers we control. The SentAI app code (`micropython_task.c`, `sentai_crazy.cc`, `flow_phase_corr.cc`, etc.) stays platform-agnostic.
3. **DO NOT vendor crazyflie-firmware (or CrazySim) into this repository.** The user keeps Crazyflie firmware in a separate folder (`/home/bogdan/work/crazyflie/crazyflie-firmware/` and friends). When Phase 3 lands (UART socket → CrazySim), we ASK the user where to clone CrazySim before doing it. SentAI ↔ CrazySim coupling lives in the socket protocol (the wire), NOT in the repo layout.
3. **One source of truth.** Algorithms live in one place. Flow algo, MP bindings, REPL parser — same file compiled for both targets.
4. **No fork of `third_party/`.** Vendored libs (FreeRTOS kernel, MicroPython, FileX/LevelX, CMSIS) are shared between ARM and SIM. Only the **port layer** of FreeRTOS differs (we select `portable/ARM_CM7` vs `portable/ThirdParty/GCC/Posix` at CMake time).
5. **SIM is a tool, not a product.** ARM remains the deliverable. SIM is for testing logic in CI, reproducing scheduler bugs (like the SAFE-MODE brick from build #1225 lesson), and developing without hardware.
6. **Defer hardware emulation.** Phase 1 stubs out everything (camera, UART, USB, TPU, NAND). Real bridges to Gazebo/CrazySim come later, only if we need them.

---

## 3. Architecture overview

```
┌────────────────────────────────────────────────────────────────────┐
│  Application layer (SentAI source, UNCHANGED)                       │
│                                                                     │
│  examples/sentai_runtime/*.c/cc, modsentai_*.c                     │
│  libs/base/{filesystem,fx_user_fs,...}.cc                          │
│  flow_phase_corr.cc, sentai_crazy.cc, micropython_task.c           │
│                                                                     │
│  Uses: FreeRTOS API (xTaskCreate, vTaskDelay, xQueueCreate, ...)   │
│        MicroPython embed API (mp_embed_*)                          │
│        FileX / LevelX API (fx_*, lx_*)                             │
│        HAL API (hal_uart_*, hal_camera_*, hal_console_*) — NEW     │
└─────────────┬──────────────────────────────┬────────────────────────┘
              │                              │
   ┌──────────▼──────────┐        ┌──────────▼──────────┐
   │  ARM build          │        │  SIM build          │
   │                     │        │                     │
   │  FreeRTOS:          │        │  FreeRTOS:          │
   │   ARM_CM7 port      │        │   POSIX port ⭐     │
   │                     │        │                     │
   │  HAL impl:          │        │  HAL impl:          │
   │   libs/base/console │        │   sim/console_stdio │
   │   libs/base/usb_*   │        │   sim/uart_socket   │
   │   libs/camera/*     │        │   sim/camera_socket │
   │   NAND driver       │        │   sim/nand_file     │
   │   NXP SDK           │        │   sim/sdk_stubs     │
   └─────────────────────┘        └─────────────────────┘
              │                              │
   ┌──────────▼──────────┐        ┌──────────▼──────────┐
   │  arm-none-eabi-gcc  │        │  gcc / clang (host) │
   │  → build/           │        │  → build-sim/       │
   │  → flash.sb file    │        │  → sentai_sim ELF   │
   └─────────────────────┘        └─────────────────────┘
```

The boundary that matters: **HAL API**. Wherever SentAI code today calls NXP SDK directly, we either route it through a thin HAL header (long-term) or wrap the include with a SIM-side stub that exposes the same symbols (short-term). See §6.

---

## 4. Repository organization

### Existing (unchanged)

```
coralmicro/
├── CMakeLists.txt                        # top-level (currently selects ARM toolchain)
├── cmake/
│   └── toolchain-arm-none-eabi-gcc.cmake # defines add_executable_m7, add_library_m7
├── apps/elf_loader/                      # ARM-only bootstrap
├── examples/sentai_runtime/              # SentAI app — UNCHANGED
├── libs/                                 # shared libs (mostly hardware-bound today)
└── third_party/
    ├── freertos_kernel/
    │   └── portable/
    │       ├── GCC/ARM_CM7/              # used by ARM build
    │       └── ThirdParty/GCC/Posix/     # ✓ vendored, used by SIM build ⭐
    └── micropython/                      # embed port, OS-agnostic
```

### New additions for SIM (proposed)

```
coralmicro/
├── Sim.md                                # this file
├── cmake/
│   ├── toolchain-x86-sim.cmake           # NEW: host gcc, defines add_executable_sim, add_library_sim
│   └── sim_options.cmake                 # NEW: SENTAI_SIM definitions, flags
├── sim/                                  # NEW: HAL implementations + SDK stubs for SIM
│   ├── CMakeLists.txt
│   ├── main_sim.c                        # entry point (replaces main_freertos_m7.cc)
│   ├── console_stdio.c                   # implements console_m7.h API on stdin/stdout
│   ├── nand_file.c                       # implements fx_nand_driver_initialize() over a file
│   ├── uart_socket.c                     # implements LPUART API over Unix socket (CrazySim deck)
│   ├── camera_socket.c                   # implements CSI hooks fed by socket frames (Gazebo bridge)
│   ├── tpu_libedgetpu.cc                 # implements sentai_load_model/invoke via Linux libedgetpu
│   ├── sdk_stubs/                        # minimal NXP SDK header stubs that compile on x86
│   │   ├── fsl_lpuart.h                  # subset of API SentAI uses
│   │   ├── fsl_gpio.h
│   │   ├── fsl_csi.h
│   │   ├── fsl_pxp.h
│   │   ├── fsl_edma.h
│   │   ├── fsl_clock.h
│   │   ├── MIMXRT1176_cm7.h              # registers we touch (PSRR, SRC, WDOG)
│   │   └── ...                           # ~15-25 headers, ~5-10 symbols each
│   └── README.md                         # quick start: how to build + run
```

### CI layout (new GitHub Actions workflow)

```
.github/workflows/
├── build_arm.yml                         # existing/missing — ensure ARM build passes
└── build_sim.yml                         # NEW — ensures SIM build passes + runs smoke test
```

---

## 5. Build separation strategy

### Toolchain selection at CMake configure time

```bash
# ARM (current)
cmake -B build -S .                                         # picks toolchain-arm-none-eabi-gcc.cmake by default
bash build.sh                                                # convenience wrapper

# SIM (new)
cmake -B build-sim -S . -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-x86-sim.cmake -DSENTAI_SIM=ON
cmake --build build-sim --target sentai_sim
```

### Top-level CMakeLists.txt change

Add at the top:

```cmake
option(SENTAI_SIM "Build the x86/Linux SIM target instead of ARM firmware" OFF)
if(SENTAI_SIM)
    add_compile_definitions(SENTAI_SIM=1)
    set(CORAL_MICRO_ARDUINO 0)              # disable Arduino path
    # Skip the apps/ subtree entirely (apps/elf_loader is ARM-only bootstrap).
    # Skip the examples/ tree except sentai_runtime which we will adapt.
endif()
```

### Conditional in `apps/CMakeLists.txt`

```cmake
if(NOT SENTAI_SIM)
    add_subdirectory(elf_loader)
endif()
```

### Conditional in `libs/CMakeLists.txt`

Many libraries are ARM-only (libs/usb, libs/cdc_*, libs/nxp/rt1176-sdk). Wrap each in `if(NOT SENTAI_SIM)`. Add a new `libs/sim_hal/` (or `sim/` at root) that compiles only for SIM.

### `add_executable_sim` macro

Mirror of `add_executable_m7`. Defined in `cmake/toolchain-x86-sim.cmake`. Links against:
- `libfreertos_posix` (built from `third_party/freertos_kernel/portable/ThirdParty/GCC/Posix/port.c` + `tasks.c` + `queue.c` + `list.c` + `timers.c` + `event_groups.c`)
- `libmicropython_embed` (existing target, OS-agnostic)
- `libsim_hal` (new — console, uart, nand, camera, tpu impls)
- pthread, rt, m

### Same source files, different libs

All `.c/.cc` in `examples/sentai_runtime/`, `libs/base/`, etc. compile for both targets. The differences are:
- Which `port.c` from FreeRTOS is linked
- Which HAL implementation (NXP vs sim) is linked
- Which SDK headers are visible (real vs stub directory)

CMake `target_include_directories` ordering controls this: SIM build adds `sim/sdk_stubs/` first in include path, ARM build uses `third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/`.

---

## 6. HAL strategy (the hard part)

The challenge: **today, SentAI code calls NXP SDK directly** (e.g. `LPUART_Init(LPUART6, ...)`). Two ways to handle this:

### Option H1: SDK header shim (FAST, less elegant)

For each NXP SDK header that SentAI includes, create a stub in `sim/sdk_stubs/` that:
- Re-declares the exact same types and function signatures
- Returns dummy values or routes to a thin sim implementation

Example `sim/sdk_stubs/fsl_lpuart.h`:
```c
#pragma once
#include <stdint.h>
typedef struct LPUART_Type LPUART_Type;
extern LPUART_Type* const LPUART6;          // dummy pointer
typedef struct { uint32_t baudRate_Bps; /* ... */ } lpuart_config_t;
void LPUART_GetDefaultConfig(lpuart_config_t* cfg);
int  LPUART_Init(LPUART_Type* base, const lpuart_config_t* cfg, uint32_t srcClock_Hz);
int  LPUART_WriteBlocking(LPUART_Type* base, const uint8_t* data, size_t len);
/* ... only the symbols SentAI actually uses */
```

The implementations live in `sim/uart_socket.c`:
```c
int LPUART_Init(LPUART_Type* base, const lpuart_config_t* cfg, uint32_t srcClock_Hz) {
    /* base is dummy; open Unix socket to CrazySim deck instead */
    return uart_socket_open(...);
}
int LPUART_WriteBlocking(LPUART_Type* base, const uint8_t* data, size_t len) {
    return uart_socket_send(data, len);
}
```

**Pros**: zero changes to SentAI source. Fast to bootstrap (~1 week to cover the dozen SDK headers SentAI uses).
**Cons**: stubs are brittle — every new SDK function used in SentAI requires a stub update.

### Option H2: Refactor through HAL boundary (CLEAN, more work)

Introduce `libs/base/hal_*.h` headers with platform-agnostic API:
```c
// libs/base/hal_uart.h
int  hal_uart_init(int port_id, uint32_t baudrate);
int  hal_uart_write(int port_id, const uint8_t* data, size_t len);
int  hal_uart_read(int port_id, uint8_t* buf, size_t max_len, uint32_t timeout_ms);
```

Two implementations: `libs/base/hal_uart_nxp.c` (calls real LPUART), `sim/hal_uart_socket.c` (calls socket).

SentAI code that previously called `LPUART_Init` is changed to `hal_uart_init`. **One-time refactor**, but durable.

**Pros**: clean architecture, no shim-decay risk, ARM build also benefits from clearer boundaries.
**Cons**: refactor across many files. Touches `sentai_crazy.cc`, `sentai_link.cc`, `modsentai_camera.c`, `flow_task.cc`, etc.

### Recommended: **H1 first, H2 incrementally**

Phase 1 uses H1 (header stubs) to bootstrap fast. As specific subsystems mature in SIM (camera, UART), we can refactor them through H2 to clean up. Each HAL refactor is its own PR, validated against ARM CI before merge.

### Key data point from inventory

Out of 66 source files in `examples/sentai_runtime/`, only **9 include NXP SDK directly**:
- `sentai_runtime.cc` (boot orchestration, WDOG)
- `modsentai_hal.cc` (catch-all hardware bindings)
- `modsentai_camera.c` (PXP scaling)
- `modsentai_diag.c` (some MMIO)
- `flow_task.cc`, `flow_task_m4.cc` (PXP, IMU)
- `sentai_fault.cc` (NVIC/SCB exception handlers)
- `sentai_health.cc` (WDOG)
- `detection_task.cc` (eDMA memcpy)

The remaining **57 files (87% codebase)** are pure logic + FreeRTOS API + MicroPython — they compile on x86 with only the FreeRTOS port swap. Phase 1 only needs to handle a tiny subset of these (REPL doesn't touch camera/PXP/eDMA).

---

## 7. Phase 1 deliverable: REPL on Linux, `1+1 → 2`

### Scope

- FreeRTOS scheduler boots on Linux x86_64
- MicroPython embed initializes
- REPL task reads from stdin, writes to stdout
- `>>> 1+1` returns `2`
- `>>> import sys; sys.implementation.name` returns `'micropython'`
- All `sentai.*` modules can be imported but most calls return stubs/errors (acceptable — Phase 2+ adds functionality)

### What we build

`sentai_sim` executable that contains:
- FreeRTOS POSIX port (`port.c`, `tasks.c`, `queue.c`, `list.c`, `timers.c`, `event_groups.c`, `heap_4.c`)
- MicroPython embed (existing build, no changes)
- `modsentai.c` (module registration only — most bindings stubbed)
- `sim/main_sim.c` (entry point)
- `sim/console_stdio.c` (replaces `console_m7.cc`'s USB-CDC paths with `read(STDIN_FILENO, ...)` / `write(STDOUT_FILENO, ...)`)
- `sim/sdk_stubs/*.h` (just enough stubs to compile what we touch)

### What we DON'T build in Phase 1

- ❌ NAND / FileX / LevelX (skip; `sentai.fs.*` returns errors)
- ❌ Crazy bridge (no UART socket yet; `sentai.crazy.*` returns errors)
- ❌ Camera (no Gazebo bridge; `sentai.camera.*` returns errors)
- ❌ TPU (no libedgetpu yet; `sentai.tpu.*` returns errors)
- ❌ Flow algorithm (no input frames; `sentai.flow.*` stubbed)
- ❌ HTTP server, USB CDC-NCM, IPC M4, network stack
- ❌ Multi-core M4 anything

### Concrete task breakdown for Phase 1

| Step | Effort | Validates |
|---|---|---|
| 1. Add `cmake/toolchain-x86-sim.cmake` defining `add_executable_sim` + `add_library_sim`, host gcc, `-pthread -lrt` | 0.5 day | toolchain works |
| 2. Add top-level `option(SENTAI_SIM ...)` + ifdef-guard ARM-only subdirs in `apps/CMakeLists.txt`, `libs/CMakeLists.txt` | 0.5 day | configure succeeds in both modes |
| 3. Build `libfreertos_posix` from POSIX port + portable kernel sources (no app code yet) | 0.5 day | FreeRTOS sources compile |
| 4. Tiny standalone test: `sim/test_freertos.c` creates 2 tasks that printf, scheduler runs | 0.5 day | port works |
| 5. Identify exact list of NXP SDK headers SentAI actually needs for REPL-only build (grep + iterate) | 1 day | scope known |
| 6. Write minimal `sim/sdk_stubs/` for those headers (likely <10 for REPL only) | 1-2 days | examples/sentai_runtime/ files compile on x86 |
| 7. `sim/console_stdio.c`: implement `_write` weak override + console init with stdin/stdout | 0.5 day | printf works |
| 8. `sim/main_sim.c`: init MP embed, create REPL task, start scheduler | 0.5 day | REPL prompt appears |
| 9. Strip down `examples/sentai_runtime/CMakeLists.txt` for SIM (subset of sources, fewer libs) | 1 day | links cleanly |
| 10. End-to-end: `./sentai_sim` shows `>>> ` prompt, `1+1\n` returns `2\n>>> ` | 0.5 day | DONE |

**Total Phase 1: ~7-10 working days** for one focused dev. Risk pad: +50% if SDK header sprawl is worse than the inventory suggests.

---

## 8. Future phases (deferred, mapped only)

### Phase 2: NAND backing file → FileX/LevelX functional in SIM
- `sim/nand_file.c` exposes the LevelX `LX_NAND_FLASH` driver API backed by a regular file (`/tmp/sentai_nand.bin`, sparse, sized to match real NAND geometry).
- All `sentai.fs.*` calls work end-to-end, including SAFE-MODE, fx_format, unflushed-write counter.
- **Big win**: bug from build #1225 (CHECK→vTaskSuspendAll silent brick) is now reproducible in SIL with GDB on a normal Linux ELF — much faster iteration than JTAG.
- ~1 week.

### Phase 3: UART socket → CrazySim deck integration
- **Prerequisite**: CrazySim cloned (separate from coralmicro/, per rule §2.3).  One-time setup via `bash sim/scripts/install_crazysim.sh` clones to `/home/bogdan/work/crazyflie/CrazySim/` and installs deps.  CrazySim plugin auto-detects Gazebo Harmonic (gz-sim8) — same engine as Phase 4 camera bridge.
- `sim/uart_socket.c` exposes LPUART API as a TCP socket.
- Recompile CrazySim (Bitcraze SITL fork) with our `app_sentai_bridge` deck driver, replacing its `uart2*` calls with the same socket interface.
- Test: `cflib → CrazySim → deck → socket → sentai_sim → MP REPL → response back`.
- First **`$1+1 → OK 2` over virtual radio**.
- ~1-2 weeks.

### Phase 4: Camera socket → Gazebo image bridge
- **Prerequisite**: Gazebo Harmonic installed on the host.  One-time setup via `bash sim/scripts/install_gazebo_harmonic.sh` (Ubuntu 24.04 Noble; OSRF apt repo).  Verifies with `gz sim --versions` (expect 8.x.x for Harmonic).
- `sim/camera_socket.c` exposes CSI hook callbacks (`coralmicro_csi_on_buffer_arm/done`) fed by frames received over a socket.
- Companion ROS 2 / gz-cli node subscribes to Gazebo camera topic and pushes frames.
- `sentai.flow.start()` runs phase correlation on real Gazebo imagery.
- Inject flow into CrazySim drone EKF; closed-loop hover.
- ~2 weeks (incl. Gazebo setup if not already running).
- **Drone-target agnostic from day one.**  The Gazebo bridge is a generic
  "image source"; the flow-output sink is selectable at runtime.  Phase 3
  ships the CrazySim sink (CRTP CH=1 over UART socket).  A future Phase 4b
  adds a PX4 sink (MAVLink `OPTICAL_FLOW_RAD` over a separate socket).
  SentAI SIM should be able to publish flow to ANY drone in Gazebo
  (Crazyflie, PX4 quad, multi-drone scenarios) without firmware-side
  conditionals.  The selection lives in MicroPython userland (e.g.
  `sentai.crazy.send_flow(...)` vs `sentai.mavlink.send_flow_rad(...)`),
  same MicroPython API contract on both ARM and SIM.

### Phase 5: TPU → libedgetpu Linux
- `sim/tpu_libedgetpu.cc` reimplements `sentai_load_model`, `sentai_invoke`, slot APIs over the C++ libedgetpu library.
- Optional: real Coral USB Accelerator on the dev machine, OR mocked outputs.
- Already documented at `~/.claude/.../memory/reference_host_coral_testing.md`.
- ~1 week.

### Phase 6: CI integration
- GitHub Actions matrix: build ARM + SIM on every PR.
- SIM smoke test: launch `sentai_sim`, assert REPL prompt appears, send `1+1\n`, verify `2\n` reply.
- More elaborate tests added incrementally (e.g. Phase 2 enables FS regression tests).

---

## 9. Risks and mitigations

| Risk | Probability | Impact | Mitigation |
|---|---|---|---|
| NXP SDK header sprawl: more shims needed than inventoried | Medium | +1-2 weeks Phase 1 | Phase 1 step 5 (inventory) catches it early. If unwieldy, escalate to H2 (HAL refactor) on the sprawling subsystem. |
| FreeRTOS POSIX port has subtle scheduler differences (signal-based "ISR" simulation, jitter) | Medium | Some bugs reproduce only on ARM | Document in `agent.md`. SIM is for logic + scheduling-pattern bugs, not hardware-timing bugs. |
| MicroPython embed thread safety on POSIX (atomic sections were RTOS-aware via `mp_embed_safe.c`) | Medium | REPL races with future task callbacks (e.g. crazy bridge) | Already have `mp_embed_safe.c` shim; adapt to call POSIX `pthread_mutex_lock` in SIM build. Validated when Phase 3 lands. |
| `_write` override conflict between SIM stdio and weak symbol from console_m7.cc | Low | Compile/link error | Conditional compile of `console_m7.cc` (ARM only) in `libs/base/CMakeLists.txt`. SIM build picks `console_stdio.c` instead. |
| `vTaskStartScheduler` behavior diff (POSIX port may need different setup) | Low | REPL task never runs | Phase 1 step 4 (standalone test) catches it before integrating MP. |
| Drift: ARM CI passes but someone introduces a new NXP SDK call that breaks SIM build | High over time | SIM rots | CI MUST run both targets on every PR. Reject merges if SIM fails. |
| Heap_4 vs Linux malloc inconsistency (FreeRTOS heap is separate from libc heap) | Low | OOM patterns differ | Use heap_4 in SIM too (FreeRTOS-managed heap), give it ample size (~4 MB). |
| SDRAM/DTCM linker sections are meaningless on Linux | N/A in Phase 1 | — | Sections are linker-script concepts; on Linux all globals go to `.bss/.data` normally. The `__attribute__((section(".sdram_*")))` annotations become no-ops or are ifdef'd via macro. |

---

## 10. Discipline rules for ongoing development

1. **Every PR that touches SentAI source must build BOTH targets.** Not optional.
2. **New NXP SDK calls in SentAI code are flagged.** PR reviewer asks: does this need a SIM-side stub? File the issue immediately if yes.
3. **HAL boundary is a one-way street.** Never call `LPUART_Init` directly from new code; always go through `hal_uart_*` once that layer exists.
4. **SIM isn't allowed to fork SentAI logic.** If the algorithm needs to behave differently in SIM, the difference goes into HAL impl, not into the algo.
5. **Document in `agent.md` §SIM-section** any divergence in behavior between ARM and SIM (e.g. timing jitter, missing hardware feature). Future maintainers must know what SIM does NOT cover.
6. **No `/tmp` for SIM artifacts.** Use `build-sim/` and `examples/sentai_runtime/experiments/sNNN_<name>/` per the existing convention.
7. **All blocking syscalls in SIM HAL must EINTR-retry.** The POSIX port
   delivers SIGALRM at `configTICK_RATE_HZ` (1 kHz) to drive the
   scheduler tick.  Any `read()`/`recv()`/`accept()`/`select()` on stdin,
   sockets, or pipes WILL be interrupted ~1000 times per second.  Standard
   buffered IO (`fgets`, `fread`, `scanf`) does NOT retry on EINTR — they
   return NULL/0 and set feof, which the caller sees as spurious EOF.
   Always wrap raw syscalls in `do { ... } while (n == -1 && errno == EINTR);`
   See `sim/main_sim.c::sim_read_line()` as the canonical example.
   Caught the hard way 2026-05-10: REPL exited instantly after the first
   tick because `fgets()` saw EOF.  Fixed in commit `e00cdf41`.

---

## 11. References

- FreeRTOS POSIX port docs: https://www.freertos.org/FreeRTOS-simulator-for-Linux.html
- POSIX port source (vendored): `third_party/freertos_kernel/portable/ThirdParty/GCC/Posix/port.c`
- MicroPython embed port: `third_party/micropython/ports/embed/`
- NXP RT1176 SDK headers (ARM-only): `third_party/nxp/rt1176-sdk/devices/MIMXRT1176/drivers/`
- CrazySim (drone-side SITL with Gazebo): https://github.com/llanesc/crazysim
- Bitcraze SITL official docs: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/sitl/
- libedgetpu Linux: https://github.com/google-coral/libedgetpu

---

## 12. Status log

Update as phases land.

| Date | Phase | Status | Build # | Notes |
|---|---|---|---|---|
| 2026-05-10 | 0 (planning) | Sim.md drafted | n/a | Architecture decided: POSIX port + HAL shim. Phase 1 next. |
| 2026-05-10 | 1 step 1-4 | DONE — kernel-only smoke test passes | n/a | `cmake/toolchain-x86-sim.cmake`, `sim/{CMakeLists.txt,FreeRTOSConfig.h,test_freertos.c}`, top-level `option(SENTAI_SIM)` + ARM-tree guard. `./build-sim/sim/sentai_sim_kernel_only` runs 2 tasks for 5s, tick rate confirmed 1 kHz, exit 0. ARM build #1226 unaffected. Notes: vendored POSIX port (V10.4.1, Cambridge Consultants) requires `configENABLE_BACKWARD_COMPATIBILITY=1` because it uses pre-V8 type names (`portTickType`, `pdTASK_CODE`). Static-alloc requires `vApplicationGetIdleTaskMemory`/`vApplicationGetTimerTaskMemory` callbacks. |
| 2026-05-10 | 1 step 5-10 | DONE — REPL `1+1 → 2` works on Linux | n/a | `sim/{main_sim.c, mpconfigport.h}` + `sentai_sim` target. Uses MP embed sources from firmware `examples/sentai_runtime/micropython_embed/` directly (same QSTR table). Stubs in main_sim.c: `mp_embed_enter/exit_critical` (no-op single-task), `mp_module_sentai` (empty module), `mp_lexer_new_from_file`/`mp_import_stat` (return ENOENT/NO_EXIST), `sentai_help_builtin_text` (placeholder). Linker: libm AFTER libmicropython for math symbols. Run: `./build-sim/sim/sentai_sim`, prompt `>>> ` interactive. ARM build #1226 unaffected. |
| 2026-05-10 | 1 fixup | DONE — REPL fgets→read+EINTR fix (commit `e00cdf41`) | SIM #1 | POSIX port SIGALRM @ 1 kHz interrupted blocking stdin read; `fgets` saw spurious EOF after first tick → REPL exited instantly.  Fix: `sim_read_line()` with `do { read(); } while (errno==EINTR);` per char.  Discipline rule #7 added to Sim.md §10 to prevent recurrence in future HAL impls (UART socket, camera socket). |
| 2026-05-10 | 1.5 | DONE — sentai.* bindings (no hardware) | SIM #1 | New `sim/modsentai_sim.c` registers real `mp_module_sentai`: `version()`, `verbose([on])`, `io.led_on/off()`, `rtos.sleep_ms(ms)` (real vTaskDelay), `diag.dmesg()`, `sys.reset()`, `fs.{write,append,read,read_str,exists,size,ls,mkdir,remove,sync}` (Phase 2 LIGHT — Linux file backing under `./sentai_sim_root/`, override via `SENTAI_SIM_ROOT` env), `camera.{init,frame_count,grabbed_id,select}` (Phase 1.5 stubs; Phase 4 will wire real Gazebo socket — backend identifier "virt_gazebo", NO ov5640 in SIM). REPL auto-`import sentai` at boot.  SIM build counter independent from ARM (sim/build_version.{h,txt}); banner prints "SentAI SIM build #N".  Help text updated.  ARM build untouched. |
| 2026-05-10 | 1.5 fixup | DONE — virtual FS root moved to build-sim/ | SIM #2+ | Default SIM_FS_ROOT was `./sentai_sim_root` in cwd → polluted repo root.  Now CMake compile-time-defines `SENTAI_SIM_FS_ROOT_DEFAULT="${CMAKE_BINARY_DIR}/sentai_fs_root"` so the FS lives at `<repo>/build-sim/sentai_fs_root/` — isolated per build, auto-gitignored, co-located with binary.  `SENTAI_SIM_ROOT` env still wins for persistent/explicit paths. |
| 2026-05-10 | 3 prep | DONE — CrazySim built + tested on Gazebo Harmonic | n/a | Gazebo Harmonic 8.11.0 installed via `sim/scripts/install_gazebo_harmonic.sh`.  CrazySim cloned to `/home/bogdan/work/crazyflie/CrazySim/` via `sim/scripts/install_crazysim.sh` (NOT into coralmicro/ per rule §2.3).  Docs say "install Garden" but the plugin CMake auto-detects gz-sim8 (Harmonic) and uses gz-msgs10/gz-transport13 — works.  SITL build via `cmake -B sitl_make/build -S sitl_make && cmake --build sitl_make/build` (NOT `make sitl_make`).  Ubuntu 24.04 quirk: `python` doesn't exist (only `python3`); fix with `apt install python-is-python3` OR `ln -sfv /usr/bin/python3 ~/.local/bin/python`.  Build outputs: `cf2` ELF (Crazyflie firmware as Linux binary, FreeRTOS POSIX port — same kernel pattern as `sentai_sim`!) + `libgz_crazysim_plugin.so`.  Launch via `bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh` — opens Gazebo GUI, spawns drone at (0,0,0.5), exposes UDP 19850 for cflib (`udp://127.0.0.1:19850`).  Topics published: `/cf_0/{imu,baro,odom}`, `/cf_0/command/motor_speed`.  cflib connect verified — `Crazyflie.open_link()` succeeds; `SyncCrazyflie` needs longer than 4s for TOC download (use callback-based connect for fast tests).  Phase 3 (UART socket → CrazySim) can now build on this foundation. |
| 2026-05-10 | 3 known-issue | OPEN — cf2 sensor calibration loop never completes on Harmonic | n/a | After all best practices applied (`stdbuf -oL`, `-v 4`, env vars set), CrazySim cf2 reaches `STAB: Wait for sensor calibration...` and stays there forever.  cflib `Crazyflie.open_link()` succeeds (state=1 = link established) but TOC never downloads because cf2 is stuck pre-task-loop.  Plugin loads (`crazysim_plugin.cpp:43 Configure() called.`), binds 19850/19950 OK, IMU/baro Gazebo topics flow.  Suspected root cause: plugin's `sendCfFirmware(IMU/baro packets)` either uses wrong addr (handshake remaddr_rcv vs remaddr) or wrong packet length/header — cf2 doesn't see the IMU samples the plugin enqueues, so `gyroBiasFound` never flips true.  Investigation requires modifying plugin C++ + rebuild + tcpdump on loopback (sudo).  Defer to a focused session; for now Phase 3 plan is "verified Harmonic ABI compatibility for plugin + cf2 binary, but operational integration needs CrazySim upstream fix or local patch."  Workarounds: (a) downgrade to Garden when CrazySim is needed (Ubuntu 24.04 manual build), (b) bypass CrazySim entirely and use Gazebo's own multicopter_velocity_control plugin (drone flies but no real Crazyflie EKF). |
| 2026-05-10 | 3 best-practices | DONE — captured during cflib TOC debug session | n/a | **Hard-won CrazySim/Gazebo Harmonic best practices.  Read these before any future debug session.**  (1) **`stdbuf -oL` is mandatory for cf2** — `cf2`'s stdout is block-buffered when redirected to a file (4 KB).  Default `sitl_singleagent.sh` does `cf2 ... > out.log 2> error.log &` and the logs stay EMPTY for minutes.  Wrap with `stdbuf -oL -eL cf2 ...` to flush per-line and see boot progress (`SOCKET_LINK: Waiting for connection with gazebo`, `Connection established`, `SYS: Software-in-the-Loop Simulator is up and running!`).  (2) **Always launch `gz sim` with `-v 4` (debug) during bringup, not the default `-v 3`** — plugin-load failures (`Failed to load system plugin [gz_crazysim_plugin] : Could not find shared library`) are logged ONLY at `-v 4`.  At `-v 3` the world boots silently with no plugin and EVERYTHING downstream (cflib TOC, motor commands, telemetry) silently times out.  (3) **`GZ_SIM_SYSTEM_PLUGIN_PATH`, `GZ_SIM_RESOURCE_PATH`, `LD_LIBRARY_PATH` MUST be set in the shell that launches `gz sim`** — `setup_gz.bash` sets them, but only inside the script's process tree.  If you run `gz sim ...` ad-hoc in another shell, the plugin is silently missing and the drone's `/cf_0/imu` topic exists but with NO subscriber on the cf2 side.  (4) **Plugin <-> cf2 handshake is `0xF3`** — cf2 SOCKET_LINK sends `0xF3` (1 byte, header only, size=0) repeatedly until plugin echos `0xF3` back.  Plugin learns cf2's ephemeral source addr from `recvfrom`'s remaddr.  cf2 then continues to system init.  Sequence visible at cf2 stdout (with stdbuf): `Create socket succeed → Binding succeed → Waiting for connection with gazebo → Connection established with gazebo → SYS: Software-in-the-Loop Simulator is up and running!`  (5) **cflib UdpDriver handshake is `\xff\x01\x01\x01`** — plugin doesn't ack this, just learns cflib's addr from recvfrom.  Subsequent CRTP packets are bidirectional. |
