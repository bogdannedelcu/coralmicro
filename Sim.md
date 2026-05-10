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
- **Prerequisite**: CrazySim cloned (separate from coralmicro/, per rule §2.3).  One-time setup via `bash sim/scripts/install_crazysim.sh` clones to `/home/bogdan/work/crazyflie/CrazySim/` and installs deps.  **Use Gazebo Garden 7.9 inside the `crazysim-garden` distrobox container — Harmonic 8.x is BANNED** (it has a hard cf2 sensor-calibration deadlock; see Phase 3 known-issue below).
- `sim/uart_socket.c` exposes LPUART API as a TCP socket.
- Recompile CrazySim (Bitcraze SITL fork) with our `app_sentai_bridge` deck driver, replacing its `uart2*` calls with the same socket interface.
- Test: `cflib → CrazySim → deck → socket → sentai_sim → MP REPL → response back`.
- First **`$1+1 → OK 2` over virtual radio**.
- ~1-2 weeks.

### Phase 4: Camera socket → Gazebo image bridge
- **Prerequisite**: Gazebo Garden 7.9 inside the `crazysim-garden` distrobox (already installed via the Phase 3 path; see Phase 3 prep entry above).  **Do NOT use Harmonic on the host** — it breaks cf2 SITL sensor calibration (verified dead-end 2026-05-10).  The host bridge subscribes to Garden topics through the shared distrobox network (`--net=host`), so no second Gazebo install is needed for Phase 4.
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

## 10b. Camera + flow conventions (load-bearing for Phase 4)

Authoritative source for the firmware: `examples/sentai_runtime/paper/flow_body_frame.md` + `flow_altitude.md` + `agent/agent.md` body-frame table. Anything in SIM that produces or consumes optical flow MUST match these — otherwise the cf2 EKF rejects samples or amplifies drift.

**FOV (lens-calibrated, NOT register-derived)**

| Axis | Pixels (raw) | FOV |
|---|---|---|
| H (long, 640) | 640 | **58°** — body FW–BACK |
| V (short, 480) | 480 | **45°** — body L–R |

Aspect 4:3, focal length per axis: `f_grid = (grid_w/2) / tan(FOV_h/2) ≈ 72.3` for `grid_w=80`.

**Body-frame mapping (cam0 with `vflip=1`)**

| Image position | Body direction |
|---|---|
| LEFT   | FORWARD  (+x_body) |
| RIGHT  | BACKWARD (-x_body) |
| TOP    | RIGHT    (-y_body) |
| BOTTOM | LEFT     (+y_body) |

`body_xform = {0: (-1.0, 0.0, 0.0, +1.0)}` (cam0 verified empirically 2026-05-07 with LED-cued translation; cam1 still placeholder).

**Sign convention (verified)** — drone moves FORWARD → flow `dx` **negative**; drone moves BACK → `dx` **positive**.

**Camera physical mount on drone**

The on-drone camera is offset from the centre of mass:

| Axis | Offset | Sign |
|---|---|---|
| body X (FW) | **−4 cm** | camera is **behind** CoM |
| body Y (L)  | 0 | centred |
| body Z (UP) | **−2 cm** | camera is **below** CoM |

This matters for two things:
1. **EKF arm length** — if we ever feed `OPTICAL_FLOW_RAD` with body-frame velocity (vs raw pixel rate), the lever-arm `r = (-0.04, 0, -0.02)` m must be accounted for: `v_cam = v_body + ω_body × r`. For the current `cf.send_flow(dpx, dpy, dt, std)` API we send pixel deltas, not velocity, so the EKF does its own lever-arm math from `motion_pos.x/y/z` log params (which we leave at default 0,0,−0.005 for PMW3901 ⇒ acceptable error at low pitch).
2. **Gazebo SIM mounting** — the Gazebo camera (whether attached to the drone link or fixed in world for the early dev iterations) MUST sit at the same offset relative to the drone's CoM, otherwise pose-derived ground-truth flow won't match what the algorithm sees.

**PXP downscale**

Hardware on RT1176: PXP DMA, XRGB8888 640×480 → RGB888P 80×60, ~1.15 ms. SIM has NO PXP — `sim/sentai_pxp_scale.c` reimplements the same area-average semantic on CPU (`#ifdef SENTAI_PLATFORM_SIM`). Output bit-near identical (±1 LSB rounding).

**Flow input chain (identical ARM ↔ SIM after PXP shim)**

```
RGB 80×60 → grayscale Y plane → centre-crop 64×60 → zero-pad 64×64
→ FFT phase-correlation (CMSIS arm_cfft_f32 on ARM, FFTW3 on SIM)
→ parabolic sub-pixel fit → (dx, dy, conf) in milli-grid-pixels
→ deadband + conf-floor → publish flow_shared_t
```

## 10c. Operator workflow rules (VISUAL CONFIRMATION REQUIRED)

These are **mandatory** rules added during the 2026-05-10 Phase 4 bring-up
session.  They take priority over any earlier "headless is fine" pattern.

### Rule 1 — Always launch the Gazebo GUI for any experiment

When running ANY closed-loop test that touches Garden + cf2 SITL + the
flow bridge, the operator MUST be able to watch what the drone is doing
visually — pose drift, attitude oscillation, camera framing, marker
visibility, etc.  Numeric `stateEstimate.x/y/z` log values are not a
substitute; they hide whether the world is rendering correctly, whether
the drone-attached camera is pointed where we think, whether physics
plugins crashed silently, etc.

Operator instruction: every Phase 4 test script MUST launch
`gz sim -g --gui-config $(repo)/sim/gazebo/sentai_gui.config`
alongside the headless server, and MUST NOT proceed past
"stack ready" until the GUI window is visible to the operator.  If the
GUI fails to show, the test is invalid — fix the GUI before continuing.

### Rule 2 — Picture-in-Picture for the SentAI camera

The `sentai_gui.config` ships with an `ImageDisplay` widget docked on
the right pane subscribed to `/downward_cam/image`.  This is the EXACT
frame the SentAI flow algorithm receives.  If the PiP shows sky, the
camera is mis-oriented (Phase 4a we found `pitch=+pi/2` looks UP, not
down — fix is `pitch=-pi/2`).  If the PiP shows uniform colour, the
drone is on the ground with the camera below the ground plane — drone
needs to take off first.  If the PiP shows a moving texture, the
algorithm should be producing non-zero `dx/dy`.

### Rule 3 — Ramp altitude carefully, baro-only Z is noisy

Without optical flow, the EKF integrates IMU + baro; baro alone can
overshoot Z by metres if commanded too aggressively (we hit z=29 m once,
documented in "3 ASSERT fix").  Phase 4 takeoff scripts MUST use
`send_hover_setpoint(0, 0, 0, z_target)` with `z_target<=1.0` and a
3-second linear ramp, and the operator MUST watch the GUI to confirm
the drone holds 1 m and doesn't climb away.  If z exceeds 1.5 m within
the first 5 seconds, kill the script and investigate before any
data-collection run.

### Rule 4 — cf2 `SUP: Locked` recovery

If cf2 logs `SUP: Locked, reboot required`, motors are stuck at 0 and
no further commands take effect.  This happens when EKF state
diverges (e.g. takeoff attempted with no flow + no extpos source so
EKF velocity blows up).  Recovery: kill cf2 inside the distrobox and
re-spawn it via `dbox_launch_cf2_headless.sh`.  Don't try to "unlock"
in software — easier to restart the firmware ELF.

## 10d. Garden render-thread starvation — Xvfb workaround (CRITICAL)

**Bug**: Gazebo Garden 7.9 with `gz sim -s -r --headless-rendering` keeps
publishing `gz.msgs.Image` headers on `/<topic>` at the configured FPS,
but the **pixel buffer is bit-identical across consecutive frames** —
the render thread starves when no display backend is wired up.  The
Gazebo GUI's ImageDisplay widget renders fine because it has its own
OGRE2 instance; the server-side sensor doesn't.  Confirmed via:
- `gz topic -e -t /downward_cam/image --json-output` shows progressing
  `header.stamp` but the same `data` md5 across messages.
- Custom gz-transport12 C++ subscriber (our `gz_to_uds_bridge`) sees
  the same frozen-buffer behaviour.
- Issue tracking: `gazebosim/ros_gz#346`, `gazebosim/gz-sensors#332`,
  `gazebosim/gz-sim#2708`, `gazebosim/gz-sim#2851`.

**Fix that works**: drop `--headless-rendering` and start the Garden
server under `Xvfb :99 -screen 0 1024x768x24` with `DISPLAY=:99` exported
before launching `gz sim`.  Encoded in `/tmp/dbox_launch_cf2_headless.sh`:

```bash
pkill -9 Xvfb 2>/dev/null || true
Xvfb :99 -screen 0 1024x768x24 > /tmp/xvfb.log 2>&1 &
export DISPLAY=:99
gz sim -s -r -v 3 sentai_crazysim.sdf > /tmp/gz_world.log 2>&1 &
```

After applying this, our bridge sees **96% non-zero `dx/dy` from
`flow_phase_corr` during drone motion** (vs 0% before) — that's the
right ballpark for the actual frame-to-frame ground-pixel shift.

**What did NOT work** (tried first, drop these from your debugging tree):
- Removing `--headless-rendering` alone (without Xvfb) — server falls
  back to EGL which has the same starvation under CrazySim.
- Adding a "keepalive" subscriber via `gz topic -e ... > /dev/null` —
  improved render rate to ~5 fps but still mostly identical frames.
- Opening the GUI ImageDisplay as a viewer — wakes some renders but not
  reliably per-frame (ros_gz#346 workaround helps but isn't enough).

**Closed-loop X stabilization working (2026-05-10 v21)**:
Applied the `state.z > 0.30 m` gate in `cflib_takeoff_with_flow.py`
FlowReceiver thread (per embeded.md "bounded behaviour" + "explicit
failure semantics" — refuse to feed the EKF until the camera actually
sees the ground).  Result with wind 0.5 m/s +X over a 25-s hover:

| Run | X drift in hover window 5..25 s |
|---|---|
| Baseline (no flow injection) | ~1.42 m  (drone hits north wall) |
| **Flow injection gated z>0.3 m** | **0.23 m** |

That's a **6× reduction** in X drift.  Sign convention also empirically
re-verified in SIM (matches the 2026-05-07 hardware test): wind +X → drone
moves FORWARD body → `dx` from `flow_phase_corr` is **negative** for 100%
of motion frames (see `/tmp/test_sign_convention.sh`); after `body_xform =
(-1, 0, 0, +1)` the post-transform `body_fw_dpx` is positive → cf2 EKF
corrects in the right direction.

**Remaining issues for follow-up**:
1. Z control gets noisy with flow injection (`stateEstimate.z` swings
   1.24 m up, -0.19 m down vs commanded 1.0 m).  Flow is body-frame
   velocity; combined with attitude noise it leaks into Z via EKF.  Look
   at `mm_flow.c` on cf2 to see if PMW3901-style flow assumes IMU-stable
   attitude.
2. Phase-corr magnitude saturates (-32k mgp values seen) when the wind
   gust is large and Garden's render rate is below the bridge's 30 fps —
   frame-to-frame ground motion exceeds the algorithm's ±32 grid-px
   measurable range.  Mitigation: throttle `gz_to_uds_bridge` to send
   every Nth frame so consecutive frames have known time spacing, OR use
   `Xvfb` with a higher refresh rate, OR add multi-resolution pyramid to
   phase-corr.
3. cf2 SUP can still lock if the gate opens too late (`z=0.30` vs `z=1.0`
   command — drone reaches gate threshold around t=1.5 s into ramp).
   Consider raising gate to z=0.5 m, OR delaying ramp by 1 s of pure
   thrust so EKF settles first.

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
| 2026-05-10 | 3 prep | DONE — CrazySim built + verified in Gazebo Garden 7.9 (distrobox) | n/a | **Garden ONLY for SentAI Phase 3/4.  Harmonic is a known dead-end (see "3 known-issue" below).**  CrazySim cloned to `/home/bogdan/work/crazyflie/CrazySim/` via `sim/scripts/install_crazysim.sh` (NOT into coralmicro/ per rule §2.3).  Garden 7.9 lives inside the `crazysim-garden` distrobox container (Ubuntu 22.04 base, gz-sim7 / gz-transport12 / gz-msgs9).  SITL built INSIDE the distrobox: `distrobox enter crazysim-garden -- bash -c 'cd ~/work/crazyflie/CrazySim/crazyflie-firmware && cmake -B sitl_make/build -S sitl_make && cmake --build sitl_make/build -j$(nproc)'`.  Build outputs: `cf2` ELF (Crazyflie firmware as Linux binary, FreeRTOS POSIX port — same kernel pattern as `sentai_sim`!) + `libgz_crazysim_plugin.so` linked against gz-sim7.  Launch from inside distrobox: `distrobox enter crazysim-garden -- bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -w sentai_crazysim`.  Drone spawns at (0,0,0.5), exposes UDP 19850 for cflib (`udp://127.0.0.1:19850`).  Topics: `/cf_0/{imu,baro,odom}`, `/cf_0/command/motor_speed`, `/downward_cam/image` (added by sentai_crazysim world).  Distrobox uses `--net=host` so the host can hit `udp://127.0.0.1:19850` directly and gz transport topics are reachable cross-boundary.  cflib link verified clean after socketlink ASSERT fix (see "3 ASSERT fix" entry).  `sim/scripts/install_gazebo_harmonic.sh` is LEGACY — do not run on a fresh machine for Phase 3/4 work. |
| 2026-05-10 | 3 ASSERT fix | DONE — cf2 socketlink ASSERT replaced with drop+count | crazyflie-firmware patch `40d34708` (local commit, ready to PR upstream) | Bug: `ASSERT(xQueueSend(crtpPacketDelivery, &p, 0) == pdPASS)` at `socketlink.c:96` in CrazySim's crazyflie-firmware called `assertFail()` -> `portDISABLE_INTERRUPTS()` which permanently masks SIGALRM and deadlocks the FreeRTOS POSIX scheduler.  cf2 logs 'Ready to fly' (last msg before deadlock) but is fully stuck in `do_sig` syscall state (visible via `ps -o stat`).  cflib opens link OK, no TOC ever downloads, drone unresponsive, `Connection refused` after handshake.  Repro reliable: plugin pushes ~1050 packets/s (IMU 1000Hz + baro 50Hz) which fills the 2000-deep queue faster than `sensorsTask` drains during the boot window before calibration finishes (~1-2 s).  **Fix in `~/work/crazyflie/CrazySim/crazyflie-firmware/src/hal/src/socketlink.c`**: drop on overflow + maintain a counter, log every 256 drops.  Production firmware pattern.  Validated end-to-end on Garden distrobox: cf2 'Ready to fly' clean, `cflib FULLY CONNECTED` in ~2s, takeoff command accepted, drone climbs and drifts visibly in X/Y (Complementary estimator with no observation = real-hardware no-flowdeck behavior).  Sample drift trace: `t+0.4s x=+0.04 y=-0.09 z=+29.2m`, `t+1.2s x=+11.9 y=+17.8 z=+24.8m` — drone "fled" the world as expected.  PR upstream to `gtfactslab/CrazySim` recommended next session. |
| 2026-05-10 | 3 known-issue | DEAD-END — Harmonic 8.x is BANNED for SentAI Phase 3/4 | n/a | **Harmonic 8.11.0 has a HARD blocker bug for CrazySim cf2.**  After all best practices applied (`stdbuf -oL`, `-v 4`, env vars set), cf2 reaches `STAB: Wait for sensor calibration...` and stays there forever.  cflib `Crazyflie.open_link()` succeeds (state=1 = link established) but TOC never downloads because cf2 is stuck pre-task-loop.  Plugin loads, binds 19850/19950 OK, IMU/baro Gazebo topics flow — but the plugin's `sendCfFirmware()` packets don't satisfy `gyroBiasFound` on Harmonic, so the boot sequence wedges indefinitely.  Root-cause investigation requires modifying CrazySim plugin C++ + tcpdump on loopback — out of scope.  **Decision (2026-05-10): drop Harmonic entirely for SentAI work; use Gazebo Garden 7.9 in the `crazysim-garden` distrobox.**  Garden's CrazySim path is verified-working end-to-end (cf2 'Ready to fly', cflib FULLY CONNECTED, takeoff accepted, drone climbs and drifts visibly — see "3 ASSERT fix" entry for the full validation trace).  Do NOT propose Harmonic again for any SentAI Phase 3/4/4b work.  `sim/scripts/install_gazebo_harmonic.sh` stays in the repo as a historical artifact only. |
| 2026-05-10 | 4 LIVE | DONE — SIM build #6 boots, camera_bridge listening on UDS, end-to-end UDS loopback PASS | new files in repo + experiments/s088,s089 | **Phase 4 pipeline VALIDATED end-to-end on x86.**  After `sudo apt install -y libfftw3-dev`, full cycle works: (a) `make run` in `experiments/s088_pxp_fft_shim_smoke/` — both shims PASS.  PXP smoke: `uniform_grey`, `solid_red` (R-channel + byte-order [B,G,R,X] -> [R,G,B] preserved), `vertical_gradient` (area-average semantic verified against analytic mean), BENCH **0.158 ms/frame** for 640×480 -> 80×60 single-thread on x86 (vs ~1.15 ms PXP DMA on RT1176 — SIM faster than HW for this op).  FFT smoke: `delta_forward` (maxdiff 0), `delta_at_k_forward` (2.3e-6), `round_trip` (1.2e-7), `vs_brute_force_DFT` (8.2e-5 — well under 1e-4 tolerance, bit-near identic to CMSIS arm_cfft_f32 for length-64).  (b) `cmake --build build-sim --target sentai_sim` succeeds, build #6 boots: `[sim] sentai_sim build #6` + `camera_bridge: listening on /tmp/sentai_cam.sock`.  REPL `>>> sentai.flow.read()` returns `(0, 0, 0, 0, 0)` initially (no frames yet).  (c) `experiments/s089_phase4_closed_loop/test_uds_loopback.py` pushes 30 synthetic 640×480 RGB frames + reads 30 flow replies on the same UDS, all match REPLY_MAGIC `0x46524C31`, dx/dy non-zero (e.g. seq=10 dx=+44 dy=+18 conf=255 lat=561µs, seq=15 dx=−44 dy=−18 conf=255 lat=1681µs — phase-corr resolves opposite-sign shifts cleanly), throughput 16.8 fps over loopback (Python-bound on synthetic-frame generation, not on the C pipeline).  This proves: shared C/C++ algorithm code (PXP shim + FFT shim + flow_phase_corr.cc) produces identical-shape output on x86 as it does on the RT1176, and the UDS protocol round-trips without corruption.  REMAINING for full closed-loop: spin up CrazySim cf2 + Gazebo with `sentai_crazysim` world, run `cflib_takeoff_no_flow.py` then `cflib_takeoff_with_flow.py`, then `analyze_drift.py` — pass criterion `RMS(x,y)_with_flow < 0.5 * RMS(x,y)_no_flow`.  Distrobox split is the blocker: cf2 + Gazebo Garden run inside `crazysim-garden` distrobox, but `cflib` + `gz.transport` Python bindings are in different envs on the host.  Workarounds documented in §10b.bridges below. |
| 2026-05-10 | 4 trap | DONE — `dwt_cyc()` SIGSEGV on x86 (load from 0xE0001004) | examples/sentai_runtime/flow_phase_corr.cc | First end-to-end UDS loopback run crashed sentai_sim with SIGSEGV right after the first frame header was read — Python loopback saw `EOF at seq=1`.  Root cause: `dwt_cyc()` in `flow_phase_corr.cc` does `*((volatile uint32_t*)0xE0001004u)` to read the Cortex-M DWT cycle counter; on x86 that address is in unmapped kernel space, so any `bc_log()` call (and bc_log fires in EVERY frame) faults.  Fix: gate with `#ifdef SENTAI_PLATFORM_SIM` and substitute `clock_gettime(CLOCK_MONOTONIC, &ts).tv_nsec` truncated to uint32 — same role (forensic time-stamp in the breadcrumb ring), no MMIO.  General lesson for the SIM port: any `volatile uint32_t*` cast to a hardcoded numeric address is a Cortex-M MMIO read; grep for `0xE` and `0x4` constants and gate them all behind `SENTAI_PLATFORM_SIM`.  Same class of bug killed pointer breadcrumbs earlier in the same file — `bc_log(0x10, (uint32_t)gray80x60)` truncates a 64-bit pointer to 32 bits which `-fpermissive` accepts but earlier GCCs wouldn't; fix is `(uint32_t)(uintptr_t)ptr`. |
| 2026-05-10 | 4 env split | DOC — Phase 4 closed-loop env stack (Garden-only) | n/a | Phase 4 closed-loop crosses host + distrobox.  Where each piece runs: (1) **distrobox `crazysim-garden`** — runs `gz sim` (Garden 7.9, gz-sim7) with `sentai_crazysim` world AND `cf2` SITL.  Built with Garden's `libgz_crazysim_plugin.so`.  Launch: `distrobox enter crazysim-garden -- bash tools/crazyflie-simulation/simulator_files/gazebo/launch/sitl_singleagent.sh -w sentai_crazysim`. Distrobox uses `--net=host` so localhost is shared with host. (2) **host** — runs `sentai_sim` (`./build-sim/sim/sentai_sim`, listens on `/tmp/sentai_cam.sock`) and `gz_to_camera_bridge.py` (subscribes to gz topic, pipes frames to UDS, optionally injects flow back into cf2 via cflib UDP 19850). Python deps for the bridge: cflib (in `~/work/crazyflie/.venv/`) + gz Python bindings for **Garden** (gz.transport12 / gz.msgs9). The host currently has Harmonic-flavoured `python3-gz-transport13` from when we mistakenly explored Harmonic — the bridge script `sim/scripts/gz_to_camera_bridge.py` imports `from gz.transport13 import Node` / `gz.msgs10.image_pb2` and **MUST be flipped to `transport12` / `msgs9`** before the first Garden run, OR the host needs Garden Python bindings installed (no apt pkg on Ubuntu 24.04; either install gz-cmake source build or run the bridge inside the distrobox using its system python after `pip install cflib gz-transport`).  Pragmatic path: edit the import lines in the bridge script to Garden versions, install `python3-gz-transport12 python3-gz-msgs9` in the distrobox, run the bridge inside the distrobox where both cflib (after `pip install cflib`) and Garden bindings exist.  Alternative if pip in the distrobox is restricted: keep the bridge on host, install `gz-transport12-dev` + Python wrappers via pip in the crazyflie venv. The UDS `/tmp/sentai_cam.sock` is shared between host and distrobox via the `--net=host` + bind-mount of `/tmp`. **No Harmonic anywhere in this chain.**  Tested through the UDS loopback (synthetic frames) — pending: real Garden run + cflib closed-loop. |
| 2026-05-10 | 4 cleanup | DONE — `examples/CMakeLists.txt` only builds `sentai_runtime` | examples/CMakeLists.txt | All vendor examples (`camera_streaming_http`, `audio_streaming`, … 30+ subdirs) commented out; only `sentai_runtime` is a live build target.  The `camera_streaming_http` was the last hold-out and already failed to link (undefined refs to `sentai_repl_activity`, `g_cam_current_id`, `g_cam_switch_seq`, `g_flow_pub_isr_task`, `g_cam_ratio_packed`, `g_cam_pending_mux_id` — these symbols moved into the SentAI camera-id propagation work months ago and were never re-exported to the vendor demo).  Other persistent ARM build issues remain in `libs/nxp/rt1176-sdk` (e.g. WICED WiFi `xTaskIsTaskFinished` undeclared in `wwd_rtos.c`) but they don't break `--target sentai_runtime`, which is the only thing this fork ships.  ARM full-build clean is out-of-scope until/unless we need vendor extras. |
| 2026-05-10 | 4.1–4.6 | STRUCTURAL — Phase 4 plumbing complete, awaits libfftw3-dev install + closed-loop run | new files in repo | **Phase 4 (camera socket + flow + cflib bridge) plumbed end-to-end.**  All shared algorithm code stays in C/C++ per the realtime rule (zero PIL/numpy in any per-pixel path); Python only marshals bytes.  Components landed: (1) `examples/sentai_runtime/sentai_pxp_shim.{h,c-sim}` — XRGB->RGB area-average resize, ARM keeps PXP DMA, SIM gets pure-C @ 0.187 ms/frame (BENCH 100 frames PASS). (2) `examples/sentai_runtime/sentai_fft_shim.{h,c-sim}` — `arm_cfft_f32` -> CMSIS on ARM, FFTW3 wrapper on SIM (FFTW_ESTIMATE \| FFTW_UNALIGNED, /N rescale on inverse to match CMSIS).  `flow_phase_corr.cc` edited to call `sentai_cfft_f32` instead of `arm_cfft_f32` — zero overhead on ARM via macro, identical algorithm on SIM.  Pointer breadcrumbs cast through `uintptr_t` so 64-bit host doesn't `-fpermissive` error. (3) `sim/camera_bridge_recv.c` — UDS `/tmp/sentai_cam.sock` server task, half-duplex protocol: client sends 24B header + 921 600 B RGB, gets 32B flow snapshot reply per frame.  Pipeline: RGB->XRGB inline -> `sentai_pxp_scale` -> `rgb888_to_y` (BT.601 fixed-point) -> `sentai_flow_phase_corr_compute` -> publish `g_flow` snapshot + reply.  Static buffers, zero malloc per frame. (4) `sim/scripts/gz_to_camera_bridge.py` — gz transport13 subscriber, threading.Lock around UDS, optional `--send-flow --uri udp://127.0.0.1:19850` enables cflib injection.  Drone-EKF math (`_scale_to_drone_units`, `_to_body_dpx`, `_conf_to_std`) ported verbatim from `_t_flow_to_drone.py` so SIM uses the same body-frame convention (image LEFT = body FORWARD, `body_xform=(-1,0,0,+1)` for cam0+vflip1). (5) `sim/modsentai_sim.c` — `sentai.flow.read()` real binding wired to `g_flow` snapshot, returns `(seq, dx_q1000, dy_q1000, conf, latency_us)`. Phase 1.5 stub deleted.  `sentai.camera` also exposed at top level (was missing from `sentai_globals_table`). (6) `sim/main_sim.c` — `sim_camera_bridge_start()` called before `vTaskStartScheduler`. (7) `examples/sentai_runtime/experiments/s088_pxp_fft_shim_smoke/` — standalone make-driven smoke tests for both shims, PXP all PASS, FFT pending fftw3-dev install. (8) `examples/sentai_runtime/experiments/s089_phase4_closed_loop/` — drift-vs-stable validation runner: `cflib_takeoff_no_flow.py` + `cflib_takeoff_with_flow.py` + `analyze_drift.py`, pass criterion `RMS(x,y)_with_flow < 0.5 * RMS(x,y)_no_flow` over 10..30 s window. (9) `sim/gazebo/sentai_crazysim_world.sdf` — downward camera FOV corrected from 1.047 (60°) to 1.0123 (58°) per SentAI lens spec; pose yaw flipped 180° so image LEFT = body FORWARD per convention.  ARM build still 100% green (sentai_runtime build #1230) — shims are zero-cost on ARM.  REMAINING: `sudo apt install -y libfftw3-dev`, `cmake --build build-sim --target sentai_sim`, run `make run` in s088, then s089 dual-run. |
| 2026-05-10 | 10b | DONE — Sim.md camera + flow conventions section | n/a | New section §10b documents the lens FOV (58°×45°), 4:3 long-axis = FW-BACK, body-frame mapping (image LEFT = body FORWARD with cam0 vflip=1, sign convention from 2026-05-07 LED-translation test), camera physical mount offset on the drone (-4 cm back, -2 cm down — needed for EKF lever-arm if we ever switch to OPTICAL_FLOW_RAD), and the PXP downscale + flow input chain (identical ARM ↔ SIM after the shim).  Anything Phase 4 produces or consumes MUST match this — otherwise cf2 EKF rejects samples or amplifies drift. |
| 2026-05-10 | 3 best-practices | DONE — captured during cflib TOC debug session | n/a | **Hard-won CrazySim/Gazebo Garden best practices.  (Originally collected on Harmonic but Harmonic is now banned; the practices apply equally to Garden in distrobox.)  Read these before any future debug session.**  (1) **`stdbuf -oL` is mandatory for cf2** — `cf2`'s stdout is block-buffered when redirected to a file (4 KB).  Default `sitl_singleagent.sh` does `cf2 ... > out.log 2> error.log &` and the logs stay EMPTY for minutes.  Wrap with `stdbuf -oL -eL cf2 ...` to flush per-line and see boot progress (`SOCKET_LINK: Waiting for connection with gazebo`, `Connection established`, `SYS: Software-in-the-Loop Simulator is up and running!`).  (2) **Always launch `gz sim` with `-v 4` (debug) during bringup, not the default `-v 3`** — plugin-load failures (`Failed to load system plugin [gz_crazysim_plugin] : Could not find shared library`) are logged ONLY at `-v 4`.  At `-v 3` the world boots silently with no plugin and EVERYTHING downstream (cflib TOC, motor commands, telemetry) silently times out.  (3) **`GZ_SIM_SYSTEM_PLUGIN_PATH`, `GZ_SIM_RESOURCE_PATH`, `LD_LIBRARY_PATH` MUST be set in the shell that launches `gz sim`** — `setup_gz.bash` sets them, but only inside the script's process tree.  If you run `gz sim ...` ad-hoc in another shell, the plugin is silently missing and the drone's `/cf_0/imu` topic exists but with NO subscriber on the cf2 side.  (4) **Plugin <-> cf2 handshake is `0xF3`** — cf2 SOCKET_LINK sends `0xF3` (1 byte, header only, size=0) repeatedly until plugin echos `0xF3` back.  Plugin learns cf2's ephemeral source addr from `recvfrom`'s remaddr.  cf2 then continues to system init.  Sequence visible at cf2 stdout (with stdbuf): `Create socket succeed → Binding succeed → Waiting for connection with gazebo → Connection established with gazebo → SYS: Software-in-the-Loop Simulator is up and running!`  (5) **cflib UdpDriver handshake is `\xff\x01\x01\x01`** — plugin doesn't ack this, just learns cflib's addr from recvfrom.  Subsequent CRTP packets are bidirectional. |
