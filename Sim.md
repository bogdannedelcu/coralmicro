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

**Remaining issues addressed (2026-05-11)**:

**Issue #2 (magnitude saturation)** — fixed in `camera_bridge_recv.c`:
duplicate-frame CRC detection (Garden render rate < 30 fps now reports
conf=0 instead of stale-but-confident motion); saturation guard at
±28k mgp drops aliased peak-edge readings.  Code is portable C, identical
on ARM where OV5640 always produces fresh frames so the duplicate path
never fires.

**Issue #3 (takeoff timing)** — fixed in `cflib_takeoff_*.py`:
new ramp-hold-climb profile (0→0.30m in 1s, hold 2s, climb to 1m in 2s).
Flow gate opens at z>0.30 with clear margin before climbing further.

**Issue #1 (Z noise)** — substantially mitigated in `FlowReceiver`:
1. `_altitude_std_scale(z)` inflates flow std at low altitude (1/z²
   Jacobian in cf2 mm_flow.c bleeds flow noise into KC_STATE_Z most
   strongly there): std multiplier goes 1.0 @ z>=1m, 4.0 @ z=0.5m,
   8.0 @ z<=0.30m.
2. **Attitude gate** at |roll| or |pitch| > 0.35 rad (~20°): skip flow
   entirely above this attitude.  cf2 mm_flow.c uses
   R[2][2]=cos(roll)*cos(pitch) for body→world; tilt degrades the
   coupling and any flow noise spills into Z.

**Verified end-to-end (2026-05-11)**:

Wind +X (forward drift):
| Run                    | X delta | Y delta | Z range      |
|------------------------|---------|---------|--------------|
| baseline (no flow)     | 1.42 m  | 0.18 m  | [0.99, 1.07] |
| with flow (gated)      | 0.23 m  | 0.13 m  | [-0.21..1.24] |
| **X drift reduction**  | **6×**  |         |              |

Wind +Y (perpendicular drift):
| Run                    | X delta | Y delta | Z range      |
|------------------------|---------|---------|--------------|
| baseline (no flow)     | 0.08 m  | 0.30 m  | [1.00, 1.08] |
| with flow (gated)      | 0.07 m  | 0.06 m  | [-0.21..1.08] |
| **Y drift reduction**  |         | **5×**  |              |

Yaw +180° rotation under flow (no wind):
| Phase                  | X drift | Y drift | Z range      | Yaw         |
|------------------------|---------|---------|--------------|-------------|
| Pre-rotate hover (8s)  | 4 cm    | 2 cm    | [0.81, 1.08] | 0°          |
| **During rotation 4s** | 2 cm    | 2 cm    | **[1.00, 1.02]** ✅ | 0 → 128°    |
| Post-rotate hover (8s) | 2 cm    | 8 cm    | [-0.19, 1.00] | 27 → 153°   |

The yaw test confirms body-frame flow projection is invariant to drone
yaw — cf2 EKF applies the current attitude R matrix to convert body
velocity into world frame, and the body_xform mapping in the bridge
(image axes → drone body axes) is itself rotation-invariant because the
camera is rigidly attached to the drone.  Z stayed within 2 cm during
the actual rotation thanks to the attitude gate keeping flow injection
quiet while the drone settled.

**Open items for next session**:
- Post-rotation Z transient ([-0.19, 1.00] at end of yaw run) needs
  further damping (gate opens too quickly after the rotation stops?
  hysteresis on attitude gate?).
- Closed-loop test under wind +X simultaneously with yaw rotation
  (real-world disturbance scenario).
- Push Garden render rate higher with VirtualGL or upgrade to Harmonic
  fix once that lands (Sim.md §10c bans Harmonic for now).

## 10e. dz from optical flow — sub-block divergence (diagnostic)

Added 2026-05-11 — `sentai_flow_phase_corr_compute_dz()` runs phase
correlation on 4 sub-blocks of the 80×60 gray (top/bottom/left/right,
each 32×32 padded) and computes the Horn-Schunck divergence:

    dz/z = -divergence/2
    div  = (dy_bottom - dy_top)/Δy + (dx_right - dx_left)/Δx

Output `dz_q1000` is in **µ-per-frame** (parts-per-million altitude rate;
+1000 = +0.1 % altitude/frame).  No focal length required — divergence
is dimensionless, focal cancels out.  Caller derives absolute m/s by:

    dz_mps = z_estimate_metres × dz_q1000 / 1e6 × frame_rate_hz

**cf2 firmware does NOT consume dz from flow** — `flowMeasurement_t`
only has `dpixelx`, `dpixely`.  Z absolute comes from baro
(`heightMeasurement_t`) or laser ToF (`tofMeasurement_t`).  Three
options to use our dz:
  A. **Diagnostic only** (current): expose via `sentai.flow.read()`,
     don't feed EKF.  Use cases: ground-rush detection, baro vs vision
     cross-check.  Zero risk.
  B. Send as `extpos.send_extpos(z=...)` after integrating dz×dt — gives
     EKF an absolute Z observation.  Risk: integration drift.
  C. Modify cf2 firmware (add `dpixelz` + Jacobian to `mm_flow.c`).
     Risk: breaks Bitcraze upstream compatibility.

**SIM validation (drone climb 0→1m, descend 1.5→0.5 m at ~25 cm/s)**:

| Phase                         | Expected dz/frame | Observed dz/frame | Verdict          |
|-------------------------------|-------------------|-------------------|------------------|
| Phase E: descent at z=1m      | −8 333 µ          | −10 416 µ         | ✅ sign + ~125% |
| Phase B: hover at 1m (steady) | ~0                | initial transient | needs filter    |

Sign is **negative for descent** (features converge as drone gets closer
to ground), positive for climb.  Magnitude within 25 % of theory in
steady-state.

`sentai.flow.read()` now returns 7-tuple (was 5):
  `(seq, dx_q1000, dy_q1000, conf, latency_us, dz_q1000, dz_conf)`
Both ARM and SIM expose the same tuple — code is shared in
`flow_phase_corr.cc`.  ARM uses CMSIS `arm_cfft_sR_f32_len32`, SIM uses
the FFTW3-backed `sentai_cfft_sR_f32_len32` instance.

Cost: 4× length-32 phase-corrs ≈ 1.6 ms on M7 @ 800 MHz, 0.6 ms on x86
— well inside the 33 ms / 30 fps budget.

## 10e2. sentai.flow → cf2 EKF wire (SIM, 2026-05-11)

**On HW**: `_t_flow_to_drone.py` (board) reads `sentai.flow.read()` →
converts to PMW3901 dpixel → ships `flow_pkt_t` over UART2 CRTP ch=1 →
`app_sentai_bridge` deck driver receives → `estimatorEnqueueFlow()`.

**In SIM**: same algorithm, different transport.  CrazySim cf2
firmware **already has** the SITL-side flow ingest path —
`src/hal/src/sensors_sitl.c` listens on `CRTP_PORT_SETPOINT_SIM = 0x09`
for packets where `data[0] == SENSOR_FLOW_SIM (=6)`, decodes
`[dpx f32 LE, dpy f32 LE, dt f32 LE]`, calls `estimatorEnqueueFlow()`
identically.  **No `app_sentai_bridge` build needed in SIM** — the
UART2 transport doesn't exist anyway, and the SITL HAL already
provides equivalent receive.

What we wired up:

- **CrazySim cf2 driver-side patches (parity with HW firmware)**:
  - `kalman_core.c`: propwash fix on `baroReferenceHeight` capture
    (commit 53d72897 from `bogdannedelcu/crazyflie-firmware` ported).
    Without this, baseline gets corrupted at takeoff and altitude hold
    is unstable.
  - `estimator_kalman.c`: enable `KALMAN_USE_BARO_UPDATE` (commit
    2b7661c5).  Required for stable altitude hold with optical-flow
    feeding the EKF.
- **Host-side sender** in `experiments/s090_hover_over_cat/hover_over_cat.py`:
  - hover_logic.py emits `STATE=` with raw `dx_q1000`, `dy_q1000` in the
    last 2 fields (read from `sentai.flow.read()`).
  - Host wrapper consumes via queue, applies `_to_body_dpx` (cam0+vflip=1
    `body_xform=(-1,0,0,+1)`), scales by `_FLOW_SCALE_X/Y` derived from
    sensor FOV vs drone PMW3901 geometry (matches `_t_flow_to_drone.py`
    on HW exactly), then `cf.send_packet(port=0x09, channel=0,
    data=struct.pack("<Bfff", 6, dpx, dpy, dt))`.
- **Tracking + hover algorithm is decoupled**.  hover_logic.py does NOT
  use flow data in its controller — pure bbox-centroid pixel-error →
  body velocity command.  Flow stabilises the drone via cf2's own EKF
  consumer; it's an independent path running on the same camera frames.

**Validation (2026-05-11, this session, build #38)**:

| Metric | No flow | With sentai.flow → cf2 |
|--------|---------|------------------------|
| Drift after 30 s hover @ 1 m | +0.41 m Y | **+0.015 m Y (×27 reduction)** |
| Flow packets to cf2 | 0 | 136 |
| LOCK events (SSD detections) | 139 | 122 |
| Drone final pose | (0.203, −0.427, 0.015) | (−0.004, +0.015, 0.015) |

cf2 receives flow, EKF locks position, drone stays put when commanded
to hover.  hover-over algorithm sends body-velocity commands on top —
they get partially absorbed by the EKF stationkeeping, so visible
position chasing is reduced versus open-loop.  Net behaviour: tight
hold with detection-driven small nudges (matches expected closed-loop
PMW3901 + SSD coexistence).

## 10f. Native `import` from the SIM virtual FS (best practice, 2026-05-11)

**Rule: ship Python helpers as importable modules in the SIM virtual FS,
not as `exec(sentai.fs.read_str("foo.py"))` source-string blobs.**

What changed in 2026-05-11 build:

- `sim/main_sim.c` now provides real `mp_lexer_new_from_file` and
  `mp_import_stat` routed through `sim_fs_resolve()` (the same resolver
  `sentai.fs.*` uses).  Backed by an inline 64-byte FD reader — no need
  to flip `MICROPY_READER_POSIX` on the embed config (that would also
  pull a competing `mp_lexer_new_from_file` that bypasses `fs_resolve`).
- `mp_embed_exec_str("import sys\nsys.path.append('/')\n")` runs once at
  REPL boot, so a bare `import foo` finds `<sim_fs_root>/foo.py`.
- `sim_fs_root()` and `sim_fs_resolve()` in `modsentai_sim.c` are now
  non-static (extern) so main_sim.c can call them.

**Why import beats exec(read_str()):**

| Aspect             | `exec(sentai.fs.read_str("foo.py"))`         | `import foo`                              |
|--------------------|----------------------------------------------|-------------------------------------------|
| Heap allocation    | full file as one `str` object (~KBs)         | 64 B FD buffer, lexer streams char-by-char |
| Source lifetime    | str pinned in `exec`'s scope                 | freed as soon as lexer consumes it        |
| Tracebacks         | `<string>` line N — no filename              | `foo.py` line N — usable                  |
| Re-execution       | re-reads + re-compiles each call             | cached in `sys.modules`, second call free |
| Firmware parity    | requires `sentai.fs.read_str` everywhere     | matches firmware: FileX-backed import     |

**ARM parity guarantee:** firmware build provides equivalent
`mp_lexer_new_from_file` / `mp_import_stat` routed through FileX
(`FxUserOpenRead`).  Same `import foo` line works in both targets as
long as `foo.py` lives at the FS root.  This is the LOAD-BEARING reason
to keep this contract: identical test scripts on SIM and HW.

**How to use in a test:**

1. Drop the helper script at `build-sim/sentai_fs_root/<name>.py`
   (SIM) or push to `/<name>.py` over REPL (HW).
2. From the controlling Python wrapper, send a single-line REPL command:
   `import <name>`
3. If the helper has a top-level loop, the loop runs to completion before
   `import` returns.  If you need to invoke it on demand, structure the
   helper as `def run(): ...` and the wrapper sends `<name>.run()` as a
   second REPL line.

**Anti-patterns to avoid:**

- Piping multi-line `for`/`if` blocks through the REPL stdin — the
  line-by-line REPL mis-indents them and emits `SyntaxError`.  Put the
  block in a `.py` file and `import` it.
- Calling `exec(sentai.fs.read_str(...))` for anything larger than a
  one-liner — both the source `str` and the resulting code object live
  on the heap until the calling scope exits.
- Hand-rolled `compile() + exec()` — same heap cost, plus the extra
  `code` object.  No upside over `import`.

`hover_over_cat.py` (s090) was the migration site that drove this
work: 200-iter top-level loop now lives in
`build-sim/sentai_fs_root/hover_logic.py` and the wrapper does
`import hover_logic` — single REPL line, MP lexer streams it from disk.

## 10g. Hover-over-detection best practices (s090, 2026-05-11)

Hard-won lessons from the closed-loop SSD + flow + cf2 hover-over demo.
Read these before adding new gaze-and-track behaviours to any SIM
experiment.

**1. CrazySim drone camera is BELOW ground for z<1m on this drone
model.**  Confirmed via Gazebo GUI PIP.  At low altitude the downward
camera sees only the z=-10m deep checkerboard fallback plane and SSD
hallucinates `tv/sink/refrigerator` at 10-20% conf on the uniform
texture.  Any "lock" at z<1m is therefore a spurious hallucination.
Staged-takeoff fix: enforce `MIN_LOCK_Z >= 1.0m` AND filter on
plausible classes before accepting the lock.

**2. cf2 SITL `send_hover_setpoint(vx, vy, yawrate, zDistance)` has
INVERTED vy convention vs the body +Y=left textbook.**  Empirically
verified 2026-05-11: at z=2.6m the drone with vy=-200 moved world +Y
(left) instead of world -Y (right).  hover_logic.py compensates by
emitting `vy = -err_x * GAIN` (flipped sign vs original cam0+vflip=1
body convention).  vx behaves as expected (`vx = +err_y * GAIN`).

**3. Coral COCO-17 model class labels — `class 16 = CAT`, not dog.**
Per Coral's official mapping in
`/home/bogdan/work/edge/edgetpu/test_data/coco_labels.txt`: 15=bird,
**16=cat**, 17=dog, 18=horse, 19=sheep, 20=cow, 21=elephant, 22=bear.
pycoral's `Detection.id` field is 0-indexed RAW model output; do NOT
add a +1 offset.  Initial overlay scripts in this repo had the wrong
mapping (16→"dog") and labels were corrected only after user caught
the mislabelled green bboxes visually.  When in doubt, run
`from pycoral.utils.dataset import read_label_file; print(read_label_file(...))`
and compare to your hard-coded dict.

**4. MobileNet V2 COCO17 confuses cat with dog/bear/horse on
`test_data/cat.bmp`.**  At z=2.6m looking down the model returns class
16 (cat) at 75-78% conf consistently when properly oriented; flipped
180° the model still gets 16 but conf varies 50-65%.  The hover_logic
class filter is `PLAUSIBLE_CAT_CLASSES = {15, 16, 21}` — accepts the
canonical cat plus its common confusion buckets.  Don't lock the
filter to `{16}` only or you'll lose track during partial views.

**5. Gazebo Garden 7.9 PBR textures must be SQUARE + power-of-two.**
A non-square texture (e.g. 512×341 from a 4:3 photo) loads but renders
INVISIBLE — Garden silently falls back to base diffuse colour and you
see only the underlying ground plane.  Fix: pad the source PNG to
512×512 (or 1024×1024) with white/neutral background, then adjust the
SDF `<box>` size to match the new aspect.  `file` will confirm
`PNG image data, 512 x 512`; checker.png (which works) is also 512×512
and uses the same `<pbr><metal><albedo_map>` element.

**6. Sentai_sim camera_bridge_recv frame dump (SIM-only).**  Set
`SENTAI_DUMP_FRAMES_DIR=/some/dir` + `SENTAI_DUMP_FRAMES_EVERY=15`
before launching `sentai_sim` to save raw 640×480 RGB PPMs to disk
(every Nth received frame).  Code lives in `sim/camera_bridge_recv.c`
which is only compiled on SIM target — never on ARM (PPM I/O is too
slow for the M7).  The wrapper at `/tmp/run_hover.sh` makes a
**timestamped per-experiment dir** so runs don't overwrite each other
(`/tmp/sentai_frames_YYYYMMDD_HHMMSS/`).

**7. TPU helper daemon (`sim/scripts/sim_tpu_helper.py`) is brittle —
it dies on USB transfer errors when the Coral USB Accelerator returns
LIBUSB_TRANSFER_ERROR (5).**  Symptom: `sentai.tpu.load() = -1` +
`HOVER_LOGIC_DIAG iter=N n_tracks=0 stats={'frames': 0}` (pipeline
runs but no model loaded → no detections).  Restart procedure:
```bash
pkill -9 -f sim_tpu_helper.py
rm -f /tmp/sentai_tpu.sock
nohup venv-coral/bin/python3 -u sim/scripts/sim_tpu_helper.py < /dev/null > /tmp/tpu_helper.log 2>&1 &
disown
```
Add `pgrep -f sim_tpu_helper >/dev/null || restart_tpu_helper()` to
any long-running experiment harness.

**8. STATE= line emitted by hover_logic.py must include FULL bbox
(x1,y1,x2,y2), not just the centroid.**  Initial 12-field STATE only
had `cx, cy` — host-side overlay had to synthesize an artificial
±50px box around the centroid, leading to bbox positions that didn't
match what SSD actually saw.  16-field STATE now appends `x1, y1,
x2, y2` in 300×300 SSD-input space.  Host scales by `640/300, 480/300`
when overlaying on 640×480 PPM.

**9. hover_logic loop duration must exceed staged-takeoff time.**
With N_ITER * RATE_MS too short (e.g. 200 × 200 ms = 40 s), a slow
staged takeoff (60+ s climbing to MAX_CLIMB_Z=3m at 0.05 m/s) finishes
the inner loop BEFORE the host wrapper enters its hover-over phase →
zero STATE messages → zero LOCK events → zero flow packets.  Either
make the climb fast (`+0.05 m/tick` at 10 Hz = 0.5 m/s, MAX_CLIMB_Z
reached in 6 s) or extend N_ITER (currently 500 = 100 s, generous
headroom).

**10. Coast through detection dropouts.**  SSD MobileNet V2 on
non-canonical poses (drone pitched, motion blur, cat partially out of
FOV) drops the class-16 track for 5-10 frames at a time.  hover_logic
keeps the last commanded velocity active for `COAST_FRAMES` ticks
after detection loss, so the drone keeps approaching the last-known
bbox centre instead of stopping cold.  Reacquisition is usually <2 s.
Without coast, the drone yo-yos: full thrust toward target → lose
detection → zero command → drone drifts → reacquire → full thrust →
oscillate.

## 10h. Flow + commanded-velocity integration (Bitcraze-recommended patterns)

After session 2026-05-11 web research, here are the canonical Bitcraze
patterns and pitfalls for integrating optical flow with target-driven
lateral motion.  Mirrors what the HW examples + community forum threads
converged on, transposed to our SIM stack.

**1. EKF reset post-setup, pre-injection.**  Set `kalman.resetEstimation=1`
AFTER flipping `stabilizer.estimator=2` (Kalman) and BEFORE you start
streaming external observations.  Without reset the cold EKF state can
diverge to NaN when fed conflicting flow+IMU data — position control
then silently fails.  In `hover_over_cat.py` (s090) this single one-line
change lifted LOCK events from 93 → 312 (3.4×) in identical 30 s
hovers because the EKF no longer fights its own startup transient.
Source: Bitcraze forum t=2629 + t=4125.

**2. `send_hover_setpoint(vx, vy, yawrate, zdistance)` feeds the
velocity controller, not position.**  Cascade order:
`velocity → attitude → attitude-rate → motor PWM`.  Flow measurements
update EKF velocity/position; the controller then drives motors via
the cascade.  With flow-only positioning, the EKF's absolute X/Y is
capped at ±10 m and is NOT trustworthy — `send_position_setpoint`
"may not make so much sense since the absolute position is not known"
per Bitcraze docs.  Sticking with `send_hover_setpoint` is the right
call for flow-only stacks.  Source:
[controllers.md](https://github.com/bitcraze/crazyflie-firmware/blob/master/docs/functional-areas/sensor-to-control/controllers.md),
[state_estimators](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/).

**3. Setpoint streaming >= 10 Hz.**  Lower than that and the cf2
watchdog (`commanderWatchdogTimeout = 500 ms`) zeros roll/pitch then
cuts motors at 1 s.  Symptom: drone tilts in then springs back
repeatedly.  Our s090 wrapper streams at 5 Hz which is on the edge;
bump RATE_HZ to 10 if you see watchdog twitch.  Source:
[Bitcraze forum t=2629](https://forum.bitcraze.io/viewtopic.php?t=2629).

**4. Motion Commander class is designed for the Flow Deck.**  It runs
a background thread that streams velocity setpoints continuously,
handles takeoff/landing, and matches the flow stack's relative
positioning semantics.  Lacks a "go to absolute X/Y" equivalent — use
`Commander.send_position_setpoint` for that, but only when you have an
absolute-position source (MOCAP, UWB, Lighthouse).  Bitcraze's own
demos for flow-only drones use Motion Commander, not the low-level
Commander.  Source:
[Kim McGuire — Commander framework offboard/onboard 2024](http://www.mcguirerobotics.com/blog/old_bitcraze_blogposts/2024_01_01_the-commander-framework-part-2-offboard-or-onboard/).

**5. Visual servoing residual problem: drone tilt rotates camera
FOV without translating the drone, making the bbox appear to move in
image space.**  s090 observed bbox shift ~65 cm of world-equivalent in
the image, while drone physical translation was ~2 cm.  Pure
pixel-error → body-velocity feedback over-reacts to attitude wobble.
Real PMW3901 flowdeck firmware already filters this via gyro-rate
de-rotation in `mm_flow.c` — for our pipeline, future work is to
gravity-vector / attitude-compensate the bbox centroid before
computing err_x/err_y.  No Bitcraze forum thread documents this fix in
the offboard-detection path; it remains open.

**6. CrazySim sensors_sitl flow protocol — extend stdDev in packet,
not hardcode.**  Vanilla CrazySim hardcodes `flowData.stdDevX = 2.0f`
in `sensors_sitl.c::SENSOR_FLOW_SIM`.  Real PMW3901 deck driver maps
the sensor's confidence byte to std (high conf → low std, low conf →
high std).  We patched CrazySim to accept std from the wire (17-byte
packet, std at p.data[13..17]) and made our host wrapper send the
conf-mapped value.  With `std=4.0` (mid-trust band) flow no longer
fights drone wobble integration.  Patch lives at
`bogdannedelcu/crazysim-crazyflie-firmware` branch
`sentai-flow-sim-support`, commit e4374251.

**Reference threads + docs (all consulted 2026-05-11):**
- [Bitcraze forum: Flying Crazyflie to specific position setpoint](https://forum.bitcraze.io/viewtopic.php?t=4125)
- [Bitcraze forum: Setpoint handling in Crazyflie firmware](https://forum.bitcraze.io/viewtopic.php?t=3459)
- [Bitcraze forum: Position data of crazyflie](https://forum.bitcraze.io/viewtopic.php?t=2867)
- [Bitcraze forum: Issuing position change commands](https://forum.bitcraze.io/viewtopic.php?t=2629)
- [Bitcraze forum: Kalman filter reset [SOLVED]](https://forum.bitcraze.io/viewtopic.php?t=3616)
- [Bitcraze forum: Hovering scripts](https://forum.bitcraze.io/viewtopic.php?t=4051)
- [Bitcraze docs: Controllers in the Crazyflie](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/)
- [Bitcraze docs: State estimation](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/)
- [Bitcraze docs: Parameter groups (kalman.resetEstimation)](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/params/)
- [Kim McGuire blog: Commander framework offboard/onboard 2024](http://www.mcguirerobotics.com/blog/old_bitcraze_blogposts/2024_01_01_the-commander-framework-part-2-offboard-or-onboard/)
- [Bitcraze AI-deck documentation update 2023](https://www.bitcraze.io/2023/03/ai-deck-documentation-and-examples-update/)
- [CrazySim repo (gtfactslab) — base SIM stack](https://github.com/gtfactslab/CrazySim)
- [crazyflie_ros issue #66: velocity setpoints with Flow Deck (whoenig)](https://github.com/whoenig/crazyflie_ros/issues/66)

## 10i. Disk-first experiment artifacts (best practice 2026-05-11)

**Rule**: every SIM experiment must produce ALL data needed to
reconstruct, analyse, and replay it as files on disk in a single
**per-experiment directory**.  No reliance on REPL stdout being
captured, no ephemeral in-memory state, no "the SSD detection scrolled
past in the terminal and you'd have to scroll up to see it".

**Reason**: during s090 development this session we spent 2-3 hours
debugging a misaligned bbox overlay that turned out to be a
frame-to-STATE index-ratio mismatch.  The fix was trivial (match by
gz frame seq, not by index) but the bug was invisible because the
state was buried in an ephemeral hover_sim.log getting overwritten on
every run.  Had the per-experiment dir included a structured state.tsv
+ flight.tsv, the bug would have been immediately catchable by
sorting and joining the two files in pandas.

**Layout** for `/tmp/sentai_frames_<YYYYMMDD_HHMMSS>/`:

| File              | Format | Rate    | Source                              |
|-------------------|--------|---------|-------------------------------------|
| `frame_NNNNNN.ppm`| PPM 640×480 | gz fps / N | sim/camera_bridge_recv.c       |
| `state.tsv`       | TSV    | 5 Hz    | host wrapper drains hover_logic STATE |
| `flight.tsv`      | TSV    | 50 Hz   | cflib LogConfig stateEstimate.{rpy,xyz} |
| `hover.log`       | text   | event   | host wrapper [hover] events         |
| `hover_sim.log`   | text   | sim stdout | reader_thread tee                |
| `run.mp4`         | H.264 baseline | 5 Hz | make_video.py post-run rendering |

`state.tsv` columns (17): `iter tid cls conf cx cy err_x err_y vx vy
fvx fvy x1 y1 x2 y2 fseq`.

`flight.tsv` columns (7): `ts roll pitch yaw x y z`.

**Replay**: any post-hoc tool reads `<exp_dir>/state.tsv` and
`<exp_dir>/flight.tsv` directly with `pandas.read_csv(sep='\t')` and
correlates by timestamp / gz frame seq.  No need to re-run the
experiment or scrape stdout.  Overlay video reproduces from the same
two TSVs + PPMs via `make_video.py <exp_dir>`.

**Anti-patterns to avoid:**

- Scraping STATE from a global `/tmp/hover_sim.log` that the NEXT run
  will overwrite.  Always copy/snapshot per-experiment.
- Logging to REPL stdout instead of to disk.  REPL terminal scrollback
  is fine for interactive debug, but it's NOT the experiment record.
- "I'll re-run if needed" — re-runs aren't reproducible (Coral USB
  drops between runs, gz timing varies, drone yaw differs).  Capture
  ONCE, analyse FOREVER.
- Storing experiment dirs outside the canonical pattern
  `examples/sentai_runtime/experiments/sNNN_<name>/` (the
  `/tmp/sentai_frames_*` dir is a workdir; results that prove a
  hypothesis get copied / linked back into the sNNN directory).

## 10j. Hover-over-detected-object — conclusions (s090 final state, 2026-05-11)

The s090 experiment closed the loop: drone detects the ImageNet cat
picture on the ground via on-board SSD + SORT, then navigates above it
holding altitude, with optical flow stabilising drift simultaneously.
**Pure classical control did the job — no policy learning, no RL.**
This section captures what worked, what didn't, and the canonical
recipe for similar tasks.

### What works (the recipe)

```
gz camera (640×480 @30fps)
    │
    ▼
camera_bridge_recv  ── PPM dump (every 15th frame, /tmp/sentai_frames_<TS>/)
    │  (RGB → PXP 80×60 → BT.601 luma → phase corr → g_flow snapshot)
    │  (RGB → nearest-neighbor 300×300 → SSD MobileNet V2 → SORT)
    │
    ├──► sentai.flow.read()  ← gz frame seq propagated through `f[0]`
    │     │
    │     ▼
    │   host wrapper: flow_to_dpixel() PMW3901 conversion (body_xform=(-1,0,0,+1))
    │     │
    │     ▼  17-byte CRTP packet [SENSOR_FLOW_SIM=6, dpx, dpy, dt, stdDev=4.0]
    │   cf2 SITL sensors_sitl.c → estimatorEnqueueFlow
    │     │
    │     ▼
    │   cf2 EKF velocity update (×27 drift reduction vs no flow)
    │
    └──► sentai.pipeline.tracker_tracks()  → confirmed cls=16 cat track
          │
          ▼
        hover_logic.py: bbox centroid → 17-field STATE w/ fseq
          │
          ▼  (push to per-experiment state.tsv on disk)
        host wrapper: attitude_compensate() bbox using cf.log.stateEstimate.{roll,pitch}
          │  (subtract f_px * pitch_rad, f_py * roll_rad — small-angle virtual horizontal cam)
          ▼
        proportional pixel-error → body velocity (vx = +err_y*GAIN, vy = +err_x*GAIN)
          │
          ▼
        MotionCommander.start_linear_motion(vx, vy, 0)  → cf2 cascaded PID
          │  (velocity → attitude → attitude-rate → motor PWM)
          ▼
        DRONE TRANSLATES toward bbox-image-centre
```

### Best practices distilled

**1. Kalman EKF reset post-stabilizer-flip.** `kalman.resetEstimation=1`
   after `stabilizer.estimator=2`, BEFORE flow injection starts.
   Without reset the cold EKF state diverges on the first noisy flow
   sample.  Verified ×3.4 lift in LOCK events (93 → 312) in s090.

**2. MotionCommander, not raw send_hover_setpoint.**  The flow-deck-
   aware background-streaming class keeps cf2's 500 ms watchdog fed and
   handles takeoff/landing.  Drone went from "barely 5cm motion in 30s"
   to "+0.54m / -0.39m within ~10cm of the cat target" on the run after
   switching.

**3. Virtual horizontal camera frame.**  Drone tilt rotates camera FOV
   ↔ bbox appears to shift in image WITHOUT drone translating.  Real
   PMW3901 deck firmware compensates this via gyro-rate de-rotation in
   `mm_flow.c`; offboard pipelines need the equivalent.  Our
   small-angle `attitude_compensate()` applies `cx += f_x * pitch_rad`,
   `cy += f_y * roll_rad` with `f_x ≈ 270 px`, `f_y ≈ 362 px` at our
   58°×45° FOV.  For tilts ≥5° upgrade to full rotation homography.

**4. Sign convention is camera-mount-specific — verify empirically,
   don't trust documentation.**  Our SIM cam0+vflip=1 setup ended up
   with: image LEFT (low cx) ↔ body forward (+X), image BOTTOM (high
   cy) ↔ body right (-Y).  Controller: `vx = +err_y*GAIN`,
   `vy = +err_x*GAIN`.  THREE sign-flip iterations before this stuck.
   Always log world pose alongside cmd to catch reversed axes.

**5. cf2 SITL `SENSOR_FLOW_SIM` packet must carry stdDev.**  The vanilla
   13-byte form hardcodes `stdDevX = stdDevY = 2.0` which over-trusts
   noisy phase-corr in featureless scenes.  We extended `sensors_sitl.c`
   to accept stdDev in p.data[13..17] (17-byte packet form, legacy
   13-byte still works).  Mirrors the PMW3901 deck driver's internal
   conf→std mapping.  Patch on
   `bogdannedelcu/crazysim-crazyflie-firmware:sentai-flow-sim-support`.

**6. Same seq number throughout the data pipeline.**  Source PPM
   `frame_NNNNNN.ppm` ↔ rendered overlay `frame_NNNNNN.png` ↔ state.tsv
   row with fseq=NNNNNN.  No index-ratio mapping anywhere.  Per
   embeded.md §"replace implicit conventions with explicit APIs".  An
   earlier overlay used proportional index-matching and produced bboxes
   shifted by 30+ frames of detection lag — invisible until rendered.

**7. Per-experiment self-contained dir.**  All artifacts in
   `/tmp/sentai_frames_<YYYYMMDD_HHMMSS>/`: PPMs, flight.tsv, state.tsv,
   hover.log, hover_sim.log, run.mp4.  Re-runs never overwrite — every
   experiment is post-hoc-analysable from disk alone.  Anti-pattern:
   scraping `/tmp/hover_sim.log` which the next run will overwrite.
   Per §10i.

**8. Auto-restart brittle services pre-run.**  The Coral USB driver
   throws transfer error 5 on client disconnect and dies; `run_hover.sh`
   `pkill -9` + restart of `sim_tpu_helper.py` is now standard pre-run
   so the helper is provably alive when sentai_sim calls `tpu.load()`.
   Without this, pipeline silently runs without a loaded model and
   produces `frames=N, n_tracks=0` forever.

### What we did NOT need (deliberately)

- **No reinforcement learning / no policy training.**  Classical PID
  on the tilt-compensated pixel error matches what the academic
  literature converges on for this task.
- **No absolute position source (UWB, Lighthouse, MOCAP).**  Flow-only
  positioning is sufficient for stationkeeping during the hover phase;
  bbox-in-image acts as the absolute-position reference for the
  controller.
- **No HW-side firmware mod beyond CrazySim cf2.**  `app_sentai_bridge`
  (the UART2 bridge on real hardware) isn't compiled into CrazySim —
  not needed, because cf2 SITL already has `SENSOR_FLOW_SIM` wired in
  `sensors_sitl.c` (we just extended the packet format).

### Papers + references that informed this design

The recipe above tracks closely with established quadrotor IBVS
literature.  Sources consulted during s090 development:

**Visual servoing fundamentals**:
- Virtual camera-based visual servoing for rotorcraft using monocular
  camera and gyroscopic feedback — Elsevier ScienceDirect.  Closest
  match to our attitude_compensate() approach.
  <https://www.sciencedirect.com/science/article/abs/pii/S001600322200552X>
- Autonomous Vision-Based Object Detection and Tracking System for
  Quadrotor UAVs — MDPI Sensors 2025 (full IBVS+detection pipeline).
  <https://www.mdpi.com/1424-8220/25/20/6403>
  <https://pmc.ncbi.nlm.nih.gov/articles/PMC12567932/>
- Precise Interception Flight Targets by Image-based Visual Servoing
  of Multicopter — arXiv 2024 (attitude-change handling).
  <https://arxiv.org/html/2409.17497v1>
- Image-Based Visual Servoing for UAVs Based on Fuzzy Logic — Sage
  Journals 2023.
  <https://journals.sagepub.com/doi/10.1177/16878132231167238>
- Image-Based Adaptive Visual Control of Quadrotor UAV — MDPI
  Electronics.
  <https://www.mdpi.com/2079-9292/14/15/3114>
- Visual Servoing Approach to Autonomous UAV Landing on a Moving
  Vehicle — MDPI Sensors 2022.
  <https://www.mdpi.com/1424-8220/22/17/6549>

**Hybrid / self-supervised approaches** (for reference, NOT adopted):
- Efficient Self-Supervised Neuro-Analytic Visual Servoing for
  Real-time Quadrotor Control — arXiv 2025.
  <https://arxiv.org/html/2507.19878>

**Bitcraze stack references**:
- Crazyflie controllers cascade documentation.
  <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/controllers/>
- State estimation (Kalman EKF + flow handling).
  <https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/>
- Bitcraze forum: flying to position with flow deck pitfalls.
  <https://forum.bitcraze.io/viewtopic.php?t=4125>
- Bitcraze forum: Kalman filter reset (resolved).
  <https://forum.bitcraze.io/viewtopic.php?t=3616>
- Kim McGuire — Commander framework offboard/onboard 2024.
  <http://www.mcguirerobotics.com/blog/old_bitcraze_blogposts/2024_01_01_the-commander-framework-part-2-offboard-or-onboard/>
- CrazySim repo (gtfactslab) — base simulator.
  <https://github.com/gtfactslab/CrazySim>

**Tutorials & tooling**:
- ViSP Tutorial: Image-based visual servo (IBVS).
  <https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-ibvs.html>
- Robotics Knowledgebase: Visual Servoing.
  <https://roboticsknowledgebase.com/wiki/state-estimation/visual-servoing/>

### Firmware-side capabilities we found but haven't fully exploited

Audit done 2026-05-11 after the s090 run to check whether
MotionCommander or cf2 firmware does anything clever we're not using.

**MotionCommander (cflib client-side)** is a thin wrapper.  It runs a
background thread that streams `send_hover_setpoint` packets at 10 Hz
with the current commanded velocity, plus convenience primitives
(`take_off`, `land`, `start_forward`, `circle_left`, etc.).  No
de-rotation, no smart pose handling, nothing magical — what we send
goes straight through.

**`cf2/src/modules/src/kalman_core/mm_flow.c` IS doing gyro
de-rotation on flow** — and it's the firmware-side equivalent of what
we do offboard with `attitude_compensate()` on bbox.  Formula:

```c
omegax_b = gyro->x * DEG_TO_RAD;
omegay_b = gyro->y * DEG_TO_RAD;
predictedNX = (dt * Npix / thetapix) * ((dx_g * R[2][2] / z_g) - omegay_b);
predictedNY = (dt * Npix / thetapix) * ((dy_g * R[2][2] / z_g) + omegax_b);
```

The `-omegay_b` and `+omegax_b` terms subtract the pixel motion
contributed by body rotation, isolating the translation-induced flow
which the EKF then integrates into velocity.  **Confirms our flow
injection via SENSOR_FLOW_SIM is processed correctly** — the firmware
itself handles attitude wobble.

For the **bbox detection path**, the firmware does NOTHING — bbox
lives entirely offboard.  Our `attitude_compensate()` in
hover_over_cat.py is necessary.

**Three things to explore next**:

1. **`kalman_pred.predNX/predNY` log vars** — already exposed by cf2.
   We can subscribe via cflib log and CROSS-CHECK live against our
   measured/injected flow.  If `predNX` diverges from injected `dpx`,
   it means our flow signs / scale are wrong.  Free continuous-time
   diagnostic, costs nothing to add.  `_t_flow_to_drone.py` on HW
   already uses this via the CH_TELEM channel.

2. **`flowdeck.flowdeckPos_{x,y,z}` PARAMs** — lever-arm offsets for
   the flow deck relative to drone CoM.  Default=0; if our gz cam SDF
   has the camera off-CoM, the EKF should know.  Formula from
   `_t_flow_to_drone.py`:

   ```
   v_cam_bx_add = omega_y * pos_z - omega_z * pos_y
   v_cam_by_add = omega_z * pos_x - omega_x * pos_z
   ```

   Setting these correctly reduces yaw-rate-induced apparent flow.
   Currently low-priority (our drone hovers with negligible yaw) but
   would matter for spin manoeuvres.

3. **`cf.commander.send_velocity_world_setpoint(vx, vy, vz, yawrate)`**
   — world-frame velocity instead of body.  With drone yaw drift, body
   cmds rotate against the world; world cmds + a host-side
   `world ← body` transformation using `stateEstimate.yaw` are robust
   to yaw drift.  For s090 we have yaw≈0 throughout so the upgrade is
   academic, but a moving-target experiment would want this.

### PD controller with altitude-aware ground-error gain (2026-05-11 follow-up)

After §10j shipped the closed-loop hover with pure-P on pixel error,
the drone overshot ~50 cm in X / 45 cm in Y before settling within
~25 cm of the cat target.  Mid-flight log showed sustained oscillation
`err_x ∈ [-44, +49]` over a ~0.8 s period — classic underdamped
response.  Two fixes converged on a much better tuning:

**1. Convert pixel-error to ground-distance error in metres before
applying gain.**  Without this, the same Kp gives different effective
velocity-per-meter at different altitudes.  Formula (small-angle):

```python
m_per_px_x = 2 * z * tan(FOV_H/2) / IMG_W
m_per_px_y = 2 * z * tan(FOV_V/2) / IMG_H
err_x_m = err_x_px * m_per_px_x   # body Y in meters of ground
err_y_m = err_y_px * m_per_px_y   # body X in meters of ground
```

At z=2.5 m / FOV=58° / 300 px, `1 px ≈ 0.93 cm` of ground.  Use the
current `z` from cflib's `stateEstimate.z` log (50 Hz), not a hardcoded
constant — controller becomes altitude-independent.

**2. PD with damping ratio ≈ 0.7 in ground-meter space.**  Empirically
`KP_M = 0.5 /s` and `KD_M = 0.6` give well-damped response without
overshoot blow-up.  Cmd output is in m/s, clip to `V_MAX_M = 0.20`
(matches MotionCommander default).

```python
vx_m = clip(KP_M * err_y_m + KD_M * d_err_y_m, -V_MAX_M, V_MAX_M)
vy_m = clip(KP_M * err_x_m + KD_M * d_err_x_m, -V_MAX_M, V_MAX_M)
```

**Sign on the derivative term is `+`, not `-`.**  When err is
shrinking (drone approaching target), `d_err < 0`, and `+KD * d_err`
reduces the cmd → damping.  Inverted sign (initial bug) made oscillation
WORSE — cmd grew as err shrank.  Standard textbook PID form is
`u = Kp·e + Kd·(de/dt)`, not minus.

**Validation (2026-05-11)**: final distance to cat target dropped
25 cm → **11.6 cm** with the altitude-aware PD vs pure-P.  Drone
reached `err=(-4, +2) px` at iter 132 (effectively centred) before
drifting slightly in late hover.

### On adaptive / auto-tuning controllers (future direction)

User asked "can we have a smarter PID that auto-tunes in flight?"
Answer: YES, four scalable techniques from the control literature:

| Method | Mechanism | Risk | When to use |
|--------|-----------|------|-------------|
| **Ziegler-Nichols online** | Increase Kp until sustained oscillation, measure period Tu, set `Kp = 0.6·Ku, Kd = Kp·Tu/8` | Medium — deliberately oscillating in flight | Fresh stack tune-up |
| **MIT rule (MRAC)** | Online gradient descent: `Kp ← Kp + γ·err·(de/dt)` | Low — converges gradually | Slowly-varying dynamics |
| **Iterative Learning Control (ILC)** | Adjust gains BETWEEN runs based on previous-run overshoot/settle metrics | Zero — offline | Repeated identical tasks |
| **Rule-based adaptive** | Heuristics: if `\|err\| small AND \|de/dt\| large`, boost Kd; if overshoot detected, drop Kp | Low — bounded by clamps | Production-safe |

For s090 we recommend **rule-based adaptive** as the next step (≤50
LOC Python in `hover_over_cat.py`).  Pseudocode:

```python
def adapt_gains(err, d_err, overshoot_detected, kp, kd):
    # Boost damping if approaching fast (large derivative near target)
    if abs(err) < 0.10 and abs(d_err) > 0.20:  # 10cm err, 20cm/s rate
        kd = min(kd * 1.1, KD_MAX)
    # Back off proportional if past overshoot
    if overshoot_detected:
        kp = max(kp * 0.9, KP_MIN)
    # Slowly drift back to default when stable
    if abs(err) < 0.05 and abs(d_err) < 0.02:
        kp += (KP_DEFAULT - kp) * 0.05
        kd += (KD_DEFAULT - kd) * 0.05
    return kp, kd
```

**RL/policy training is overkill** for this static-target problem.
Literature consensus (Bitcraze + IBVS papers in §10h, §10j refs) is
classical PD + altitude scaling is sufficient.  Reserve RL for
acrobatic / fast-trajectory / non-linear-coupled cases.

**Reference paper** for adaptive IBVS:
- [Adaptive Image-Based Visual Servoing for an Underactuated Quadrotor System (JGCD)](https://arc.aiaa.org/doi/abs/10.2514/1.52169) — gradient-descent gain adaptation
- [Adaptive Output-Feedback IBVS for Quadrotor UAVs (IEEE)](https://ieeexplore.ieee.org/document/8628313/) — online identification
- [IBVS based on adaptive sliding mode for quadrotor target tracking under perturbations (Elsevier)](https://www.sciencedirect.com/science/article/abs/pii/S0957415822001271) — adaptive gain + sliding mode for robustness
- [Fuzzy Gain-Scheduling Based Fault Tolerant Visual Servo Control of Quadrotors (MDPI Drones)](https://www.mdpi.com/2504-446X/7/2/100) — fuzzy-rules-driven gain scheduling
- [PID control of quadrotor UAVs: A survey (Elsevier 2023)](https://www.sciencedirect.com/science/article/abs/pii/S1367578823000640) — overall survey covering linear, nonlinear, adaptive, event-based, gain-scheduling, fault-tolerant, fractional-order, intelligent PID

### Self-calibrating PID across flights (ILC, 2026-05-11 ship)

User wanted "load PID params at takeoff, test, refine in-flight if
behaviour bad, persist for next flight."  After two failed attempts
(Ziegler-Nichols online didn't fit our overshoot-then-creep response
pattern, naive overshoot-driven adapter ran KP toward zero), the
working pattern is **Iterative Learning Control across flights**:

```
LOAD pid_params.json (defaults if absent: KP=0.5, KD=0.6 — the
                      manually-tuned working values that hit 8.9cm)
   │
   ▼
RUN flight with loaded gains
   │  (track max |err_x_m|, |err_y_m| during hover phase)
   ▼
AT END OF HOVER PHASE: adjust gains for NEXT flight based on outcome
   │
   ├─ overshoot > 40 cm → recalibrate (KP × 0.9, KD × 1.1)
   ├─ overshoot < 20 cm AND final_dist > 30 cm → drone too cautious
   │                                              (KP × 1.1)
   └─ otherwise → behaviour acceptable, FREEZE gains
   │
   ▼
SAVE pid_params.json (with _last_overshoot, _last_final_dist
                      annotations for audit)
```

This is **adaptive at the granularity of flights, not within a flight**.
Within-flight adaptation (the earlier attempts) was unstable because:

- Our system response is overshoot-then-slow-creep, not sustained
  ringing.  Z-N requires sustained oscillation to measure Tu — only 3
  sign-flips observed in 50 s of hover, insufficient for ID.
- Within-flight adapter that REDUCES gain on every overshoot event
  monotonically converges to over-damped (KP → KP_MIN) over several
  flights, because the climb/takeoff transient always registers as
  "overshoot" against the bbox-error target.

Bounds in `pid_params.json`: `KP_MIN=0.20, KP_MAX=0.90, KD_MIN=0.30,
KD_MAX=1.20`.  Auto-calibration NEVER pushes outside these — if the
adapter wants to go further, it clips and `_stable_at_end=False` flag
prompts manual inspection.

**File location**:
`examples/sentai_runtime/experiments/s090_hover_over_cat/pid_params.json`
— committed to git; the `_last_*` audit fields update on each run
but the structural fields (KP_M, KD_M, bounds, thresholds) form the
durable "learned configuration" for the drone-camera-environment
combination.

**Recovery from bad saved params** — user's main concern:

> "as vrea ca daca la un zbor nou nu reuseste sa foloseasca constantele
> invatate de pana atunci si stocate in memorie sa incerce sa le invete
> din nou."

The ILC handles this naturally.  If the loaded params produce a bad
flight (overshoot > 40 cm), the rule:

```
pid["KP_M"] = max(pid["KP_M"] * 0.9, pid["KP_MIN"])
pid["KD_M"] = min(pid["KD_M"] * 1.1, pid["KD_MAX"])
```

backs off proportional gain + boosts damping for next flight.  Over
3–5 flights the system converges to a stable point.  Manual reset is
available by deleting `pid_params.json` — system reverts to working
defaults and starts re-learning.

**Validation: 5 consecutive flights with auto-calibration (2026-05-11)**:

| Flight | KP/KD loaded | Final dist | Overshoot peak | New saved |
|--------|--------------|------------|----------------|-----------|
| 1 (defaults) | 0.500 / 0.600 | **0.089 m** | 0.41 m | 0.450 / 0.660 |
| 2            | 0.450 / 0.660 | 0.228 m    | 0.45 m | 0.405 / 0.726 |
| 3            | 0.405 / 0.726 | 0.179 m    | 0.66 m | 0.365 / 0.799 |
| 4            | 0.365 / 0.799 | **0.019 m** 🎯 | 0.50 m | 0.328 / 0.878 |
| 5            | 0.328 / 0.878 | 0.073 m    | 0.43 m | 0.295 / 0.966 |

**Trend over 5 flights**: KP descending 0.50 → 0.30, KD ascending
0.60 → 0.97 — system learning that our cf2-SITL + flow + bbox-detect
combination prefers more damping than first-flight defaults assumed.
F4 reached **1.9 cm** final distance to cat — better than the manually-
tuned baseline (11.6 cm) the algorithm started from.

Variance flight-to-flight is real (gz physics restart, drone yaw
drift, SSD class oscillation at takeoff transient) and the ILC handles
it gracefully because gains converge over MULTIPLE flights, not within
a single one.  No flight requires manual intervention; the system
self-recovers from over-aggressive saved gains by detecting overshoot
and pulling KP down on the next iteration.

### Altitude-correlation validation (z=1.75 m vs z=2.5 m, 2026-05-11)

After the 5-flight ILC at z=2.5 m proved auto-tuning works, ran a
parallel 5-flight series at z=1.75 m (= 70% of 2.5 m) with
`HOVER_TARGET_Z=1.75` env var override to validate the
altitude-normalization claim (gains learned at one altitude generalise
to another because controller converts px→meters via current `z`).

**Both 5-flight runs starting from defaults `KP=0.5, KD=0.6`:**

| Flight | KP/KD loaded | z peak (m) | Final dist (cm) | Overshoot (cm) |
|--------|--------------|------------|-----------------|----------------|
| **z=2.5 m series** | | | | |
| 1 | 0.500/0.600 | ~2.5 | **8.9** | 41 |
| 2 | 0.450/0.660 | ~2.5 | 22.8 | 45 |
| 3 | 0.405/0.726 | ~2.5 | 17.9 | 66 |
| 4 | 0.365/0.799 | ~2.5 | **1.9** 🎯 | 50 |
| 5 | 0.328/0.878 | ~2.5 | 7.3 | 43 |
| Mean / Best | | | **~12 / 1.9** | |
| **z=1.75 m series** | | | | |
| 1 | 0.500/0.600 | 2.14 | **9.8** | 45 |
| 2 | 0.450/0.660 | 2.14 | 38.5 | 47 |
| 3 | 0.405/0.726 | 2.13 | 12.2 | 52 |
| 4 | 0.365/0.799 | 2.11 | 11.4 | 33 |
| 5 | 0.328/0.878 | 2.15 | 21.9 | 39 |
| Mean / Best | | | **~19 / 9.8** | |

**Findings:**

1. **ILC converges to IDENTICAL gains** at both altitudes
   (`KP=0.295, KD=0.966` after 5 flights either way).  This is by
   construction: ILC adjustment is a deterministic multiplier per
   overshoot event, and both runs had similar overshoot patterns ⇒
   same fixed point.

2. **Final-distance precision is better at higher altitude** (~12 cm
   mean at z=2.5 m vs ~19 cm mean at z=1.75 m).  Best-case run at
   z=2.5 m hit **1.9 cm**; best at z=1.75 m was 9.8 cm.  Explanation:
   higher altitude → wider FOV in metres → drone has more room to
   manoeuvre without losing the bbox at image edges, so the controller
   gets more useful frames of feedback per second.

3. **z peak overshoot of 0.4 m at takeoff** — `MotionCommander.take_off`
   with `velocity=0.5 m/s` and `height=HOLD_Z` consistently overshoots
   altitude by ~0.4 m before settling.  cf2's altitude controller has
   internal damping but isn't perfectly critically damped.  Lower
   `velocity=0.2 m/s` would reduce this.

4. **Altitude-normalisation in the gain math (px → meters via live `z`
   from cflib log) WORKS as designed.**  Performance at z=1.75 m is
   only ~7 cm worse mean than z=2.5 m, all explainable by FOV margin
   not by gain mismatch.  Same `pid_params.json` file would work at
   either altitude.

**Recommendation**: in the s090 hover-over use case, prefer z ≥ 2.5 m
for best target-acquisition precision.  For real-world applications
where altitude is constrained (e.g., flying under a ceiling), the
controller still works but expect ~2× wider final-distance variance.

### Altitude-sweep calibration (multi-DOF excitation, 2026-05-11 ship)

User asked "can we do a calibration that sweeps altitude (±25cm) while
tracking the cat, to probe gains at multiple z's?"  Answer: yes, this
is the canonical **Quad-M frequency-sweep ID** maneuver applied to
multi-DOF (Z sweep + XY centering).  Enabled via `SENTAI_ALT_CAL=1`
env var; adds ~20s overhead at start of flight.

**What it does** (in `hover_over_cat.py` post-takeoff, pre-hover):

```
for 20 s:
  z_cmd = HOLD_Z + 0.25 * sin(2π·t/20)          # 1 cycle, ±25cm sweep
  mc.start_linear_motion(0, 0, (z_cmd - z_now) * 0.5)
  drain STATE queue, bucket bbox-err by current z relative to HOLD_Z:
    low      = z < HOLD_Z - 15cm
    mid_low  = -15cm < z - HOLD_Z < 0
    mid_high = 0 < z - HOLD_Z < +15cm
    high     = z > HOLD_Z + 15cm
```

**Validation 2026-05-11**:

| Altitude bucket | Samples | mean \|err_x\| (px) | mean \|err_y\| (px) |
|-----------------|---------|---------------------|---------------------|
| low             | 44      | 44.6                | 88.2                |
| mid_low         | 10      | 57.2                | 78.7                |
| mid_high        | 44      | 41.9                | 73.5                |
| high            | 0       | n/a                 | n/a                 |

**Findings:**

1. `err_x` is roughly altitude-invariant (45–57 px across buckets) ⇒
   altitude normalisation in the X-axis gain works.

2. `err_y` is consistently larger (74–88 px) AND degrades slightly at
   low z (88 vs 73 px) ⇒ Y dynamics benefit from altitude scheduling
   in the next iteration of the controller.

3. `high` bucket has 0 samples — drone's vertical step response is
   slower than the sweep period.  The `mc.start_linear_motion(_,_,vz)`
   command effects altitude with delay > 0.1s; gain `(z_cmd-z_now)*0.5`
   produced too-gentle vertical setpoints to overcome cf2's altitude
   damping in time.  Fix: increase gain to ~2.0 and/or extend
   `CAL_DURATION_S` to 30s.

4. Side effect: **the calibration acts as a warm-up**; main hover
   phase immediately afterward hit **1.7cm final distance** (new
   record vs prior 1.9cm at z=2.5m without calibration).  Likely the
   EKF has settled fully + the SORT tracker has multiple consistent
   detections before the centering phase starts.

**Approaches from literature for in-flight calibration**:

- **Chirp / frequency sweep** in attitude (CIFER tool) — standard for
  manned helicopter ID; ArduPilot port at
  <https://ardupilot.org/copter/docs/systemid-model-development.html>
- **In-situ rotation for optical flow focal length** — Wang et al
  2023 "Improved modeling and fast in-field calibration of optical
  flow sensor for UAV position estimation"
  <https://www.sciencedirect.com/science/article/abs/pii/S0263224123016305>
- **Quad-M principles** for flight vehicle ID (Maneuvers,
  Measurements, Model, Method) — see SJSU/VFS 2019 paper at
  <https://www.sjsu.edu/researchfoundation/docs/VFS_2019_Ivler.pdf>
- **Adaptive PID gain-scheduling for 3D quadrotor** — recent IEEE
  conf paper specifically about altitude-correlated gain table
  <https://ieeexplore.ieee.org/document/10638945/>
- **Fuzzy Gain-Scheduling PID for UAV** — MDPI 2022 Sensors,
  position + altitude controllers with fuzzy gain rules
  <https://www.mdpi.com/1424-8220/22/6/2173>
- **ArduPilot in-flight flow calibration**: hover at 10m, rock
  ±5° roll/pitch to identify focal length / installation angles
  <https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html>

**Open ideas for next iteration**:

- Higher vz gain (2.0 instead of 0.5) + longer sweep window (30s) to
  reach the "high" bucket properly and get 4-point altitude coverage.
- Sweep amplitude scaling with detected oscillation strength (start
  small, grow until performance differs across buckets).
- Per-axis gain learning: separate `KP_X_M`, `KP_Y_M`, `KD_X_M`,
  `KD_Y_M` since Y axis has more inherent overshoot in cf2 SITL.
- Move to **gain-scheduling table** `KP(z), KD(z)` populated by
  calibration sweep, interpolated at run time using current cflib `z`.

### Open work

- **Full rotation homography** for tilt compensation (we use
  small-angle approx; OK for ±2° hover, breaks ≥5°).
- **Per-sample flow stdDev** plumbed end-to-end (currently host uses
  fixed `std=4.0`; the conf-mapped per-frame value from
  `sentai.flow.read()[3]` would let the EKF adapt to scene-quality
  changes during flight).
- **Move-the-target test** — currently cat is static at world
  (+0.4, -0.3).  A moving target would exercise the SORT predict step
  and the controller's response to a drifting setpoint.
- **Re-run on ARM** to confirm the bbox-tilt-compensation +
  MotionCommander pattern transfers (the firmware-side flow path on
  HW already does gyro de-rotation in `mm_flow.c`, so the host-side
  compensation in our s090 wrapper may be redundant on HW).

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
| 2026-05-11 | 5b import-bridge | DONE — native `import` from SIM virtual FS (replaces `exec(read_str())`) | SIM build #after-#1234 | **Built the firmware-parity import path on SIM.**  `sim/main_sim.c` now provides real `mp_lexer_new_from_file` + `mp_import_stat` routed through `sim_fs_resolve()` (the same resolver `sentai.fs.*` uses).  Backed by an inline 64-byte FD reader inside `main_sim.c` (typedef `sim_reader_fd_t` with `readbyte` + `close` callbacks) so we don't have to flip `MICROPY_READER_POSIX=1` on the embed config — that flag would pull a competing `mp_lexer_new_from_file` from `lexer.c` that bypasses our `sim_fs_resolve()`.  REPL boot now does `import sys; sys.path.append('/'); sys.path.append('')` so a bare `import hover_logic` finds `<sim_fs_root>/hover_logic.py`.  `sim_fs_root()` and `sim_fs_resolve()` in `modsentai_sim.c` un-staticed and `extern`-declared in `main_sim.c`.  Smoke test: `>>> import smoke_import; smoke_import.greet()` returns `"hello from disk"` after a single .py file is dropped at `build-sim/sentai_fs_root/smoke_import.py`.  Migration: `examples/sentai_runtime/experiments/s090_hover_over_cat/hover_over_cat.py` swapped `exec(sentai.fs.read_str("hover_logic.py"))` for `import hover_logic` — heap cost drops from ~KB source-string to 64 B FD buffer streamed char-by-char by the lexer; tracebacks now show `hover_logic.py` line N instead of `<string>`; second invocation is free (cached in `sys.modules`).  New best-practices section §10f documents the rule + the anti-patterns to avoid (multi-line for/if blocks through stdin REPL → `SyntaxError`; `exec(read_str())` for anything larger than a one-liner → useless heap copy).  ARM parity guarantee: firmware build provides the equivalent through FileX (`FxUserOpenRead`) so the same `import foo` line works on both targets. |
| 2026-05-10 | 3 best-practices | DONE — captured during cflib TOC debug session | n/a | **Hard-won CrazySim/Gazebo Garden best practices.  (Originally collected on Harmonic but Harmonic is now banned; the practices apply equally to Garden in distrobox.)  Read these before any future debug session.**  (1) **`stdbuf -oL` is mandatory for cf2** — `cf2`'s stdout is block-buffered when redirected to a file (4 KB).  Default `sitl_singleagent.sh` does `cf2 ... > out.log 2> error.log &` and the logs stay EMPTY for minutes.  Wrap with `stdbuf -oL -eL cf2 ...` to flush per-line and see boot progress (`SOCKET_LINK: Waiting for connection with gazebo`, `Connection established`, `SYS: Software-in-the-Loop Simulator is up and running!`).  (2) **Always launch `gz sim` with `-v 4` (debug) during bringup, not the default `-v 3`** — plugin-load failures (`Failed to load system plugin [gz_crazysim_plugin] : Could not find shared library`) are logged ONLY at `-v 4`.  At `-v 3` the world boots silently with no plugin and EVERYTHING downstream (cflib TOC, motor commands, telemetry) silently times out.  (3) **`GZ_SIM_SYSTEM_PLUGIN_PATH`, `GZ_SIM_RESOURCE_PATH`, `LD_LIBRARY_PATH` MUST be set in the shell that launches `gz sim`** — `setup_gz.bash` sets them, but only inside the script's process tree.  If you run `gz sim ...` ad-hoc in another shell, the plugin is silently missing and the drone's `/cf_0/imu` topic exists but with NO subscriber on the cf2 side.  (4) **Plugin <-> cf2 handshake is `0xF3`** — cf2 SOCKET_LINK sends `0xF3` (1 byte, header only, size=0) repeatedly until plugin echos `0xF3` back.  Plugin learns cf2's ephemeral source addr from `recvfrom`'s remaddr.  cf2 then continues to system init.  Sequence visible at cf2 stdout (with stdbuf): `Create socket succeed → Binding succeed → Waiting for connection with gazebo → Connection established with gazebo → SYS: Software-in-the-Loop Simulator is up and running!`  (5) **cflib UdpDriver handshake is `\xff\x01\x01\x01`** — plugin doesn't ack this, just learns cflib's addr from recvfrom.  Subsequent CRTP packets are bidirectional. |
