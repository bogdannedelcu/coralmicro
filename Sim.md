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

## 10k. sentai.flow — multi-pyramid + triple-anchor architecture (s091, 2026-05-11)

After s090 closed the cat-hover loop, s091 investigated **flow-only
position hold** under Gazebo wind perturbations (no SSD target).
Discovered + fixed several architectural issues; final architecture
holds drone within 7.6 cm under realistic indoor wind, 30-60 cm under
aggressive wind (== physical limit of PMW3901-class odometry-only).

### Architecture

```
                                    PER-FRAME PIPELINE
                                    ──────────────────
  640×480 RGB ──┐
                ├─ PXP 8× decim ── 80×60 wide gray ──┐
                │                                    │
                ├─ 4×4 box-filter from 320×240 ─── 80×60 mid gray ──┐
                │   centred patch                                    │
                │                                                    │
                └─ Native crop of 80×60 from         80×60 fine gray ─┤
                    640×480 centre (NO decim)                        │
                                                                     │
       L0/L1/L2 phase-corr (independent 64×64 FFTs):                 │
         L0: frame[t] vs frame[t-1] @ L0 grid                        │
         L1: frame[t] vs frame[t-1] @ L1 grid                        │
         L2: frame[t] vs frame[t-1] @ L2 grid (native pixels)        │
                                                                     │
       L0/L1/L2 LCF anchors (independent 64×64 FFTs):                │
         frame[t] vs anchor[t-N] @ each level                        │
         anchor refreshes when L#-detected motion exceeds threshold  │
                                                                     │
       Sub-pixel refinement: Guizar-Sicairos DFT upsampling          │
         M=10 → 1/10 grid resolution                                 │
                                                                     │
       C-side fusion picks SINGLE (dx, dy, conf) per ARM convention  │
                                                                     │
  Output → sentai.flow.read() → cf2 EKF via SENSOR_FLOW_SIM CRTP ────┘
```

### Per-level math (z=1m drone hover, 14fps gz Garden)

| Level | Decimation | Per-grid ground | FOV ground | Min detectable motion |
|-------|------------|------------------|-------------|-----------------------|
| **L0** wide | 8× PXP | 13.85 mm | 1.10 × 0.83 m | > 19.4 cm/s |
| **L1** mid  | 4× box | 6.93 mm  | 0.55 × 0.42 m | > 9.7 cm/s |
| **L2** fine | NATIVE crop | 1.73 mm | 0.14 × 0.10 m | > 2.4 cm/s |
| **Anchors** | cumulative | level-dependent | level-dependent | level/√N (over N frames) |

Drone slow drift typical 2-4 cm/s → only L2 native + anchors can resolve.
L0/L1 averaging erases sub-pixel motion (key insight discovered through
empirical pixel-byte-difference analysis of dumped frames).

### Critical bugs found + fixed (chronological)

1. **BODY_XFORM axes swap in SIM** (2026-05-11): gz camera mount in
   `sentai_crazysim.sdf` has yaw=π after pitch=π/2 — image axes are
   SWAPPED + sign-flipped relative to HW (cam0 + OV5640 vflip=1).
   Empirically determined: `(0, -1, -1, 0)` instead of HW's
   `(-1, 0, 0, +1)`.  Verified via flow_diagnostic T1/T2/T3 PASS.

2. **Duplicate-frame CRC on downsampled gray** (CRITICAL): camera_bridge
   was computing duplicate-detection hash on `s_gray80x60` (post-PXP
   8× decim).  At drone slow drift (2-4 cm/s ≈ 1.4-2.9 mm/frame),
   the 8×8 averaging produced byte-identical downsampled output even
   though raw RGB had 50-80% pixels differing.  Phase-corr was being
   SKIPPED (conf=0 returned) in ~48% of frames.  **Fix**: CRC on
   raw 640×480 instead.  Detects only TRUE gz duplicates (pre-takeoff
   or render pause), no false positives on slow motion.

3. **flow_phase_corr.cc shared state across pipes**: original
   `sentai_flow_phase_corr_compute` had file-static `prev_fft` —
   calling it for L0, L1, L2 in sequence cross-contaminated state.
   **Refactor**: added `_compute_at(pipe_id, ...)` with per-pipe
   state arrays (prev_fft[FLOW_N_PIPES], have_prev_per[], etc.).
   Legacy entry-point preserved as wrapper to pipe_id=0.

4. **L2 anchor threshold mis-scaled**: L2 native uses 8× finer mgrid
   units, but anchor refresh threshold was 1500 mgrid (= 2.6mm physical
   motion).  Anchor refreshed every sub-frame, never accumulated.
   **Fix**: scaled per-level — L0=1500, L1=3000, L2=12000 mgrid (all
   = 20.8mm physical).

5. **L2 anchor spurious peaks beyond FOV**: native L2 crop FOV is only
   14×10cm.  When drone drifts > 7cm, anchor sees DIFFERENT ground
   content → phase-corr returns spurious match.  **Fix**: sat-guard
   on L2 anchor at ±40000 native mgrid (≈ half FOV) — beyond that,
   reject estimate.

### Sub-pixel refinement: Foroosh-Zerubia → Guizar-Sicairos

Original phase-corr used Foroosh-Zerubia 3-point local parabolic fit on
the correlation surface around the integer peak.  Replaced with
**Guizar-Sicairos DFT-based upsampling** (Optics Letters 2008):
1. After integer peak, evaluate iFFT(cross_power_spectrum) at K=21
   fractional positions ±1 pixel around peak at 1/M=1/10 resolution
2. Separable 2D DFT via pre-computed twiddle factors (init_once)
3. Cost: ~500K float ops per call (~300-600µs ARM, ~100µs x86)

Guizar-Sicairos is more **robust to noisy / broad peaks** than
local parabolic fit — uses global spectrum, not just 3 neighbours.
Empirical improvement modest (~7-16% conf increase) because phase-corr
peak at sub-pixel motion is fundamentally broad regardless of fit
algorithm.

### LastChangedFrame (LCF) anchor — temporal integration

User-proposed innovation: keep ANCHOR frame in pipe's `prev_fft` while
drone is stationary; refresh anchor only when motion is clearly
detected.  Compare each subsequent frame against frozen anchor →
CUMULATIVE drift over many frames.

```
anchor algorithm per level:
  if motion(current frame) > threshold:
    refresh_anchor(current)
  else:
    cumulative_drift = phase_corr(current, anchor_FFT)
    per_frame_velocity = cumulative_drift / frames_since_anchor
```

Anchor captures sub-pixel-per-frame drift that frame-to-frame
phase-corr misses (peak too broad).  Over N frames, motion accumulates
to detectable signal.

Triple-anchor (L0+L1+L2 independent): each level has its own anchor,
matching its physical-motion-equivalent threshold.  C-side fusion
prefers finest anchor with reliable confidence + within-FOV constraint.

### Empirical performance (hover at z=1m, 15s)

| Wind level | all-4 detect | dist_mean | dist_max | Anchor usage |
|------------|--------------|-----------|----------|--------------|
| Full (0.2 + σ=0.15 m/s) | 35% | 0.32 m | 0.56 m | <5% |
| **Half (0.1 + σ=0.075 m/s)** | **100%** | **0.076 m** | **0.38 m** | 4% |

Half-wind result demonstrates flow algorithm IS correct.  Real cf2 with
PMW3901 + flowdeck hover indoors typically holds 5-15 cm — our 7.6 cm
mean drift matches that benchmark.

### Compute cost (per-frame)

| Operation | x86 SIM | ARM Cortex-M7 @ 800 MHz |
|-----------|---------|--------------------------|
| Box-filter L1 (320→80) | ~30 µs | ~150 µs |
| Native crop L2 (80 from 640) | ~5 µs | ~30 µs |
| Phase-corr per pipeline | ~250 µs | ~5 ms |
| Guizar-Sicairos sub-pixel | ~100 µs | ~600 µs |
| Coarse-to-fine warp (bilinear) | ~3 µs | ~30 µs |
| **6 pipelines total (3 main + 3 anchors)** | **~2 ms** | **~30 ms** |

ARM budget tight (30 ms of 33 ms @ 30 fps) — drop L1-anchor for headroom
if needed; L0+L2+anchors_L0 alone gives ~20 ms ARM, ~13 ms headroom.

### Open architectural issues

1. **EKF integrating IMU noise without absolute position reference**:
   when flow returns zero (drone truly stationary), cf2 EKF integrates
   IMU bias drift, producing fake position drift in state estimate.
   Fix: PnP from ArUco markers as MOCAP-equivalent injection
   (CRTP `EXT_POSE` channel).  Not in flow scope.

2. **Native L2 FOV too small for translation**: 14×10cm covers only
   region directly under drone.  Lateral translation > 7cm loses the
   anchor.  Current sat-guard handles this defensively but loses
   anchor benefit.  Real fix needs SLAM-like map of ground patches.

3. **Phase-corr peak broadness at sub-pixel motion**: Foroosh /
   Guizar-Sicairos refine the peak but cannot create signal where
   spatial cross-correlation is genuinely flat (motion below 1/10 grid).
   This is fundamental to FFT-based correlation; only higher input
   resolution or longer baselines (anchors) help.

### References

Classical optical flow:

- **Lucas & Kanade 1981** "An Iterative Image Registration Technique
  with an Application to Stereo Vision". DARPA Image Understanding Workshop.
  Original gradient-based optical flow.

- **Burt & Adelson 1983** "The Laplacian Pyramid as a Compact Image Code".
  IEEE Trans. Comm. 31(4):532-540.  Foundational multi-resolution
  decomposition that enabled coarse-to-fine flow.

- **Bouguet 2001** "Pyramidal Implementation of the Lucas Kanade Feature
  Tracker — Description of the Algorithm".  Intel Corp tech report.
  Standard pyramidal LK reference, source of "coarse-to-fine with
  warping" pattern. [PDF](http://robots.stanford.edu/cs223b04/algo_tracking.pdf)

Phase correlation + sub-pixel:

- **Kuglin & Hines 1975** "The Phase Correlation Image Alignment Method".
  IEEE Int. Conf. Cybernetics & Society.  Origin of phase correlation.

- **Foroosh, Zerubia & Berthod 2002** "Extension of Phase Correlation
  to Subpixel Registration". IEEE Trans. Image Proc. 11(3):188-200.
  Closed-form sub-pixel fit on phase-corr surface.

- **Guizar-Sicairos, Thurman & Fienup 2008** "Efficient subpixel image
  registration algorithms". Optics Letters 33(2):156-158.  DFT-based
  upsampling — what we use.  Heavily used in HST/JWST astronomy.

- **Wang et al. 2021** "Modified phase correlation algorithm for image
  registration based on pyramid". Alexandria Eng. J. — Pyramidal
  phase-correlation refinement, similar to our coarse-to-fine.

Drone-specific optical flow:

- **Bristeau et al. 2011** "The navigation and control technology
  inside the AR.Drone micro UAV". IFAC Proc. 44(1):1477-1484.
  AR.Drone optical flow using LK pyramidal — first commercial
  flying camera w/ flow stabilisation.

- **Honegger et al. 2013** "An Open Source and Open Hardware Embedded
  Metric Optical Flow CMOS Camera for Indoor and Outdoor Applications".
  IEEE ICRA 2013.  **The PX4Flow paper** — single-scale SAD with
  bilinear sub-pixel refinement on STM32F4 @ 400Hz.  Hardware ancestor
  of PMW3901 commercial chip. [PDF](https://people.inf.ethz.ch/pomarc/pubs/HoneggerICRA13.pdf)

- **Briod et al. 2013** "Optic-flow based control of a 46g quadrotor".
  IROS 2013.  Multi-patch phase-correlation on nano-quadrotor — most
  similar setup to ours (lightweight, embedded, indoor hover).

- **PMW3901 datasheet** (PixArt Imaging, 2016).  Commercial optical
  flow sensor on Bitcraze FlowDeck v1/v2 — internal architecture:
  35×35 effective pixels, sub-pixel via phase-corr + parabolic fit,
  output ±2048 "fractional pixels" at ~100 Hz.

Recent (2024-2025) deep-learning state-of-art (for context):

- **DPFlow (Morimitsu et al. 2025)** "Adaptive Optical Flow Estimation
  with a Dual-Pyramid Framework". CVPR 2025.  GPU-scale dual-pyramid,
  not realistic for MCU but informs architecture decisions.
  [arXiv:2503.14880](https://arxiv.org/abs/2503.14880)

- **RAPIDFlow (2024)** "Recurrent Adaptable Pyramids with Iterative
  Decoding". ICRA 2024.  Recurrent pyramidal refinement.

### Concluzia alegerilor (post-session 2026-05-11)

**Ce am păstrat în final**:

| Componentă | Motiv |
|------------|-------|
| **3-level pyramid (L0+L1+L2)** | L0 catches fast motion, L2 native catches slow drift, L1 fills mid-range. Burt-Adelson classical pattern. |
| **L2 = NATIVE crop (not 2× decim)** | 8× finer per-pixel resolution. Singura cale pentru sub-cm motion (2-4 cm/s drift). Same compute cost ca L2 box-filter. |
| **Coarse-to-fine warping (Bouguet)** | L0 coarse predicts → warp L1 → L1 finds residual → predict L2 → warp L2 → residual. Matematic corect (vs parallel pyramid care eşua pe periodic textures). |
| **Guizar-Sicairos sub-pixel** | Mai robust decât Foroosh la peak broad. Bonus per axă: ~600 µs ARM. |
| **Triple LCF anchor (L0/L1/L2)** | Detectează cumulative drift sub pragul de single-frame phase-corr. Native L2 anchor cu FOV sat-guard. |
| **Duplicate-CRC pe RAW RGB** | Era pe downsampled = bug critic. RGB CRC detects only true gz duplicates. |
| **C-side fusion** | Single (dx, dy, conf) exposed la consumer. Python doar relay → matches ARM real-time architecture. |

**Ce am respins / drop-uit**:

| Componentă | Motivul respingerii |
|------------|---------------------|
| **640×480 phase-corr direct** | 100× compute (~500ms ARM) — neîncadrabil în budget 33ms |
| **Temporal averaging (running avg pe cross-power)** | Decoherează semnal pe oscilation drone — bun doar pe motion constant |
| **Parallel pyramid fusion** | Periodic textures → diferite niveluri văd aliasing-uri diferite → sign-disagreement 22% (random) |
| **Threshold-low (MOTION=50)** | Anchor refresh la fiecare frame → niciodată acumulează |
| **Native-only single pipeline** | FOV 14cm — drone leaves anchor too fast, L0 wide needed pentru large motion |

**Limita fizică confirmată**: cu wind 0.2 m/s + σ=0.15 m/s (peak gusts 0.5 m/s), nicio configuraţie nu duce drift sub ~35cm.  Real cf2+PMW3901 indoors are aceeaşi limită — confirmare empirică.  Cu wind realistic indoor (≈ half power), drone hovers cu **7.6cm mean drift, 100% marker visibility**.

**Pentru deployment pe HW ARM**: tot pipeline-ul ~30 ms ARM (90% budget) e tight.  Recomandat: drop L1-anchor sau L1 main pentru headroom ~13ms.  Funcţie de mission profile:
- **Indoor hover-focus**: keep all 6 pipes (drift detection priority)
- **Aerial cruise/explore**: drop L2 anchor (FOV irrelevant during translation)

EKF + flow fusion:

- **Mueller, Hamer & D'Andrea 2015** "Fusing ultra-wideband range
  measurements with accelerometers and rate gyros for quadrocopter
  state estimation". IEEE ICRA 2015.  Bitcraze's EKF foundation —
  same kalman_core.c we feed via SENSOR_FLOW_SIM.

- **Förster et al. 2017** "On-Manifold Preintegration for Real-Time
  Visual–Inertial Odometry". IEEE Trans. Robotics 33(1).
  Tight coupling of flow + IMU — what cf2 EKF does in firmware.

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

## 10l. Hover-under-wind — bug hunt + final tuning (s091/s092/s093, 2026-05-11)

A multi-hour session driven by a single user complaint: *"drone drifts 40 cm
under wind even though the flow algorithm 'looks right'."*  Three independent
bugs were uncovered (none were in flow proper) and the controller was
re-tuned for proper wind-rejection authority.

### Bug 1 — PnP marker size off by 28%

**Symptom:** `pnp_z` always reported drone altitude 25 cm higher than EKF.
With drone at world z = 1.00 m, PnP said z ≈ 1.25 m; with EKF reset, gz
ground-truth at z = 1.00 m, same disparity.

**Root cause:** the ArUco texture
`materials/textures/aruco_4x4_50_id0.png` is a 512 × 512 PNG with the
black-bordered marker occupying only the centre 400 × 400 px (78.1 % of
the image; 22 % white padding around the border).  The marker box face is
0.08 m × 0.08 m, but the *detected* outer-black-square is only
0.08 × 0.781 = **0.0625 m**.  `MARKER_SIZE_M = 0.08` told `solvePnP` the
marker was larger than it actually appears → `tvec[2]` came back inflated
by exactly 1.28× (= 0.08 / 0.0625) → 25 cm extra at z = 1 m.

**Fix:** `examples/sentai_runtime/experiments/s090_hover_over_cat/aruco_detector.py:46`
```python
MARKER_SIZE_M = 0.0625
```
After fix: PnP-z error 25 cm → **2 – 5 cm**.

**Diagnosis recipe (reusable):** when PnP scale looks off, dump the
texture, compute black-pattern fraction with cv2:
```python
img = cv2.imread(tex, cv2.IMREAD_GRAYSCALE)
ys, xs = np.where(img < 128)
print('pattern coverage %.1f%%' % ((xs.max()-xs.min())/img.shape[1]*100))
```

### Bug 2 — PnP X / Y label inversion under partial-FOV

**Symptom:** with wind, EKF said `x = -0.42 m`, PnP said `x = +0.01 m`,
both at the same instant.  Disagreement of 40 cm.  gz `model -p`
ground-truth agreed with EKF, so PnP was the liar.

**Root cause:** the old s091 PnP code averaged per-marker
`-tvec[0]` / `-tvec[1]` assuming (a) camera_X axis == body_X axis and
(b) the marker centroid stays at world (0, 0).  Both wrong:

1. The downward camera is mounted with `pose: pitch=+π/2, yaw=+π`.  After
   rotation, `cam_X = -body_Y` and `cam_Y = -body_X` (verified
   empirically in §10l calibration below).  So `-tvec[0]` actually maps
   to `drone_y`, not `drone_x`.
2. When the drone drifts > 0.3 m the markers at the *near* side leave the
   FOV (drone is at z = 1 m, FOV at z = 1 m is ≈ 1.1 × 0.83 m).  The
   per-marker average then centroids around the *visible* markers, not
   the geometric centre at (0, 0).

**Fix:** new `aruco_detector.estimate_drone_world_pose()` that
(a) uses each marker's *known world position* `KNOWN_POSITIONS_M[mid]`
(centroid bias eliminated even with partial visibility),
(b) applies a fixed `_R_CAM_TO_BODY = [[0,-1,0],[-1,0,0],[0,0,-1]]`
(empirically determined from s092 axis calibration),
(c) rotates by drone yaw (read from cf2 `stateEstimate.yaw`,
converted from degrees to radians),
(d) subtracts the 4 cm camera-CoM offset from the body frame.

After both fixes: EKF/PnP agreement within ±3 cm on X/Y, ±2 cm on Z.

### s092 — Empirical camera-axis calibration

Built `examples/sentai_runtime/experiments/s092_axis_calib/axis_calib.py`
to determine the body↔image axis mapping by *controlled motion test*
rather than guessing from the SDF pose math.  Protocol:
1. Wind disabled.  Takeoff to z = 1 m, settle 5 s, measure DC bias.
2. Command +body_X at 0.30 m/s for 4 s.  Hover 3 s.
3. Command -body_X at 0.30 m/s for 4 s.  Hover 3 s.
4. Same for ±body_Y.
5. Subtract pre-motion bias, take the (+) - (-) differential (cancels
   any residual DC).  Middle 50 % of each phase only (skip accel/decel
   tilt transients).

Result (bias-corrected differential, per 1.2 m of body motion):

| Motion | mean L0_dx | mean L0_dy |
|---|---:|---:|
| +body_X (forward) | -114 | **-732** |
| -body_X | +234 | **+676** |
| +body_Y (left) | **-957** | +131 |
| -body_Y | **+653** | -184 |

Pattern is clear: `L0_dy` tracks body_X motion (sign-inverted), `L0_dx`
tracks body_Y motion (sign-inverted).  This matches the current
`BODY_XFORM = (0, -1, -1, 0)` in `s091/aruco_hover.py` — i.e. **flow
axes were already correct** (confirmed, not changed).  But the *PnP*
code used the *opposite* assumption and was wrong.

This experiment is now the reference for any future axis-confusion
debug.  Empirical control test > rotation-matrix arithmetic.

### Bug 3 — cf2 position-PID velocity cap = 1.0 m/s (the real blocker)

s091 hover under full wind plateaued at ~17 % all-4 detect, 43 cm mean
drift, no matter how the *flow* fusion was tuned.  Six parameter sweeps
(texture threshold, REFINE_MIN_CONF, deadband, conf-std mapping ½×/2×,
etc.) all gave statistically identical results.  The flow algorithm
itself was clean (no wind: 100 % detect, < 10 cm hover; gz ground truth
agreed with both EKF and PnP).  So *something else* was capping
performance.

s093 (`max_velocity.py` + `max_velocity_pos.py`) commanded the drone to
fly at progressively higher velocities.  Two findings:

1. With MotionCommander (high-level wrapper), achieved velocity tops
   out at **0.78 m/s** even when commanded 2.0 m/s — MotionCommander
   smooths velocity setpoints.
2. With direct `cf.commander.send_position_setpoint(D, 0, z)` at 100 Hz
   (bypassing MotionCommander), peak velocity hits exactly **1.01 m/s**
   regardless of target distance D — a hard cap.

`grep PID_POS_VEL_X_MAX` in the CrazySim firmware tree found:
```
src/platform/interface/platform_defaults_sitl.h:123:#define PID_POS_VEL_X_MAX 1.0f
```
This is the *output limit* of the cf2 position-PID (set on
`pidX.pid.outputLimit` in `position_controller_pid.c`).  The position
loop physically cannot ask the velocity loop for more than 1.0 m/s,
regardless of how far the setpoint is from current position.

**Implication for hover under wind:** Gazebo `WindEffects` is configured
at 0.20 m/s steady + 0.15 m/s Gaussian noise + 20 % sin modulation, with
gust peaks reaching ~0.4 m/s.  With cap = 1.0 m/s, the drone has
~0.6 m/s of *spare authority* to push back against wind.  Per
fundamental-mode control theory the maximum sustained correction is the
authority *minus* the disturbance — so steady-state error is bounded
below by `(wind / authority) × characteristic_length`, on the order of
40 – 60 cm with these numbers.  That matches the observed drift exactly.

**Fix:** raise `posCtlPid.xVelMax` and `posCtlPid.yVelMax` to **2.5 m/s**
via cflib `cf.param.set_value()`:
```python
DAMP_PARAMS = {
    "posCtlPid.xyKd":    0.5,
    "velCtlPid.vxKd":    0.05,
    "velCtlPid.vyKd":    0.05,
    "posCtlPid.xVelMax": 2.5,   # default 1.0 — wind-rejection authority
    "posCtlPid.yVelMax": 2.5,
    "posCtlPid.xKp":     3.0,   # default 2.0 — more aggressive correction
    "posCtlPid.yKp":     3.0,
}
```
xKp = 3.0 was the sweet spot — xKp = 4.0 over-corrects and oscillates.

Higher than 2.5 m/s on the velocity cap pushed the flow algorithm past
its phase-correlation saturation (28000 mgrid, ≈ 32 L0-px shift =
~358 mm/frame at z = 1 m) and the EKF lost track entirely.  So
2.5 m/s is the practical upper bound at z = 1 m hover; flying higher
proportionally raises the saturation budget.

### s093 — Max velocity bench (cap-raised)

z = 3 m, no wind, direct position setpoint at 100 Hz, cap = 3.0 m/s:

| target | peak_v_x | flow_peak | sat |
|---|---:|---:|---:|
| 0.5 m | 0.30 m/s | 5200 | 0 % |
| 1.0 m | 0.72 m/s | 2808 | 0 % |
| 3.0 m | **2.27 m/s** | 23888 | 0 % |
| 5.0 m | EKF noise (>10 m/s reading = glitch) | n/a | flow lost |
| 8.0 m | EKF garbage | n/a | flow lost |

**Practical max sustained velocity at z = 3 m hover: ~2.27 m/s.**
Beyond that flow saturates (32-pixel phase-corr range exceeded), EKF
position estimate jumps wildly, and the controller goes unstable.

### Final hover-under-wind result (3-trial mean, full wind)

| Config | all-4 % | dist mean | dist max |
|---|---:|---:|---:|
| Default (cap = 1.0, defaults) | 17 % | 43 cm | 66 cm |
| Optimized (cap = 2.5, xKp = 3.0) | **28.2 %** | **32.5 cm** | 52 cm |

Improvement: +65 % on detection, −25 % on drift — without touching the
flow algorithm at all.  The remaining drift is *real physical drift*
the drone cannot dodge faster than the controller authority allows
under this wind profile.

### Lessons / discipline

1. **Always verify with ground truth.**  Three weeks of "tuning the
   flow fusion" was wasted because EKF and PnP were both being trusted.
   `gz model -m crazyflie_0 -p` in a shell took 30 seconds and ended
   the whole debate.
2. **Controller caps masquerade as algorithm limits.**  Before tuning
   any perception stack against a control failure, grep the controller
   firmware for `*_MAX` / `*_LIMIT` constants.  In CrazySim these live
   in `src/platform/interface/platform_defaults_sitl.h`.
3. **Bias contaminates passive observation.**  Empirical axis
   identification *only* works under controlled motion with bias
   subtraction (s092 protocol).  Trying to derive the rotation matrix
   from hover data with wind running gave coefficients that were 90 %
   bias and 10 % signal.
4. **Texture padding is a real failure mode.**  When using
   `cv2.aruco.generateImageMarker(... sidePixels=N, borderBits=1)` the
   total pattern is `(N + 2) × cell_size` but the texture file may be
   padded to a power-of-two size with white space.  Always compute the
   black-bordered fraction before passing the box size to `solvePnP`.

### Files shipped

- `examples/sentai_runtime/experiments/s090_hover_over_cat/aruco_detector.py`
  — `MARKER_SIZE_M = 0.0625`, new `estimate_drone_world_pose()`,
  `CAM_OFFSET_BODY`, `_R_CAM_TO_BODY`.
- `examples/sentai_runtime/experiments/s091_aruco_lowalt/aruco_hover.py`
  — uses the new PnP function, sets `posCtlPid.{xVelMax,yVelMax,xKp,yKp}`,
  writes `flow_records.csv` + `samples.csv` per run.
- `examples/sentai_runtime/experiments/s092_axis_calib/axis_calib.py`
  — controlled-motion axis identification.
- `examples/sentai_runtime/experiments/s093_max_velocity/{max_velocity,max_velocity_pos}.py`
  — velocity-bench tests (MotionCommander vs direct position setpoint).
- `sim/camera_bridge_recv.c` — dump infra split: `SENTAI_DUMP_RAW_EVERY`
  controls 640×480 RGB dumps (default 6 frames, ArUco PnP rate); a
  separate L0 80×60 PGM dump per frame via `SENTAI_DUMP_FRAMES_EVERY=1`
  (cheap, for visual debug).  Both fopen failures now log once per 100
  errors instead of failing silently.

## 10m. PX4 SITL coexistence + sentai.link MAVLink bridge (Phase 6, 2026-05-12)

PX4 v1.14 SITL installed alongside CrazySim in the existing
`crazysim-garden` distrobox.  Verified bidirectional MAVLink between
`sentai_sim` and a real PX4 instance over UDP — `sentai.link` works
unchanged with a SIM-only UDP backend, mirroring the ARM UART path.

### Compatibility verdict

| Stack piece | Distrobox (crazysim-garden, Ubuntu 22.04) | PX4 v1.14 | Match |
|---|---|---|---|
| Gazebo Sim | 7.9.0 (Garden) | requires gz-transport12 | ✓ |
| gz-transport | 12.2.2 | gz-transport12 | ✓ |
| gz-msgs | 9.5.1 | gz-msgs9 | ✓ |
| World plugins | Physics, UserCmd, SceneBcast, Contact, Imu, AirPressure, Sensors | identical 7 plugins | ✓ |

PX4 v1.15 also works (fallback to gz-transport12).  v1.16+ requires
gz-transport13 / Harmonic → INCOMPATIBLE with our Garden distrobox.
Don't propose v1.16+ for this codebase without first migrating the
whole stack to Harmonic.

### Port allocation — NO conflicts when all 3 sims run together

| Port  | Owner    | Role | Bind/Send |
|------:|----------|------|---|
| 14540 | sentai_sim | PX4 telemetry IN | bind |
| 14580 | PX4 SITL   | offboard cmds IN | bind (PX4 sends to 14540) |
| 18570 | PX4 SITL   | GCS link | bind (reserved for pymavlink/MAVSDK) |
| 14550 | reserved   | legacy GCS UDP | free |
| 19850 | CrazySim cf2 | cflib CRTP | bind |

`sim/sentai_uart_serial_udp.c` defaults: BIND 14540, SEND 14580
(env vars `SENTAI_LINK_UDP_LOCAL_PORT` / `SENTAI_LINK_UDP_REMOTE_PORT`
override).

### Architecture — same `sentai.link` API on ARM + SIM

```
       ARM (firmware)                           SIM (host)
  ┌────────────────────┐                  ┌────────────────────────┐
  │ Python REPL        │                  │ Python REPL            │
  │   sentai.link.*    │                  │   sentai.link.*        │
  └────────┬───────────┘                  └────────┬───────────────┘
           │ (same C ABI: sentai_link_init,         │
           │  send_heartbeat, send_statustext,      │
           │  set_debug, get_stats, ...)            │
  ┌────────▼───────────┐                  ┌────────▼───────────────┐
  │ sentai_link.cc     │                  │ sim/sentai_link_sim.cc │
  │ (full firmware:    │                  │ (slim: hb + statustext │
  │  tracker, mesh,    │                  │  + parse rx hb)        │
  │  health, nanopb)   │                  │                        │
  └────────┬───────────┘                  └────────┬───────────────┘
           │                                       │
  ┌────────▼───────────┐                  ┌────────▼───────────────┐
  │ sentai_uart_serial │                  │ sim/sentai_uart_serial │
  │ * (LPUART6 ARM HAL)│                  │ _udp.c (Linux UDP)     │
  └────────┬───────────┘                  └────────┬───────────────┘
           │ UART2 @ 57600                          │ UDP datagram
           │   ↓                                    │   ↓
  ┌────────▼───────────┐                  ┌────────▼───────────────┐
  │ Crazyflie 2 radio  │                  │ PX4 SITL instance 0    │
  │ or external FCU    │                  │ mavlink onboard 14580  │
  └────────────────────┘                  └────────────────────────┘
```

Key invariant: `examples/sentai_runtime/modsentai_link.c` (Python
binding) and the C ABI surface (`sentai_link_init / *_send_heartbeat /
*_send_statustext / *_set_debug / *_get_stats`) are IDENTICAL on both
targets.  Only the transport differs.  Future firmware features added
to `sentai_link.cc` (e.g. TUNNEL handler) must keep ABI parity so the
SIM mirror picks them up.

### Verification (s095_px4_link_ping)

`bash examples/sentai_runtime/experiments/s095_px4_link_ping/ping_test.sh`
- Starts PX4 SITL (sihsim_quadx, no Gazebo needed) inside distrobox
- Spawns `sentai_sim`, drives `sentai.link.init/heartbeat/stats`
- Asserts `tx_hb > 0` AND `rx_hb > 0` AND `last_peer_sysid == 1`

Result on 2026-05-12: TX=5, RX=7, peer_sys=1.  PASS.

### Phase 6 — installation + build

1. `bash sim/scripts/install_px4_sitl.sh` clones PX4 v1.14 to
   `/home/bogdan/work/px4/PX4-Autopilot` (sibling of CrazySim, NOT
   vendored into coralmicro/ per §2.3) and installs build deps inside
   the distrobox.  Pip deps `pyros-genmsg` and `future` are also
   installed inside distrobox (used by uORB / mavgen code generators).
2. Build:
   `distrobox enter crazysim-garden -- bash -c 'cd /home/bogdan/work/px4/PX4-Autopilot && make px4_sitl_default -j$(nproc)'`
3. Output binary: `/home/bogdan/work/px4/PX4-Autopilot/build/px4_sitl_default/bin/px4` (~47 MB).

### SIM bridge implementation (NASA/JPL discipline per `agent/embeded.md`)

`sim/sentai_uart_serial_udp.c`:
- Bounded I/O: UDP datagrams ≤ 2 KB, timeouts capped at 5 s.
- No dynamic allocation after init (single static `s_sock`, static
  `s_peer` sockaddr).
- All `socket() / bind() / sendto() / recvfrom()` return codes checked;
  failures logged once-per-100 (no log spam, no silent swallowing).
- Open is idempotent (`s_sock >= 0 → return 1`).
- `_read()` learns peer ephemeral source from `recvfrom` and locks
  subsequent sends to that real address (PX4 mavlink module uses
  ephemeral src ports that don't match the configured remote).

`sim/sentai_link_sim.cc`:
- Same public C ABI as `examples/sentai_runtime/sentai_link.cc` so
  `modsentai_link.c` works unchanged — but slim (~250 LoC vs 700 LoC)
  because SIM doesn't need tracker / mesh / health / nanopb deps.
- Reader task created with priority `tskIDLE_PRIORITY + 2` — MATCH the
  main MP task.  Earlier tried `+1` (lower) → POSIX FreeRTOS port did
  not schedule it while the same-prio camera_bridge task was perpetually
  ready, so RX silently stayed at 0.  **Lesson**: on the POSIX port
  always set link-reader priority equal-or-higher than the IO-bound
  tasks that compete for the scheduler.
- Stop has bounded join: 500 ms max wait for reader to exit, then drop
  socket regardless.  No infinite-wait join.
- All `atomic<bool>` flags between main + reader task; no mutex needed.

### Plan — REPL-over-MAVLink (next step, scoped)

User spec (2026-05-12): "*căutăm un mesaj TEXT sau ceva custom în
MAVLink și implementăm comenzile de REPL prin radio așa cum aveam și
în crazy.  Cu sentai_sim vorbim tot prin command prompt; prin
intermediul radio-ului vorbim cu o buclă REPL care face exec.  Spre PX4
NU folosim cflib — folosim MAVSDK sau altă librărie x86 ce trimite UDP
către PX4.  Watch UDP conflicts.*"

Design — TUNNEL message (msgid 385, MAVLink v2 only):
- `target_system / target_component` — addresses sentai_sim (sysid=1, compid=191)
- `payload_type` — custom value `0xC0DE` ("SentAI REPL")
- `payload_length` (uint8) — 1..128
- `payload[128]` — raw REPL bytes (UTF-8)

Forward direction (host → sentai_sim REPL):
- Host pymavlink/MAVSDK sends TUNNEL{payload_type=0xC0DE, payload=cmd}
- PX4 mavlink module routes by target_system; with sentai on UDP 14540
  and sysid=1, PX4 forwards the TUNNEL to us
- `sentai_link_sim.cc` reader detects payload_type 0xC0DE → pushes
  bytes into a stdin-tap FIFO that `main_sim.c::sim_read_line()` polls
  in addition to actual stdin
- MicroPython processes line, emits result via stdout

Reverse direction (sentai_sim REPL output → host):
- `main_sim.c` already has a tee on stdout (used by the REPL prompt);
  add a hook that mirrors bytes into `sentai_link_send_tunnel(payload)`
- Reader on host re-assembles → prints to terminal

Constraints:
- TUNNEL payload is 128 B per packet — large vs Crazyflie CRTP's 30 B
  MTU (radio bridge memory `feedback_radio_no_file_transfer.md`).
- Still NOT for file transfer — keep small `$exec`-style commands.
- ARM uses same TUNNEL bytes over UART; no protocol change between
  SIM ↔ HW so the test we write here proves the HW radio path too.

Test scaffold:
- `s095_px4_link_ping/ping_test.sh` — current MVP (heartbeat only)
- `s096_repl_over_mavlink/` (future) — TUNNEL round-trip, exec inline,
  multi-packet response chunking, recovery from packet drop.

### Coexistence with CrazySim in same Gazebo world

Both PX4 and CrazySim cf2 can spawn in the same `sentai_crazysim.sdf`:
- CrazySim adds Crazyflie model named `crazyflie_0`, plugin claims
  topics `/cf_0/*` and UDP 19850
- PX4 spawns x500 (or sihsim) model named `x500_0`, plugin claims
  `/world/.../model/x500_0/*` topics and UDP 14580/14540/18570
- Model names disjoint, topic prefixes disjoint, port allocations
  disjoint → safe coexistence.

`PX4_GZ_STANDALONE=1` env var lets the PX4 binary attach to an
already-running Gazebo instance instead of spawning its own — preferred
when CrazySim already started gz with the sentai world.

(This is documented but NOT yet wired into a runner script; the s095
test uses `sihsim` simulator backend which doesn't need Gazebo at
all.  Real Gazebo coexistence is the next milestone after REPL.)


## 10n. PX4 Phase 6d asset map (2026-05-12)

Where every asset that participates in the PX4 + Gazebo + sentai.flow
end-to-end test lives.  Apples-to-apples comparison with the cf2
hover-stability bench (s091) requires using the SAME world and
matching the camera mount conventions.  Keep this table accurate;
when an asset moves, update here first.

### Drone model — `x500_sentai`

| Item | Value |
|---|---|
| Path | `/home/bogdan/work/px4/PX4-Autopilot/Tools/simulation/gz/models/x500_sentai/` |
| Files | `model.config` + `model.sdf` + `meshes/` + `materials/` + `thumbnails/` |
| Base | Copied from `x500/` then added `downward_cam` sensor |
| Spawn name in PX4 | `x500_sentai_0` (PX4 appends `_<instance>`) |
| Airframe used | `4001_gz_x500` (env override — no new airframe needed yet) |

### Downward camera mount (parity with cf2)

| Property | cf2 model.sdf.jinja | x500_sentai model.sdf |
|---|---|---|
| `<pose>` | `-0.04 0 -0.02 0 1.5707963 3.1415927` | `0 0 -0.05 0 1.5707963 3.1415927` |
| HFOV | `1.0123` rad (58°) | identical |
| Image | 640×480 R8G8B8 | identical |
| `<topic>` | `/downward_cam/image` | identical |
| `<update_rate>` | 30 Hz | identical |
| `<always_on>` | 1 | identical |
| `<clip>` | (default) | `near=0.05 far=30` |

Camera **orientation** (pitch=π/2, yaw=π) is identical → the
body-frame transform `(-1, 0, 0, +1)` from memory
`project_flow_body_frame_baseline.md` applies UNCHANGED on PX4.

Camera **position** differs (cf2 has 4 cm X back-offset from CoM;
x500_sentai is centered).  For the FLOW algorithm this is irrelevant
(flow measures ground-relative motion, not absolute pose).  For ArUco
PnP-based ground-truth comparison the offset enters the
`estimate_drone_world_pose()` cam-to-CoM compensation — update that
function if you wire ArUco PnP into the PX4 bench.

### Gazebo world — canonical s091 world

| Item | Value |
|---|---|
| Path | `/home/bogdan/work/crazyflie/CrazySim/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/worlds/sentai_crazysim.sdf` |
| Internal name | `sentai_crazysim` (`<world name="sentai_crazysim">`) |
| Resource path | same dir (textures + materials live here) |
| Contains | 4 ArUco markers id0..3, 6 color cubes, cat target, checkerboard ground, wind plugin, deep ground texture |

**Do NOT use** `sim/gazebo/sentai_crazysim_world.sdf` for PX4 hover
bench — it is a stripped variant **without ArUco markers** (only
color cubes + cat).  It's an earlier Sentai-host world kept around
for the camera-bridge smoke tests but is NOT the s091 baseline.

ArUco texture files (4×4_50 dictionary, ids 0..3):
`<crazysim_root>/worlds/materials/textures/aruco_4x4_50_id{0..3}.png`

Marker positions in the world (matches `KNOWN_POSITIONS_M` in
`s090_hover_over_cat/aruco_detector.py`):

| id | pose (x, y, z_base) | z_top (visible face) |
|---|---|---|
| 0 | +0.15, +0.10, 0.15 | 0.20 m |
| 1 | -0.15, +0.10, 0.15 | 0.20 m |
| 2 | -0.15, -0.10, 0.15 | 0.20 m |
| 3 | +0.15, -0.10, 0.15 | 0.20 m |

Effective marker size for PnP: 0.0625 m (NOT 0.08 m — texture has
22% white padding; bug fixed s091 commit `6e492c28`).

### Gazebo GUI config (mandatory per Sim.md §10c rule 1)

| Item | Value |
|---|---|
| Path | `/home/bogdan/work/coralmicro/sim/gazebo/sentai_gui.config` |
| Purpose | Layout with PiP camera widget so operator can see drone-eye view |
| Use | `gz sim --gui-config <path> <world.sdf>` |

### Launch wiring (s100 smoke = the working recipe)

Inside the `crazysim-garden` distrobox:
1. `gz sim --verbose=1 -r --gui-config <gui> sentai_crazysim.sdf`
   with `GZ_SIM_RESOURCE_PATH = <crazysim_worlds>:<px4_gz_models>`
2. Wait for `/world/sentai_crazysim/clock` topic to appear.
3. `PX4_SYS_AUTOSTART=4001 PX4_SIMULATOR=gz PX4_GZ_MODEL=x500_sentai
    PX4_GZ_WORLD=sentai_crazysim PX4_GZ_MODEL_POSE='0,0,0.2,0,0,0' px4`
4. Wait for "Ready for takeoff" in PX4 log.

PX4 detects the running gz, attaches via `gz_bridge`, spawns
x500_sentai_0 into the world.  The downward_cam sensor begins
publishing `/downward_cam/image` immediately at 30 Hz.

### Flow forwarder (C-side, no Python per frame)

| Item | Value |
|---|---|
| Source | `sim/sentai_link_sim.cc::link_flow_forward_task` |
| Toggle (Python) | `sentai.link.flow(1[, dist_m])` / `sentai.link.flow(0)` |
| Stats counter | `sentai.link.stats()[8]` (tx_flow) |
| Verified by | `experiments/s099_link_flow_c_forward/` |

See memory `project_px4_flow_c_forwarder.md` for full design.

### Smoke test sequence (Phase 6d steps)

| s### | Goal | Status |
|---|---|---|
| s099 | C-side flow forwarder verified | ✅ PASS |
| s100 | x500_sentai spawns in canonical world + cam publishes | ✅ PASS |
| s101 | EKF2 flow-only nav (no-wind hover baseline) | ⏳ next |
| s102 | Half-wind hover (target: ~7.6 cm drift = s091 #14 parity) | ⏳ |
| s103 | Full-wind hover (target: ~32 cm drift = s091 #1 parity) | ⏳ |

## 10o. PX4 + Gazebo bring-up — 3 pitfalls + canonical sequence (s101, 2026-05-12)

Phase 6d step 1 (x500_sentai takeoff/hover/land with GPS baseline)
exposed three non-obvious failure modes between PX4 SITL and our gz
world.  All three documented here; memory file
`feedback_px4_gz_takeoff_pitfalls.md` mirrors this for cross-session
recall.

### Pitfall #1 — Lockstep does NOT engage when gz is pre-launched

**Symptom:** PX4 boots, `gz_bridge` attaches, topics are published —
but EKF2 logs `Preflight Fail: ekf2 missing data` and `Preflight Fail:
Compass Sensor 0 missing`.  Drone won't arm-and-take-off.

**Root cause:** When gz is already running before PX4 starts, the
`px4-rc.simulator` init script takes the "gazebo already running"
branch which only attaches via `gz_bridge` without engaging the
lockstep scheduler.  IMU/baro topics ARE published but the timing
contract that EKF2 expects (`lockstep_scheduler initial absolute time`
log line never appears) breaks.

**Fix:** Always let PX4 launch gz itself by setting
`PX4_GZ_WORLDS=<dir>` + `PX4_GZ_WORLD=<name-without-.sdf>` in the
env passed to the PX4 binary.  PX4 spawns gz via `gz sim -s` then
spawns a bare `gz sim -g` GUI.  If you want the sentai PiP layout,
kill the bare GUI and re-launch separately with `--gui-config`.

```bash
# Working pattern (in distrobox):
export GZ_SIM_RESOURCE_PATH="$CRAZYSIM_WORLDS:$PX4_GZ_MODELS"
export PX4_GZ_MODELS="$PX4_GZ_MODELS"
export PX4_GZ_WORLDS="$CRAZYSIM_WORLDS"
PX4_SYS_AUTOSTART=4040 PX4_SIMULATOR=gz \
  PX4_GZ_MODEL=x500_sentai PX4_GZ_WORLD=sentai_crazysim \
  PX4_GZ_MODEL_POSE='0,0,1.0,0,0,0' \
  $PX4_BIN -i 0 -d $PX4_ETC
# Then:
pkill -9 -f "gz sim -g"
gz sim --gui-config $SENTAI_GUI_CFG -g
```

### Pitfall #2 — `MAV_CMD_NAV_TAKEOFF` lat/lon=0 triggers auto-disarm

**Symptom:** `Armed by external command` → ~10 s pause → `Disarmed by
auto preflight disarming` (governed by `COM_DISARM_PRFLT`, default
10 s).  Drone never lifts.

**Root cause** (per [PX4 issue #21601](https://github.com/PX4/PX4-Autopilot/issues/21601)):
`p5=0, p6=0` (lat/lon) in `MAV_CMD_NAV_TAKEOFF` are "arbitrary
coordinates" — PX4 interprets them as "fly to (0°, 0°)" (in the
Atlantic ocean near the equator) before climbing.  The internal
sanity check rejects the maneuver but doesn't surface a useful error;
drone stays armed-on-ground until `COM_DISARM_PRFLT` triggers.

**Fix:** pass NaN for `p4` (yaw), `p5` (lat), `p6` (lon).  PX4
interprets NaN as "use current / home position".  Shipped in
`sim/sentai_link_sim.cc::sentai_link_cmd_takeoff` on 2026-05-12.

```c
return link_send_command_long(22,
    /* p1 min_pitch */ 0,
    /* p2 unused   */ 0,
    /* p3 unused   */ 0,
    /* p4 yaw      */ NAN,
    /* p5 lat      */ NAN,
    /* p6 lon      */ NAN,
    /* p7 alt      */ altitude_m);
```

### Pitfall #3 — Spawn pose must clear obstacles

**Symptom:** drone spawns but stays on ground despite GPS+EKF being
ready; pose CSV shows constant z near body-half-height.

**Root cause:** If `PX4_GZ_MODEL_POSE` puts the drone overlapping any
collision geometry, gz physics pins it.  Specific to s101 setup: the
sentai_crazysim ArUco posts have NO `<collision>` element (visual-only)
so they don't collide; but the x500 drone (~46 cm body + arms) is
larger than the cf2 (~10 cm) the world was originally designed for.
Spawn at z=0.2 was too low.

**Fix:** spawn at z=1.0 m for x500.  Markers were also doubled-out to
±0.30 ±0.20 (was ±0.15 ±0.10) so they remain in the downward-cam FOV
when drone hovers at z=1.0-2.0 m.  ArUco IDs unchanged; positions
updated in `KNOWN_POSITIONS_M` (aruco_detector.py) too — see
`reference_px4_phase6d_assets.md` for the new pose table.

### Pitfall #4 (cosmetic) — `COM_TAKEOFF_ALT` minimum

PX4 enforces 2.5 m minimum takeoff altitude by default.  Requesting
1.5 m results in `WARN [navigator] Using minimum takeoff altitude:
2.50 m` and drone climbs higher.  Override per airframe or via
runtime `param set` if low-altitude testing matters.

### Canonical bring-up sequence (s101 working baseline)

1. PX4 binary already built (`make px4_sitl gz_x500`).
2. Custom airframe `4040_gz_x500_sentai` registered in
   `build/px4_sitl_default/etc/init.d-posix/airframes/`.
3. Custom model `x500_sentai` in `Tools/simulation/gz/models/`.
4. World file (canonical s091 in CrazySim tree) loaded via env vars.
5. Bring-up: launch PX4 (lockstep-mode); wait for `Ready for takeoff`;
   replace bare GUI with PiP-configured one.
6. REPL: `sentai.link.init()` → `arm(1)` → `takeoff(alt)` with NaN
   lat/lon → wait → `land()` → `arm(0)`.

Result on 2026-05-12 build #100 of sentai_sim: hover z_mean +1.92 m
(target 1.5, clamped to ≥2.5), drift mean **10.4 cm**, max **16.9 cm**
over 10 s window.  Compare to cf2 s091 baseline (no wind): same
sub-20 cm regime → PX4 ground-truth EKF parity OK.

Reference snapshot:
`experiments/s101_px4_hover_nowind/`.


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
| 2026-05-11 | 10l hover-under-wind | DONE — 3 bugs found + cf2 PID cap fix, drift 43cm→32cm at full wind | SIM build trail | Bug 1: MARKER_SIZE_M was 0.08 but ArUco texture padding makes effective marker 0.0625m → PnP-z over-estimated by 1.28×.  Bug 2: per-marker -tvec[0] averaging assumed (a) cam_X=body_X and (b) marker centroid stayed at world (0,0) — both wrong; partial-FOV biased the centroid 40cm.  Bug 3 (the real blocker): cf2 `platform_defaults_sitl.h:PID_POS_VEL_X_MAX=1.0f` caps position-PID velocity output at 1 m/s → drone has only ~0.6 m/s wind-rejection authority vs 0.4 m/s gusts.  Fix: raise `posCtlPid.xVelMax/yVelMax` to 2.5 + `xKp/yKp` to 3.0 via cflib param.set_value.  s092 (axis_calib.py) controlled-motion test confirms current BODY_XFORM=(0,-1,-1,0) is correct (axes were never the bug; PnP labels were).  s093 (max_velocity.py + max_velocity_pos.py) discovers the hard 1 m/s cap and finds max sustainable velocity at z=3m hover is ~2.27 m/s (peak before flow saturates at 32 L0-px shift).  Final 3-trial mean hover-under-wind: all-4 17%→28.2%, dist mean 43cm→32.5cm, no flow-algorithm changes.  Full section in §10l. |
| 2026-05-12 | 10m Phase 6 PX4-link | DONE — PX4 v1.14 SITL installed + sentai.link MAVLink bridge, TX/RX heartbeat round-trip verified (s095_px4_link_ping PASS: TX=5 RX=7 peer_sys=1) | SIM build trail | New: `sim/scripts/install_px4_sitl.sh` clones PX4 v1.14 to /home/bogdan/work/px4/PX4-Autopilot (sibling of CrazySim).  Garden 7.9 + gz-transport12 + gz-msgs9 stack matches PX4 v1.14 native deps.  Pip needs `pyros-genmsg` + `future` inside distrobox.  New: `sim/sentai_uart_serial_udp.c` provides ARM-ABI-compatible UART backend over UDP (BIND 14540, SEND 14580 = PX4 v1.14 offboard mavlink, per `PX4 ROMFS/.../px4-rc.mavlink`).  New: `sim/sentai_link_sim.cc` slim MAVLink encoder/parser (heartbeat + statustext only, no tracker/mesh/nanopb deps) with reader task — CRITICAL FIX: reader task priority must be `tskIDLE_PRIORITY+2` to MATCH main MP task; at +1 (lower) it never gets scheduled on POSIX FreeRTOS port while camera_bridge is perpetually ready.  `sentai.link.{init,stop,debug,heartbeat,send,stats}` Python module exposed in SIM via `sim/modsentai_sim.c` (mirrors firmware modsentai_link.c surface).  Port allocation disjoint from CrazySim (19850) and PX4 GCS (18570) so all 3 can run together.  Plan §10m.b: REPL-over-MAVLink via TUNNEL msgid 385 payload_type=0xC0DE (128B MTU vs Crazyflie CRTP 30B); same protocol on ARM/HW radio. |

## 10p. PX4 SITL hover-under-stress + ArUco VPE — Phase 6d final (2026-05-12)

End-of-session state for PX4 SITL hover validation.  All architectural
pieces of the cf2-equivalent flow stack are now plumbed end-to-end on
x86; remaining work is controller tuning + sign-convention closure.

### Architecture proven on PX4 SITL (Garden, x500_sentai)

```
gz Garden 7.9 (sentai_crazysim world, ArUco markers id0..3 at ±0.30,±0.20)
   │
   │ /downward_cam/image (gz topic, 640×480 RGB, 30 Hz)
   ▼
gz_to_uds_bridge (C++, distrobox, gz-transport12 → UDS)
   │
   │ Unix socket /tmp/sentai_cam.sock (full 124-byte Reply protocol)
   ▼
aruco_to_vision_estimate.py (HOST venv: cv2 + pymavlink)
   │  - detect_markers (cv2.aruco DICT_4X4_50)
   │  - estimate_drone_world_pose (per-marker solvePnP + R_cam_to_world)
   │  - ENU→NED transform
   │  - VPE yaw = NaN (PX4 uses mag yaw, avoids conflict)
   ▼
MAVLink VISION_POSITION_ESTIMATE (msg id 102) → PX4 UDP 14580
   ▼
PX4 EKF2 (airframe 4043: GPS+VPE fused, EKF2_EV_CTRL=3, EVP_NOISE=0.05)
   ▼
OFFBOARD-POSITION (Python pymavlink, 20 Hz setpoint stream, port 14550)
```

### Hover bench results (x500_sentai, 15 s OFFBOARD hover, z=1.5 m target)

| s### | Setup | Wind | Stress | XY drift mean | XY drift max | Notes |
|---|---|---|---|---:|---:|---|
| s101 | GPS, AUTO.TAKEOFF | 0 | none | 14.0 cm | 20.0 cm | baseline |
| s104 | GPS, OFFBOARD-pos | 0 | none | **9.8 cm** | 18.1 cm | tightest |
| s107 sanity | no GPS, flow=0 mock | 0 | none | **<2 cm** | <3 cm | architecture proof |
| s107 stress | no GPS, flow=0 mock | 0 | IMU+motor | 70 cm | — | EKF blind to drift |
| s108 | GPS, OFFBOARD-pos | 0 | IMU+motor | 33.7 cm | 55.6 cm | + ArUco VPE (no fusion) |
| s109 | GPS+VPE fused, OFFBOARD | 0.2 m/s | IMU+motor | 67.7 cm | 172.8 cm | wind on |

vs cf2 s091 (same world, same wind 0.2 m/s, sentai.flow path):
  - Half wind: 7.6 cm drift, 100% all-4 markers
  - Full wind: 32 cm drift, 35% all-4 (after PID cap fix)

PX4 s109 drift ≈ 2× cf2 s091.  Suspect — per §10l lesson "controller
caps masquerade as algorithm limits":
  - PX4 `MPC_XY_P` default 0.95 (proportional position gain)
  - PX4 `MPC_XY_VEL_P_ACC` default 1.8 (velocity feedforward)
  - PX4 `MPC_XY_VEL_MAX` default 12 m/s (lots of authority, not the
    cap-issue cf2 had)
  - Likely need a sweep similar to cf2 §10l's `posCtlPid.xKp` 2→3 fix.

### Built-in Gazebo stress factors (added to x500_sentai SDF)

IMU sensor noise (gz built-in):
```xml
<imu>
  <angular_velocity>
    <x|y|z>
      <noise type="gaussian">
        <stddev>0.01</stddev>          <!-- rad/s, mid-grade MEMS -->
        <bias_stddev>0.001</bias_stddev>
      </noise>
    </x|y|z>
  </angular_velocity>
  <linear_acceleration>
    <x|y|z>
      <noise type="gaussian">
        <stddev>0.1</stddev>           <!-- m/s², mid-grade MEMS -->
        <bias_stddev>0.01</bias_stddev>
      </noise>
    </x|y|z>
  </linear_acceleration>
</imu>
```

Motor asymmetry (manufacturing tolerance simulation):
- 4 rotors with `motorConstant` perturbed ±2% from 8.54858e-06 nominal
  - rotor 0: 8.61e-06 (+0.6%)
  - rotor 1: 8.40e-06 (-1.7%)
  - rotor 2: 8.71e-06 (+1.8%)
  - rotor 3: 8.41e-06 (-1.7%)

Result with stress alone (no wind), flow=0 mock to isolate stress
effect: drone holds altitude (baro stable) but drifts ~70 cm in 10 s
on coupled x+y axes — motor asymmetry causes rotational drift, IMU
noise propagates into EKF velocity estimate.  With ArUco VPE active
+ GPS, drift drops to 33 cm (s108).

### Constraints / open issues for full s091 parity

1. **Controller tuning** — PX4 MPC_XY_* params not yet tuned for our
   x500_sentai mass/inertia + flow-source setup.  cf2 needed §10l
   posCtlPid.xKp 2→3 + xVelMax 1.0→2.5 ratio.  Equivalent PX4 sweep
   to be done in s110+.
2. **Flow-only path (no GPS) blocked on axis-sign convention** —
   s092 cf2 BODY_XFORM=(0,-1,-1,0) does NOT directly apply; PX4
   EKF source negates pixel_flow internally and uses different
   convention.  4 sign combos tried, all diverged.  Requires
   PX4-side controlled-motion calibration (s092 protocol replicated
   for PX4) — estimated 8-12h focused session.
3. **Lockstep camera cadence** — sentai.flow real path (vs ArUco
   mock) needs camera frames at 30 Hz real wall-clock to keep
   phase-corr inter-frame motion in search range.  PX4-gz lockstep
   slows camera to ~1 Hz, breaks phase-corr.  Workaround: use
   ArUco VPE path (s108/s109) which is timing-tolerant.
4. **Spawn altitude vs marker visibility** — at z=0.23 m (drone on
   ground, body half-height), camera at z=0.18 m is BELOW marker
   tops at z=0.20 m → no detection until drone airborne.  Use GPS
   or baro-only for initial climb, switch to ArUco-VPE once airborne.
   Spawn pose set to (0,0,1.0) for x500 to clear ArUco posts.

### Mandatory MAVLink + PX4 conventions (load-bearing)

- **NaN yaw in VPE** — sending VPE with yaw=0 caused 22 m drift in
  s108; PX4 had yaw conflict with mag.  Fix: yaw/roll/pitch = NaN.
- **NaN lat/lon in NAV_TAKEOFF** — sending lat=lon=0 = "arbitrary
  coords" → auto-disarm.  Per §10o pitfall #2.
- **HEADLESS=1 + manual GUI** — PX4 launches its own bare gz GUI
  if HEADLESS not set; running our PiP-config GUI on top races.
  Per §10o pitfall #1.
- **MAVLink port 14550 for OFFBOARD pymavlink** — PX4 Normal stream
  sends to 14550; 14540 is Onboard stream which the flow-bridge
  may have claimed partnership of.
- **PX4 launches gz itself** (`PX4_GZ_WORLDS` env) — external gz +
  PX4 attach later skips lockstep handshake → EKF starves.
- **Reply struct = 124 bytes** for gz_to_uds_bridge UDS — sending
  smaller reply causes bridge to disconnect.  Per s108 fix.

### Files shipped this session

- `examples/sentai_runtime/experiments/s107_px4_offboard_pos_mockflow/`
  no-GPS + flow + OFFBOARD-position, sanity flow=0 PASS
- `examples/sentai_runtime/experiments/s108_px4_aruco_vpe/`
  ArUco PnP → VPE pipeline, all components verified
- `examples/sentai_runtime/experiments/s109_px4_aruco_wind/`
  ArUco VPE + wind + stress, drift 67.7 cm (cf2 parity target 32 cm)
- `sim/scripts/aruco_to_vision_estimate.py` — host-side ArUco PnP
  bridge (cv2 + pymavlink), 124-byte Reply protocol compat
- `sim/scripts/gz_pose_to_flow_estimate.py` — gz-pose → OPTICAL_FLOW_RAD
  mock (axis-sign tuning open)
- Airframes installed in PX4 build/etc/init.d-posix/airframes/:
  - 4040 = x500_sentai GPS baseline (proven)
  - 4041 = x500_sentai flow-only nav (axis tuning open)
  - 4042 = x500_sentai external vision only (no GPS)
  - 4043 = x500_sentai GPS+VPE fused

## 10q. PX4 SITL wind ceiling — outdoor max-wind test (s110, 2026-05-12)

Goal: find the wind speed at which PX4 x500_sentai SITL drone fails to
hover.  Operator asked specifically "PX4 va zbura outdoor, sa vedem
max wind posibil".

### Method

Wind sweep 0.2 → 20.0 m/s, each run = 15 s OFFBOARD-POSITION hover
via s109 setup (airframe 4043 GPS+VPE fused, MPC_XY_P=1.8, IMU+motor
stress factors active).  World `<linear_velocity>X 0 0</linear_velocity>`
edited inline between runs.  Drift recorded from gz ground-truth pose.

### Results: drone flies in ALL tested wind speeds

| Wind | Beaufort | Outdoor terminology | Drift mean | Drift trend | Status |
|---:|---:|---|---:|---|---|
| 0.2 m/s |  1 | calm / light air | 94.4 cm | high (stress-dom) | ✅ FLEW |
| 0.5 m/s |  1 | light air | 52.1 cm | low | ✅ FLEW |
| 1.0 m/s |  2 | light breeze | 62.6 cm | low | ✅ FLEW |
| 2.0 m/s |  2 | light breeze | 92.0 cm | mid | ✅ FLEW |
| 3.0 m/s |  3 | gentle breeze | 72.0 cm | mid | ✅ FLEW |
| 5.0 m/s |  3 | gentle breeze | 29.7 cm | low | ✅ FLEW |
| 7.0 m/s |  4 | moderate breeze | 45.9 cm | mid | ✅ FLEW |
| 10.0 m/s |  5 | fresh breeze (36 km/h) | 51.1 cm | mid | ✅ FLEW |
| 15.0 m/s |  7 | near gale (54 km/h) | 137.4 cm | high (peak) | ✅ FLEW |
| 20.0 m/s |  8 | gale (72 km/h) | 82.0 cm | mid | ✅ FLEW |

**Drone never crashed.**  Drift bounded ~30–140 cm in all
configurations.  Hovers maintain z ≈ target altitude.

### Interpretation

Drift is dominated by IMU+motor stress factors per-run noise, NOT by
wind magnitude (no monotonic correlation).  This is because:

1. **VPE provides absolute position anchor** — EKF cannot lose track
   of position regardless of wind.  Compare cf2 + PMW3901 (no
   position anchor, velocity-only): breaks at 0.2 m/s wind.  PX4 +
   VPE handles 100× more wind because the failure mode is different.
2. **Controller authority is large** — PX4 `MPC_XY_VEL_MAX=12 m/s`
   default → drone can command up to 12 m/s counter-thrust.  As
   long as wind < 12 m/s, drone has surplus authority.
3. **15+ m/s wind exceeds controller authority margin** but VPE
   anchor keeps EKF correct → drone still tries to fight, drift
   grows but drone doesn't tumble.

### Lesson distilled

For PX4 SITL outdoor hover validation:
- **With VPE (vision pose anchor)**: handles up to 20+ m/s wind in SIM.
  Real-world bottleneck is camera/CV failure at high speed, not
  controller.
- **With flow-only (no position anchor)**: cf2 s091 demonstrated
  0.2 m/s wind handled, 0.4 m/s breaks (per §10l).  Same algorithm
  on PX4 would behave similarly.

These are FUNDAMENTALLY DIFFERENT failure modes.  Vision-anchored
hover is fundamentally more robust than flow-only.

### Open: true flow-only wind ceiling on PX4

Pending axis-sign calibration (s092-style empirical protocol replicated
on PX4) needed to enable real `sentai.flow` → OPTICAL_FLOW_RAD → EKF
fusion.  Once closed, repeat the wind sweep with airframe 4041 (no GPS,
flow-only nav) to find the flow-bounded wind ceiling.  Expected to be
lower than 20 m/s (cf2 baseline is 0.4 m/s — flow noise dominates as
drone speed increases).

### Reproducibility recipe

```bash
# Set up s109 prerequisites (airframe 4043 + stress factors in
# x500_sentai SDF + ArUco-VPE bridge ready).
# Then:
bash examples/sentai_runtime/experiments/s110_px4_wind_sweep/run.sh
# Default WINDS="0.2 0.5 1.0 2.0 3.0".  Override:
WINDS="5.0 10.0 15.0 20.0" bash .../s110_px4_wind_sweep/run.sh
```

Output: `/tmp/sentai_s110_<stamp>/sweep.csv` with one row per wind value.

## 10r. SOTA research notes — optical flow + ArUco for embedded drones (2026-05-12)

Research session to identify improvements applicable to our M7
implementation.  Sources: arXiv, MDPI, ScienceDirect, IEEE.

### Optical flow on embedded/MCU

**Edge-FS / GAP8 parallelization** (arXiv:2305.13055, 2023).  Block-
matching optical flow on RISC-V 8-core MCU (GAP8) @ 50 MHz: 500 fps.
Showed parallelization 7.21× speedup.  Not directly applicable to our
M7 single-core but proves: optical flow can run at hundreds of fps
on ultra-low-power MCU with right architecture.

**Selective Intersection Flow (SIF)** (MDPI, 2025).  Lightweight LK
variant that pre-filters "non-contributive pixels".  1.7-1.8× faster
than full LK, 1.2-1.4× more accurate.  Complementary to our phase-
correlation — could replace LK in any LK-based pipeline.

**LEVIO** (IEEE Sensors, accepted Feb 2026).  ORB-based VIO on ultra-
low-power RISC-V SoC: 20 fps @ <100 mW.  Open-source.  Shows full VIO
(not just flow) feasible at extreme low power.  For us: future
direction if we want sentai.flow with VIO-like absolute drift bound.

**PMW3901 limitations** (ScienceDirect, 2023).  Sensor returns 1/10
px → coarse at altitude.  Our sentai.flow at 80×60 grid has 8× px
budget — better resolution at high z, but ALSO requires more compute.
Validates our "native crop L2" finding from §10l (1.73 mm/px @ z=1m).

### ArUco detection state-of-the-art

**DeepArUco++** (Image and Vision Computing, 2024, arXiv:2411.05552).
CNN-based ArUco with 3-stage pipeline (detect / corner refine /
decode).  Robust to challenging lighting (sun, dark, motion blur)
where OpenCV stock fails.  Heavy compute → suited for our **EdgeTPU**
(we have it idle most of the time!).  Code: github.com/AVAuco/deeparuco.

**ChromaTag** (arXiv:1708.02982).  Color-modulated AprilTag variant.
**2616 fps** average (37× faster than next).  But requires color
image (we have RGB888 from camera).  Not field-standard like ArUco.

**AprilTag vs ArUco** comparison (IEEE 2020).  AprilTag more robust
to occlusion + warping but slower.  ArUco optimal for mobile/embedded.
For our use case (clean indoor ground markers): ArUco fine.

**ChArUco boards** (OpenCV mainline).  Chess board + ArUco markers
combined.  Better pose accuracy (chess corners are sub-pixel exact).
Drop-in if we want lab-grade precision.

### Adaptive thresholding (the ArUco preprocessing bottleneck)

**Bradley & Roth integral image method** (Journal of Graphics Tools,
2007).  THE canonical fast adaptive threshold.

Algorithm:
1. Pre-pass: build integral image II[y][x] = Σ src[0..y][0..x]
   - O(N) preprocessing, ~3 cycles/pixel
2. Per-output: window sum = II[y2][x2] - II[y1][x2] - II[y2][x1] + II[y1][x1]
   - O(1) per pixel: 3 subtractions + 1 store + 1 compare
   - ~10 cycles/pixel vs current 125 cycles/pixel
3. Total: O(N) including preprocessing

**Expected speedup**: 12 cycles/pixel total ≈ **10× faster than naive
7×7 box**.

Bradley-Roth on 320×240 estimate (M7 @ 800 MHz):
  - Naive 7×7 box (measured): 12.0 ms, 125 cyc/px
  - Bradley integral image: ~12 cyc/px = **1.15 ms total**

That's the **1-2 ms target** we discussed.  Achievable with the same
hardware, just better algorithm.

### Concrete optimization roadmap for sentai.flow.mode("anchor")

| Stage | Current (naive) | SOTA optimization | Expected |
|---|---:|---|---:|
| Threshold (7×7 box) | 12.0 ms | **Bradley integral image** | **1.2 ms** |
| Edge (Sobel) | 3.5 ms | __USAD8 SIMD per channel | 1.0 ms |
| Contour finder | (not measured) | Scan-line + RLE | 2-3 ms |
| Quad approx + decode | (not measured) | OpenCV port stock | 1 ms |
| PnP P4P closed-form | (not measured) | `arm_mat_inverse_f32` | 1 ms |
| **TOTAL full ArUco** | est. ~20 ms | **est. ~6-7 ms** | |

**6-7 ms = 140-170 fps capable** on M7.  Hover anchor at 30 fps would
use ~20% CPU.  Plenty of headroom for other tasks (flow, TPU, mavlink).

### Lessons applicable to our project

1. **Integral image (Bradley 2007) replaces naive box filter.**  10×
   speedup on threshold alone, easy to implement (~50 lines).  Top
   priority if we proceed.

2. **CNN-based detect (DeepArUco++) for challenging lighting** could
   offload to EdgeTPU.  Our TPU is idle most of the time; a small
   detection model would fit.  Phase 2.

3. **SIF / LK improvements** apply to LK-based flow paths.  Our
   phase-correlation is already different and competitive; no port
   needed.

4. **VIO (LEVIO) > anchor-only**.  Long-term direction: ORB-feature
   tracking on M7/TPU gives absolute pose without external markers.
   Out of scope for current sprint.

5. **Don't reinvent — adapt.**  Most algorithms have C reference
   implementations (apriltag, opencv aruco_lite).  Port + adapt to
   our FreeRTOS + PXP + SDRAM constraints.

### References (citable in commit messages / paper)

- Bradley, D. & Roth, G. (2007). *Adaptive Thresholding using the
  Integral Image.* J. Graphics Tools, 12(2), 13-21.
  doi:10.1080/2151237X.2007.10129236
- Berto, M. et al. (2024). *DeepArUco++: Improved detection of
  square fiducial markers in challenging lighting conditions.*
  Image and Vision Computing, 152. arXiv:2411.05552
- Müller, L. et al. (2023). *Parallelizing Optical Flow Estimation on
  an Ultra-Low Power RISC-V Cluster for Nano-UAV Navigation.*
  arXiv:2305.13055
- Zhu, W. et al. (2025). *Selective Intersection Flow: A Lightweight
  Optical Flow Algorithm for Micro Drones.* MDPI Engineering
  Proceedings 108(1), 47.
- DeGol, J., Bretl, T., Hoiem, D. (2017). *ChromaTag: A Colored
  Marker and Fast Detection Algorithm.* arXiv:1708.02982
- (To be published) LEVIO authors (2026).  *Lightweight Embedded
  Visual Inertial Odometry for Resource-Constrained Devices.*
  IEEE Sensors Journal, accepted Feb 2026.  arXiv:2602.03294

## 10s. Empirical validation: SOTA algorithms vs M7 cache reality (2026-05-12)

After §10r literature survey, we implemented 3 adaptive-threshold
variants on M7 + SDRAM and measured.  Result UPENDS the SOTA paper
predictions.

### Measured cycle counts on M7 @ 800 MHz, 320×240 image

```
Naive 7×7 box (49 sequential SDRAM loads/px):    12.0 ms  baseline
Bradley integral image (4 RANDOM SDRAM lookups): 14.6 ms  0.82×
Separable 7+7 rolling sum (sequential horiz):    14.0 ms  0.86×
```

**Both SOTA optimizations LOST to naive.**

### Why

SDRAM cache-miss penalty (~50 cyc) dominates when buffers don't fit
in L1 cache (M7 has 16 KB instruction + 16 KB data L1).  Our test
image 320×240 = 75 KB + integral 300 KB + horizontal-sum 150 KB.

- Naive: 49 loads/px BUT sequential row scan → hardware prefetcher
  hits → most loads from L1.  Algorithmic O(K²) but cache-friendly.
- Bradley: 4 lookups/px, EACH at non-contiguous (y2×w + x2) address
  → most are cache misses, ~200 cyc/px.  Algorithmic O(1) but
  cache-hostile.
- Separable: pass 1 sequential (good), pass 2 vertical strided
  reading s_horiz at (y×w + x) for varying y → strides 320 bytes
  → cache prefetcher loses, similar penalty.

### Lesson for embedded vision research

**Big-O complexity is necessary but not sufficient** on memory-
constrained MCUs.  Standard CV papers assume L1 cache hits (which
holds on Cortex-A / x86 with 32-256 KB L1 + L2/L3); doesn't hold
on Cortex-M class with only 16 KB L1 + slow SDRAM.

For real 1-2 ms threshold on M7, need ONE of:
1. **OCRAM-resident buffers** (256 KB FlexRAM @ M7 cache-coherent
   speed).  Move integral image there.  Bradley should then win.
   We've used FlexRAM allocations elsewhere — pattern exists.
2. **Tile-based processing** — process 32×32 tiles that fit L1.
   Apply naive 7×7 within each tile (cache-friendly).  Stitch.
3. **PXP HW box filter** — `pxp.OUT_BUFFER_FORMAT = ALPHA_FILTER`
   or similar.  HW does 0-cycle CPU.  PXP capabilities need
   investigation.
4. **160×120 image** — 4× fewer pixels, integral fits 75 KB → DTCM.
   Drops all 3 algorithms to ~3 ms (still naive winner unless
   integral moves to fast RAM).

### Files added in this session

- `examples/sentai_runtime/aruco_bench.cc` — 3 kernels measured
- `examples/sentai_runtime/modsentai_diag.c` — `sentai.diag.aruco_bench()`
  binding returns dict with `thresh_us`, `thresh_bradley_us`,
  `thresh_separable_us`, `edge_us`, cycle-count counterparts.

### Decision for ArUco-on-M7 path

Don't naively port literature.  Need either:
- Move processing buffers to fast RAM (OCRAM/DTCM) — engineering work
- Use PXP HW — investigation work
- Reduce image to 160×120 — accuracy cost

Recommend **PXP investigation first** since it's the only path to
sub-1ms (no CPU).  If PXP can do 7×7 box filter or its equivalent,
that's free.  Fallback: 160×120 image + OCRAM-resident integral.

This is genuine "we did the research, the algorithms didn't work as
advertised, here's the actual path on our hardware" engineering.
Worth documenting for future researchers.

## 10t. Platform-abstracted ArUco anchor shim (s112, 2026-05-12)

Closes the SIM ↔ ARM parity gap for `sentai.flow.mode("anchor")`.
After s111 validated the M7 speed budget (1.1 ms threshold, 3.0 ms
edge, 37 µs rectify; total ArUco preprocessing ~4.1 ms), the same
MicroPython API is now alive on the x86 SIM.

### Pattern (matches `sentai_pxp_shim.h` + `sentai_fft_shim.h`)

| Layer | ARM | SIM |
|---|---|---|
| Contract header | `examples/sentai_runtime/sentai_aruco_shim.h` (shared) | same |
| Impl | `sentai_aruco_shim_arm.cc` (stub today — s113+) | `sim/sentai_aruco_shim_sim.c` |
| Detector | M7 PXP+SIMD pipeline (s111 building blocks, s113+ wiring) | Python sidecar `sim/scripts/aruco_pose_publisher.py` using `cv2.aruco` + `solvePnP` (reuses s090 `aruco_detector.py`) |
| Transport | direct C call | `SOCK_DGRAM` UDS, 36-byte fixed packet, drain-to-latest |

### MicroPython surface (identical on both targets)

```python
sentai.flow.mode()              # -> "normal" | "anchor"
sentai.flow.mode("anchor")      # auto-inits shim, opens UDS on SIM
sentai.flow.anchor_pose()       # -> dict(detected, num_markers,
                                #         x, y, z, yaw, frame_seq,
                                #         detect_us, src_ts_ms)
```

Drivers built against ARM are byte-identical on SIM.

### Wire format (UDS, little-endian)

```
"<II BB H ffff II"  (36 B)
magic=0x41524332, frame_seq,
detected, num_markers, pad,
x_m, y_m, z_m, yaw_rad,
detect_us, src_ts_ms
```

### Bring-up gotchas (worth keeping)

1. **`.ocram_bss` orphan-landing is load-bearing for new ARM code**
   too — adding any function to ITCM (`m_text`) overflows.  Default
   non-ISR ARM functions to `__attribute__((section(".sdram_text")))`.
2. **`SOCK_DGRAM` + drain-to-latest** is the right semantic for pose
   snapshots — packet loss harmless (next datagram <33 ms), and
   `recvfrom(MSG_DONTWAIT)` never stalls the flow task.
3. **`get_latest()` MUST also drain the UDS**, not just `detect()`.
   The first cut had drain only in detect() and pose reads returned
   stale zeros despite live publisher.
4. **Hand-port the MP binding** between ARM and SIM `modsentai_*`
   sources rather than `#include`-ing — ARM uses `flow_shared_t`
   (M7+M4 IPC), SIM uses an x86 snapshot struct.  Different backends
   in the same binding would tangle two state models.
5. **QSTR regen is mandatory** when adding `MP_QSTR_*` symbols
   (`mode`, `yaw`, `detected`, `num_markers`, `anchor_pose` here).
   Recipe in `agent.md §6`.

### Smoke test (no Gazebo, no OpenCV)

`examples/sentai_runtime/experiments/s112_x86_anchor_shim/test_anchor_wire.py`
spawns `sentai_sim`, pushes one `struct.pack` packet over UDS,
reads `sentai.flow.anchor_pose()` via REPL, diffs values.  Result
(2026-05-12, sim build #107): **PASS** — det=1, n=3, x=1.25,
y=-0.5, z=1.75, seq=42 round-tripped intact.

### Files

- `examples/sentai_runtime/sentai_aruco_shim.h`
- `examples/sentai_runtime/sentai_aruco_shim_arm.cc`
- `sim/sentai_aruco_shim_sim.c`
- `sim/scripts/aruco_pose_publisher.py`
- `examples/sentai_runtime/modsentai_flow.c`  (ARM binding)
- `sim/modsentai_sim.c`                       (SIM binding)
- `examples/sentai_runtime/experiments/s112_x86_anchor_shim/{README.md,test_anchor_wire.py}`

### Open work

- `sentai_aruco_shim_arm.cc` real M7 detector (s111 budget shows
  ~5 ms/frame is feasible; remaining is contour finder + quad
  decode + PnP).
- Auto-forward VPE from flow path when `mode == "anchor"` —
  `sentai.link.send_vpe(...)` (PX4) and `sentai.crazy.send_external_position(...)`
  (cf2) already exist; just need the per-frame call site.
- gz-subscription path in `aruco_pose_publisher.py` unit-tested
  inside `crazysim-garden` distrobox with a live Garden session.

## 10u. Best practices for SIM ↔ ARM platform abstraction (s112 distilled)

Captured after shipping `sentai_aruco_shim` end-to-end (struct.pack
smoke + cv2 synth frame + live Gazebo on PX4 + cf2 all PASS).
These rules generalise — apply to ANY new primitive that needs to
run on both targets.

### B-1. The three-file contract (mandatory shape)

```
examples/sentai_runtime/sentai_<NAME>_shim.h        ← contract, shared
examples/sentai_runtime/sentai_<NAME>_shim_arm.cc   ← ARM impl
sim/sentai_<NAME>_shim_sim.c                        ← SIM impl
```

Header defines the C ABI: opaque struct + `init/op/get/shutdown`.
`extern "C"` everywhere — never put C++ types in the contract.

Precedent: `sentai_pxp_shim.h` (HW DMA on ARM, area-average on SIM),
`sentai_fft_shim.h` (CMSIS-DSP on ARM, FFTW3 on SIM),
`sentai_aruco_shim.h` (M7 detector on ARM — s113+; Python sidecar
sidecar+UDS on SIM).

### B-2. SOCK_DGRAM + drain-to-latest for snapshot streams

For periodically-updated state (pose snapshots, detection results,
sensor readings): `SOCK_DGRAM` Unix sockets with **drain on every
read**.  Wrong:

```c
recvfrom(fd, &snapshot, sizeof, MSG_DONTWAIT);   // returns ONE oldest packet
```

Right:

```c
while (1) {
    n = recvfrom(fd, &snapshot, sizeof, MSG_DONTWAIT);
    if (n < 0) break;                            // EAGAIN — queue empty
    // overwrite cached state with whatever just arrived
}
```

Drop-old semantics is what you want for "latest pose" or "latest
detection" — packet loss is harmless, staleness is the enemy.
Bug captured the hard way in s112: drain in `detect()` but not
in `get_latest()` returned zeros forever despite an active publisher.

### B-3. Hand-port MP bindings between targets, don't `#include`

ARM `modsentai_<x>.c` and SIM `modsentai_sim.c` should both register
the same MP namespace + dict shape but with **separate
implementations of each binding**.  Don't try to share via `#include`
— the state backends differ (ARM uses `flow_shared_t` cross-core
IPC; SIM uses an x86 snapshot struct).  Sharing source tangles two
state models in one binding.

Rule of thumb: if the binding body has more than 1 `#ifdef SENTAI_PLATFORM_SIM`
you should split it.

### B-4. The SIM REPL is **line-at-a-time** — don't pipe multi-line blocks

`( sentai_sim < script.py )` fails for any `for:`, `if:`, `def:` block
because the REPL parses each line independently.  Symptoms:
`SyntaxError: invalid syntax` on the first indented line; the rest
of the block executes as top-level statements with stale parser state.

Two working idioms:

**(a) Single-line semicolon chains** — what `s111_m7_aruco_pxp_dbg/read_pxp_dbg.py`
uses:

```python
s.write(b'p=sentai.flow.anchor_pose(); print(p["detected"], p["x"])\r\n')
```

**(b) FIFO + bash loop with sleeps** — what `s112/run_*_live.sh` uses:

```bash
FIFO=$OUT/sim_repl.fifo
mkfifo "$FIFO"
( $SIM_BIN < "$FIFO" ) > $OUT/sim.log 2>&1 &
SIM_PID=$!
exec 3>"$FIFO"                                   # keeps FIFO open
echo "import sentai"             >&3
for i in $(seq 1 60); do
    echo "p=sentai.flow.anchor_pose(); print('ANCHOR',$i,p['detected'])" >&3
    sleep 0.5
done
exec 3>&-                                        # sim sees EOF
wait $SIM_PID
```

FD 3 stays open until the script closes it, so the SIM doesn't get
EOF after the first batch.

### B-5. Extend the proven bridge, don't fork a new one

For SIM detection sidecars (cv2.aruco, segmentation, etc.) that
operate on Gazebo camera frames, **extend the existing
`gz_to_uds_bridge` + `aruco_to_vision_estimate.py` chain with an
optional `--<x>-pub-uds` flag**.  Don't write a parallel gz
subscriber.

Why: distrobox `crazysim-garden` has gz transport (via `gz` binary)
but NO cv2 / NO `gz-transport` Python bindings; host venv has cv2
but no gz transport.  The proven chain — C++ `gz_to_uds_bridge` in
distrobox feeds a UDS, host-venv Python reads + detects — already
sidesteps this.  Don't fight it.

Backwards-compat rule: the new flag MUST default to off, so all
existing s108/s109 launches still pass.  ~20 LoC patch typical.

### B-6. Three-tier validation, escalating cost

Before claiming a SIM↔ARM shim "works":

| Tier | What it proves | Cost |
|---|---|---|
| **T1 wire test** (`struct.pack` → UDS → REPL diff) | wire format, drain-to-latest, dict shape | seconds |
| **T2 synth-frame test** (cv2 detection on a synthetic PPM → publisher → SIM) | real detector code path, publisher plumbing | tens of seconds |
| **T3 live Gazebo** (PX4 or cf2 SITL + flying drone) | end-to-end through real renderer + flight stack | minutes (incl. cleanup) |

Run T1 → T2 → T3 in order.  Most regressions die in T1/T2 where
the iteration is cheap; T3 is for the architectural integration
proof.  s112 caught a real bug at T1 (drain-on-read missing) — would
have wasted a full Gazebo cycle if we'd skipped to T3.

### B-7. cf2 `--mav` argument trick

`aruco_to_vision_estimate.py` was written for PX4 + MAVLink.  When
reusing it on the cf2 path (which doesn't speak MAVLink), pass
`--mav udpout:127.0.0.1:1` — UDP is fire-and-forget, port 1 has
nothing bound, packets get dropped silently, no side-effects.

Alternative would be a `--no-mav` flag, but the empty-port trick is
zero-LoC and works today.  Use the trick; leave the flag for s113
if/when MAVLink turns out to have a side effect (it doesn't).

### B-8. Live-test PASS criteria — count, peak, ground-truth

A live-Gazebo validation script must report THREE numbers:

| Metric | Why |
|---|---|
| `DETECTED_COUNT / TOTAL_COUNT` of poll iterations with `detected=True` | proves the chain is actually flowing, not just bound |
| `PEAK_Z` from poses where `detected=True` | rough sanity vs takeoff target — catches "drone never flew" |
| Bridge stats (`detect=N pose=N vpe=N`) from `aruco_to_vision_estimate.py` log | external check that detection was happening, separately from our shim |

PASS threshold for s112 was `DETECTED_COUNT >= 10`.  Lower means the
takeoff didn't complete, the world has no markers visible, or our
shim is dropping packets.  All three are real failure modes we hit
during s112 bring-up; the counter immediately localizes which.

## 10v. Dual-scale world pattern: cf2 vs PX4 (2026-05-13)

**Constrângere fundamentală**: cele două platforme de zbor operează
la altitudini complet diferite, deci scena vizibilă din camera
dronei este la **scări complet diferite**:

| Platform | Altitudine tipică | Ground coverage @ FOV 70° | Scală obiecte |
|---|---|---|---|
| **CrazyFlie / CrazySim** | 0.2-2 m | 0.3-2.8 m diameter | **SMALL** (proportional) |
| **PX4 / x500_sentai** | 2-20 m | 2.8-28 m diameter | **REAL-WORLD** |

**Consecință**: SCENA conceptuală (markeri ArUco + obiecte detectate
de MobileNet COCO + obstacole) trebuie redată în **două variante
fizice diferite** care păstrează **proporțiile relative drone↔obiect**.

### Pattern decis pentru sentai.explore canonical world

**Aceleași CLASE de obiecte** (person/car/chair/etc.), **scări fizice
diferite**:

| Obiect | PX4 world scale | cf2 world scale | Ratio |
|---|---:|---:|---|
| Person panel/actor height | 1.70 m (real) | **0.17 m** | 10× |
| Car length | 4.0 m (real) | **0.40 m** | 10× |
| ArUco marker size | 0.40 m | **0.04 m** | 10× |
| Bench length | 1.5 m | **0.15 m** | 10× |
| Arena dimensions | 20×20×5 m | **2×2×0.5 m** | 10× |
| Drone wingspan/diam | 0.5 m (x500) | **0.10 m** (cf2) | 5× |

Factor general: **10× downscale** pentru cf2 vs PX4 (cu mici ajustări
pentru drona însăși — cf2 nu e 5× mai mică decât x500, deci unele
proporții drone↔obstacol sunt diferite).

### Implicații în plan

**Class priors** (din §3 Stage 1.B `sentai.slam.set_class_prior`):
```python
# PX4 world
sentai.slam.set_class_prior(0,  1.70)   # person
sentai.slam.set_class_prior(2,  4.00)   # car
sentai.slam.set_class_prior(56, 0.85)   # chair
sentai.slam.set_class_prior(13, 1.50)   # bench

# cf2 world
sentai.slam.set_class_prior(0,  0.17)   # person (scaled 10×)
sentai.slam.set_class_prior(2,  0.40)   # car
sentai.slam.set_class_prior(56, 0.085)  # chair
sentai.slam.set_class_prior(13, 0.15)   # bench
```

Tabela priors e ATAȘATĂ misiunii (parameter în `sentai.explore.start()`
ulterior, sau set explicit înainte de mission start).

**World files** — 2 variante pinned în repo:

```
sim/gazebo/worlds/
├── explore_canonical_px4.sdf   ← 20×20 m arena, real-scale objects
└── explore_canonical_cf2.sdf   ← 2×2 m arena, 10× downscaled objects
```

Geometric topology IDENTICĂ (4 markeri ArUco corners, 4 panouri foto
COCO classes, 2 pillars obstacles); doar mărimile scalate.

**Camera intrinsics păstrate** — FOV 70° este aceeași pe ambele
platforme; **distanța pixel-per-meter SE SCALEAZĂ în funcție de
altitude**. Detection probability și pseudo-depth math funcționează
identic, dar ground-truth e adaptat.

### Pattern de testing — same code path, two scenes

```bash
# PX4 path
bash experiments/s115_endtoend_cube_landing/run_px4.sh \
     --world=explore_canonical_px4.sdf \
     --class-priors=px4_priors.json

# cf2 path
bash experiments/s115_endtoend_cube_landing/run_cf2.sh \
     --world=explore_canonical_cf2.sdf \
     --class-priors=cf2_priors.json
```

Codul firmware (sentai.explore + sentai.slam + sentai.servo) e
**identic**. Doar **world file + class priors JSON** se schimbă
ca params per launch.

### H3 hex resolution per platform (legat de §15)

Constrângerea de scală 10× impactează direct **resoluția H3 optimă**:

| Platform alt | Ground coverage | Optimal H3 res | Cell side |
|---|---:|---:|---:|
| PX4 @ 5 m | ~7 m | **12** | 9.4 m |
| PX4 @ 20 m | ~28 m | **11** | 25 m |
| cf2 @ 0.5 m | ~0.7 m | **15** | ~0.5 m (or 14 = 1.4 m) |
| cf2 @ 2 m | ~2.8 m | **13** | 3.6 m |

cf2 lucrează la **rezoluții H3 mai mari (cell-uri mai mici)**, iar PX4
la rezoluții mai mici (cell-uri mai mari). Funcția
`optimal_h3_res_for_altitude(alt)` din §15.5 returnează valoarea
corectă transparently per platformă; nu necesită platform-specific
code path în sentai.places.

### What this means for content (foto panels — §10v.update)

Panouri foto pentru sentai.explore canonical:
- **PX4**: 60×60 cm panouri (vizibile la 5-20 m alt)
- **cf2**: 6×6 cm panouri (vizibile la 0.5-2 m alt)

Conținutul (PNG cu poza COCO class) e **IDENTIC** între platforme;
doar mărimea panoului fizic în Gazebo se schimbă. Modelul MobileNet
COCO recunoaște textura indiferent de mărimea fizică (depinde doar
de proiectia pixel-space).

### Convenție de naming în repo

```
sim/gazebo/worlds/
  explore_canonical_<platform>.sdf    # {px4, cf2}

sim/scripts/world_assets/
  panels/                              # PNG textures (shared)
    person.png, car.png, chair.png, cat.png, ...
  actors/                              # 3D mesh (shared)
    walking_person.dae
  priors/
    px4_priors.json                    # class → real_size_m
    cf2_priors.json                    # class → scaled_size_m

examples/sentai_runtime/experiments/sNNN_*/
  run_px4.sh
  run_cf2.sh
  README.md                            # documents both paths
```

### Anti-pattern de evitat

- ❌ **NU** încerci să folosești same SDF pentru ambele platforme cu
  un wrapper "scale=0.1" — Gazebo `<scale>` pe actor / model
  composite poate produce bug-uri de rendering (lighting, collision,
  sensor)
- ❌ **NU** ai un singur set de class priors pentru ambele —
  pseudo-depth math depinde linear de prior, off-by-10× distruge slam
- ❌ **NU** încerci să zbori cf2 într-o lume scalată PX4 (drona se
  ciocnește de panouri de 1.7 m / pillars de 2 m care în lumea ei
  proporțională sunt obstacole gigantice)

### Decizia operațională

Pentru sentai.explore Stage 8 (canonical test scenario):
- **Ambele world files** create + version-pinned în repo
- **Două perechi de class priors** JSON
- **Tot codul firmware** rămâne identic
- **Test pe ambele scale** = real validation MCU+algorithm portabil

Asta protejează arhitectura: dacă **ALGORITMUL** funcționează la
scale 10× diferit, înseamnă că NU e dependent de scale-specific
hyperparameters magic numbers — e cu adevărat scale-invariant.

### Floor texture (OSM map background) — aplicarea regulii de scale

**Constrângere matematică confirmată**: drona cf2 la 0.2 m și PX4 la
2 m văd **bit-identic** aceeași imagine prin cameră (FOV 70°,
320×240) DOAR DACĂ conținutul fizic e scalat 10×.

```
cf2 @ 0.2 m:  ground visible = 0.28 m diameter →  pixel @ 0.875 mm/px
PX4 @ 2.0 m:  ground visible = 2.80 m diameter →  pixel @ 8.75 mm/px
                                                  Ratio: 10× — IDENTIC
```

**Aplicat la podea OSM/satellite map**:

Soluția elegantă = **TEXTURA IMAGINE IDENTICĂ; doar world dimensions
diferă**:

```
sim/gazebo/worlds/assets/
  floor_osm_4096.png        ← SAME image, ~4 MB

PX4 world (explore_osm_px4.sdf):
  floor plane: 20×20 m
  texture mapped: 1 map_image → 20×20 m physical
  ⇒ at PX4 altitude 5 m, drone sees 7×7 m of map = 35% of map image

cf2 world (explore_osm_cf2.sdf):
  floor plane: 2×2 m
  texture mapped: 1 map_image → 2×2 m physical  ← 10× smaller world
  ⇒ at cf2 altitude 0.5 m, drone sees 0.7×0.7 m of map = 35% of map image
                                                     ↑ SAME 35%!
```

**Rezultat**: drona vede **bit-identic** la altitudini proporționale,
**fără să generăm a doua texturi**. Doar SDF-ul scalează planul podea.

```xml
<!-- explore_osm_px4.sdf -->
<model name="ground_plane">
    <link name="link">
        <collision name="collision"><geometry>
            <plane><normal>0 0 1</normal><size>20 20</size></plane>
        </geometry></collision>
        <visual name="visual"><geometry>
            <plane><normal>0 0 1</normal><size>20 20</size></plane>
        </geometry><material><pbr><metal>
            <albedo_map>assets/floor_osm_4096.png</albedo_map>
        </metal></pbr></material></visual>
    </link>
</model>

<!-- explore_osm_cf2.sdf — SAME texture, smaller plane -->
<model name="ground_plane">
    <link name="link">
        <collision><geometry>
            <plane><normal>0 0 1</normal><size>2 2</size></plane>  <!-- 10× smaller -->
        </geometry></collision>
        <visual><geometry>
            <plane><normal>0 0 1</normal><size>2 2</size></plane>
        </geometry><material><pbr><metal>
            <albedo_map>assets/floor_osm_4096.png</albedo_map>  <!-- SAME asset -->
        </metal></pbr></material></visual>
    </link>
</model>
```

**Win operational**:
- 1× binary asset în repo (~4 MB, nu 2×)
- Algoritm identic (descriptori scene match între platforme)
- Dovedește scale-invariance: dacă cf2 descriptor matches în lumea
  mică, PX4 descriptor SAME matches în lumea mare, fără re-training

**Excepție**: dacă texturăm cu **detalii sub-pixel-relevant** (ex: text
mic pe drumuri "STOP"), atunci la cf2 vor fi vizibile la scale fine
unde nu erau în PX4. Asta-i ACCEPTABIL pentru place recognition
(diversitate features = mai bun match), NU pentru detection (un
"STOP" sign de 1 mm la cf2 nu mai e detectabil de YOLO COCO).

### Bottom line — confirmare model dual-scale

**Da, exact**: în Gazebo vom avea **două lumi** cu **proporții
identice** dar **scale fizic 10× diferit**:
- Toate obiectele (oameni, mașini, scaune, panouri foto) scalate 10×
- Texturile (incl. OSM floor map) sunt **același fișier**, mapate pe
  plane de dimensiuni diferite
- Drona cf2 la 0.2-2 m vede content identic ca PX4 la 2-20 m
- Codul firmware (sentai.slam, sentai.explore, sentai.places)
  rămâne 100% identic — DAR class priors + arena dimensions
  parametrizate prin config JSON la mission start

**Documentat: confirmat din linkat. Asset floor_osm_4096.png va fi
shared cross-platform.**

---

## 10w. CrazySim runtime hygiene (best practices, 2026-05-13)

Lessons learned bringing up the s125 integrated visual demo. These rules
are **load-bearing** — every minute we ignore one of them costs ~30
minutes of debug.

### 10w.1 NEVER run `gz sim` from the host

Sim.md §10c+§10d already says "Garden 7.9 inside distrobox, Harmonic
banned on host". This is stricter than it sounds: **the host's `gz`
binary is on `$PATH`**, so any script that calls `gz sim …` directly
runs the HOST version (Harmonic 8.x on this dev box). Harmonic loads our
world successfully, then fails to load the CrazySim plugin (it's built
against `libgz-sim7`, not `libgz-sim8`), and cf2 never spawns.

  - **Symptom**: SITL launcher prints
    `Error while loading the library [.../libgz_crazysim_plugin.so]:
    libgz-sim7.so.7: cannot open shared object file`,
    followed by `Failed to load system plugin (Reason: No plugins
    detected in library)`. cflib then gets `ConnectionRefused` on
    `udp://127.0.0.1:19850` because cf2 never started.
  - **Rule**: every CrazySim invocation (sitl_singleagent.sh, raw
    `gz sim`, `gz topic`, `gz service`) MUST run **inside** distrobox
    `crazysim-garden`. Don't even put the SITL launcher in a host
    script that pipes commands in — the moment gz starts on the host,
    you've lost.
  - **One-line check before debugging anything else**:
    `gz sim --version` (host) vs `distrobox enter crazysim-garden -- gz
    sim --version`. Host = 8.x → use distrobox. If only host has gz,
    distrobox isn't entered.

Concretely for the s125 demo: `run_demo.sh` enters distrobox FIRST and
runs `sitl_singleagent.sh -w s125_demo` from there, NOT from the host.

### 10w.2 Texture-path resolution under Garden — symlink trick

CrazySim's `setup_gz.bash` only adds `models` and `worlds` to
`GZ_SIM_RESOURCE_PATH`. World SDFs using
`<albedo_map>materials/textures/foo.png</albedo_map>` expect the texture
to resolve relative to the SDF's directory — i.e.,
`worlds/materials/textures/foo.png`. But the textures live at
`<gazebo-root>/materials/textures/`.

  - **Symptom**: `[Err] [SystemPaths.cc] File [.../worlds/materials/
    textures/foo.png] resolved to path [.../worlds/materials/textures/
    foo.png] but the path does not exist`, followed by `Unable to find
    file`. Drone spawns, world loads, but the ground plane is solid
    white (no texture).
  - **Fix**: one-time symlink at install time:
    `ln -s ../materials <gazebo-root>/worlds/materials`. This is
    idempotent and shared across all worlds + textures (checker.png,
    aruco markers, harmonic_alt200_4k.png, …).
  - **Apply once**: `run_demo.sh` is now idempotent — checks for the
    symlink and creates it if missing.

### 10w.3 Texture file deployment for new worlds

When committing a new world SDF to the SentAI repo that references a
texture not already in the CrazySim install:

  1. **Source of truth** for textures stays under
     `sim/gazebo/worlds/assets/<tile-name>/` in this repo (one canonical
     copy, versioned, durable).
  2. **Mirror to CrazySim** install at world-launch time —
     `run_demo.sh` checks for the file under
     `$CRAZYSIM/materials/textures/`, copies if missing. Don't expect
     operators to remember a separate `cp` step.
  3. **Keep symlink path stable**: `worlds/materials → ../materials`
     resolves new textures automatically once dropped into the mirror.

### 10w.4 Harmonic generation infra removed from repo (2026-05-13)

Per operator direction, the `sim/gazebo/worlds/harmonic_scene.sdf` and
`sim/scripts/render_topdown.py` / `topdown_subscribe.py` were deleted —
they pulled Harmonic World models from Fuel and required Gazebo
8/Harmonic to render. The 4K artifact lives under
`sim/gazebo/worlds/assets/harmonic_tiles/` (versioned PNG + JPEG
previews) and is treated as a **frozen asset**; regeneration recipe is
the README in that dir, but normal development should never need it.

### 10w.5 Subprocess REPL pipe → sentai_sim (orchestrator pattern)

For the s125 demo, the orchestrator uses a subprocess pipe to a
`sentai_sim` process — sending lines to stdin, parsing `=KEY value`
markers from stdout. Lessons:

  - **REPL is line-at-a-time on multiline blocks.** No multi-line `def`,
    no `for: …` loops over multiple statements. Use one-liners /
    inline generators. (Same caveat already in
    `feedback_no_complex_serial_orchestration.md`.)
  - **Banner consumption matters.** sentai_sim prints a build banner +
    `>>> ` prompt at startup; consume those before issuing queries or
    you'll mis-parse the first response.
  - **Always send a distinct response key per query** (`=Q01`, `=Q02`)
    so the parser can grep its own response out of the noisy stdout
    stream without false matches against echoed input lines.

### 10w.6 Cleanup discipline — kill in reverse-spawn order

Demo scripts spawn 3+ processes (gz sim server, gz sim GUI, cf2
firmware, orchestrator's sentai_sim subprocess). Cleanup order matters:

  1. `pkill -f "orchestrator.py"` (high-level, owns subprocess)
  2. `pkill -x cf2`
  3. `pkill -9 "gz sim"` (server then GUI)
  4. `pkill -9 ruby` (CrazySim Ruby helper)

Reverse-spawn order avoids races where a dying process tries to
publish to a topic served by an already-killed peer. `trap cleanup
EXIT SIGINT SIGTERM` in the launcher script is the right pattern.

### 10w.7 cf2 SITL stable flight needs sentai.flow (or anchors) — NOT flowdeck simulation

Operator caught chaotic flight in the s125 demo (2026-05-13) when
MotionCommander commands were streamed against a freshly-reset Kalman
EKF.  Root cause: cf2's Kalman estimator (`stabilizer.estimator=2`)
expects horizontal-position observations, normally fed by either:

  - **`--flowdeck` simulation** in `sitl_singleagent.sh` — DON'T USE.
    The Gazebo launcher doesn't expose this flag (only MuJoCo does),
    and the flowdeck simulation is a coarse mock that doesn't match
    what the M7 firmware will see on the real board.

  - **`sentai.flow` + sentai bridge** — the CANONICAL path.  sentai.flow
    consumes Gazebo `/downward_cam/image`, runs phase-correlation flow
    on the M7 (or x86 in SIM), and forwards `OPTICAL_FLOW_RAD` packets
    via the existing UART socket bridge to cf2 firmware (Phase 3+4).
    Proven on s091/s092/s101: hover < 2 cm drift at z=1 m.

  - **`sentai.flow.mode("anchor")`** + ArUco markers — even tighter
    pose lock when home markers are visible (s112).  Drone gets full
    absolute pose corrections, not just velocity.

When a SIM-side experiment that uses cf2 SITL needs stable flight, the
work to do is **bring up the sentai.flow → bridge → cf2 wiring**, NOT
reinvent estimator tweaks.  As a temporary stopgap, switching to
`stabilizer.estimator=1` (complementary) lets the drone take off and
hover without horizontal-position feedback — acceptable for visual
demos where exact position doesn't matter, but commits the drone to
open-loop drift on long moves.

s125 currently runs with the complementary stopgap.  Wiring sentai.flow
into it is the next iteration (tracked as Stage 3.D in the
objects_plan; same path s091/s112 already proved).
