# TD-S10-A8 - Research ARM Emulator Runtime Path

## Goal

Pause the B7 POSIX/x86 runtime refactor path and research a closer simulator
target: run the ARM firmware image, ARM FreeRTOS port, Cortex-M exception model,
ISR/task wakeups, and ARM-oriented memory layout inside an emulator on the host.

The objective is not "another x86 SIM".  A8 asks whether SentAI can move from:

```text
shared runtime code + FreeRTOS POSIX port + x86 backends
```

to:

```text
ARM firmware ELF + Cortex-M emulator + modeled board/peripherals
```

so that the scheduler, NVIC/SysTick/PendSV/SVC path, ISR notifications, stack
layout, and ARM build assumptions are much closer to the physical RT1176 board.

## Why Now

B7 made progress on namespace parity, shared bindings, virtual camera, TFL/TPU
experiments, and SIM backend cleanup.  But the current POSIX/x86 path reached a
dead end for virtual-camera + pipeline + TPU/Flow stability.  The failure mode
looks architectural: POSIX scheduler semantics, host helper I/O, and partially
migrated task/event ownership are different enough from the ARM runtime that
debugging the x86 simulator is no longer the best next move.

A8 is a research reset: find the best path to emulate the actual ARM runtime
model rather than continue layering fixes onto POSIX.

## Research Questions

1. Which emulator path can run a Cortex-M7 FreeRTOS image with realistic
   SysTick/PendSV/SVC/NVIC behavior?
2. Can we boot the SentAI ARM ELF, or do we first need a reduced firmware
   profile?
3. How much of MIMXRT1176/EVK must be modeled before SentAI reaches the REPL?
4. How should emulator-side camera frames enter the same frame-buffer/ISR path
   used by the physical camera?
5. How should filesystem artifacts be staged and recovered: emulated flash/SD,
   semihosting, or host bridge?
6. Is USB host + Coral EdgeTPU feasible in an emulator, or should TPU be
   deferred/stubbed for the first ARM-emulated slice?
7. What should remain in POSIX SIM after an ARM emulator path exists?

## Desired Fidelity Order

The emulator should prioritize fidelity in this order:

1. ARM Cortex-M exception/task model:
   SysTick, PendSV, SVC, BASEPRI/PRIMASK, NVIC interrupt delivery, task stacks.
2. Static memory layout:
   OCRAM/ITCM/DTCM/SDRAM-ish regions, linker symbols, no MP heap surprises.
3. Camera/PrepTask event shape:
   frame buffers filled, camera ISR fired, PrepTask woken the same way as ARM.
4. REPL and mission execution:
   UART/console or semihosted command channel, import from staged FS image.
5. Flight recorder artifacts:
   deterministic logs/events/scalars exported after each run.
6. TPU/USB:
   only after the above is stable; full USB Coral emulation is likely a later
   research branch, not the first milestone.

## Non-Goals

- Do not preserve the B7 POSIX/x86 simulator as the primary proof path for
  task/ISR behavior.
- Do not emulate camera optics or Gazebo physics inside the ARM emulator.
- Do not require cycle-accurate PXP/CSI/USB modeling in the first slice.
- Do not try to emulate the Coral USB protocol end-to-end before the basic
  ARM firmware boots and runs deterministic camera/PrepTask tests.
- Do not move compute-intensive loops into MP to make emulation easier.

## Candidate Emulator Families

### QEMU

QEMU supports Arm M-profile architecture emulation, including Armv7-M/Armv8-M
style Cortex-M targets, and FreeRTOS documents an official QEMU Cortex-M3 MPS2
demo path.  This makes QEMU a credible tool for FreeRTOS kernel/exception
smoke tests.

Likely fit:

- good for a first "ARM FreeRTOS task/ISR proof" using an existing QEMU board
  such as MPS2/AN385;
- useful for GDB, CI, and catching ARM-only compiler/linker/runtime bugs;
- poor fit for direct MIMXRT1176 board fidelity unless we write substantial
  custom machine/peripheral models.

Risk:

- MIMXRT1176 is not a standard QEMU machine target;
- camera CSI, PXP, SEMC, FlexSPI, SDHC, EHCI host, and GPIO/IOMUX modeling
  would require custom QEMU device work;
- QEMU's strength is CPU/system emulation, not turnkey modern MCU board
  peripheral fidelity.

### Renode

Renode is designed around configurable virtual platforms, custom peripheral
models, host integration, GDB/debugging, tracing, deterministic execution, and
CI.  It has NXP platform ecosystem support and a Zephyr dashboard entry for
MIMXRT1170-EVK/EVKB, which is a strong signal that the target family is not
alien to the toolchain.

Likely fit:

- best candidate for an A8 SentAI virtual platform;
- model missing peripherals incrementally in C#/Python;
- inject deterministic camera frames by modeling the CSI/PXP-facing boundary;
- expose UART/console and FS artifacts cleanly for tests;
- use Robot Framework/CI-style automation later.

Risk:

- still requires platform work for the exact SentAI/MCUXpresso firmware shape;
- may not model NXP CSI/PXP/USB host/EdgeTPU to the level we need out of the box;
- full USB host + Coral device behavior remains hard.

### Hybrid Path

Use a Cortex-M emulator for ARM firmware/task/ISR fidelity, but model only the
minimum platform boundary needed per test:

- camera provider model writes frame buffers and triggers the camera ISR;
- filesystem starts as a prebuilt flash/SD image or semihosted fixture bridge;
- TPU starts as not-ready/stub or CPU TFL smoke;
- USB Coral remains on hardware or on the old host baseline until the emulator
  path is proven.

This is the recommended A8 direction.

## Peripheral Strategy

### Camera

The first camera model should not emulate OV5640, MIPI CSI, and PXP
cycle-by-cycle.  It should emulate the effect the rest of SentAI depends on:

1. a 640x480 XRGB frame buffer becomes available in the same memory region the
   ARM code expects;
2. the camera frame metadata/camera ID is updated;
3. the same ISR/notification path fires as on ARM;
4. PrepTask wakes, performs enabled preprocessing, and publishes static slots.

Implementation options:

- Renode peripheral: C# model loads deterministic BMP/sequence files, writes
  guest RAM frame buffers, sets CSI-like status, triggers the interrupt line.
- QEMU custom device: memory-mapped peripheral with a timer/frame producer and
  interrupt; more effort than Renode for this project.
- Early compromise: compile a firmware profile where a board camera provider
  calls the same ISR-facing publication helper from task context.  This is less
  pure, but still preserves ARM FreeRTOS and task layout.

### USB / Coral EdgeTPU

Full USB host + Coral EdgeTPU emulation is not the first milestone.

Reasons:

- the physical path includes host controller, USB PHY/EHCI behavior, endpoints,
  DFU/runtime transitions, model upload phases, instructions/weights, and TPU
  responses;
- the Coral protocol is already complex enough on real hardware;
- an emulator would need either a real USB passthrough stack, a USB/IP bridge
  that matches the guest host controller, or a modeled EdgeTPU device.

Recommended first A8 policy:

- keep `sentai.tpu` present but return clear not-ready in ARM-emulator mode;
- validate camera/PrepTask/Flow/Markers/FR first;
- later evaluate a host-side "EdgeTPU mailbox peripheral" that preserves the
  ARM task API but does not pretend to be real USB;
- only after that consider real USB/IP or a modeled Coral device.

### Filesystem

Preferred first path:

- build a deterministic FS image per experiment;
- attach it as emulated flash/SD/block storage;
- export the image or mounted artifact directory after the emulator exits.
- B8 confirmed the preferred path as production FileX/LevelX over an emulated
  raw-NAND image.  For the first emulator milestones it is acceptable to stage
  host files into that NAND image before boot, then let the guest read them
  through normal `sentai.fs` APIs.  USB/MSC is useful later for board-parity
  workflows, but it is not required to unblock model/image/mission loading in
  the emulator.

Fallback:

- semihosted or Renode host-file bridge for development speed, as long as it
  is documented as a test fixture and does not become the ARM production path.

The key rule from B7 remains: mission source lives in the per-run FS root/image,
and stdin/console only imports and runs it.

### UART / REPL / Radio

Use a real emulated UART or semihosting console for early bring-up:

- boot log and REPL over UART;
- host script sends `import mission; mission.run()`;
- later, CRTP/MAVLink bridges can be modeled as UART/socket peripherals if
  needed.

## Expected A8 Output

A8 should produce:

1. a researched recommendation: Renode-first, QEMU backup;
2. a minimal emulator bring-up plan;
3. a peripheral fidelity matrix;
4. a first B8 implementation slice;
5. a clear "what not to emulate yet" list, especially USB Coral.

## Source Notes

- QEMU Arm docs: M-profile architecture support includes Armv6-M, Armv7-M,
  Armv8-M, and Armv8.1-M, but board/peripheral support is machine-specific.
  https://qemu-project.gitlab.io/qemu/system/arm/emulation.html
- FreeRTOS has official emulation/simulation material and a QEMU Cortex-M3 MPS2
  demo.  This validates QEMU for kernel-level FreeRTOS smoke, not necessarily
  for MIMXRT1176 board fidelity.
  https://freertos.org/Documentation/02-Kernel/03-Supported-devices/04-Demos/03-Emulation-and-simulation/01-Emulation-and-simulation
  https://www.freertos.org/Why-FreeRTOS/Quick-connect/qemu-mps2-an385-demo
- Renode supports virtual SoC construction, Cortex-M-family targets, host
  integration, deterministic control, GDB/debugging, and custom peripherals.
  https://antmicro.com/platforms/renode/
  https://renode.readthedocs.io/en/latest/advanced/writing-peripherals.html
- Zephyr documents MIMXRT1170-EVK/EVKB as a maintained board with MIMXRT1176,
  Cortex-M7/M4, SysTick/NVIC, SEMC, FlexSPI, USB, CSI, PXP, and related
  peripherals.  This is useful as a peripheral map and as evidence that the
  target family is actively described in modern embedded tooling.
  https://docs.zephyrproject.org/latest/boards/nxp/mimxrt1170_evk/doc/index.html
