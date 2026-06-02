# TD-S10-B8 - Research And Spike ARM Emulator Runtime

## Purpose

Execute the A8 research path: decide how SentAI should run closer to the ARM
firmware model on x86 host machines, without relying on the FreeRTOS POSIX port
as the primary task/ISR proof.

B8 is research plus a small spike.  It should not refactor production SentAI
runtime code until the emulator choice and first boot target are clear.

## Current Recommendation

Use **Renode-first**, with **QEMU as a smaller kernel/FreeRTOS smoke backup**.

Why:

- QEMU can emulate Arm M-profile CPU architecture and has FreeRTOS Cortex-M
  demos, so it is good for proving ARM FreeRTOS basics.
- QEMU does not currently give us an off-the-shelf MIMXRT1176 SentAI board.
  Modeling RT1176 peripherals in QEMU would be a substantial custom-machine
  project.
- Renode is designed for custom virtual platforms and custom peripherals.  It
  is a better fit for "run our ARM ELF and gradually add the peripherals that
  SentAI touches".
- Renode's host integration, deterministic execution, GDB support, and
  peripheral modeling are closer to what we need for camera frames, FS images,
  UART/REPL, and CI-style experiments.

## 2026-06-02 Research Update

B8 should not assume that "ARM emulator" automatically means "all real board
peripherals are available".  The critical split is:

```text
ARM task/ISR fidelity          -> emulator can plausibly help
RT1176 board/peripheral parity -> custom platform work
Coral USB parity              -> likely not first-slice emulator work
```

Current host state:

- `renode` is not installed in the local PATH, but source was cloned locally at
  `/home/bogdan/work/renode` (`ac18b5a`, branch `master`).
- `qemu-system-arm` is not installed in the local PATH.
- Zephyr source was cloned locally at `/home/bogdan/work/zephyrOS`
  (`0570f6d6b`, branch `main`) to mine RT1176 board/devicetree/platform data.
- no ready `sentai_runtime.elf` was found in the current build directories; the
  current tree has SIM builds and ARM build definitions, but the B8 spike must
  first produce or locate a CM7 ELF.

Current source-boundary observations:

- `examples/sentai_runtime` builds an ARM CM7 `sentai_runtime` executable with
  `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`.
- The runtime pulls in real board services early: `BOARD_InitHardware`,
  FileX/FxUser storage, `CameraTask`, CSI/PXP camera support, USB/EdgeTPU
  manager code, MicroPython task, FR, and several SentAI runtime tasks.
- The real camera path uses the NXP CSI receiver queue plus ISR bookkeeping in
  `libs/camera/camera_support.c`, and consumers eventually call
  `CAMERA_RECEIVER_GetFullBuffer` through `CameraTask`.
- Therefore the first emulated camera provider should inject into the same
  "full buffer available + interrupt/queue event" boundary, not into MP and not
  into a separate POSIX-only buffer model.

Clarification: Zephyr **does** document an RT1176 board target:
`mimxrt1170_evk@A/mimxrt1176/cm7`.  The Zephyr board page identifies the SoC
as `mimxrt1176`, the MCU as `MIMXRT1176DVMAA`, and lists the Cortex-M7/M4,
SDRAM, flash, USB connectors, PXP, CSI/MIPI camera-related nodes, clocks, and
other peripherals.  This is useful evidence that the RT1176 platform is well
described in modern embedded tooling.

Local Zephyr files of immediate interest:

```text
/home/bogdan/work/zephyrOS/boards/nxp/mimxrt1170_evk/
  mimxrt1170_evk_mimxrt1176_cm7.dts
  mimxrt1170_evk.dtsi
  mimxrt1170_evk-pinctrl.dtsi
  xip/*flexspi_nor_config*

/home/bogdan/work/zephyrOS/dts/arm/nxp/imxrt/nxp_rt1170*.dtsi
```

The CM7 DTS confirms useful anchors for our emulator profile:

- `compatible = "nxp,mimxrt1176"`;
- `zephyr,sram = &sdram0`;
- `zephyr,dtcm = &dtcm`;
- `zephyr,itcm = &itcm`;
- `zephyr,console = &lpuart1`;
- `zephyr,flash-controller = &ext_flash_ctrl`;
- `zephyr,flash = &is25wp128`;
- `sdram0` at `0x80000000`, size `64M`;
- enabled `systick`, `gpt_hw_timer`, `wdog1`, `lpuart1`, `usdhc1`,
  `usb1`/`usbphy1`, `mipi_csi2rx`, and `csi` references.

The important caveat is that **Zephyr board support is not emulator support**.
It gives us:

- a validated RT1176 peripheral map and devicetree-style naming reference;
- build/clock/memory clues for an emulator profile;
- possible comparison points for what a minimal RT1176 CM7 bring-up needs.

It does not give us, by itself:

- a Renode RT1176 machine;
- a QEMU RT1176 machine;
- USB host passthrough for Coral;
- CSI/PXP/SEMC/FlexSPI device models.

So B8 should use Zephyr as a platform map and sanity reference, not as proof
that the SentAI ELF can boot in an emulator out of the box.

Local Renode findings:

```text
/home/bogdan/work/renode/platforms/boards/mimxrt1064_evk.repl
/home/bogdan/work/renode/platforms/cpus/imxrt1064.repl
/home/bogdan/work/renode/platforms/boards/mimxrt700_evk.repl
/home/bogdan/work/renode/scripts/single-node/mimxrt700_evk.resc
```

Renode does not currently show an RT1170/RT1176 board platform in the cloned
tree.  It does have useful NXP/i.MX RT material:

- `CPU.CortexM` with `cpuType: "cortex-m7"` in `imxrt1064.repl`;
- `IRQControllers.NVIC` with SysTick frequency and priority mask;
- `UART.NXP_LPUART` model for LPUART instances;
- `GPIOPort.IMXRT_GPIO`;
- `SPI.IMXRT_FlexSPI`;
- `Timers.IMX_GPTimer`;
- broad MMIO tags for SEMC, USB, CSI, PXP, USDHC, FlexSPI FIFOs, etc.

This makes RT1064 the best local Renode starting template, while Zephyr/SDK
RT1176 sources provide the address map and board-specific deltas.

## B8 Objective Shift

We are pausing the POSIX SIM path as the primary proof vehicle.  B8's objective
is now:

```text
Boot a SentAI/RT1176-like ARM firmware image in an emulator, using the ARM
FreeRTOS port and ARM exception/task model, then incrementally add enough
peripheral models to reach REPL, FS, camera-provider frames, PrepTask, Flow,
and later Crazyflie transport.
```

The POSIX SIM remains a historical/auxiliary tool, not the direction of record
for B8.

## Direction Decision For B8

### Primary path: Renode virtual platform

Renode remains the best fit for a SentAI-specific emulator because we can build
the board boundary incrementally:

1. create a minimal RT1176-like platform with CM7, RAM regions, SysTick/NVIC,
   and UART;
2. load a reduced or current CM7 ELF;
3. add just enough MMIO stubs for boot code that touches clocks, GPIO, cache,
   SEMC/FlexSPI, and storage;
4. add a camera-frame peripheral/provider that writes guest RAM and raises the
   same interrupt/event path expected by the ARM camera code;
5. add filesystem as a deterministic image or a documented host bridge.

This is the path most likely to preserve the ARM FreeRTOS exception model while
still allowing custom peripherals.

### Backup path: QEMU Cortex-M smoke

QEMU is useful for an ARM FreeRTOS kernel smoke, especially with known Cortex-M
boards such as MPS2/AN385.  It is not currently the best way to model RT1176
camera/PXP/USB host because that would require a custom QEMU machine and custom
device models.

QEMU should be used to answer one narrow question:

```text
Can our ARM FreeRTOS/MicroPython task assumptions survive on an ARM emulator at
all, independent of RT1176 board peripherals?
```

### Not first slice: real Coral USB inside emulator

Full Coral USB in an ARM MCU emulator is too large for the first B8 milestone.
It requires all of:

- an emulated RT1176 USB host controller/PHY path;
- USB device attach/enumeration semantics;
- DFU/runtime transitions;
- EdgeTPU endpoint protocol, model upload, instruction/weight flow, and output
  response behavior;
- nonblocking integration with the guest RTOS.

The current B8 first milestone should keep `sentai.tpu` present but not use it
as the gate.  TPU returns as one of these later tracks:

1. **host mailbox peripheral**: guest InferTask writes tensors/commands to a
   modeled peripheral; host runs PyCoral/libedgetpu and writes outputs back.
   This tests SentAI scheduling and parsing, but it is not USB parity.
2. **hardware-in-loop Coral**: run the EdgeTPU path on the physical board or
   the existing host baseline while the emulator validates camera/PrepTask/Flow.
3. **USB model/pass-through**: later research only, after basic ARM-emulated
   SentAI reaches REPL and camera/FS are stable.

## Peripheral Plan

| Subsystem | B8 first slice | Reason |
| --- | --- | --- |
| CM7/FreeRTOS | real ARM port in emulator | This is the core reason to pause B7. |
| UART/REPL | emulated UART to host console/PTY | Needed for `import mission; mission.run()`. |
| Filesystem | first: host bridge or RAM-backed image; later: flash/SD image | We need reproducible per-run mission + FR artifacts. |
| Camera | custom provider -> guest frame buffers + ISR/queue | Preserves ARM producer/consumer shape. |
| PXP | stub/bypass only if boot requires it; real prep code later | Avoid modeling PXP registers before boot/REPL. |
| Flow/Markers | real ARM tasks after PrepTask output exists | Consumer fidelity matters after camera works. |
| Crazyflie UART/CRTP | emulated UART/socket bridge after REPL | Lets emulator drive a real/simulated radio bridge. |
| EdgeTPU USB | deferred | Too much protocol + controller fidelity for first slice. |

## Crazyflie / UART Strategy

The whole Crazyflie experiment needs at least two communication channels:

1. operator/mission REPL channel;
2. drone/radio channel, likely UART/CRTP/CPX-shaped from SentAI's perspective.

B8 should model these as distinct UARTs or one UART plus one explicit bridge.
The emulator side should expose bytes to host PTYs/sockets.  A host bridge can
then forward to:

- a real Crazyradio/Crazyflie stack;
- a simulator bridge;
- a deterministic test harness that records CRTP/MAVLink packets.

Do not merge the REPL transport and drone transport in the emulator; doing so
would hide the exact class of coupling we are trying to uncover.

## Filesystem Strategy

First development option:

```text
iterNNN/
  fs_root/              # host-side staging directory
  fs.img or fs_bridge/  # emulator-visible storage fixture
  fr/                   # exported/decoded artifacts
  uart.log
  emulator.log
```

The mission rule stays unchanged:

```python
import mission
mission.run()
```

No MP-side `exec` and no large byte-array file loads in MP.

Open decision:

- if Renode can expose a simple host-file or block-device model quickly, use it
  for the spike;
- otherwise build a small board-profile storage shim that presents an in-memory
  image to FxUser/FileX and exports it at shutdown.

## Step Plan

### Step 0 - Tooling And ELF

- install or locate Renode and QEMU;
- build `examples/sentai_runtime/sentai_runtime.elf` for CM7;
- record ELF entry point, vector table address, RAM/flash sections, and linker
  memory map;
- confirm whether the RAM linker profile can boot without boot ROM/FlexSPI.

### Step 1 - Boot Boundary Inventory

Create and maintain:

```text
todo/A8_arm_emulator_boot_boundary.md
```

It must list every board service touched before the REPL is usable and classify
each as:

```text
real required | stub allowed | defer | unknown
```

### Step 2 - Renode Minimal Platform

- start from a generic Cortex-M7/RT-like platform, or create a custom RT1176
  platform description;
- map ITCM/DTCM/OCRAM/SDRAM/FlexSPI-style regions enough for the linker;
- add UART console;
- load ELF and capture first fault/boot log.

Success is not "full app": success is a reproducible fault or first UART line.

### Step 3 - Reduced SentAI Profile If Needed

If the full runtime blocks on too many peripherals before scheduler start,
create a reduced ARM-emulator profile:

- FreeRTOS;
- MicroPython REPL;
- FR;
- FS fixture;
- no camera, no USB, no Wi-Fi/Bluetooth, no EdgeTPU.

This is still ARM FreeRTOS, not POSIX SIM.

### Step 4 - Camera Provider

- implement modeled camera frame delivery at the ARM camera receiver boundary;
- support one 640x480 BMP repeated from memory;
- then support a deterministic frame sequence;
- verify PrepTask frame counters and FR artifacts.

### Step 5 - Drone Transport

- add a second UART/socket bridge for CRTP/MAVLink/Crazyflie commands;
- record all bytes as experiment artifacts;
- only then run an end-to-end Crazyflie-style mission in emulator.

### Step 6 - TPU Choice Gate

After REPL + FS + camera + PrepTask are stable, decide between:

- mailbox EdgeTPU host bridge;
- hardware-in-loop TPU validation;
- real USB model/pass-through research.

## Source Notes Added For This Update

- Renode custom peripherals are explicitly supported by the platform model
  workflow: https://renode.readthedocs.io/en/latest/advanced/writing-peripherals.html
- Renode supports connecting UART analyzers and host I/O fixtures:
  https://renode.readthedocs.io/en/latest/basic/using-renode.html
- QEMU documents Arm M-profile CPU/system emulation, but board support is
  machine-specific: https://qemu-project.gitlab.io/qemu/system/arm/emulation.html
- QEMU USB host passthrough exists for machines with usable USB host support,
  but that does not provide an RT1176 EHCI/PHY/board model by itself:
  https://qemu-project.gitlab.io/qemu/system/devices/usb.html
- FreeRTOS has an official QEMU MPS2/AN385 Cortex-M demo, useful as a kernel
  smoke rather than board parity:
  https://www.freertos.org/Why-FreeRTOS/Quick-connect/qemu-mps2-an385-demo
- Zephyr documents the maintained `mimxrt1170_evk@A/mimxrt1176/cm7` target and
  its RT1176 hardware/peripheral inventory, but this is board/toolchain support,
  not emulator support:
  https://docs.zephyrproject.org/latest/boards/nxp/mimxrt1170_evk/doc/index.html

## B8 Research Tasks

### 1. Inventory The ARM Firmware Boundary

Map the minimum board/peripheral functions touched before REPL is usable:

- reset vector, boot ROM assumptions, vector table placement;
- clock init and DCD/FCB/FlexSPI startup expectations;
- SDRAM/SEMC initialization;
- UART console;
- FreeRTOS tick source and interrupt priorities;
- FileX/LittleFS or storage init path;
- camera init path, but only mark what can be deferred;
- USB/EdgeTPU init path, but mark as deferred unless it blocks boot.

Deliverable:

```text
todo/A8_arm_emulator_boot_boundary.md
```

### 2. QEMU Kernel Smoke

Build or reuse a minimal Cortex-M QEMU target to prove:

- ARM GCC build;
- ARM FreeRTOS port, not POSIX;
- SysTick/PendSV/SVC task switching;
- UART output;
- GDB attach;
- at least two tasks plus one interrupt-like wakeup.

This can use QEMU MPS2/AN385 and does not need SentAI yet.

Success:

```text
qemu-system-arm boots a tiny ARM FreeRTOS ELF and logs task switches.
```

### 3. Renode SentAI Boot Spike

Try to load the current SentAI ARM ELF or a reduced SentAI profile into Renode:

- define memory map for ITCM/DTCM/OCRAM/SDRAM/FlexSPI-like regions;
- map UART;
- model enough reset/clock/storage behavior to reach boot log;
- avoid camera/USB initially;
- capture UART/FR output as artifacts.

Success ladder:

1. CPU starts from vector table and reaches early boot log.
2. FreeRTOS scheduler starts.
3. MicroPython REPL starts.
4. `import mission; mission.run()` works from an emulated/staged FS.

### 4. Camera Provider Model

Once REPL boots, add the first camera provider model:

- load one 640x480 BMP from host or emulated FS;
- write XRGB8888 into the same guest frame-buffer contract used by ARM;
- trigger the same camera ISR/notification path;
- verify PrepTask consumes frames without needing MP to schedule them.

Initial tests:

```text
camera only -> PrepTask RGB/gray slot increments
camera repeated static frame -> Flow reports zero motion
camera shifted sequence -> Flow reports non-zero motion
```

Do not add TPU yet.

### 5. Filesystem Strategy

Choose one:

- preferred: prebuilt per-experiment FS image attached to emulated flash/SD;
- acceptable spike: Renode/semihosted host-file bridge documented as a test
  fixture;
- avoid: MP reading large host files into heap.

Artifacts must land beside the experiment run exactly as B7 wanted:

```text
iterXYZ/fs_root_or_image/
iterXYZ/fr/debug.log
iterXYZ/fr/events.csv
iterXYZ/fr/scalars.csv
```

### 6. TPU Strategy

Do not model full Coral USB in the first B8 slice.

Candidate later options:

1. `sentai.tpu` not-ready in emulator mode, used only for non-TPU task tests.
2. TFLite Micro CPU model smoke in ARM emulator for tensor-path tests.
3. Host mailbox peripheral: guest writes input tensor and command to a modeled
   peripheral; host side runs PyCoral/libedgetpu and writes output tensors back.
   This tests SentAI task scheduling and output parsing but is not USB parity.
4. USB/IP or modeled Coral device: future research only; likely expensive.

## Peripheral Fidelity Matrix

| Peripheral / subsystem | First B8 fidelity | Later fidelity |
| --- | --- | --- |
| Cortex-M7 CPU | real emulated ARM core | same |
| FreeRTOS scheduler | ARM port with SysTick/PendSV/SVC | same |
| NVIC interrupts | modeled enough for UART/camera/tick | expand as needed |
| UART/REPL | emulated UART or semihost console | radio/CRTP bridge later |
| SDRAM/OCRAM | mapped memory regions | MPU/cache details if needed |
| Filesystem | FS image or host bridge | production-like block device |
| Camera | frame-buffer producer + ISR trigger | CSI/PXP register model if useful |
| PXP | initially bypassed or coarse modeled effect | register-level model only if needed |
| FlowTask | real ARM task consuming PrepTask output | same |
| Markers | real ARM task consuming PrepTask/camera output | same |
| USB host | disabled/stubbed | maybe USB/IP/model later |
| Coral EdgeTPU | disabled/stubbed | mailbox or USB model later |

## First Practical Milestone

The first B8 milestone should be:

```text
ARM-emulated SentAI boots, starts FreeRTOS + MP REPL,
loads a mission from staged FS, starts virtual/emulated camera frames,
PrepTask consumes N frames, FR artifacts are exported.
```

This gives us the key thing B7 lacked: the ARM task/ISR model, without getting
blocked immediately by Coral USB.

## Decision Gate

After the first spike, decide:

- if Renode can reach REPL within a bounded effort, continue Renode;
- if Renode setup blocks on boot/peripheral details, keep QEMU for kernel-level
  ARM regression and return to ARM hardware for full pipeline validation;
- if neither path reaches REPL quickly, stop and document exactly which
  peripherals block boot.

## Open Questions

- Can the current MCUXpresso/Sentai ELF be loaded directly, or do we need a
  reduced "emulator board" linker/profile?
- How much boot ROM/FlexSPI/DCD behavior must be bypassed or modeled?
- Does FileX require a production-like block device, or can the emulator provide
  a clean fixture layer?
- Can the camera ISR path be triggered cleanly without modeling all CSI
  registers?
- How much of the M4/shared-memory path matters for current SentAI tests?
- Is the Coral EdgeTPU worth modeling at USB level, or should emulator TPU
  parity stop at tensor/output mailbox semantics?

## Verification To Avoid B7 Repeat

Every B8 run should record:

- exact emulator and version;
- exact ELF/profile;
- full UART log;
- FR logs/artifacts;
- task list or trace if available;
- pass/fail condition;
- which peripheral models were real, stubbed, or bypassed.

No "it probably ran" results.
