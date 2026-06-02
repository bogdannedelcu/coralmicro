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
| Camera | virtual provider -> guest frame buffers + ISR/event/queue boundary | Do not emulate MIPI-CSI first. Preserve ARM producer/consumer shape. |
| PXP | stub/bypass only if boot requires it; real prep code later | Avoid modeling PXP registers before boot/REPL. |
| Flow/Markers | real ARM tasks after PrepTask output exists | Consumer fidelity matters after camera works. |
| Crazyflie UART/CRTP | emulated UART/socket bridge after REPL | Lets emulator drive a real/simulated radio bridge. |
| EdgeTPU USB | deferred | Too much protocol + controller fidelity for first slice. |

## 2026-06-02 SentAI Boot Source Review

The current ARM `sentai_runtime` artifact is a CM7 RAM image loaded from:

```text
build/examples/sentai_runtime/sentai_runtime.stripped
```

Important linker/runtime anchors:

```text
examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld

.interrupts        0x00000800
.ramfunc           0x00000c00
.text              0x00000e10
.data              0x20008000
.noinit_boot       0x2000ad28  size 0x14
.bss               0x2000ad40
.usb_host          0x20240000
.tpu_input         0x20243400
.ocram_bss         0x20324400
.heap              0x80000000  size 0x00e00000
.sdram_text/data   0x80e00000+
.ncamera           0x82000000  size 0x004b0000 in current ELF
```

The vector table is at `0x00000800`, the reset vector points to
`0x00000ecd`, and the initial MSP is `0x20040000`.

Boot flow, from source:

```text
startup_MIMXRT1176_cm7.S
  Reset_Handler
    -> SystemInit()
       -> SystemInitHook()
          -> MCMGR_EarlyInit()
    -> C/C++ runtime init
    -> real_main(...)

libs/base/main_freertos_m7.cc
  real_main()
    g_boot_persist.prev_progress = progress
    progress = 0
    BOARD_InitHardware(true)
    SEMA4_Init
    Timer/GPIO/IPC/Console init
    sentai_boot_progress_mark(0x01)
    storage-mode latch
    LfsInit/LfsUserInit
    USB device/host + EdgeTPU tasks
    LPI2C5/LPI2C6 setup
    CameraTask::Init(...)
    PmicTask::Init(...)
    xTaskCreate(app_main)
    sentai_boot_progress_mark(0x06)
    vTaskStartScheduler()

examples/sentai_runtime/sentai_runtime.cc
  app_main()
    sentai_boot_progress_mark(0x10)
    SentAI runtime / FR / MicroPython setup
    micropython_start_repl_task(...)
```

The debug console is **LPUART6**, not the Zephyr EVK default LPUART1:

```text
third_party/modified/nxp/rt1176-sdk/board.h
  BOARD_DEBUG_UART_INSTANCE_M7 = 6
  BOARD_DEBUG_UART_BAUDRATE_M7 = 115200

third_party/nxp/rt1176-sdk/devices/MIMXRT1176/MIMXRT1176_cm7.h
  LPUART6_BASE = 0x40090000
  LPUART6_IRQn = 25
```

`BOARD_InitHardware(true)` is too board-specific for a first emulator slice:

```text
libs/nxp/rt1176-sdk/board_hardware.c
  MCMGR_Init()
  BOARD_InitBootPins()
  BOARD_InitBootClocks()
  BOARD_ConfigMPU()
  BOARD_InitDebugConsole()
  BOARD_InitSEMC()
  memcpy/memset SDRAM sections
  BOARD_InitCAAM()
  BOARD_InitNAND()
```

Renode can map SDRAM directly, so B8 does not need to model SEMC timing before
it can run the firmware.  Similarly, NAND, CAAM, USB host, LPI2C, PMIC, and
physical CameraTask init should not be blockers for the first REPL milestone.

Current Renode spike result:

```text
emu/renode/sentai_rt1176.repl
emu/renode/sentai_rt1176.resc
```

The provisional platform now loads the current ELF, maps ITCM/DTCM/OCRAM/SDRAM,
uses a Cortex-M7 with 16 MPU regions, includes DWT, and exposes LPUART6.
Running for 1 emulated second no longer aborts on missing DWT/MPU support, but
the UART log is still empty.  Reading `.noinit_boot_persist` after the run
shows all zeros, so the current production image has not reached
`sentai_boot_progress_mark(0x01)`.  The next spike should not continue adding
random peripheral stubs; it should build an explicit emulator board profile.

## Emulator Board Profile Decision

B8 should introduce a deliberate `SENTAI_EMU_ARM`/`BOARD_EMU` style build
profile instead of trying to run the production board initialization unchanged.

First milestone rule: **boot to FreeRTOS idle before initializing any real
board peripheral**.  This is intentional.  The first proof should answer only:

```text
Can the current ARM startup, vector table, MPU/SysTick/NVIC shape, C/C++
runtime, FreeRTOS scheduler, and an idle/heartbeat task run under the emulator?
```

Until that is true, the emulator build should not initialize SEMC, NAND/FileX,
USB device, USB host, EdgeTPU, LPI2C, PMIC, physical CameraTask, CSI, MIPI, PXP,
or Crazyflie transport.  Each of those becomes a separate opt-in milestone once
the scheduler heartbeat is stable.

The profile should preserve:

- CM7 startup, vector table, ARM FreeRTOS port, SysTick/NVIC, MPU shape;
- `real_main`/`app_main` structure;
- MicroPython as a low-priority command layer;
- SentAI runtime modules that do not require physical peripherals;
- FlightRecorder logging and mission import semantics.

The profile should bypass or replace:

- SEMC hardware initialization, while keeping the linker SDRAM layout mapped by
  the emulator;
- NAND/FileX/LevelX physical driver, replaced initially by a RAM-backed or
  host-backed filesystem fixture;
- USB device/host and EdgeTPU task init for the first REPL milestone;
- LPI2C/PMIC physical setup;
- physical `CameraTask::Init()`/MIPI-CSI/OV5640 setup.

Recommended staged boot gates:

```text
B8.1  CM7 Reset_Handler -> main -> vTaskStartScheduler -> idle heartbeat
B8.2  add UART/debug-console output, still no storage/camera/USB
B8.3  add MicroPython REPL task over UART, no filesystem mission yet
B8.4  add emulator filesystem fixture and import mission; mission.run()
B8.5  add virtual camera provider -> common frame-ready boundary
B8.6  add PrepTask consumers
B8.7  add Flow/markers consumers
B8.8  decide EdgeTPU path: host mailbox, HIL, or USB model
```

The first camera milestone in emulator is **not** MIPI-CSI emulation.  It is a
runtime virtual camera provider that writes frames into the same frame-ready
boundary consumed by PrepTask/Flow/InferTask.  This can be implemented with a
provider interface:

```text
FrameProvider
  file-sequence provider      -> BMP/fixture frames
  memory-repeat provider      -> one loaded frame, repeated at configured FPS
  gazebo-provider bridge      -> host/simulator frames, same publication API
  future physical camera      -> ISR-backed CSI producer on real ARM
```

All providers publish through one camera-frame event/queue boundary.  Consumers
do not know whether a frame came from MIPI-CSI, a file sequence, or Gazebo.

This keeps B8 aligned with the original ARM model without requiring an
impractical MIPI-CSI peripheral model.

## B8.1 Minimal ARM FreeRTOS Spike

The first buildable emulator spike is intentionally smaller than production
`sentai_runtime`.  It proves:

```text
CM7 startup/vector table -> C/C++ runtime -> ARM FreeRTOS scheduler ->
static heartbeat task -> SysTick-backed tick progress
```

It does **not** initialize real board peripherals:

```text
no SEMC/NAND/FileX/USB/EdgeTPU/LPI2C/PMIC/CameraTask/CSI/MIPI/PXP/Crazyflie
```

Files added:

```text
emu/CMakeLists.txt
emu/sentai_emu_idle.cc
emu/sentai_emu_freertos_hooks.c
emu/renode/sentai_emu_idle.resc
```

Build:

```sh
cmake -S . -B build_emu -DSENTAI_ARM_EMU=ON -DSENTAI_SKIP_SDK_PATCHES=ON
cmake --build build_emu --target sentai_emu_idle -j$(nproc)
```

Run:

```sh
/home/bogdan/work/renode_portable/renode --plain --console --disable-xwt \
  emu/renode/sentai_emu_idle.resc
```

Validated result on 2026-06-02:

```text
sentai_emu_idle boot_state: 0x00000300
sentai_emu_idle heartbeat:  0x0000002F
sentai_emu_idle last_tick:  0x000001CC
```

Interpretation:

- startup reaches `main()`;
- `xTaskCreateStatic()` succeeds;
- `vTaskStartScheduler()` starts the first task;
- SysTick/FreeRTOS tick advances under Renode;
- the heartbeat task wakes repeatedly through `vTaskDelay()`.

Implementation notes:

- the target uses the existing RT1176 startup file and ARM FreeRTOS port;
- CMSIS Core include paths are explicit in the emu target;
- `vPortSetupTimerInterrupt()` is overridden only for the B8.1 emu smoke so
  Renode's Cortex-M SysTick frequency matches the SDK's default
  `SystemCoreClock`;
- `vPortSuppressTicksAndSleep()` is stubbed for this smoke because B8.1 is not
  testing low-power tickless idle.

Remaining B8.1 caveat:

- Renode still logs a benign priority-mask warning while FreeRTOS sets system
  handler priority.  The scheduler/tick proof works, but B8.2 should either
  confirm the priority-mask model is acceptable or adjust the `.repl` with a
  documented rationale.

## Console / REPL Transport Clarification

The current SentAI runtime has three related but distinct concepts:

```text
debug console UART  -> LPUART6, BOARD_InitDebugConsole(), DbgConsole_*
USB CDC ACM console -> ConsoleM7 CDC ACM endpoint, default REPL target
sentai.uart         -> raw UART serial bridge, available only when REPL is USB
```

Important code references:

```text
third_party/modified/nxp/rt1176-sdk/board.h
  BOARD_DEBUG_UART_INSTANCE_M7 = 6
  BOARD_DEBUG_UART_BAUDRATE_M7 = 115200

libs/base/console_m7.h
  enum class ReplTarget { kUsb, kUart };
  repl_target_ = ReplTarget::kUsb;

examples/sentai_runtime/bindings/modsentai_top.c
  sentai.console()        -> "usb" or "uart"
  sentai.console("usb")   -> sentai_console_set_target(0)
  sentai.console("uart")  -> sentai_console_set_target(1)

examples/sentai_runtime/bindings/modsentai_uart.c
  sentai.uart.open() requires REPL on USB

examples/sentai_runtime/bindings/modsentai_usb.c
  sentai.usb.open() requires REPL on UART
```

Therefore B8 should not treat `sentai.uart` as the REPL transport.  The correct
reading is:

- default REPL is USB CDC ACM on the real board;
- UART/LPUART6 is the debug console and can be selected as REPL target through
  `sentai.console("uart")`;
- the non-REPL side becomes available for raw Python serial I/O.

`examples/sentai_runtime/SENTAI_API.md` still contains an older statement that
the REPL is on UART and `/dev/ttyACM0` is not the REPL.  That conflicts with
the current code and `agent.md`, which says MicroPython REPL is over USB CDC
ACM by default.  B8 should follow the code and update the stale API note later.

For the emulator path:

- B8.2 validates LPUART6 output as a debug/console smoke;
- B8.3 may force REPL target to UART as an intermediate milestone because
  modeling USB CDC ACM is larger;
- a later milestone can model/bridge USB CDC ACM if we need exact real-board
  REPL transport parity.

## B8.2 Minimal UART Smoke

`sentai_emu_uart` extends the B8.1 heartbeat target with direct LPUART6 writes.
It is intentionally not full `ConsoleM7`; it only proves that the Renode
RT1176-like platform can capture the SentAI debug UART.

Files:

```text
emu/renode/sentai_emu_uart.resc
examples/sentai_runtime/experiments/s213_arm_emulator_idle/
```

Experiment run:

```sh
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target uart
```

Validated result:

```text
examples/sentai_runtime/experiments/s213_arm_emulator_idle/
  iter03_renode_uart_heartbeat/
    verdict_s213.json  -> pass: true
    uart.log           -> boot + task online + heartbeat messages
```

The UART log contains:

```text
SentAI EMU UART boot
SentAI EMU UART task online
SentAI EMU UART heartbeat
```

This confirms:

- Renode `UART.NXP_LPUART` at `0x40090000` works for LPUART6 TX;
- file-backend logging works for experiment artifacts;
- FreeRTOS task execution and UART output coexist.

Next gate: B8.3 should start a reduced MicroPython REPL task on UART, without
filesystem/main.py/crazy/USB/storage/camera/TPU.

## B8.3 Minimal MicroPython REPL On UART

`sentai_emu_repl` is the first emulator target that runs the MicroPython VM.
It boots through the same CM7 startup + ARM FreeRTOS path as B8.1/B8.2, then
hands control to a single REPL task on LPUART6.

Files:

```text
emu/sentai_emu_repl.cc            # main + heap + REPL loop
emu/sentai_emu_mphalport.c        # mp_hal_stdin_rx_chr / mp_hal_stdout_tx_*
emu/sentai_emu_stub_modules.c     # empty `sentai` module (linker satisfier)
emu/mp_inc/mpconfigport.h         # minimal MP config for emu_repl
emu/renode/sentai_emu_repl.resc
examples/sentai_runtime/experiments/s213_arm_emulator_idle/
```

Build:

```sh
cmake --build build_emu --target sentai_emu_repl -j$(nproc)
```

Run (experiment-driven):

```sh
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py \
    --target repl
```

Validated result (`iter05_renode_repl_oneplusone`):

```text
boot_state              = 0x00000500  (kBootReplBanner)
repl_lines              = 1
last_tick               > 0
uart_log raw bytes      = "\r\nSentAI EMU REPL B8.3\r\n"
                          "MicroPython embed ready\r\n"
                          ">>> 1+1\r\n"
                          "2\r\n"
                          ">>> "
pass                    = true
```

What this proves:

- the emulator can host the MicroPython embed port on a true ARM build (not
  POSIX), driven by the production CM7 startup and ARM FreeRTOS scheduler;
- LPUART6 RX delivers characters into `mp_hal_stdin_rx_chr` via Renode's
  `lpuart6 WriteChar` injection;
- `mp_embed_init` (GC heap, stack ctrl, QSTR pool, module table) completes;
- `mp_embed_exec_str` compiles and evaluates a top-level Python expression
  and the result reaches LPUART6 TX via `mp_hal_stdout_tx_strn_cooked`.

Notes and gotchas captured by B8.3:

- The shared `genhdr/moduledefs.h` references `mp_module_sentai` and the rest
  of the production module set unconditionally.  The emu mpconfigport must
  therefore enable the same module set; disabling `MICROPY_PY_SYS`,
  `MICROPY_PY_ARRAY`, `MICROPY_PY_MATH`, `MICROPY_PY_COLLECTIONS`, or
  `MICROPY_PY_STRUCT` would not save flash — it would only produce
  "undefined reference to mp_module_X" link errors.  An empty `mp_module_sentai`
  stub is supplied in `sentai_emu_stub_modules.c` so we do not pull modsentai.c
  or any binding for B8.3.
- The QSTR pool is shared with production.  `MICROPY_PY_BUILTINS_STR_OP_MODULO`
  must be enabled in the emu config so `hex()`/`oct()` use `%#x`/`%#o` instead
  of the `{:#x}` form, which is not in the baked QSTR table.
- `mp_embed_exec_str` parses with `MP_PARSE_FILE_INPUT`, yet the embed VM still
  prints the top-level expression result.  B8.3 confirms this empirically:
  `1+1\r` causes `2\r\n` to be written to the UART.  No `MP_PARSE_SINGLE_INPUT`
  variant is required for the smoke.
- `Renode read_text()` strips `\r` via newline translation; verdict checks
  must read the UART log in binary mode and assert against raw bytes, or the
  REPL answer assertion silently flips to false.  Fixed in `run_s213.py`.
- The B8.1 critique that the emu target should use `ARM_CM7/r0p1/port.c`
  instead of `ARM_CM4F/port.c` was wrong: `libs/FreeRTOS/CMakeLists.txt`
  shows production firmware itself compiles `ARM_CM4F/port.c` for the M7.
  The `ARM_CM7/r0p1` directory is only added as an include path, not as a
  source.  Emulator and production are aligned on this point.

Open caveats:

- Boot-time priority-mask warning from Renode persists (same as B8.1).  Now
  that the build emits proper FreeRTOS interrupt priorities, B8.4+ can either
  raise `cpu PRIGROUP` in the `.repl` or document the model gap.
- The REPL is single-line only: no history, no multi-line blocks, no Ctrl+C,
  no watchdog hook, no scheduler drain.  All deferred to a later milestone
  if the emulator path becomes a daily driver.
- `mp_embed_init` is fed a 32 KiB GC heap.  Enough for `1+1` and simple
  expressions; will need raising before importing a mission module.

Next gate: B8.4 should mount a RAM-backed or host-bridged filesystem fixture
so `import mission; mission.run()` works without USB/NAND/FileX.

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

## Initial SentAI Boot Inventory

Local source review of the actual SentAI ARM boot path shows that the first
B8 emulator slice must follow the SentAI board configuration, not the generic
Zephyr EVK defaults:

- `third_party/modified/nxp/rt1176-sdk/board.h` sets the M7 debug UART to
  **LPUART6**, baud `115200`.
- `MIMXRT1176_cm7.h` maps `LPUART6_BASE` to `0x40090000` and `LPUART6_IRQn`
  to `25`.
- `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld` places the vector
  table at `0x00000800`, code in ITCM, data in DTCM, USB/tensor buffers in
  OCRAM, and the heap/large runtime sections in SDRAM.
- The current stripped ARM artifact has MSP `0x20040000` and reset vector
  `0x00000ecd`, so Renode must set `VectorTableOffset` to `0x800`.
- `BOARD_InitHardware(true)` touches pin mux, clocks, MPU, debug console,
  SEMC/SDRAM, SDRAM section copy/clear, CAAM, FlexSPI NAND.
- `real_main()` then initializes SEMA4, timer/GPIO/IPC/ConsoleM7, LFS,
  USB device/host, EdgeTPU tasks, LPI2C5/6, camera task, PMIC task, and only
  then starts `app_main`.
- `app_main()` initializes SentAI runtime state and starts the MicroPython REPL
  task.

This means the first milestone is not "USB REPL" yet.  It is:

```text
Load ELF -> run Reset_Handler -> pass enough board init to see UART6 output
```

The second milestone can then decide whether to stub/replace NAND/LFS and USB
for emulator mode or model them more fully.

## Emulator Config Added

Initial Renode bring-up files were added under:

```text
emu/
  README.md
  renode/sentai_rt1176.repl
  renode/sentai_rt1176.resc
```

The `.repl` maps the SentAI linker memory layout:

- ITCM `0x00000000`, `512 KiB`;
- DTCM `0x20000000`, `512 KiB`;
- OCRAM `0x20200000`, `2 MiB`;
- SDRAM `0x80000000`, `64 MiB`;
- LPUART6 `0x40090000`, IRQ `25`.

It also includes host-side Renode stubs for early SDK MMIO regions such as
watchdogs, CCM/SRC/IOMUXC, SEMC, FlexSPI1, CAAM, SEMA4, LPI2C5/6, USB, CSI,
MIPI CSI2RX, PXP, and GPIOs.  These are provisional emulator models whose
purpose is to expose the first real missing behavior boundary.

First run result:

- Renode portable `v1.16.1.4546` was installed under
  `/home/bogdan/work/renode_portable`.
- The `.resc` loads the current stripped SentAI RAM ELF and sets initial
  `PC = 0x00000ecd`, `SP = 0x20040000`.
- The first hard loop was at MU-A: repeated reads from `0x40C48020`, reached
  from PC `0x10278`.  This matches early `MCMGR_EarlyInit()` /
  `MCMGR_Init()` behavior, so MU-A was added as a provisional stub at
  `0x40C48000`.
- DCDC/ANATOP coverage was also widened because boot read `0x40CAC960`.
- After adding MU-A, boot reached clock/pinmux setup and looped on
  `0x40C84560` after writes under `0x40C94000`.  These addresses map to the
  RT1176 ANADIG/OSC/PLL/PMU block and IOMUXC_SNVS, so provisional stubs were
  added for `0x40C84000`, `0x40C90000`, `0x40C94000`, and `0x40C98000`.
- After adding ANADIG/IOMUXC_SNVS, boot reached SDK delay code polling
  `DWT->CYCCNT` at `0xE0001004`, so Renode's built-in `Miscellaneous.DWT`
  model was added at `0xE0001000`.
- After adding DWT, boot reached `BOARD_ConfigMPU()` and Renode aborted because
  the default Cortex-M model exposed only 8 MPU regions.  The local SentAI/NXP
  board code configures regions 0 through 15, so the platform now sets
  `numberOfMPURegions: 16`.

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
