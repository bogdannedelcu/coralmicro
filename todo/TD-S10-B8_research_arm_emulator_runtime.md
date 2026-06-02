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
