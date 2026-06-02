# A8 ARM Emulator Boot Boundary

## Purpose

Inventory the minimum ARM firmware boundary needed before SentAI can run under
an ARM emulator close enough to the physical RT1176 board to be useful.

This is not a SIM/POSIX inventory.  It is the list of board/runtime contracts
that an ARM emulator or reduced emulator profile must satisfy before we can
trust task/ISR behavior.

## Local Baseline

- Current working tree has no `renode` or `qemu-system-arm` in PATH.
- No ready `sentai_runtime.elf` was found in the current build directories.
- External research repos are now local:

```text
/home/bogdan/work/zephyrOS  # Zephyr main, 0570f6d6b
/home/bogdan/work/renode    # Renode master, ac18b5a
```

- ARM build is selected when `SENTAI_SIM=OFF`; `examples/sentai_runtime`
  defines `sentai_runtime` as a CM7 executable linked with
  `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`.
- The runtime includes real board code: NXP RT1176 SDK startup, clocks, GPIO,
  FileX/FxUser storage, camera/CSI/PXP, USB/EdgeTPU, MicroPython, and SentAI
  runtime tasks.
- Zephyr has a maintained `mimxrt1170_evk@A/mimxrt1176/cm7` target that can be
  used as a platform-map reference, but it is not an emulator target by itself.
- Renode has `mimxrt1064_evk` and `mimxrt700_evk` platforms locally, but not an
  RT1170/RT1176 platform.  Use RT1064 as the closest Cortex-M7/i.MX RT template
  and Zephyr/SDK RT1176 files for the deltas.

## Local ARM Workflow Notes

From `.claude/skills/arm-flash`, `.claude/skills/qstr-regen`, and
`.claude/agents/arm-builder.md`:

- Build ARM firmware from repo root with:

```bash
bash build.sh
# or incremental:
cmake --build build --target sentai_runtime
```

- Verify ISR/hot-path placement after ARM build:

```bash
arm-none-eabi-objdump -h build/examples/sentai_runtime/sentai_runtime.elf \
  | grep -E '\.(ramfunc|itcm)'

arm-none-eabi-objdump -d build/examples/sentai_runtime/sentai_runtime.elf \
  | grep -E 'csi_irq|CSI_DriverIRQHandler|USB_OTG1_IRQHandler' \
  | head -5
```

- Flashing is a separate safety workflow:

```bash
python3 scripts/flashtool.py -e sentai_runtime
# optional RAM-only fast test:
python3 scripts/flashtool.py -e sentai_runtime --ram
```

- After flashing, verify USB enumeration and build counter.  `1fc9:c0a1` is
  expected NXP runtime; `18d1:9307` is Google Coral ROM bootloader and means
  an anti-brick failure path.
- Regenerate MicroPython QSTR only after binding changes, using the documented
  embed recipe under `examples/sentai_runtime`, not by editing generated files.

## Boundary Classification

| Boundary | Current ARM source path | First emulator status | Notes |
| --- | --- | --- | --- |
| Reset/vector table | `startup_MIMXRT1176_cm7.S`, linker script | real required | Emulator must start from the correct vector table/entry. |
| SysTick/PendSV/SVC/NVIC | FreeRTOS ARM CM7 port | real required | Core reason for B8. |
| Clock init | `BOARD_InitHardware`, `clock_config.c` | stub allowed | Need enough behavior to keep SDK code moving. |
| Cache/MPU/barriers | `fsl_cache.c`, `DCACHE_*` calls | stub allowed initially | Must not fault; exact cache behavior can be deferred. |
| SDRAM/SEMC | linker + board hardware | mapped memory required | We need memory regions; SEMC register fidelity can be stubbed. |
| FlexSPI/boot flash | linker/boot headers if flash image is used | unknown | RAM linker may avoid most boot ROM/FlexSPI work. |
| UART console/REPL | `console_m7`, SDK debug console/LPUART | real required | Route to host console or PTY. |
| FxUser/FileX FS | `libs/base/filesystem.*`, `fx_user_fs.*` | required for REPL missions | Host bridge or image acceptable in first slice. |
| Flight recorder | `sentai_fr.*` | required | Must export `debug.log`, `events.csv`, `scalars.csv`. |
| MicroPython task | `micropython_task.c` | real required | MP remains high-level command layer. |
| CameraTask queue | `libs/camera/camera.cc` | required for camera tests | Keep same task-facing API. |
| CSI receiver queue | `camera_support.c`, NXP `fsl_csi.c` | modeled effect | Provider should mark full buffers and trigger interrupt/event. |
| PXP | `fsl_pxp`, PrepTask resize paths | defer/stub initially | Enable once camera provider works. |
| PrepTask | `sentai_runtime.cc`, `sentai_prep.*` | real required after camera | Target for first functional camera test. |
| FlowTask | `flow_task.cc` | later real consumer | Consume PrepTask output after camera is stable. |
| Markers | runtime marker code | later real consumer | Same as FlowTask. |
| USB host controller | `libs/usb/usb_host_task.*` | defer | Too expensive for first boot. |
| EdgeTPU manager | `libs/tpu/edgetpu_manager.*` | defer/stub | Use mailbox/HIL later, not first boot gate. |
| Crazyflie/CRTP UART | SentAI link/crazy modules | later bridge | Needs separate UART/socket from REPL. |
| Wi-Fi/Bluetooth | NXP/Coral libs | defer | Disable/stub in emulator profile if it blocks boot. |

## First Questions To Answer

1. Can the current RAM-linked `sentai_runtime` ELF be built and loaded without
   boot ROM/FlexSPI emulation?
2. What is the earliest blocking peripheral touch before scheduler start?
3. Can UART output be captured before FileX/FS comes up?
4. Does MicroPython REPL require mounted user FS, or can it boot with a minimal
   fixture?
5. Can camera tests be compiled behind an emulator provider without changing
   MP APIs?

## First Spike Success Ladder

1. Build CM7 ELF and record map/entry.
2. Emulator reaches reset handler.
3. Emulator emits first UART/boot line.
4. FreeRTOS starts at least one task.
5. MicroPython REPL banner appears.
6. `import mission; mission.run()` works from staged FS.
7. Camera provider publishes N frames and PrepTask consumes them.

## Explicit Deferrals

- Full Coral USB protocol in emulator.
- Full CSI/PXP register-level modeling.
- Real Crazyflie flight loop.
- Wi-Fi/Bluetooth.
- Cycle-accurate performance claims.
