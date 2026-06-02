# SentAI ARM Emulator Bring-Up

This directory holds host-side emulator configuration for B8.  The goal is to
boot the ARM `sentai_runtime` firmware model, not to extend the POSIX SIM path.

Current first milestone:

1. build and boot `sentai_emu_idle`, a minimal CM7/ARM-FreeRTOS heartbeat
   image;
2. prove startup, scheduler, SysTick, and `vTaskDelay()` under Renode;
3. build and boot `sentai_emu_uart` to prove LPUART6 output capture;
4. build and boot `sentai_emu_repl` to prove MicroPython embed runs on the
   ARM emulator and evaluates `1+1 -> 2` over LPUART6;
5. build and boot `sentai_emu_mission` to prove `import mission;
   mission.run()` against an in-firmware VFS;
6. build and boot `sentai_emu_camera` to prove the full ARM camera ISR
   path: VCam Renode peripheral DMA + NVIC IRQ → strong override of
   `Reserved110_IRQHandler` → `vTaskNotifyGiveFromISR` → consumer task
   validates frame bytes;
7. build and boot `sentai_emu_pipeline` to prove the two-stage task
   pipeline topology (`Stage1Task` → `Stage2Task`) behind the same
   VCam IRQ.  Stage names are deliberately neutral; production
   `PrepTask` / `InferTask` / `FlowTask` / `CameraTask` symbols are
   reserved for the real algorithms.  No TPU/InferTask in this gate —
   EdgeTPU USB is still explicitly deferred;
8. build and boot `sentai_emu_fanout` to prove the multi-reader
   fan-out topology (`Stage1Task` → {`Stage2ATask`, `Stage2BTask`})
   driven by a real seqlock.  Both readers see identical data per
   frame on every IRQ;
9. only then re-evaluate the TPU strategy.

Caveat (operator note 2026-06-02): production REPL is USB CDC ACM, not
LPUART6.  The emu uses LPUART6 because Renode has no CDC ACM endpoint, but
LPUART6 will eventually carry CRTP for Crazyflie — at that point the
emulator should either model USB CDC ACM or drop the interactive REPL and
drive missions only through `import + run` from a baked/staged file.

Production-image inventory milestone:

1. load the existing CM7 RAM ELF/stripped ELF;
2. map the SentAI linker memory regions correctly;
3. expose the firmware debug console on LPUART6;
4. identify the first missing board peripheral model with evidence.

The first interactive target is UART REPL.  USB CDC REPL, filesystem-backed
missions, camera provider, and EdgeTPU are later milestones.

## Source Anchors

- Linker: `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`
- Board init: `libs/nxp/rt1176-sdk/board_hardware.c`
- Main boot: `libs/base/main_freertos_m7.cc`
- App boot: `examples/sentai_runtime/sentai_runtime.cc`
- Debug UART: `third_party/modified/nxp/rt1176-sdk/board.h`

Important local facts:

- vector table is at `0x00000800`;
- reset vector currently points to `0x00000ecd`;
- MSP starts at `0x20040000`;
- SentAI debug console uses **LPUART6**, not Zephyr's LPUART1;
- LPUART6 base is `0x40090000`, IRQ `25`;
- SDRAM starts at `0x80000000` and must cover heap, SDRAM code/data, and the
  camera no-cache area at `0x82000000`.

## Layout

- `renode/sentai_rt1176.repl` - provisional RT1176-like platform.
- `renode/sentai_rt1176.resc` - loads the current SentAI firmware artifact.
- `renode/sentai_emu_idle.resc` - loads the minimal B8.1 heartbeat target.
- `renode/sentai_emu_uart.resc` - loads the minimal B8.2 LPUART6 target.
- `renode/sentai_emu_repl.resc` - loads the B8.3 MicroPython REPL target.
- `renode/sentai_emu_mission.resc` - loads the B8.4 mission import target.
- `renode/sentai_emu_camera.resc` - loads the B8.5 camera frame provider
  target and triggers 5 frame deliveries via the VCam peripheral.
- `renode/sentai_emu_pipeline.resc` - loads the B8.6 Stage1Task +
  Stage2Task pipeline target and triggers 5 frames through the full
  task chain.
- `renode/sentai_emu_fanout.resc` - loads the B8.7 Stage1Task +
  Stage2ATask + Stage2BTask multi-reader fan-out target.
- `renode/sentai_emu_flowest.resc` - loads the B8.7c flow-offset
  target and feeds 6 cat scenes panned by 1 px/frame via
  `sysbus LoadBinary`.
- `scripts/prepare_cat_scenes.py` - preprocesses the B7 reference
  cat BMP into 32x32 Y8 scene .bin files under `output/scenes/`.
- `mp_inc/mpconfigport.h` - B8.3 emu MicroPython config.
- `mp_inc_mission/mpconfigport.h` - B8.4 config (adds external import +
  `sys.path` attribute delegation on top of B8.3).
- `sentai_emu_repl.cc` / `sentai_emu_mphalport.c` / `sentai_emu_stub_modules.c`
  - B8.3 REPL implementation, LPUART6 mphal port, and empty `sentai` module
  stub that satisfies the shared `genhdr/moduledefs.h`.
- `sentai_emu_fs.c` - B8.4 in-firmware VFS (mission.py baked into .rodata,
  exposed through `mp_import_stat` + `mp_lexer_new_from_file`).
- `sentai_emu_camera.cc` - B8.5 consumer task + strong override of
  `Reserved110_IRQHandler` driven by VCam Renode peripheral writes to
  the NVIC ISPR2 register.
- `sentai_emu_pipeline.cc` - B8.6 Stage1Task + Stage2Task + shared
  scalar slot.  Same VCam IRQ as B8.5; the ISR now wakes a multi-stage
  chain via `vTaskNotifyGiveFromISR` + `xTaskNotifyGive`.  Stage names
  are deliberately distinct from production `PrepTask` / `FlowTask`.
- `sentai_emu_fanout.cc` - B8.7 Stage1Task + Stage2ATask + Stage2BTask
  with a real seqlock (`version++` odd-then-even writer; `v1 == v2`
  reader retry loop).  Same B8.5 VCam IRQ feeding three FreeRTOS
  tasks now.
- `sentai_emu_flowest.cc` - B8.7c brute-force SAD block-match flow
  estimator over host-fed cat scenes.  Disables VCam FILL_MODE so
  the bytes loaded by Renode into `g_frame_buffer` survive between
  IRQs.  NOT a production-format-equivalent flow path: production
  is XRGB8888 → PrepTask → Y8 80x60 → FlowTask USADA8 / phase
  correlation; B8.7c is Y8 32x32 → SAD brute force.

The `.repl` intentionally uses Renode host-side stub peripherals for early MMIO
that the NXP SDK touches during boot.  These are not firmware filesystem code
and do not change the SentAI runtime; they are emulator-only board models.

## Run

Build and run the minimal B8.1 heartbeat target:

```sh
cmake -S . -B build_emu -DSENTAI_ARM_EMU=ON -DSENTAI_SKIP_SDK_PATCHES=ON
cmake --build build_emu --target sentai_emu_idle -j$(nproc)
/home/bogdan/work/renode_portable/renode --plain --console --disable-xwt \
  emu/renode/sentai_emu_idle.resc
```

Expected B8.1 proof:

```text
boot_state = 0x00000300
heartbeat  > 1
last_tick  > 0
```

Build and run the B8.2 UART target:

```sh
cmake --build build_emu --target sentai_emu_uart -j$(nproc)
/home/bogdan/work/renode_portable/renode --plain --console --disable-xwt \
  emu/renode/sentai_emu_uart.resc
sed -n '1,80p' emu/output/sentai_emu_uart.log
```

Expected UART proof:

```text
SentAI EMU UART boot
SentAI EMU UART task online
SentAI EMU UART heartbeat
```

Build and run the B8.3 MicroPython REPL target:

```sh
cmake --build build_emu --target sentai_emu_repl -j$(nproc)
/home/bogdan/work/renode_portable/renode --plain --console --disable-xwt \
  emu/renode/sentai_emu_repl.resc
xxd emu/output/sentai_emu_repl.log | head
```

Expected REPL proof:

```text
SentAI EMU REPL B8.3
MicroPython embed ready
>>> 1+1
2
>>>
```

Build and run the B8.4 mission import target:

```sh
cmake --build build_emu --target sentai_emu_mission -j$(nproc)
/home/bogdan/work/renode_portable/renode --plain --console --disable-xwt \
  emu/renode/sentai_emu_mission.resc
xxd emu/output/sentai_emu_mission.log | head
```

Expected B8.4 proof:

```text
>>> import mission
>>> mission.run()
MISSION OK from B8.4 5
>>>
```

For experiment-style archived runs:

```sh
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target uart
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target repl
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target mission
```

Run the current production artifact inventory script:

```sh
/home/bogdan/work/renode_portable/renode --console --disable-xwt emu/renode/sentai_rt1176.resc
```

Expected early blockers are SEMC/NAND/LFS/USB peripheral fidelity, not the
ARM CPU model itself.
