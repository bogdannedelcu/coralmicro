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
- `host/sentai_crazy_cpx_udp_bridge.py` - B9/s219 host bridge that
  terminates emulator CPX-over-UART, acknowledges CPX CTS, forwards CRTP
  datagrams to CrazySim/cf2 UDP `127.0.0.1:19850`, and wraps inbound CRTP
  back into CPX frames for the guest.
- `host/test_sentai_crazy_cpx_udp_bridge.py` - protocol-only smoke test for
  the CPX/CRTP parser/wrapper.  It does not require Renode or cf2.
- `sentai_emu_crazy_serial_bridge.cc` - guest-side `SENTAI_ARM_EMU` serial
  backend for `sentai_uart_serial_*`.  It lets `sentai.crazy` use a dedicated
  MMIO transport instead of sharing LPUART6 with the REPL.
- `renode/crazy_cpx_udp_mmio_bridge.py` - Renode PythonPeripheral backing the
  guest serial MMIO contract and forwarding CPX/CRTP to cf2 UDP.
- `renode/sentai_emu_camera_markers_stage_assets.resc` - stages the S230
  WhyCon PGM/BMP assets into the emulated FileX/LevelX NAND image through the
  guest FxUser stack.
- `renode/sentai_emu_camera_markers.resc` - boots the S230 MicroPython target
  with shared `sentai.camera` and `sentai.markers`, then runs the camera +
  WhyCon autorun smoke.
- `renode/sentai_emu_camera_markers_ui.resc` - same S230 camera/markers target
  with Renode UART analyzer enabled for interactive debugging.
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

## SentAI Namespace Verification (B9)

B9 verifies the MicroPython-facing `sentai.*` surface in small emulator
profiles.  Use `--renode-ui` when you want the Renode UART analyzer visible
while the same file backend records artifacts:

```sh
python3 examples/sentai_runtime/experiments/s216_arm_emulator_namespace_inventory/run_s216.py --renode-ui
python3 examples/sentai_runtime/experiments/s218_arm_emulator_core_namespaces/run_s218.py --renode-ui
python3 examples/sentai_runtime/experiments/s228_arm_emulator_hw_stub_namespaces/run_s228.py --renode-ui
python3 examples/sentai_runtime/experiments/s229_arm_emulator_gazebo_calib_precursor/run_s229.py --renode-ui
```

Current checkpoints:

- S216 `iter05_namespace_inventory`: PASS, wide inventory profile with
  `fs/fr/rtos/io/sys/tpu/pipeline/crazy`, 14 shared exports present, 26 still
  missing, 0 unexpected.
- S218 `iter01_core_namespaces`: PASS, 25/25 checks for top-level helpers,
  `fs`, `fr`, `rtos`, `io`, and `sys`.
- S228 `iter02_hw_stub_namespaces`: PASS, 45/45 checks for safe emulator
  stubs under `usb`, `uart`, `imu`, `mic`, `sleep`, `servo`, `calib`,
  `object_lifter`, and `safety`.
- S229 `iter01_gazebo_whycon_calib_precursor`: PASS, boots Renode with UART UI,
  starts CrazySim/cf2 in `sentai_whycon_small`, probes the current
  `sentai.calib` stub boundary, and runs a conservative
  `sentai.crazy.fly(0.35, 1800, 2200, 2200)` through the bridge.
- S230 `iter03_camera_markers`: PASS, stages a synthetic WhyCon marker into
  emulator FileX, verifies `sentai.markers.detect_pgm()`, then selects a
  staged BMP through `sentai.camera`, runs `prep_once()`, and verifies
  `sentai.markers.detect_from_camera()` plus a 10-frame repeated loop.
- S233 `iter31_gazebo_flow_whycon`: PASS, feeds live Gazebo VGA
  `640x480` RGB888 frames into the ARM emulator through a format-aware
  CameraTask bridge, then verifies PrepTask, FlowTask, WhyCon, and cf2 commands
  together.  FlowTask consumes PrepTask's `FLOW_GRAY_80x60` slot; it does not
  process the VGA frame directly.  RGB888 fast path result:
  `cam_last_format=0`, `cam_frames=644`, Prep/Flow `5.17/4.79 FPS`.
  `iter29_gazebo_flow_whycon` validates the same bridge in ARM-like XRGB mode
  (`cam_last_format=1`, Prep/Flow `4.54/4.22 FPS`).  ARM keeps the physical
  camera XRGB contract; RGB is an emulator-only profiling/throughput path.

Run the S230 camera + WhyCon marker smoke:

```sh
python3 examples/sentai_runtime/experiments/s230_arm_emulator_camera_markers/run_s230.py
```

Expected proof:

```text
pass=True
pgm_n=1
cam_n=1
loop_ok=10/10
loop_fps_x100=291
```

The S230 target uses the shared runtime camera and markers bindings.  The
emulator-specific boundary is `emu/sentai_emu_camera_runtime_bridge.cc`, which
supplies the board/HAL/filesystem callbacks needed by the shared runtime.  The
experiment runner archives generated assets, Renode logs, UART logs, and
`verdict_s230.json` under
`examples/sentai_runtime/experiments/s230_arm_emulator_camera_markers/iterNN_*`.
It removes the S230 UART files from `emu/output` after archiving, so persistent
run artifacts stay under the `s230` experiment folder.

S216 and S228 are intentionally separate profiles.  The wide inventory profile
already includes TPU/Pipeline/Crazy bridge code; adding every hardware stub to
the same binary overflows the 256 KiB RT1176 RAM text region used by this
bring-up linker script.  The split keeps each behavior explicit without
pretending the emulator has real USB MSC, UART serial, IMU, microphone, or
actuator hardware.

Run the current production artifact inventory script:

```sh
/home/bogdan/work/renode_portable/renode --console --disable-xwt emu/renode/sentai_rt1176.resc
```

Expected early blockers are SEMC/NAND/LFS/USB peripheral fidelity, not the
ARM CPU model itself.

## Crazyflie Bridge (B9/s219)

`sentai.crazy` is the active drone-control priority for B9.  The intended
guest path remains the shared ARM runtime path:

```text
sentai.crazy -> CRTP -> CPX -> UART
```

The emulator-specific part lives on the host side and translates that UART
byte stream to cf2 CRTP UDP.  Run the bridge smoke test with:

```sh
python3 emu/host/test_sentai_crazy_cpx_udp_bridge.py
```

Once Renode exposes a second UART/PTY/socket for Crazyflie traffic, start the
host bridge like:

```sh
python3 emu/host/sentai_crazy_cpx_udp_bridge.py \
  --serial <renode-crazy-pty> \
  --udp-host 127.0.0.1 \
  --udp-port 19850 \
  --verbose
```

Do not use LPUART6 for this while the interactive REPL is on LPUART6.  The
Crazy bridge is for `sentai.crazy`/cf2.  `sentai.link`/PX4/MAVLink is a later
B9 step and remains deferred until a PX4 simulator endpoint is installed.

The current B9 implementation also has a lower-friction MMIO route for Renode:
`sentai_emu_crazy_serial_bridge.cc` implements the guest serial ABI and
`renode/crazy_cpx_udp_mmio_bridge.py` forwards those requests to cf2.  This is
an emulator transport substitution at the serial boundary, not a new
MicroPython API and not a replacement for the production `sentai.crazy` logic.
The guest-side serial backend can be compile-checked with:

```sh
cmake -S . -B build_emu -DSENTAI_ARM_EMU=ON -DSENTAI_SKIP_SDK_PATCHES=ON
cmake --build build_emu --target sentai_emu_crazy_serial_bridge_obj -j$(nproc)
```

The current end-to-end smoke is archived under experiment `s219`:

```sh
python3 examples/sentai_runtime/experiments/s219_arm_emulator_crazy_bridge/run_s219.py
```

That runner compile-checks the bridge code, respawns CrazySim/cf2 in the
`sentai_whycon_small` world by default, boots
`sentai_emu_crazy_ping_smoke` in Renode, and copies the logs into the next
`s219/iterNN_*` directory.  The first archived PASS is:

```text
examples/sentai_runtime/experiments/s219_arm_emulator_crazy_bridge/iter01_renode_crazy_cf2_bridge
boot_state=0x0B00 init_rc=0 ping_ms=5 serial_tx=54 serial_rx=22
bridge: guest->udp crtp_len=5, udp->guest crtp_len=5
```

The first archived MicroPython `sentai.crazy` namespace PASS with Renode UI
enabled is:

```text
examples/sentai_runtime/experiments/s219_arm_emulator_crazy_bridge/iter03_renode_crazy_mp_cf2_bridge
boot_state=0x0500 CRAZY_MP_INIT=0 CRAZY_MP_PING_MS=1 CRAZY_MP_STOP=0
serial_tx=56 serial_rx=22 bridge: guest->udp=1, udp->guest=1
```

Use `--world sentai_crazysim` only for CRTP-only bring-up where the visual
marker pad is irrelevant.  For interactive debugging with Renode's UI/analyzer
enabled:

```sh
python3 examples/sentai_runtime/experiments/s219_arm_emulator_crazy_bridge/run_s219.py \
  --renode-ui
```

`--renode-ui` selects the matching `_ui.resc` script and opens
`showAnalyzer lpuart6`, while still writing the UART file backend archived by
the experiment runner.  Use `--mode mp --renode-ui` for the MicroPython
`sentai.crazy` namespace smoke.

This target is still primarily a C++ smoke harness.  The next B9 step is to
broaden the MicroPython namespace target from `init`/`ping` to telemetry and
safe arm/disarm commands, logging mission results through `sentai.fr`.

The first Gazebo/WhyCon flight precursor is archived under `s229`:

```sh
python3 examples/sentai_runtime/experiments/s229_arm_emulator_gazebo_calib_precursor/run_s229.py \
  --renode-ui
```

Archived PASS:

```text
examples/sentai_runtime/experiments/s229_arm_emulator_gazebo_calib_precursor/iter01_gazebo_whycon_calib_precursor
CALIB_GZ_INIT=0 CALIB_GZ_PING_MS=2 CALIB_GZ_FLY_RC=0 CALIB_GZ_STOP=0
serial_tx=131 serial_rx=50 bridge: guest->udp=5, udp->guest=3
```

`CALIB_GZ_ALT_AFTER=-999.0` is expected in this profile: the current
`sentai.crazy.altitude()` binding uses the SentAI deck telemetry channel, not
the cf2 standard log path exposed by the emulator bridge.  The visual
calibration stack is also not complete yet; `sentai.calib` is only a stable
emulator stub until `sentai.camera` and `sentai.markers` are ported into the
same profile.
