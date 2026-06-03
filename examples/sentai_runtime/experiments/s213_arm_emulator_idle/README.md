# s213 ARM Emulator Idle

B8 experiment for the Renode-first ARM emulator direction.

Goal: prove the smallest useful ARM firmware slice before any real board
peripheral is initialized:

```text
CM7 startup -> C/C++ runtime -> ARM FreeRTOS scheduler -> heartbeat task
```

The experiment builds a minimal emulator target, runs it under Renode, and
stores each run in an `iterNNN_*` folder with logs, verdict, and the Renode
configuration snapshot used for that run.

Run:

```sh
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target uart
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target repl
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target usbhost_task
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target edgetpu_manager_probe
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target edgetpu_task_probe
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target edgetpu_opendevice_probe
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target edgetpu_synth_enum_probe
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target edgetpu_mmio_enum_probe
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target edgetpu_mmio_opendevice_probe
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target fx_storage
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target fs_repl
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target fs_stage_assets
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target fs_asset_check
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_cat_repl
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_fps_repl
```

Expected verdict:

```text
idle / uart PASS when boot_state == 0x300, heartbeat > 1, last_tick > 0
            UART target additionally requires boot + heartbeat strings in uart.log
repl       PASS when boot_state == 0x500, repl_lines >= 1, last_tick > 0,
            and uart.log raw bytes contain "MicroPython embed ready" and "\r\n2\r\n"
```

Targets:

- `idle` - B8.1 scheduler/SysTick heartbeat.
- `uart` - B8.2 LPUART6 file-backend smoke, still without full ConsoleM7,
  USB CDC, or MicroPython REPL.
- `repl` - B8.3 MicroPython embed REPL on LPUART6 RX+TX.  Injects `1+1\r`
  via `lpuart6 WriteChar` and asserts the VM prints `2`.
- `mission` - B8.4 `import mission; mission.run()` against in-firmware VFS.
  Asserts `MISSION OK from B8.4 5` (the `5` proves the function body ran,
  not just the import side effect).
- `camera` - B8.5 VCam Renode peripheral DMA + NVIC IRQ 94 + consumer
  task.  Triggers 5 frame deliveries and asserts
  `FRAME N seq=N byte=0x0N ok=1` for N=1..5 plus
  `frames_consumed == frames_valid == irq_count == 5`.
- `pipeline` - B8.6 Stage1Task → Stage2Task chain behind the VCam IRQ.
  Stage1Task scans the frame, publishes `sum` + `avg` into a shared
  slot, notifies Stage2Task.  Asserts `irq_count == stage1_processed
  == stage2_consumed == 5`, `pipeline_errors == 0`, `last_sum == 320`,
  and that the UART contains `STAGE2 1..5 frame_seq=1..5 sum=64..320`.
  Names are deliberately neutral: production sentai_runtime already
  owns `PrepTask` / `InferTask` / `FlowTask` / `CameraTask` and those
  do real algorithm work that this spike does not implement.
  InferTask/TPU is explicitly out of scope for the emulator path
  (EdgeTPU USB stays deferred).
- `fanout` - B8.7 Stage1Task → {Stage2ATask, Stage2BTask} multi-reader
  fan-out via a real seqlock.  Both consumers read the same scalar
  slot on every IRQ and emit matching `STAGE2A` / `STAGE2B` UART
  marker pairs with identical arithmetic per frame.  Verdict asserts
  `irq_count == stage1_processed == stage2a_consumed == stage2b_consumed
  == 5`, `pipeline_errors == 0`, `seqlock_torn_reads == 0`, and
  `last_sum == 320`.
- `flowest` - B8.7c/d cat-flow offset detection.  Host preprocesses
  `cat_640x480.bmp` from the B7 s209 reference into 6 Y8 32x32
  scenes panned along a varied 2D trajectory.  Renode `LoadBinary`s
  them into `g_frame_buffer` between IRQs; VCam runs with
  `FILL_MODE = 0` so the host bytes survive.  Firmware brute-force
  SAD block matcher detects per-frame motion.  Verdict asserts the
  full per-frame sequence:
  `detected_dx == [0, +2, 0, -3, 0, +4]` and
  `detected_dy == [0, 0, +2, -1, -3, 0]` with `sad == 0` for every
  frame.  Mixes pure-X, pure-Y, diagonal-negative, and positive
  motions so the matcher cannot fake-pass by always reporting the
  same value.  The equivalent B7 SIM run reported `dx = 243 / -129`
  — broken; the emulator returns the textbook answer.  Note: 32x32
  Y8 is NOT production format (production is XRGB8888 from CSI
  → PrepTask Y8 80x60 → FlowTask USADA8 phase correlation).  This
  gate validates the camera-to-flow chain, not those algorithms.
- `usbhost_probe` - B8.8 P1 production USB host constructor probe.
  Links `libs/usb/usb_host_task.cc` and the required NXP USB host
  middleware into the emu profile, calls the production constructor, and
  does not start the scheduler.  Verdict asserts `boot_state == 0xCAFE`,
  `usbhost_step == 2`, and `uart.log` contains
  `UsbHostTask constructor returned`.
- `usbhost_task` - B8.8 P2 production USB host task scheduler smoke.
  Calls the same constructor, calls `UsbHostTask::Init()`, starts the ARM
  FreeRTOS scheduler, and runs a lower-priority heartbeat task.  No USB
  device/model is attached yet.  Verdict asserts `boot_state == 0x600`,
  `usbhost_step == 5`, `heartbeat > 1`, `last_tick > 0`, and that the
  UART log contains `UsbHostTask::Init returned` plus
  `heartbeat task online`.
- `edgetpu_manager_probe` - B8.8 P3a production `EdgeTpuManager`
  singleton above the USB host task.  Starts from the P2 topology, constructs
  the real manager, does not call `OpenDevice()` yet, and then starts the
  scheduler heartbeat.  Verdict asserts `boot_state == 0x600`,
  `usbhost_step == 8`, `heartbeat > 1`, `last_tick > 0`, and that
  `uart.log` contains `EdgeTpuManager singleton returned`.
- `edgetpu_task_probe` - B8.8 P3b production `EdgeTpuTask` QueueTask above
  P3a.  Starts the real EdgeTPU task, lets it register the Coral VID/PID
  callback with the production `UsbHostTask`, and then starts the scheduler
  heartbeat.  No USB device/model is attached yet.  Verdict asserts
  `boot_state == 0x600`, `usbhost_step == 11`, `heartbeat > 1`,
  `last_tick > 0`, and that `uart.log` contains
  `EdgeTpuTask::Init returned`.
- `edgetpu_opendevice_probe` - B8.8 P4a production
  `EdgeTpuManager::OpenDevice()` no-device/error path.  Starts from P3b,
  then calls `OpenDevice()` from a FreeRTOS task with the manager already
  marked as USB-error/no-device.  Verdict asserts `boot_state == 0x600`,
  `usbhost_step == 13`, `heartbeat > 1`, `last_tick > 0`, and that
  `uart.log` contains `OpenDevice returned nullptr after error`.
- `edgetpu_synth_enum_probe` - B8.8 P4b synthetic Coral
  attach/enumeration.  Starts from P3b, creates NXP USB host descriptor
  structs for a Coral-like device (`18d1:9302`, vendor class/subclass,
  bulk IN, bulk OUT, interrupt IN), and injects attach/enumeration events
  through the real `UsbHostTask::HostEvent()` callback path.  Verdict asserts
  `boot_state == 0x600`, `usbhost_step == 16`, `heartbeat > 1`,
  `last_tick > 0`, and that `uart.log` contains
  `synthetic EdgeTPU enum done`.  This is not an EHCI Renode device model yet.
- `edgetpu_mmio_enum_probe` - B8.8 P5a Renode MMIO Coral descriptor block.
  Starts from P3b, reads Coral-like VID/PID, vendor-class interface, and
  endpoint descriptors from the Renode-side `coral_usb` MMIO block at
  `0x40900400`, then injects attach/enumeration through the real
  `UsbHostTask::HostEvent()` callback path.  Verdict asserts
  `boot_state == 0x600`, `usbhost_step == 19`, `heartbeat > 1`,
  `last_tick > 0`, and that `uart.log` contains
  `Renode Coral MMIO enum done`.  This is a descriptor-provider proof, not an
  EHCI Renode device model and not physical Coral USB passthrough.
- `edgetpu_mmio_opendevice_probe` - B8.8 P5b `OpenDevice()` after Renode
  MMIO enum.  Starts from P5a and also calls production
  `EdgeTpuManager::OpenDevice()` from a FreeRTOS task.  The emu-scoped
  `TpuDriver::Initialize()` stub returns success for this target only, so the
  verdict proves manager/task sequencing after an emulator-provided Coral
  descriptor, not real TPU transfers.  Verdict asserts `boot_state == 0x600`,
  `usbhost_step == 22`, `heartbeat > 1`, `last_tick > 0`, and that `uart.log`
  contains `OpenDevice returned context after MMIO enum`.
- `fx_storage` - B8.9 production FileX/LevelX over a Renode raw-NAND bridge.
  This is not FileX-in-RAM: `FxUser*` runs in guest firmware, and only the
  raw NAND page read/program/block erase operations are served by a persistent
  host-backed NAND image.  Verdict asserts `boot_state == 0x600`,
  `fs_size == 41`, `fs_read_ok == 1`, nonzero NAND reads/writes/erases,
  zero NAND errors, and that `uart.log` contains `FX_STORAGE PASS`.
- `fs_repl` - B8.9b MicroPython `sentai.fs` over the same FileX/LevelX
  raw-NAND bridge.  The VM executes `sentai.fs.format/mkdir/write/append/sync/
  size/read_str/exists/ls`; verdict asserts `boot_state == 0x500`,
  `fs_smoke_ok == 1`, `fs_smoke_size == 11`, and UART contains
  `FS_READ hello world` plus `FS_REPL_DONE`.
- `fs_stage_assets` - B8.9c pre-boot asset staging into the production
  FileX/LevelX raw-NAND image.  Firmware writes the COCO EdgeTPU model,
  `cat_640x480.bmp`, and `/mission.py` through `FxUser*`; the source bytes are
  served by a Renode host asset bridge.  Verdict asserts three staged files,
  zero staging errors, zero FileX/NAND bridge errors, and `FS_STAGE PASS`.
  Re-runs are idempotent: if a guest file already exists at the expected size,
  firmware logs `FS_STAGE SKIP` and does not rewrite it.  Current skip verdict:
  `iter54_renode_fs_stage_assets_filex_levelx_nand`, with `stage_bytes == 0`,
  `fx_writes == 0`, and `fx_erases == 0`.
- `fs_asset_check` - B8.9d post-boot readback of the staged NAND image.
  Mounts without formatting and verifies `sentai.fs` can read the model,
  image, and mission from their guest paths.  This is the current accepted
  emulator asset path; USB MSC is not required for first boot/test loops.
- `tpu_cat_repl` - B8.10 guest FS to physical USB Coral proof.  Imports
  `/mission.py`, loads the staged COCO EdgeTPU model and cat BMP through
  `sentai.tpu`, streams them to the Renode host bridge, invokes the existing
  no-PyCoral C++/libusb physical-Coral smoke, returns detections to
  `sentai.pipeline.detections()`, and writes `/detections.txt` in guest FS.
  Current PASS: `iter55_renode_tpu_cat_repl_filex_physical_coral`, top
  detection `class=16 score=0.828`.
- `tpu_fps_repl` - B8.10b steady-state physical USB Coral benchmark through
  the same guest FS and host bridge path.  The FPS window starts after
  `sentai.tpu.load()`, `sentai.tpu.load_image()`, and one warmup invoke.  It
  runs 5 measured invokes in one host C++/libusb smoke process.  Current PASS:
  `iter60_renode_tpu_fps_repl_filex_physical_coral`, `TPU_FPS_X100 == 58`
  (~0.58 FPS), `completed_runs == 5`, and `DETECTIONS_COUNT == 20`.
