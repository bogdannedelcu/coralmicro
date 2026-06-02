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
