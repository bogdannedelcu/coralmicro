# S228 - ARM Emulator Hardware Stub Namespaces

Purpose: verify that hardware-dependent `sentai.*` namespaces are visible in
the ARM emulator and fail safely when the real peripheral is not modeled.

This is not a physical peripheral test.  It documents B9 emulator behavior for:

- `sentai.usb`
- `sentai.uart`
- `sentai.imu`
- `sentai.mic`
- `sentai.servo`
- `sentai.calib`
- `sentai.object_lifter`
- `sentai.safety`

Run:

```sh
python3 examples/sentai_runtime/experiments/s228_arm_emulator_hw_stub_namespaces/run_s228.py --renode-ui
```

Expected result: all checks pass, with no claims of real actuator/USB/audio
success.  The runner stores Renode/UART logs, copied `.resc` files, and
`verdict_s228.json` under the next `iterNN_hw_stub_namespaces` folder.

Latest B9 run:

- `iter07_hw_stub_namespaces`
- Renode UI/analyzer enabled
- PASS
- checks: 69
- failures: 0
- build: #1496 at 2026-06-11 18:35:05
- storage setup: existing `sentai_emu_fx_storage` PASS before S228
- FileX/FlightRecorder smoke: PASS (`/fr/s228_events.csv`,
  `/fr/s228_scalars.csv`, `/b9/s228_status.txt`)

This is a separate build profile from S216.  S216 includes the current
TPU/Pipeline/Crazy bridge surfaces; S228 focuses on hardware-sensitive
namespaces.  `sentai.usb`, `sentai.uart`, `sentai.object_lifter`,
`sentai.servo`, `sentai.calib`, and `sentai.safety` use shared
`sentai_runtime` bindings with either real shared logic or explicit unavailable
backends.

Build-only follow-up after the shared-binding cleanup:

- `sentai.usb` uses shared `modsentai_usb.c`; S228 links explicit unavailable
  `sentai_usb_*` backend functions for MSC, CDC serial, and IP.
- `sentai.uart` uses shared `modsentai_uart.c`; S228 links weak unavailable
  `sentai_uart_serial_*` backend functions, while Crazy/Gazebo targets can
  override the same ABI with the Renode serial bridge.
- `sentai.object_lifter` uses shared `modsentai_object_lifter.c` and
  `sentai_object_lifter.cc`; no local EMU lifter module remains.
- `sentai.servo`, `sentai.calib`, and `sentai.safety` use the shared runtime
  bindings/state implementations; only missing worker/peripheral backends are
  linked as explicit unavailable stubs in this target.
- The runner resets the host-backed Renode raw NAND image and runs the existing
  `sentai_emu_fx_storage` setup before S228.  That keeps storage handling at
  the emulator flash boundary; the S228 runtime smoke then uses normal
  `sentai.fs`/`sentai.fr` calls and stores logs under the mission `iterNN`
  artifact folder.
