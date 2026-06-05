# S228 - ARM Emulator Hardware Stub Namespaces

Purpose: verify that hardware-dependent `sentai.*` namespaces are visible in
the ARM emulator and fail safely when the real peripheral is not modeled.

This is not a physical peripheral test.  It documents B9 emulator behavior for:

- `sentai.usb`
- `sentai.uart`
- `sentai.imu`
- `sentai.mic`
- `sentai.sleep`
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

- `iter02_hw_stub_namespaces`
- Renode UI/analyzer enabled
- PASS
- checks: 45
- failures: 0

This is a separate build profile from S216.  S216 includes the current
TPU/Pipeline/Crazy bridge surfaces; S228 includes hardware-safe stub modules.
Keeping them separate avoids overflowing the 256 KiB RT1176 RAM text region
used by the emulator linker script while preserving explicit coverage.
