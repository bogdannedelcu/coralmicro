# S216 - ARM Emulator Namespace Inventory

Goal: record the current ARM-emulator-visible `sentai.*` surface before moving
the emulator root to the shared production `modsentai.c` table.

Run:

```sh
python3 examples/sentai_runtime/experiments/s216_arm_emulator_namespace_inventory/run_s216.py
```

For live UART while Renode runs:

```sh
python3 examples/sentai_runtime/experiments/s216_arm_emulator_namespace_inventory/run_s216.py \
  --renode-ui
```

The target boots `sentai_emu_namespace_inventory`, autoruns a MicroPython
inventory, writes `/b9/s216_inventory.txt` inside the emulator FileX volume,
prints the same inventory on LPUART6, and archives all logs under the next
`iterNN_*` directory.

This is intentionally a baseline.  It may pass even while many production
namespaces are missing, as long as the inventory completes and the differences
are recorded.

Latest B9 run:

- `iter05_namespace_inventory`
- Renode UI/analyzer enabled
- PASS
- present shared exports: 14
- missing shared exports: 26
- unexpected emulator exports: 0

Note: this target is the "wide runtime bridge" inventory profile and includes
the current `tpu`/`pipeline`/`crazy` emulator bridge surfaces.  Hardware-safe
stub namespaces such as `usb`, `uart`, `imu`, `mic`, `sleep`, `servo`,
`calib`, `object_lifter`, and `safety` are verified separately by S228 so this
inventory target stays under the RT1176 RAM linker limit.
