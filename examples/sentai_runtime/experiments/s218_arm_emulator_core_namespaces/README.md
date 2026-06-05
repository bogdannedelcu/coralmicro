# S218 - ARM Emulator Core Namespaces

Goal: verify low-risk core `sentai.*` namespaces in the ARM emulator:

- top-level `version`, `verbose`, `debug`, `console`, `run`;
- `sentai.fs`;
- `sentai.fr`;
- `sentai.rtos`;
- `sentai.io`;
- `sentai.sys`.

Run:

```sh
python3 examples/sentai_runtime/experiments/s218_arm_emulator_core_namespaces/run_s218.py
```

For live UART in Renode UI:

```sh
python3 examples/sentai_runtime/experiments/s218_arm_emulator_core_namespaces/run_s218.py \
  --renode-ui
```

The smoke writes `/b9/s218_core_report.txt` and FlightRecorder files into the
emulator FileX volume, then archives UART/Renode logs under `iterNN_*`.

Latest B9 run:

- `iter01_core_namespaces`
- Renode UI/analyzer enabled
- PASS
- checks: 25
- failures: 0
