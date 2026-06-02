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
