# S219 - ARM Emulator Crazy Bridge

Goal: prove that the ARM-emulated SentAI runtime can talk to CrazySim `cf2`
through the shared `sentai_crazy.cc` path.

The tested path is:

```text
guest FreeRTOS task -> sentai_crazy.cc -> sentai_uart_serial_* ->
SENTAI_ARM_EMU serial MMIO bridge -> Renode PythonPeripheral ->
cf2 CRTP UDP 127.0.0.1:19850
```

This is a transport substitution at the serial boundary only.  It does not use
the POSIX SIM `sentai.crazy` implementation and does not introduce a new
MicroPython API.

Run:

```sh
python3 examples/sentai_runtime/experiments/s219_arm_emulator_crazy_bridge/run_s219.py
```

The default world is `sentai_whycon_small`, matching the circle/WhyCon marker
missions (`s187`, `s207`, and the B3/B4/B5 calibration/control line).  Use
`--world sentai_crazysim` only for a CRTP-only smoke where the visual marker
pad is irrelevant.

For interactive debugging, keep the Renode UI/analyzers enabled:

```sh
python3 examples/sentai_runtime/experiments/s219_arm_emulator_crazy_bridge/run_s219.py \
  --renode-ui
```

With `--renode-ui`, the runner selects the matching `_ui.resc` script and
opens `showAnalyzer lpuart6`, so UART output is visible live in the Renode UI.
The file backend stays enabled, so the same UART text is still archived into
the experiment folder.

The MicroPython namespace smoke can use the same UI mode:

```sh
python3 examples/sentai_runtime/experiments/s219_arm_emulator_crazy_bridge/run_s219.py \
  --mode mp --renode-ui
```

The runner:

- compile-checks the host CPX/CRTP parser;
- configures and builds `sentai_emu_crazy_ping_smoke`;
- respawns CrazySim/cf2 with `sim/scripts/respawn_sitl.sh sentai_whycon_small`;
- runs Renode with `emu/renode/sentai_emu_crazy_ping_smoke.resc`;
- archives Renode, UART, cf2, and bridge logs into the next `iterNN_*`
  directory;
- stops the cf2/Gazebo helper processes unless `--keep-sitl` is passed.

PASS criteria:

- `sentai_crazy_init(576000)` returns 0;
- `sentai_crazy_ping(1500)` returns a non-negative ping time;
- Renode boot state reaches `0x0B00`;
- bridge log shows both guest-to-cf2 and cf2-to-guest CRTP traffic.

Current status:

- C++ smoke path passes against cf2.
- First archived run:
  `iter01_renode_crazy_cf2_bridge/verdict_s219.json`.
- Result: `boot_state=0x0B00`, `init_rc=0`, `ping_ms=5`,
  `serial_tx=54`, `serial_rx=22`, bridge `guest->udp=1`,
  `udp->guest=1`.
- Re-run after switching the B9 default world to WhyCon/circle markers:
  `iter02_renode_crazy_cf2_bridge/verdict_s219.json`.
- Result: `world=sentai_whycon_small`, `boot_state=0x0B00`,
  `init_rc=0`, `ping_ms=1`, `serial_tx=54`, `serial_rx=22`,
  bridge `guest->udp=1`, `udp->guest=1`; cleanup left no cf2/Gazebo/Renode
  processes running.
- MicroPython namespace run with Renode UI/analyzer enabled:
  `iter03_renode_crazy_mp_cf2_bridge/verdict_s219.json`.
- Result: `world=sentai_whycon_small`, `renode_script=sentai_emu_crazy_repl_ui.resc`,
  `boot_state=0x0500`, `CRAZY_MP_INIT=0`, `CRAZY_MP_PING_MS=1`,
  `CRAZY_MP_STOP=0`, `serial_tx=56`, `serial_rx=22`, bridge traffic in both
  directions, and no leftover cf2/Gazebo/Renode processes after cleanup.
- Next step is to extend the MicroPython namespace smoke to telemetry and
  conservative arm/disarm commands, with mission metrics logged through
  `sentai.fr`.
