# s229 - ARM Emulator Gazebo Calib Precursor

This B9 experiment boots the ARM-emulated SentAI runtime in Renode, starts the
CrazySim `cf2` Gazebo world with WhyCon circle markers, and runs a conservative
MicroPython mission through the shared `sentai.crazy` binding.

Scope:

- Validate Renode guest -> CPX/UDP bridge -> CrazySim `cf2` while Gazebo is
  running the WhyCon world.
- Run `sentai.crazy.init`, `ping`, and a small `fly(0.35m)` takeoff/hold/land.
- Probe `sentai.calib` only as the current emulator stub boundary.

Non-goal for this iteration:

- Full visual calibration.  That still needs `sentai.camera`,
  `sentai.markers`/WhyCon, and the real C++ calibration task ported into the
  emulator profile.

Run:

```bash
python3 examples/sentai_runtime/experiments/s229_arm_emulator_gazebo_calib_precursor/run_s229.py --renode-ui
```

Artifacts are written under `iterNN_gazebo_whycon_calib_precursor/`.

Status:

- `iter01_gazebo_whycon_calib_precursor`: PASS.
- Renode UI/analyzer enabled.
- `CALIB_GZ_INIT=0`, `CALIB_GZ_PING_MS=2`, `CALIB_GZ_FLY_RC=0`,
  `CALIB_GZ_STOP=0`.
- Bridge counters: `guest_to_udp=5`, `udp_to_guest=3`, `serial_tx=131`,
  `serial_rx=50`.
- Cleanup check found no leftover Renode/Gazebo/cf2/bridge processes.

Known limitation:

- `CALIB_GZ_ALT_AFTER=-999.0` because this profile validates CRTP/cf2 traffic;
  the current `sentai.crazy.altitude()` binding expects the SentAI deck
  telemetry channel.
