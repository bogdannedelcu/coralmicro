# s203 - B4 HL Marker Control With Flow

Purpose: continue B4 after s201.  s201 proved that the A3 calibration artifact
can be loaded, ExtPos can warm up the estimator, and HL handoff can be made
stable, but it did not prove smooth A4 lateral navigation.  s203 keeps the
s201 handoff sequence and adds `sentai.flow` as an estimator aid, following the
older smooth-navigation experiments.

Seed calibration:

```text
../s197_sota_calib_orientation_guarded/
  iter27_camera_landing_contract/
    mission_s197_calibration.json
```

Runtime shape:

```text
run.sh
  -> stage calib.ini + mission_s203.py
  -> launch Gazebo GUI + sentai_sim
  -> mission_s203 reads sentai.flow in REPL/runtime
  -> mission_s203 injects SENSOR_FLOW_SIM into CF2 EKF
  -> A3 RPYT bootstrap + WhyCon ExtPos warmup
  -> HL handoff
  -> +x, -x, center, +y, -y, center
  -> soft land
```

Estimator inputs are intentionally distinct:

- WhyCon/PnP calibration path sends absolute position via
  `sentai.crazy.send_extpos(x, y, z)`.
- `sentai.flow` sends optical-flow increments to the CF2 EKF through the same
  runtime CRTP path as ExtPos.  In SIM this is `sentai.crazy.send_crtp(...)`;
  on ARM the equivalent packets go over the UART/CPX link to Crazyflie.
- CF2 Kalman fuses both with IMU/baro; GT remains post-mortem only.
- There is no host flow forwarder in the validation path.  Host scripts may
  inspect artifacts post-mortem only.

s203 explicitly sets `locSrv.extPosStdDev` to a conservative CV value before
the first ExtPos injection.  The firmware default is `0.01 m`, which is too
optimistic for live WhyCon/PnP during tilted motion; local Crazyflie notes
recommend `0.02-0.05 m` for good computer-vision position input.  The first
s203 value is `0.04 m` and is logged as an experimental parameter.

Success is not just command return code success.  The post-mortem verdict must
show real GT displacement along the commanded axes, bounded cross-axis motion,
and lower roll/pitch oscillation than s201 iter17/iter18.

Run:

```bash
bash examples/sentai_runtime/experiments/s203_hl_marker_control_with_flow/run.sh iter1_flow_hl_axis_smoke
```
