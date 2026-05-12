# s099 — C-side flow forwarder (Python on/off only)

Verifies the new architecture introduced 2026-05-12:

  `sim/sentai_link_sim.cc::link_flow_forward_task`

reads the camera-bridge `g_flow` snapshot in C, converts mgrid→radians
in C, calls `sentai_link_send_flow()` (also C) — no MicroPython VM in
the hot loop.  Python exposes `sentai.link.flow(0/1[, dist_m])` as the
on/off switch only.

Motivation: parity with ARM firmware where the Crazy radio bridge
auto-inits in firmware (memory: `project_crazyflie_radio_bridge.md`),
so the equivalent PX4 path on SIM should not depend on Python being
scheduled per frame.

## Pass criteria

1. `link.flow(1)` returns 1 and prints `flow forwarder task started`.
2. With a synthetic camera feeder bumping `g_flow.seq` at ~30 Hz, the
   sniffer on UDP 14580 sees ≥ 10 MAVLink OPTICAL_FLOW_RAD (msgid 106)
   frames over a 1-second window.
3. `link.stats()[8]` (the new `tx_flow` counter) matches the sniffer
   count ±1.
4. `link.flow(0)` returns 1 and stats freeze.

## How to run

```bash
cd /home/bogdan/work/coralmicro
python3 examples/sentai_runtime/experiments/s099_link_flow_c_forward/run.py
```

The script handles its own sim spawn, synthetic feeder, sniffer, and
teardown.  No Gazebo needed.
