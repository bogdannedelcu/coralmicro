---
name: PX4 flow forwarder is C-side (Python on/off only)
description: SIM `sentai.link.flow(0/1[, dist])` toggles a FreeRTOS C task that reads g_flow snapshot and sends OPTICAL_FLOW_RAD — zero Python in the per-frame path. Parity with ARM Crazy bridge auto-init.
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
Architecture decision (2026-05-12): the flow → PX4 OPTICAL_FLOW_RAD
forwarder runs entirely in C/C++ inside `sim/sentai_link_sim.cc`.

**Why:** parity with the ARM HW path where the Crazyflie radio bridge
auto-inits in firmware before /main.py (memory
`project_crazyflie_radio_bridge.md`).  Operator request: "*nu mai
trecem prin Python odata ce e activat*".

**How to apply:**
- The hot loop (camera_bridge_recv.c → flow_phase_corr.cc →
  `g_flow` snapshot → MAVLink encode → UDP send) is all C/C++.
- Python role on PX4 SIM is on/off only:
  ```python
  sentai.link.init(57600, 1, 191)
  sentai.link.flow(1, 1.0)   # enable; second arg = distance in m
  sentai.link.flow(0)        # disable
  ```
- The C task `link_flow_forward_task` runs at `tskIDLE_PRIORITY + 2`,
  polls `sim_camera_flow_snapshot()` every 20 ms, dedups by `seq`,
  converts mgrid → rad with `MGRID_TO_RAD = 12.6e-6f` (1 L0 grid-px
  = 12.6 mrad at HFOV 58° / 640px / 8× decim).
- New stats field at index 8: `tx_flow` counter
  (`sentai.link.stats()[8]`).
- The Python binding reuses the existing `flow` QSTR (no regen needed)
  by overloading `sentai.link.flow()` — `link` namespace disambiguates
  from `sentai.flow` (read flow snapshot).
- DO NOT add a Python loop calling `sentai.link.send_flow()` per frame
  — that was the rejected approach.  Use `link.flow(1)` instead.

**Verified by:** `examples/sentai_runtime/experiments/s099_link_flow_c_forward/`
— synthetic 30 Hz camera feeder → sim → UDP sniffer on :14580
caught 42 OPTICAL_FLOW_RAD frames in ~2 s, `stats[8]` matched exactly.

**On ARM HW (future work):** mirror the same pattern — add a
`sentai_link_flow_forward()` C entry in `examples/sentai_runtime/sentai_link.cc`
that auto-starts alongside the existing Crazy bridge for MAVLink
peers.
