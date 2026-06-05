---
name: project-emu-transport-bridge-pattern
description: "Architectural pattern for B8 emulator transports — emu runs REAL firmware code, Renode peripheral wraps host transport (libusb, TCP, file backend, sysbus injection) to either hardware-in-loop (real silicon) or simulator-in-loop (cf2-SITL, Gazebo)."
metadata: 
  node_type: memory
  type: project
  originSessionId: 8b2903f8-f695-46ef-9f13-edba1cccdfeb
---

Established 2026-06-02 during B8.8 TPU pivot conversation.

The emulator must run the **real production firmware code path** for any
peripheral/transport gate.  Renode peripheral provides the MMIO/IRQ
boundary; the bytes are bridged to a host endpoint that is either:

- **Hardware-in-loop**: real silicon (e.g., Coral USB Edge TPU,
  real Crazyradio dongle).
- **Simulator-in-loop**: another simulator (e.g., cf2-SITL inside
  CrazySim/Gazebo, real-time IMU model).
- **File/log**: capture for verdict (e.g., LPUART6 console -> file
  backend, camera scene .bin -> sysbus LoadBinary).

Why: this gives the emulator predictive value for on-board behavior —
the same C++ path that runs on the M7 in silicon runs in the emulator,
and bugs surface in either place identically.  A "mailbox bypass"
(host script does the work, firmware never runs the real code) is
explicitly NOT acceptable per
`feedback_emu_tpu_must_run_real_edgetpu_manager`.

Mapping table for the SentAI subsystems we expect to bridge:

| Transport     | Production path (real silicon)                    | Renode peripheral                | Bridge to host                       | Host endpoint                                  |
| ------------- | ------------------------------------------------- | -------------------------------- | ------------------------------------ | ---------------------------------------------- |
| LPUART6 console | `libs/base/console_m7` -> LPUART6                | `UART.NXP_LPUART` (built-in)     | `CreateFileBackend`                  | log file (already done in B8.2)                |
| Camera CSI    | OV5640 -> MIPI CSI2RX -> CSI receiver -> CameraTask | VCam Python peripheral (custom) | `sysbus LoadBinary` per frame        | scene .bin per frame (already done in B8.5)    |
| TPU USB       | `libs/tpu/edgetpu_manager.cc` -> `usb_host_task.cc` -> `usb_host_ehci.c` -> USB OTG2 -> Coral | NXP-EHCI model (TBD: write or extend USBDeprecated) | libusb or USB/IP plugin (TBD: probably fork Renode) | **physical Coral USB Edge TPU** plugged into host USB |
| Crazyflie CRTP | `libs/sentai/sentai_crazy*` -> LPUART2 -> CPX -> CRTP -> nrf radio -> cf2 | `UART.NXP_LPUART` (built-in)  | Renode `UartConnector` to TCP socket or PTY | **cf2-SITL inside CrazySim/Gazebo** on host    |

The Crazyflie bridge is structurally simpler than TPU USB because:
- Renode's `UART.NXP_LPUART` is built-in and matches the LPUART2 register
  layout already.
- Bridging UART to a host TCP socket is a feature Renode supports
  out-of-the-box (UartConnector + TCP backend), so no Renode fork
  should be required.
- cf2-SITL already exposes CRTP over a TCP socket in production SIM
  setups (see `project_no_flow_deck_camera_imu_only` memory entry).

The TPU bridge is structurally harder than Crazyflie because:
- Renode does not ship an NXP RT1176 EHCI model; the closest is the
  generic `USBDeprecated.EHCIHostController`.
- USB device passthrough to a host USB device (libusb / USB/IP) is
  not in renode_portable scripts/plugins.  Likely requires a fork.

How to apply:

- When planning a new emu gate that needs an external transport,
  first identify which row of the table it matches.
- Match the production C++ path exactly (link the same source files;
  do not write parallel emu-only drivers that talk to the same MMIO).
- Document the Renode model + host bridge choice in the B8 doc and
  the gate's commit message.
- If a Renode patch is required, follow the
  `external-repo-patch-log` policy (patches under `patches/`, or
  fork under a separate org repo).

Cross-refs:
- [[feedback_emu_tpu_must_run_real_edgetpu_manager]] (no mailbox bypass)
- [[feedback_emu_must_not_reuse_production_task_names]] (emu test
  scaffolding stays visibly distinct)
- [[feedback_emu_repl_transport_caveat]] (UART will eventually carry
  CRTP, so UART-as-REPL is test scaffolding only)
