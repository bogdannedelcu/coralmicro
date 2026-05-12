# s095 — sentai_sim ↔ PX4 MAVLink ping (Phase 6 / Sim.md §10m)

End-to-end verification that `sentai.link` (with the SIM-only UDP
backend `sentai_uart_serial_udp.c` + slim `sentai_link_sim.cc` MAVLink
encoder) successfully exchanges heartbeats with a real PX4 SITL
instance running in the `crazysim-garden` distrobox.

## Architecture

```
  ┌────────────────────────┐         ┌──────────────────────────┐
  │  sentai_sim (host)     │         │  PX4 SITL  (distrobox)   │
  │  ----------------     │         │  ---------------------    │
  │  sentai.link.init()   │         │  mavlink instance 0      │
  │  sentai.link.hb()     │         │  bind  :14580 (offboard) │
  │                        │         │  send  :14540 (telem)    │
  │  UDP bind :14540 ──────┼─────────┼─→ heartbeat / statustext │
  │  UDP send :14580 ←─────┼─────────┼─  heartbeat (1 Hz)       │
  │                        │         │                          │
  │  Reader task parses    │         │  Standard MAVLink v2     │
  │  HEARTBEAT → stats     │         │  no signing, no XRCE     │
  └────────────────────────┘         └──────────────────────────┘

   Separate channel (not used in s095, kept free for later GCS):
  ┌────────────────────────┐
  │  pymavlink/MAVSDK host │  ←→  PX4 GCS @ :18570 (reserved)
  └────────────────────────┘
```

## Port allocation (Sim.md §10m)

| Port  | Owner   | Direction | Notes |
|------:|---------|-----------|-------|
| 14540 | sentai  | bind      | companion listen (PX4 telem) |
| 14580 | PX4     | bind      | offboard listen (commands) |
| 18570 | PX4     | bind      | GCS legacy (free for pymavlink) |
| 19850 | CrazySim| bind      | cflib CRTP (separate, no conflict) |

## Run

```bash
# 1. Distrobox terminal 1 — start PX4 SITL (no gazebo, just core PX4)
distrobox enter crazysim-garden -- bash -c \
   'cd /home/bogdan/work/px4/PX4-Autopilot && PX4_SIMULATOR=none ./build/px4_sitl_default/bin/px4 -i 0 build/px4_sitl_default/etc'

# 2. Host terminal 2 — start sentai_sim, then in REPL:
./build-sim/sim/sentai_sim
>>> import sentai
>>> sentai.link.debug(1)               # print rx/tx
>>> sentai.link.init()                 # bind :14540, peer :14580
>>> sentai.link.heartbeat()            # send 1 hb to PX4
>>> import time; time.sleep(2)
>>> sentai.link.stats()                # expect rx_heartbeat > 0
```

## Pass criteria

- `stats()[0]` (tx_heartbeat) > 0
- `stats()[3]` (rx_heartbeat) > 0  ← critical: PX4 talks back
- `stats()[6]` (last_peer_sysid) == 1 (PX4 default sysid)

## Next phase: REPL-over-MAVLink (Sim.md §10m.b)

Once heartbeat round-trip is green, we use the **TUNNEL** message
(msgid 385) to carry raw REPL bytes:
- payload_type = 0xC0DE (custom — "SentAI REPL")
- payload[0..127] = REPL byte stream

Direction:
- host → sentai_sim: TUNNEL packet with REPL command (e.g. `1+1\n`)
- sentai_sim's link reader detects payload_type 0xC0DE, feeds bytes
  into MicroPython stdin queue
- sentai_sim stdout/stderr → captured → packaged into TUNNEL → host

This gives a `screen /dev/ttyACM0` equivalent over MAVLink, suitable
for both SIM (UDP) and HW (UART2 radio — same `sentai.link` API).

Mirrors the existing Crazyflie radio CRTP `$exec` pattern (memory
`feedback_radio_no_file_transfer.md`) but with MAVLink TUNNEL
instead of CRTP (128 B vs 30 B MTU, ~4× larger replies per packet).
