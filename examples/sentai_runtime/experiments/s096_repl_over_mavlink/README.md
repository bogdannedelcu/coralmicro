# s096 — REPL over MAVLink TUNNEL (Phase 6a)

Drive the `sentai_sim` MicroPython REPL remotely via MAVLink TUNNEL
messages.  Same protocol on x86 SIM (UDP) and ARM HW (UART2 radio).

## Mechanism

```
  ┌──────────────────────┐
  │  host                │
  │  (this script)       │
  │  hand-rolled MAVLink │
  │  v2 TUNNEL encoder   │
  └─────────┬────────────┘
            │ UDP :14540 (sentai bind)
            │ TUNNEL{msgid=385, payload_type=0xC0DE,
            │        target_sys=1, payload=bytes}
            ▼
  ┌──────────────────────┐
  │ sentai_sim           │
  │  ↓ link_reader_task  │
  │  detects TUNNEL,     │
  │  payload_type=0xC0DE │
  │  → repl_fifo_push    │
  │                      │
  │  ↓ sim_read_line()   │
  │  drains FIFO + stdin │
  │  → MP REPL exec      │
  └──────────────────────┘
```

`payload_type = 0xC0DE` is in the >32767 "local experimental" block
per MAVLink spec — won't collide with registered tunnel types.

## Run

```bash
# Terminal 1 — start sentai_sim and arm sentai.link
mkfifo /tmp/sim_repl_fifo
nohup ./build-sim/sim/sentai_sim < /tmp/sim_repl_fifo > /tmp/sentai.log 2>&1 &
( tail -f /dev/null > /tmp/sim_repl_fifo ) & disown
echo "import sentai" > /tmp/sim_repl_fifo
echo "sentai.link.init()" > /tmp/sim_repl_fifo

# Terminal 2 — inject REPL commands via TUNNEL
python3 host_tunnel_send.py "print(2+2)"
python3 host_tunnel_send.py "sentai.version()"
python3 host_tunnel_send.py "a=10; b=32; print(a*b)"

# Terminal 1 — see output in tail of /tmp/sentai.log
```

Verified 2026-05-12: all 3 commands round-trip cleanly, output
appears in sentai_sim's console.

## Constraints

- 128 B per TUNNEL payload (MAVLink message MTU).  Larger lines must
  be chunked (host script does this automatically).
- Currently UNIDIRECTIONAL (host→sentai).  Stdout capture + reverse
  TUNNEL is Phase 6c work.  For now the operator reads command results
  from the sentai_sim console.
- Same TUNNEL protocol works on ARM over UART2 to a real radio link
  (Crazyflie / SiK / RFD900), no changes to `sentai_link.cc`.

## Future (Phase 6c — bidirectional)

Capture MicroPython stdout (via `mp_sys_stdout_obj` redirect to a
ring buffer in `main_sim.c`), and after each line exec send the
captured bytes back as TUNNEL with payload_type=0xC0DE → host.
Host script blocks waiting for response → real terminal feel.
