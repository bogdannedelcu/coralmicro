# s086_radio_smoke

End-to-end smoke test for the Crazyflie radio bridge: prove the host
can drive the SentAI MicroPython REPL purely over Crazyradio PA, with
the board on the drone's UART2 and **no USB cable**.

## What this proves

Path under test:

```
host PC (cflib)
  └─► Crazyradio PA  ──CRTP port 0x0E ch=0──►  Crazyflie 2.1 STM32
                                                    │ (sentai deck driver)
                                                    └─► UART2 @ 576 000 baud
                                                          ├─► SentAI rx_task
                                                          │   (0xAA frame parser)
                                                          ├─► dispatch_push(EXEC)
                                                          ├─► mp_sched_schedule
                                                          └─► crazy_run_exec
                                                                ├─ compile + eval "1+1"
                                                                └─► link_send(0, b'OK 2')
                                                                      └─► UART2 ─► STM32 ─► radio ─► host
```

Builds required:

- Board firmware ≥ #1223 (crazy bridge auto-inits in firmware, not in `/main.py`).
- Drone fork: `bogdannedelcu/crazyflie-firmware`, branch `sentai-deck-driver`,
  with `CONFIG_ENABLE_CPX=n` in `examples/app_sentai_bridge/app-config`
  (otherwise CPX races the deck driver for UART2 and TX wedges).

## Files

- `test_radio_1plus1.py` — sends `$1+1`, expects `b'\x00OK 2'` back.
- `check_bridge_counters.py` — reads `deck.sentai{R2U,U2R,Ucrc,Ubad,Flow,…}`
  PARAMs to localise which leg of the chain is broken when the smoke test fails.

## Run

```bash
# from the repo root
/home/bogdan/work/crazyflie/.venv/bin/python \
    examples/sentai_runtime/experiments/s086_radio_smoke/test_radio_1plus1.py
```

Expected on success:

```
sending port=0x0E ch=0 data=bytearray(b'$1+1')
got 1 fragment(s) totalling 5 bytes:
  [0] b'\x00OK 2'
PASS: $1+1 -> OK 2 via radio
```

## Diagnosing failures (from `check_bridge_counters.py`)

| `R2U` | `U2R` | `Ucrc` / `Ubad` | Likely cause |
|-------|-------|------------------|--------------|
| 0     | 0     | 0                | deck driver not loaded on drone, or wrong URI |
| ≥1    | 0     | 0                | board hung, build w/o crazy auto-init, or UART RX wire broken |
| ≥1    | 0     | ≥1               | board IS sending bytes, but UART framing mangled (CRC/bad len) — re-seat connector |
| ≥1    | ≥1    | 0                | bridge healthy; if `test_radio_1plus1.py` still fails, dispatcher race or MP REPL silent (see agent.md §17.5) |

`cf.param.get_value()` returns CACHED values — to refresh, reconnect
(re-run the script). Per agent.md §18, for live counters use a LOG
block instead of PARAM polling.

## Historical context

The original 2026-05-09 validation script (under `/tmp` at the time) is
the one referenced in `agent/experiment.md` §"Session 2026-05-09 — FS
hardening + radio auto-init (build #1223)". Same payload, same wire
format; the on-board bridge has not changed since.
