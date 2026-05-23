---
name: crazy-anti-patterns-check
description: Audit Crazyflie radio bridge (UART2 + CPX + CRTP) code for known anti-patterns that have cost real debug time. Use when touching libs/sentai/sentai_crazy*, modsentai_crazy.c, the CRTP packet path, or before shipping outdoor PX4 + cf2 work.
---

# /crazy-anti-patterns-check

The Crazyflie radio bridge runs over UART2 at 576k baud with CRTP-style framing (`0xAA LEN CH DATA CRC`).  It shares the UART2 hardware with three potential consumers — REPL multi-frame, flow board→drone, telem req-resp — and the race surface is non-obvious.  These anti-patterns each cost hours.

## Audit checklist

### A. CPX must be disabled at Kconfig level

```bash
grep -E 'CONFIG_ENABLE_CPX' /home/bogdan/work/coralmicro/libs/crazy/Kconfig
# expect:  CONFIG_ENABLE_CPX = n  (or absent)
```

**Why**: enabling CPX wires a UART2 static-global that races with our flow / telem channels.  The race window is ~200 µs but recurs on every CRTP fragment.  Symptom: random byte corruption in `0xAA` headers, framing errors that appear as if the cf2 firmware is buggy.

### B. No `cpxSendPacketBlocking()` in RX callback

```bash
grep -rn 'cpxSendPacketBlocking' /home/bogdan/work/coralmicro/libs/crazy/ /home/bogdan/work/coralmicro/examples/sentai_runtime/
```

**Why**: `cpxSendPacketBlocking()` calls the CPX semaphore wait path.  Called from an RX callback (which runs in the UART2 ISR / RX task context), it can deadlock on the same UART2 mutex the callback is holding.  Symptom: board appears alive (REPL responds) but flow packets stop forever.

If you need to send from RX, queue the message and let the periodic sender task pick it up.

### C. No `appchannel` polling from `appMain`

```bash
grep -rn 'appchannelHasOverflowOccured\|appchannelReceivePacket' /home/bogdan/work/coralmicro/examples/sentai_runtime/ /home/bogdan/work/coralmicro/libs/crazy/
```

Calls from `appMain` are wrong context.  Symptom: cf2 firmware reports buffer overflow on every other tick.  appchannel must be consumed via the dedicated app handler task.

### D. Don't assume `link_send(ch, data)` order matches Bitcraze

Bitcraze's `link_send` is `(channel, data, len)`.  Ours is `(data, len, channel)` in some legacy call sites.  Mismatched arg order silently routes a flow packet to REPL multi-frame channel (ch 0) where it gets interpreted as REPL bytes.

```bash
grep -rn 'sentai_crazy_link_send\|sentai.crazy.send_crtp' /home/bogdan/work/coralmicro/ \
  | grep -v 'test\|paper\|\.md'
```
Sanity-check each call site against the current header signature.

### E. No `MICROPY_BEGIN_ATOMIC_SECTION` override

```bash
grep -n 'MICROPY_BEGIN_ATOMIC_SECTION\|MICROPY_END_ATOMIC_SECTION' \
  /home/bogdan/work/coralmicro/examples/sentai_runtime/mpconfigport.h
```

**Why**: overriding MP's atomic section primitive disables the scheduler-drain hooks (`MICROPY_BEGIN/END_ATOMIC` is one of the three required drain paths per `[[async-dispatch-audit]]`).  Symptom: MP `schedule()` callbacks fire silently late or never.

### F. Wire-format invariants

Fixed constants — any code that asserts a different value is wrong:
- Baud rate: 576 000 (NOT 115200 — that's the host CDC-ACM; UART2 is different)
- Frame start byte: `0xAA`
- CRTP port number: `0x0E`
- CRTP MTU: 30 bytes
- Per-fragment payload: 29 bytes (1 MF byte + 29 data)
- `flow_pkt_t` size: 16 bytes (fits one fragment, no MF stitching)

### G. Channel allocation

| CH | Direction | Purpose |
|---|---|---|
| 0 | bidirectional | REPL multi-frame (the Bitcraze appchannel) |
| 1 | board → drone | flow_pkt_t (flow estimate at ~30 Hz) |
| 2 | bidirectional | telem request/response |
| 3 | reserved | future |

Any new channel assignment must extend, not collide with, this table.

## Quick run

```bash
cd /home/bogdan/work/coralmicro
# A
grep -E 'CONFIG_ENABLE_CPX' libs/crazy/Kconfig
# B + C + D + E together
grep -rn -E 'cpxSendPacketBlocking|appchannelHasOverflowOccured|MICROPY_BEGIN_ATOMIC_SECTION' \
    libs/crazy/ libs/sentai/ examples/sentai_runtime/ \
    | grep -v -E '\.md|paper/'
```

Any non-empty grep above (except for the Kconfig `=n` line) → investigate.

## Hardware debug: snoop UART2

If you suspect framing corruption but the source is unclear, attach a USB→serial adapter (CP2102 / FTDI) at 576 000 baud to UART2:
- TX2 = PA2 (board → drone) — adapter RX
- RX2 = PA3 (drone → board) — adapter TX (or just monitor)
- Common GND

Python sniffer:
```python
import serial
s = serial.Serial('/dev/ttyUSB0', 576000, timeout=0.1)
while True:
    b = s.read(64)
    if not b: continue
    print(b.hex())  # expect 0xAA-prefixed frames
```

Isolates board-side framing bug from drone-side parsing bug.

## Reject patterns

- "It works in my test, ship it" — UART2 races appear only under combined REPL + flow + telem load.
- Disabling the watchdog to "give the radio more time" (see `[[watchdog-estimate]]`).
- Re-enabling CPX "because the Bitcraze examples need it".

## See also

- `agent.md` §Crazyflie anti-patterns (~lines 2493-2523)
- `agent.md` §Wire format + channel allocation (~lines 2174-2200)
- `[[async-dispatch-audit]]` skill (related: MP scheduler drain paths)
