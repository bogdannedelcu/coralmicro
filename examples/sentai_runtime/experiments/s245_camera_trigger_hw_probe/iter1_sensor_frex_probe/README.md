# s245 iter1 — OV5640 FREX / snapshot block probe (sensor side)

**Hypothesis**: the OV5640 FREX (frame-exposure / snapshot) block is reachable
and configurable over I²C on both sensors, so a sensor-side hardware trigger is
at least *possible* on this board.
**Change vs prior iter**: baseline — first iter of s245.
**Date**: 2026-09-04
**Build**: `SentAI v1.0 build 1593 (2026-07-14 10:46:19)` — no firmware change,
`sentai.camera.reg_read/reg_write` already expose raw SCCB access.

## Result: ✅ PASS

- **0 read errors** across 27 registers × 2 sensors.
- **0 disagreements** between cam0 and cam1 — the two OV5640s are configured
  identically, which any dual-camera trigger scheme depends on.
- **FREX block is LIVE, not a read-only shadow**: write-readback-restore on
  `0x3B04`/`0x3B05` (FREX_EXPOSURE) with patterns 0x5A and 0xA5 read back exact
  on both sensors, and both restored cleanly to their original values.

## What the register values say

| Register | Value (both cams) | Meaning |
|---|---|---|
| `0x3B00` SRB_CTRL | `0x00` | strobe control **disabled** |
| `0x3B07` FREX_MODE | `0x08` | reset default — FREX not armed |
| `0x3B08` FREX_REQUEST | `0x00` | no frame-exposure request pending |
| `0x3016` PAD_OUTPUT_ENABLE00 | `0x00` | **FREX/STROBE pads not enabled as I/O** |
| `0x3019` PAD_OUTPUT_VALUE00 | `0xF0` | pad output values (pads disabled anyway) |
| `0x3008` SYSTEM_CTROL0 | `0x02` | normal operation, not in software standby |
| `0x300E` MIPI_CONTROL00 | `0x45` | MIPI active |
| `0x3035/0x3036` PLL | `0x14`/`0x54` | VGA45 timing, matches `camera.fps()` = 45 |

So the sensors ship with the whole snapshot/strobe path **present but switched
off** — exactly what the firmware audit predicted, since nothing in the tree ever
programs `0x3B00-0x3B0C`.

## The finding that changes the plan

`0x3B08` is **FREX_REQUEST** — a *software* frame-exposure request. If that
behaves as its name implies, the sensor can be snapshot-triggered over I²C
**without the GPIO being routed at all**, which would make the whole
GPIO8_IO27 routing question optional rather than blocking.

That does not make it *fast*: an I²C transaction is ~0.1 ms/register at 400 kHz,
the frame still has to traverse MIPI at the sensor's line rate, and FREX adds
shutter-delay + reset-length settling (`0x3B02/0x3B03`, `0x3B0A/0x3B0B`). The
value of a sensor trigger is **determinism and cross-camera synchronisation**,
not throughput. Throughput is what iter3 measures at ISR level.

## Files captured in this folder

- `probe_frex.py` — the probe (host-side, drives the REPL)
- `frex_dump.csv` — all 27 registers × 2 sensors
- `verdict.json` — machine-readable verdict
- `run_probe.log` — full run transcript

## Caveat

Register liveness proves **I²C reach**, not that FREX *functions*. Arming it
requires enabling the pad (`0x3016`) and either driving the FREX pin or issuing
`FREX_REQUEST`. Whether the pin is physically routed is iter2.
