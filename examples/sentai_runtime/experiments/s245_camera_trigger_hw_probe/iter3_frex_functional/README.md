# s245 iter3 — functional test: does GPIO8_IO27 reach an OV5640 pad?

**Hypothesis**: iter2 showed nothing *drives* GPIO8_IO27, but could not exclude
a high-impedance OV5640 FREX input on the far end. Close the gap from the other
side: make the **sensor** drive, and see whether our pin notices.
**Change vs iter2**: iter2 listened for a driver; this iter creates one. Needs
`pin_probe(park_input)` (build 1596).
**Date**: 2026-09-04
**Build**: `SentAI v1.0 build 1596 (2026-09-04 09:57:41)` — persistent flash.

## Method

OV5640 `0x3016` (PAD_OUTPUT_ENABLE00) enables auxiliary pads as outputs and
`0x3019` (PAD_OUTPUT_VALUE00) sets their level. For each bit:

```
0x3016 = (1 << b)   enable that pad as an output
0x3019 = 0x00       drive low   -> read our pin
0x3019 = 0xFF       drive high  -> read our pin
```

If our pin's reading **follows** `0x3019`, the pad is routed to GPIO8_IO27 — a
*correlated* response, so a positive would be conclusive rather than suggestive.
This needs no knowledge of FREX bit semantics (the OV5640 datasheet has them;
this repo does not), and it produces a static DC level instead of a per-frame
strobe pulse that a millisecond-scale sampler could miss.

Safety: the pin is parked as a high-impedance input for the whole sweep, since
an MCU output driving 0 against a sensor output driving 1 is a short through
both drivers. `0x3017`/`0x3018` are left alone — the NXP driver sets them to
0x00 to high-Z the DVP pads for MIPI (`fsl_ov5640.c:997`), so writing them would
break the link. Every config is restored immediately and camera health is
re-checked after each step.

## Result: ✅ PASS (test valid) — negative finding

16 configs × 2 sensors. The pull-up/pull-down readings were **identical (1, 0)
in every single one**. The pin never tracked what the sensor drove.

Camera stayed streaming throughout (`frame_count` 4916 → 8561), and both
sensors' `0x3016`/`0x3019` verified restored to their originals.

| | cam0 | cam1 |
|---|---|---|
| configs tested | 8 | 8 |
| pin tracked `0x3019` | **0** | **0** |
| registers restored | OK | OK |

## Two corrections made during this iter — both worth recording

**1. The first run was invalid and its verdict was meaningless.** Every probe
returned `None`, including the baseline, yet the script still printed a
confident "NO RESPONSE". The cause was a host-side parse bug: the REPL echoes
the command before the result, so `pin_probe(1)` puts a literal `(1)` into the
stream and the regex matched *that* instead of the real 11-element tuple. iter2
never hit it because it called `pin_probe()` with no argument.
Fix: take the longest parenthesised number group, and **abort if the baseline
read fails** — an instrument that reads nothing cannot produce a negative
result.

**2. The second run produced a false positive.** It reported `RESPONDED` at
`0x3016` bit4 — a bit the pre-flight had just shown is *not implemented*, so the
write went nowhere and the "response" could not be causal. The cause was
judging on the full tuple including the **no-pull** column. A floating CMOS
input has no defined level: it holds residual charge and drifts, which is
exactly what it did (0 → 1, then stayed). Fix: judge only on the pull-up and
pull-down columns, which are well-defined; a genuinely driven pad forces both.

The drifting no-pull column is itself corroborating evidence that the pin is
floating, so it is still recorded — just not used as the discriminator.

## Coverage — why "no response" is not "we missed the right bit"

Pre-flight write-readback established which bits physically exist:

| register | writable bits | note |
|---|---|---|
| `0x3016` PAD_OUTPUT_ENABLE00 | **0x03** (bits 0,1 only) | writing 0xFF reads back 0x03; 0x55 reads back 0x01 |
| `0x3019` PAD_OUTPUT_VALUE00 | 0xFF (all 8) | fully writable |

So the sweep exercised **every pad-enable bit that exists** on this register.
The masked bits are not untested — they are absent. And because `0x3019` is
fully writable, the value bit was covered whatever the pad-to-bit mapping is.

Without this pre-flight, a sensor that silently ignored `0x3016` would have
produced a clean-looking negative that tested nothing.

## Conclusion, with its limit stated

Across all three iters:

- **iter2**: nothing drives GPIO8_IO27 (method validated against a known-routed
  control pin).
- **iter3**: no OV5640 output pad reaches it either, on either sensor, across
  every implemented pad-enable bit.

The practical answer is that **`Gpio::kCameraTrigger` is an inherited name from
upstream coralmicro with nothing behind it on the SentAI board.** There is no
usable hardware trigger line.

The one case this cannot exclude remains a FREX **input** on the sensor: an
input never drives, so it is invisible to a test that listens. Settling that
last case needs the board schematic or a scope — not more firmware.

## Files captured in this folder

- `probe_frex_functional.py` — host-side driver, includes the pre-flight
- `frex_functional.csv` — per-config readings
- `verdict.json` — machine-readable, includes the implemented-bit masks
- `run_probe.log` — full transcript of the final (valid) run
