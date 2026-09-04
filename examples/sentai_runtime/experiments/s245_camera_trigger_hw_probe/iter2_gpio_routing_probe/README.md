# s245 iter2 — is GPIO8_IO27 (`kCameraTrigger`) routed to anything?

**Hypothesis**: `Gpio::kCameraTrigger` (GPIO8_IO27, pad `GPIO_EMC_B2_17`) is an
inherited name from upstream coralmicro with nothing behind it on the SentAI
board — no firmware drives it and no experiment has ever used it.
**Change vs iter1**: iter1 probed the *sensor* side over I²C with stock
firmware; this iter probes the *pin* and needs new firmware.
**Date**: 2026-09-04
**Build**: `SentAI v1.0 build 1595 (2026-09-04 09:22:21)` — RAM-flashed
(`flashtool.py --ram`) per the anti-brick rule, board re-enumerated as
`1fc9:c0a1`.

## Firmware added

- `sentai_pin_probe_cam_trigger()` in `sentai_runtime.cc` — the probe.
- `sentai.io.pin_probe()` in `bindings/modsentai_io.c` — thin binding,
  QSTR table regenerated.

Nothing on the boot path, nothing pre-scheduler, no ISR touched, so none of the
anti-brick triggers apply. `.ramfunc` verified intact after build (0x208 B at
`0x00000c00`, ITCM).

### One design decision worth recording

The SDRAM canary was first written as a `static` buffer in `.sdram_bss`. That is
wrong here, and the tree already says so: the comment above `g_slot_arena` in
`sentai_runtime.cc` warns that **adding static buffers to `.sdram_bss` shifts the
FileX/LevelX state and silently breaks NAND I/O**. Rewritten to `malloc()` from
`m_heap`, which is SDRAM-backed (`ORIGIN = 0x80000000` in the linker script) and
is the alternative that same comment sanctions. The first form also produced an
assembler section-attribute warning; the `malloc` form builds clean.

## Method

Weak-pull flip test: configure the pad as an input, read it under the internal
pull-up, then under the internal pull-down. A pad with nothing on it follows the
pull (1 then 0). A pad tied to a rail or driven by another device ignores the
~50 kΩ internal pull and reads the same both times.

**Methodology control**: the identical sequence runs on `Gpio::kEdgeTpuPgood`
(GPIO8_IO26, pad `GPIO_EMC_B2_16`) — the physically adjacent pad in the same
bank, known routed to the EdgeTPU power-good signal. Without a control, a
"floating" reading could just mean the test does not work.

**Drive safety**: the capacitance sub-test turns the pad into an output. It is
gated on the pull test having already shown nothing drives the net — two
push-pull drivers on one net is a short through both. That gate is what keeps
the probe from ever driving the TPU's power-good output.

## Result: ✅ PASS (method validated, question answered as far as it can be)

5 runs, perfectly stable:

| pin | pad | no-pull | pull-up | pull-down | charge (cyc) | verdict |
|---|---|--:|--:|--:|--:|---|
| `kCameraTrigger` (under test) | `GPIO_EMC_B2_17` | 0 | 1 | 0 | 876 | **FLOATING-or-HIGH-Z** |
| `kEdgeTpuPgood` (control) | `GPIO_EMC_B2_16` | 1 | 1 | 1 | n/a | **TIED/DRIVEN** |

- **The control behaved as a routed, driven pin must** — it held 1 against the
  internal pull-down. So the test genuinely distinguishes the two cases; a
  floating reading on the pin under test is a real measurement, not an artefact.
- **`kCameraTrigger` follows the internal pulls exactly**, in both directions,
  identically across all 5 runs. **Nothing on the board is driving this net.**
- **SDRAM canary: 0 mismatches in all 5 runs.** This confirms empirically the
  static analysis that `SEMC_DM03` is unused at `kSEMC_PortSize16Bit` — re-muxing
  the pad does not disturb memory.

## What this does and does not settle

**Settled**: no device output, pull resistor, or rail is connected to
GPIO8_IO27. Any scheme that expected to *read* a trigger or handshake signal on
this pin is dead.

**Not settled, by construction**: a high-impedance *input* on the far end — which
is exactly what an OV5640 FREX trigger input is — is indistinguishable from an
absent trace by this method. So this result cannot rule out that the pad reaches
the sensors' FREX pins.

The charge-time column was meant as a secondary hint (more copper and more pins
on a net = more capacitance = slower charge through the pull-up). It is **not
usable as measured**: the 876 cycles (~1.1 µs at 800 MHz) include the
`GpioSetMode()` call itself — `GPIO_PinInit` plus two IOMUXC writes — which is
very likely the dominant term. Separating the RC from that fixed overhead would
need a calibration run against a pad known to be unrouted, which we do not have.
Recorded for completeness, not relied on.

**The conclusive test remains functional**: enable the FREX pad on the sensor
(`0x3016`), arm FREX mode, toggle the GPIO, and see whether the sensor responds.
That is a positive-only test — a null result would still be ambiguous.

## Why this may no longer matter

iter1 found `0x3B08` **FREX_REQUEST**, a software frame-exposure request over
I²C. If it works as named, the sensor can be snapshot-triggered with **no
trigger pin at all**, which retires this whole line of questioning.

## Files captured in this folder

- `probe_pin.py` — host-side driver
- `pin_probe.csv` — per-pin summary
- `verdict.json` — machine-readable, includes all 5 raw runs
- `run_probe.log` — full transcript
