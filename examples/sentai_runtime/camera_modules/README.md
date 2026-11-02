# Camera module registry (OV5640 modules in stock) — WBS OP-S10-W24-T22

**This folder is the project's database of camera modules.  It is kept, versioned and
extended for every module we own: one assigned ID per physical module, its register set(s)
and its measured characterisation.**  Operator decision 2026-11-02.  Nothing here is
derivable from the code or the sensor: the modules carry no factory identifier (the OTP of
both current modules reads all zero, chip ID `0x5640` is the same on all), so this registry
is the only link between a physical module, its calibration and the board connector it sits on.

## Files

- `modules.json` — machine-readable registry (one entry per module; schema below).
- `regs/<ID>_<reference>_<date>.json` — register sets as `{"0xADDR": value}` (109 registers:
  AEC/AGC freeze, AWB gains, gamma, CMX, SDE, LENC), replayable after `camera.init()` with
  `camera.reg_write(cam, addr, val)` (s265 `tune_colors.py --apply`, group write 0x3212
  recommended for production).
- Measurements live with the experiment that produced them (s265 `iter1/`, `iter3_kamvas/`);
  this folder only references them.

## ID scheme

- Human ID `SAI-CAM-NNNN` (sequential, append-only, never reused) + a 128-bit UUID4.
- Planned OTP layout (not programmed yet, see below): OTP bytes `0x3D05..0x3D17` = ASCII `SA`
  (2) + UUID (16) + CRC-8 (1) = 19 of the 27 bytes OmniVision leaves to the user
  (`0x3D00..0x3D04` are reserved by OmniVision; 32 bytes = 256 bits in total).

## OTP status — NOT programmed (hardware constraint, 2026-11-02)

The OV5640 datasheet (v2.03 §4.11, table 4-11 footnote a) requires **AVDD = 2.1 V ± 5 % while
programming** the OTP ("otherwise there will be reliability issues"; the Arducam application
note says 2.5 V ± 5 %); reading has no such requirement.  On the SentAI board AVDD is the
PMIC rail `VDD_2V8_CAM1/2` (MCP16701 LDO3 / LDO2) at a fixed 2.8 V; the firmware only switches
the rails on and off (`sentai.camera.power_rails()`).  The MCP16701 LDOs are programmable
0.6–3.8 V in 25 mV steps, so a programming mode (rail to 2.1 V → program → back to 2.8 V) is
possible in principle but needs the PMIC LDO voltage register map, a multimeter check of the
rail and operator approval — OTP programming is irreversible and the two sources disagree on
the voltage.  Until then the module identity is **connector position + fingerprint** (below).

Programming sequence per the datasheet, for when the rail can be set: `0x3004` bit 4 OTP clock
on (init table sets `0xFF`), write the data bytes to `0x3D05..`, set `0x3D20 = 0x01`
(program enable, fast), poll `0x3D20` bit 7 (busy) until clear, power-cycle, read back with
`0x3D21 = 0x01` (load) and compare.

## How to identify a module without OTP

1. Connector: bus 1 = cam0 (`CAM1` on the board), bus 2 = cam1 (`CAM2`).  Stable unless the
   modules are swapped.
2. Fingerprint: the manual AWB gains needed for a neutral grey on the same display differ
   between modules (table below), and so does the stock colour (SAI-CAM-0002 is purple at
   stock settings, with a RAW-domain G/B crosstalk that no ISP register corrects).
3. Procedure for a new module: `s265 locate.py` → `tune_colors.py --centre --stages expo,awb`
   on the Kamvas; the resulting R/B gains are the fingerprint; then the full stage list for
   the register set; add an entry here.

## Modules

| ID | UUID | sensor | board / connector | mount | AWB fingerprint (R / B gain, Kamvas grey, G = 1.000) | colour quality (Kamvas centre mode, ΔE00 all / greys) | notes |
|---|---|---|---|---|---|---|---|
| SAI-CAM-0001 | 0aab8082-1be5-4cef-b565-d6c8bdbc3570 | OV5640, OTP empty | 58eb / cam0 (bus 1) | upright | 1.282 / 1.314 (AGC 2.8×, exposure 886 lines) | **4.5 / 3.8** (Philips grid ΔE76 8.4, ΔE00 5.7) | reference module; LENC flattened to ±4 % |
| SAI-CAM-0002 | ad0a0d03-f20c-4b4a-b3ef-346fd4189762 | OV5640, OTP empty | 58eb / cam1 (bus 2) | rotated 180° | 1.092 / 1.198 (AGC 3.4×, exposure 886 lines) | **7.8 / 4.4** (Philips grid ΔE76 17.4–19.6, ΔE00 10.7) | stock frame purple; RAW-domain G/B crosstalk (gamma-off diagnostic: a 3x3 matrix would fix it, rms 13.4 → 2.5, but the OV5640 gamma runs before the CMX); not correctable in-sensor |

Register sets: `regs/SAI-CAM-000N_kamvas_2026-11-01.json` (Kamvas 16 sRGB reference, centre
mode — the production set) and `regs/SAI-CAM-000N_philips_2026-11-01.json` (Philips 234E5
reference, grid mode + LENC).  Both include the AEC/AGC freeze (`0x3503 = 0x03`) and the
exposure/gain measured at tuning time; for production apply everything except
`0x3500–0x3503` / `0x350A–0x350B` unless the exposure is meant to be frozen.

## Schema of `modules.json`

```
{ "id": "SAI-CAM-0001", "uuid": "...", "sensor": "OV5640", "otp": {"programmed": false, "read": "all zero"},
  "placement": {"board": "58eb", "connector": "cam0", "bus": 1, "rotation_deg": 0},
  "fingerprint": {"awb_r": 1.282, "awb_g": 1.0, "awb_b": 1.314, "agc_x": 2.81, "exposure_lines": 886,
                  "reference": "Kamvas 16 sRGB, grey patches, centre mode", "date": "2026-11-01"},
  "register_sets": [{"file": "regs/....json", "reference": "...", "mode": "centre", "regs": 109, "date": "..."}],
  "quality": {"kamvas_centre_de00": 4.5, "kamvas_centre_de00_greys": 3.8, "philips_grid_de76": 8.4},
  "notes": "..." }
```

## History

- 2026-11-02 — registry created with the two modules on board 58eb (EXP-s265 calibration of
  2026-11-01); OTP programming deferred (AVDD constraint).
