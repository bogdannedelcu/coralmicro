---
name: project_camera_module_registry_w24_t22
description: "2026-11-02 operator decision: every OV5640 module gets a unique ID and the project keeps a database in the source tree (examples/sentai_runtime/camera_modules/ — IDs, register sets, characterisation); OTP programming analysed (AVDD 2.1 V requirement, rail fixed 2.8 V) and DECLINED by the operator: identity = connector + AWB fingerprint"
metadata:
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-02T04:57:09.858Z
---

**Operator decision (2026-11-02):** assign a unique ID to each camera module, write it into the
sensor OTP, and keep the list of assigned IDs + register sets + characterisation **in the source
tree as a permanent project database** (`examples/sentai_runtime/camera_modules/`: README.md,
modules.json, regs/).  WBS OP-S10-W24-T22.  IDs `SAI-CAM-NNNN` + UUID4, append-only.
SAI-CAM-0001 = board 58eb cam0 (upright, AWB R 1.282 / B 1.314, Kamvas ΔE00 4.5);
SAI-CAM-0002 = cam1 (rotated 180°, R 1.092 / B 1.198, ΔE00 7.8, RAW G/B crosstalk).

**Operator decision later the same day: the OTP stays unprogrammed (identity = connector + fingerprint).**  Background: OV5640 datasheet v2.03 §4.11 footnote: AVDD must be 2.1 V ± 5 %
while programming (Arducam app note says 2.5 V) — reading has no such requirement.  The board's
AVDD is the MCP16701 LDO2/LDO3 rail `VDD_2V8_CAM1/2` at 2.8 V; firmware has on/off only
(`sentai_cam_rails_set`).  MCP16701 LDOs are programmable 0.6–3.8 V (25 mV steps) → a
programming mode is feasible after confirming the PMIC LDO voltage register map and measuring the
rail; needs explicit operator approval (irreversible, sources disagree on the voltage).
OTP facts: 256 bits = 0x3D00–0x3D1F, 0x3D00–0x3D04 reserved by OmniVision (27 user bytes),
program: data → 0x3D20 = 0x01, poll bit 7; load/read: 0x3D21 = 0x01.  Planned layout
0x3D05.. = "SA" + UUID(16) + CRC-8.

**How to apply:** when a module is calibrated, moved or replaced, update modules.json + README
table and copy the register set into regs/; identify modules by connector + AWB fingerprint
until the OTP is written.

**Full register scan 2026-11-02 (`camera_modules/scan_regs.py`, 0x3000–0x60FF, 2 runs with a reset between):** NO factory identifier anywhere — every register that differs between the modules also changes between runs of the same module (AFC memory 0x6019–0x603F random per power-up, BLC black levels 0x402C–0x4033 / 0x40AC–0x40B3 drift, flicker detection 0x3C0C–0x3C18, undocumented 0x3051/52, 0x3A53=0x3AD3, 0x4617, 0x4763, 0x56A2). OTP read with reset released (0x3000[4]=0) + clock on + fast/slow: all zero. Weak analog fingerprint: SAI-CAM-0002 black level ≈ +40 LSB higher (1991–2009 vs 1953–1971), consistent with its purple stock frame. Do not re-investigate: identity = connector + AWB fingerprint.
- 2026-11-08: production sets switched to `philipsL_2026-11-08` (Philips landscape monitor rig), uploaded with `--keep-exposure` (frozen tuning exposure; auto exposure clips on this monitor). Switch the production flag back to the kamvas sets and re-run `make_regs.py --upload` if the Kamvas rig returns. AEC-target variant for auto exposure = W24-T23.
