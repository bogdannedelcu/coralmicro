---
name: project_camera_module_registry_w24_t22
description: "2026-11-02 operator decision: every OV5640 module gets a unique ID and the project keeps a database in the source tree (examples/sentai_runtime/camera_modules/ — IDs, register sets, characterisation); OTP programming blocked by AVDD 2.1 V requirement (board rail fixed 2.8 V)"
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

**Why OTP is not programmed yet:** OV5640 datasheet v2.03 §4.11 footnote: AVDD must be 2.1 V ± 5 %
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
