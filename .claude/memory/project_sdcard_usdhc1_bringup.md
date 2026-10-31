---
name: sdcard-usdhc1-bringup
description: "microSD on the new board rev is on USDHC1 with GPIO-controlled supply (SD_PWREN_B) and pad rail (SD1_VSELECT); Google's libs/sdcard assumed USDHC2 + wrong rails. Bring-up PASSED 2026-10-30 build 1605 — 64 GB exFAT card read via sentai.sdcard."
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-10-30T19:06:57.118Z
---

**Hardware (new schematic by the electronics colleague, sheet 16 "Interface"
+ sheet 6 "RT1176 PART2", dated 10/30/2026):**

- microSD socket is on **USDHC1**, pads `GPIO_SD_B1_00..05` (CMD, CLK, D0..D3).
  `GPIO_SD_B2_*` (USDHC2) are SPARE / FlexSPI boot — NOT the card.
- **`SD_PWREN_B` = `GPIO_AD_12`** (GPIO_MUX3_IO11 → `GPIO3` pin 11): gate of
  P-FET U12 that switches VDD_3V3 → VSD_3V3 (card supply). **Active LOW**
  (0 = card powered).  C78 soft-start on the gate → wait ~30 ms after enable.
- **`SD1_VSELECT` = `GPIO_AD_16`** (GPIO_MUX3_IO15 → `GPIO3` pin 15): select of
  NX3L1T3157 analog switch U6 feeding `NVCC_SD1` (pad I/O rail).
  **0 = VDD_3V3, 1 = VDD_1V8.**  Card is only ever supplied 3.3 V, so keep
  VSELECT=0 and **never enable UHS-I 1.8 V switching** (`ioVoltage = nullptr`,
  `maxFreq = 50 MHz`).
- No card-detect switch wired → `cardDetected` always true; `SD_Init()` fails
  cleanly on empty socket.
- External 10k pull-ups only on CMD and D0 (R373/R374); D1..D3 use internal
  pad pull-ups (`PULL(1)`).
- Neither GPIO3 pin 11 nor 15 is used elsewhere in `libs/base/gpio.cc` (checked).

**Why:** the colleague warned that Google's `libs/sdcard/sdcard.cc` "did not set
the SD voltages correctly" — it targeted USDHC2, drove the Wi-Fi power GPIOs
(GPIO13_12 / WL_REG_ON) as "card power", and requested host-controlled 1.8 V
switching at 200 MHz.  Operator asked to follow the schematic exactly.

**What was done (build 1605, 2026-10-30, RAM-loaded, not committed yet):**
- `libs/sdcard/sdcard.cc` rewritten to USDHC1 + the two GPIOs above, power
  cycle (off 30 ms → on 30 ms) before `SD_Init`, 3.3 V-only.
- `examples/sentai_runtime/sentai_sdcard_glue.cc` (C shim, NonCacheable 512 B
  sector buffer) + `bindings/modsentai_sdcard.c` → `sentai.sdcard.{init,
  deinit, info, probe, hexdump}` — read-only, scalars/strings only.
- Linker: new `.sdcard` section (`liblibs_sdcard_freertos`, `sdmmc`,
  `middleware_sdmmc_sd`, `fatfs`, glue) → **SDRAM**.  Without it m_text
  (ITCM) overflowed.  Adds 17.4 KB SDRAM; ITCM `.text` = 240.8 KB / 259 KB.
- QSTR regen done (`sdcard, probe, hexdump, init, deinit` were new).

**Card found in the socket:** `SD64G`, 124 735 488 blocks × 512 B = 60 906 MB
(SDXC).  MBR, partition 0 type 0x07, start LBA 32768, 124 702 720 sectors,
**exFAT**, 512 B/sector, 256 sectors/cluster (128 KB clusters).
`SD_Init` rc=0, USDHC1 root clock 198 MHz.

**How to apply:**
- Treat `libs/sdcard` as board-specific now; do not "fix" it back to USDHC2.
- FatFs config (`third_party/modified/nxp/rt1176-sdk/fatfs/ffconf.h`, R0.14
  rev 86631) already has `FF_FS_EXFAT=1`, `FF_USE_LFN=2`, `FF_VOLUMES=1`,
  `FF_MULTI_PARTITION=0` (auto-selects first MBR partition) → the exFAT card
  can be mounted as-is, no reformat needed.
- SIM has no `sentai.sdcard` (ARM-only, intentionally; `sim/modsentai_sim.c`
  does not include the binding).
- Related: [[new-board-rev-2026-10-open-items]], [[itcm-budget]],
  [[no-heavy-data-through-mp]].
