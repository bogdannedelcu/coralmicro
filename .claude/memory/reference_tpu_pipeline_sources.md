---
name: TPU pipeline — external reference sources for RT1176/EdgeTPU optimization
description: Primary sources and key findings from research on decongesting Coral Dev Board Micro CSI/PXP/USB on SEMC/AXBS fabric. Consult these before proposing new bus-level optimizations.
type: reference
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
External documents and their load-bearing findings — use as ground
truth when reasoning about RT1176 bus architecture or EdgeTPU tuning.

## NXP AN12437 — "i.MX RT Series Performance Optimization"

**URL:** https://www.nxp.com/docs/en/application-note/AN12437.pdf
**Mirror:** https://community.nxp.com/pwmxy87654/attachments/pwmxy87654/imxrt/10784/1/AN12437.pdf

**Key findings:**
- **Master ID table (Table 5)** — definitive answer why AxQOS fails on RT1176:
  - `000b`: Core platform
  - `001b`: eDMA
  - `010b`: DCP
  - `011b`: **ALL OTHERS — PXP, USB, CSI, LCD, etc. share this ID**
- CSI, PXP, USB are arbitrated as a SINGLE master by AXBS.  Per-master AxQOS
  cannot prioritize between them — they're architecturally indistinguishable.
- Only eDMA (001b) and DCP (010b) have independent arbitration slots —
  route traffic through these to get a distinct priority lane.
- SEMC fabric is SIM_M7 (64-bit @ 132 MHz); FlexSPI is on SIM_EMS (separate).

## NXP IMXRT1170RM — Reference Manual

**URL:** https://mm.digikey.com/Volume0/opasdata/d220001/medias/docus/6465/IMXRT1170RM%20manual%20REV3.pdf

**Key findings:**
- SEMC supports 4 CS × 4 banks × 8 row-pages open; bank-interleaving
  lets concurrent accesses to different banks overlap (one precharges
  while another transfers).
- CSI CR3: `RxFF_LEVEL` (FIFO fill threshold), `STATFF_LEVEL` — tunable
  for CSI DMA burst behavior, but no hardware decimation before DMA write.
- USBHS `SBUSCFG.AHBBRST` field exists but is write-during-reset only;
  post-init writes cause hard-faults (tried 3 and 7, both crashed).

## NXP AN12077 — "Using the i.MX RT FlexRAM"

**URL:** https://www.nxp.com/docs/en/application-note/AN12077.pdf

**Key findings:**
- FlexRAM 512 KB splittable ITCM/DTCM/OCRAM via `IOMUXC_GPR_GPR17` +
  `GPR16.FLEXRAM_BANK_CFG_SEL=1`.  32 KB bank granularity.
- Current config: 256 KB ITCM + 256 KB DTCM + 0 KB FlexRAM-OCRAM
  (dedicated OCRAM1+OCRAM2 = 1 MB provides the OCRAM).
- Max ITCM 512 KB if we steal all DTCM; realistic 384/128 already hits
  DTCM usage ceiling.

## Google libedgetpu — `tflite/public/edgetpu.h`

**URL:** https://github.com/google-coral/libedgetpu/blob/master/tflite/public/edgetpu.h

**Key findings:**
- `Usb.MaxBulkInQueueLength` option range 0..255, default **32** —
  libedgetpu on desktop pipelines up to 32 URBs IN direction.
- Our MCU uses 2 URB-slot async pipeline in `BulkOutTransferPipelined`
  (much shallower).  URB depth 2→3 or 4 is a plausible micro-optimization
  within existing QH/QTD=8 pool.

## Coral Dev Board Micro datasheet / hardware facts

**URL:** https://www.cnx-software.com/2023/02/04/coral-dev-board-micro-nxp-i-mx-rt1176-mcu-edge-tpu-raspberry-pi-zero/

**Key findings:**
- Board = RT1176 (M7 @ 1 GHz + M4 @ 400 MHz) + Coral Accelerator
  Module (TPU) over internal USB 2.0 HS.
- No external HyperRAM/PSRAM on FlexSPI (FlexSPI used for NOR flash only).
- SDRAM is 64 MB @ 166 MHz on SEMC.
- Coral Accelerator Module datasheet specifies USB 3.1/PCIe on the
  MODULE side, but the Coral Dev Board Micro only connects via USB 2.0
  HS (480 Mbps wire limit, ~40 MB/s effective) — this is the ceiling
  no amount of software tuning can move.

## How to apply

When user proposes a new bus/arbitration optimization, check first:
1. Does the lever depend on separating CSI/PXP/USB at AXBS level?
   → Won't work, they share master ID 011b.  Redirect to SDRAM bank
   interleaving (physical parallelism) or eDMA/DCP offload.
2. Does it assume USB 3.0 / higher wire speed is unlockable?
   → No, hardware limit 480 Mbps.
3. Does it propose SBUSCFG post-init tuning?
   → Already tried, hard-faults.  Only testable via NXP EHCI init patch.
4. Does it propose more URBs in flight?
   → Depth 2→3 is safe within pool of 8; depth 8→16 crashed pipeline.

Keep consulting these sources before burning effort on dead-ends.
