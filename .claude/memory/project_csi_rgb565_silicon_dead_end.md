---
name: CSI RGB565 SDRAM 2 BPP — silicon dead-end (ERR051248)
description: 2026-04-25 forum-confirmed: i.MX RT1170/RT1176 MIPI CSI-2 RX → CSR pixel-link is hard-wired 24-bit. RGB565 always stored as XRGB8888. No software fix.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
CSI camera→SDRAM 2 BPP path is permanently blocked on RT1176 silicon.

**Why:** errata **ERR051248** (i.MX RT1170 errata §2.22) — the VIDEO_MUX/CSR pixel-link from MIPI CSI-2 RX to the CSI parallel block is hard-wired 24-bit with no programmable width register. NXP engineers confirm in multiple forum threads that "if the input data format is RGB565, the MIPI_CSI converts it to RGB888 internally and sends to CSI" — i.e. capture is ALWAYS 32 bpp regardless of what dataType the sensor sends.

`CFG_DISABLE_PAYLOAD_RGB888` (bit 20 of `MIPI_CSI2RX->CFG_DISABLE_PAYLOAD_0`) only tells the RX to drop incoming RGB888 packets at the protocol layer; it does NOT alter the CSR pixel-link width or stop the MIPI→24-bit upconvert. UYVY YUV422 has the same trap — stored as XYUV8888 (4 BPP).

The empirical "4-quadrant garbage" we saw with BPP=2 is the exact symptom: CSR still emits 24-bit pixels but CSI tries to read 16-bit, addressing skews systematically.

**How to apply:** 
- Do NOT spend more time searching for a CSR width register, MIPI dataType filter trick, or fsl_csi.c patch — all dead-ends, multiple NXP engineers confirm.
- The only paths to halve camera→SDRAM traffic on this SoC are: (a) lower resolution (already at VGA), (b) parallel-CSI/DVP port instead of MIPI (Coral Dev Board Micro doesn't expose this — sensor is wired to MIPI), (c) sensor-side binning + smaller frames.
- For SDRAM contention reduction, prefer arena/.tpu_input OCRAM placement (already shipped V22) over fighting the camera path.

**Sources:**
- https://community.nxp.com/t5/i-MX-RT-Crossover-MCUs/RT1166-RT1170-MIPI-CSI-data-convert-between-RGB888-RGB565/m-p/2135738
- https://community.nxp.com/t5/i-MX-RT-Crossover-MCUs/iMXRT1176-MIPI-CSI2-to-UVC-reducing-the-frame-buffer-s-size/m-p/1352717
- https://community.nxp.com/t5/i-MX-RT-Crossover-MCUs/RT1170-Using-MIPI-CSI-with-grayscale-or-raw-image-sensors/m-p/1270310
- AN13573 i.MX 8/RT MIPI DSI/CSI-2 Application Note (Aug 2024)
