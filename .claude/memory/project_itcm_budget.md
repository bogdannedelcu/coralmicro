---
name: ITCM/m_text budget (Cortex-M7, 252 KB) — what's already SDRAM, what's left
description: m_text=ITCM is the bottleneck; lwIP/jpeg/MP/TFLM/etc. ALREADY in SDRAM from 2026-04-22. s113 P2.5 moved another 22 KB (LittleFS/IMU/TPU DFU/USB MSC) freeing margin from ~10 KB to ~32 KB.
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
**ITCM = m_text on Cortex-M7 RT1176**: hard-wired 256 KB (m_text spans
0x00000C00..0x00040000 = 252 KB).  Single-cycle from CPU; no way to
extend.  Vector table claims the first 3 KB.

**Already in SDRAM** (don't try to "move them" — done in 2026-04-22):
- `.lwip` 56 KB · `.libjpeg` 102 KB · `.tensorflow` 55 KB
- `.micropython` 219 KB · `.cmsis_dsp` 3.6 KB · `.libm`
- `.audio` 4.9 KB · `.shine` 17 KB · `.aifes` 24 KB
- `.sentai_slow` 66 KB (tracker/mesh/link/httpd/tfl_bridge)
- `.camera` 5.9 KB
- ~564 KB code+rodata already relocated

**Moved 2026-05-12 (s113 P2.5)** — adding to the list above:
- `.littlefs` 15.4 KB
- `.tpu_dfu` 12.9 KB (one-shot at boot, dormant after)
- `.imu` (lis2du12) 2.9 KB (REPL on-demand)
- `.usb_msc` 1.5 KB (only in storage mode)

**Current ITCM state (build #1298)**: `.text = 224 544 B`, free
margin ≈ **32 KB**.  Verified board boots, camera+I2C+MIPI CSI all
work post-relocation.

**Available for future moves if margin gets tight again** (~14 KB
total, lower-confidence):
- USB device enum: `usb_device_ch9.c` (1.85 KB), `usb_device_msc_ufi.c`
- Boot config: `clock_config.c` (1.6 KB), `fsl_clock.c` (2.9 KB)
- `libg_nano.a` 22 KB (newlib) — risky, pulled by everything

**Cannot move (HOT path — would regress 42 fps TPU pipeline)**:
- `tasks.c` / `queue.c` / `stream_buffer.c` (FreeRTOS scheduler)
- `fsl_edma.c`, `fsl_csi.c`, `fsl_lpi2c.c`, `fsl_semc.c`, `fsl_pxp.h`
- `liblibs_tpu_freertos.a` (15 KB) — invoke hot path
- `liblibs_usb_host_edgetpu_freertos.a` + EHCI (already partially in
  `.usb_host` OCRAM)
- `sentai_runtime.cc` (mixed — capture_jpeg/capture_rgb/PXP all hot)
- `flow_task.cc`, `flow_phase_corr.cc`, `detection_task.cc`

**Anti-pattern when adding new ARM-side code**: do NOT default to
m_text.  ALWAYS prefix `__attribute__((section(".sdram_text"),
noinline))` and put data in `.sdram_bss`.  Avoid `printf("...")`
string literals in standalone .o files (they land in m_text/.rodata
by default).  Use `sentai_dmesg` (already SDRAM) for runtime logs.
ITCM is for proven hot paths only — earn the placement.

Reference: `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`
sections `.littlefs / .imu / .usb_msc / .tpu_dfu` (newly added)
and the older `.lwip / .libjpeg / .tensorflow / .micropython /
.cmsis_dsp / .libm / .audio / .shine / .aifes / .sentai_slow /
.camera`.
