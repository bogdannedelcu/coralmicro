# ARM memory budget — i.MX RT1176 M7, SentAI firmware (OP-S10-W15)

**Date:** 2026-10-31.  **Tool:** `python3 scripts/mem_budget.py [--top N]` (region
usage from the ELF; exit 1 when ITCM/DTCM/OCRAM free space drops under the
guard, default 4 KB).  Run it after every build that adds a static buffer or a
new hot function; run it with `--top 20` when a region is tight.

## Regions (linker: `MIMXRT1176xxxxx_cm7_ram_mp.ld`)

| region | address | size | what lives there | why |
|---|---|--:|---|---|
| ITCM `m_text` | 0x0000_0800 | 254 KB | vector table, `.ramfunc` (CSI ISR, hooks, deferred flip), **all `.text` not routed elsewhere** | single-cycle instruction fetch; ISR latency |
| DTCM `m_ncache` | 0x2000_0000 | 32 KB | USB EHCI QH/QTD/frame list, debug console state, SD bounce/sector | non-cacheable, M7-private; EHCI descriptors here were the TPU-throughput fix |
| DTCM `m_data` | 0x2000_8000 | 224 KB | `.data`, `.bss`, main stack, crash-text slot | fastest data memory; lwIP pbuf pool, NCM state, TPU bulk staging live here |
| OCRAM `m_ocram` | 0x2024_0000 | 1008 KB | `.usb_host` (EHCI host code), `.tpu_input` (900 KB TPU tensor), `.ocram_bss` (ArUco fill stack, rolling sums) | DMA-reachable without touching SEMC: the TPU tensor here is what took the pipeline from 1.8 to 41 FPS |
| OCRAM `rpmsg` | 0x2033_C000 | 16 KB | M4 shared memory | cross-core |
| SDRAM `m_heap` | 0x8000_0000 | 14 MB | FreeRTOS/newlib heap (task stacks, MicroPython GC 1 MB) | |
| SDRAM `m_sdram` | 0x80E0_0000 | 18 MB | M4 image, lwIP, TFLM, libjpeg, MicroPython, libm, audio, SD/FileX/LittleFS code, `.sentai_slow`, `.sdram_text`, logs, `.sdram_bss` (frame-recorder pools, camera prep, cold buffers) | everything that is not hot |
| SDRAM `m_ncamera` | 0x8200_0000 | 24 MB | camera DMA frame buffers (non-cacheable) | CSI writes here |

## Usage, build 1686 (before W15) → build 1688 (after)

| region | before used / free | after used / free | change |
|---|--:|--:|---|
| ITCM | 256 744 / **~2.3 KB** | 224 224 / **35.9 KB** (86 %) | CMSIS-NN kernels (21 KB) → `.cmsis_nn`, newlib float formatting (11.5 KB) → `.libc_cold`, both in SDRAM |
| DTCM `m_data` | 225 740 / **~0** (linker overflowed twice that day) | 195 296 / **34.1 KB** (85 %) | `s_decim` 19.2 KB (GIST scratch), `s_play_paths` 8 KB (camera replay), `s_scalar_linebuf` 3 KB (FR drain) → `.sdram_bss` |
| DTCM `m_ncache` | 20 836 / 11.9 KB | same | — |
| OCRAM | 1 030 668 / **~1.5 KB** | 953 868 / **78.3 KB** (92 %) | `s_test_gray` 76.8 KB (ArUco synthetic bench frame) → `.sdram_bss` |
| SDRAM `m_sdram` | 90 % | 90.8 % | +0.13 MB |
| runtime heap | 14.57 MB free (`sentai.rtos.heap_info()`), MicroPython GC 1 MB with 1.6 KB used | | |

Nothing on the TPU/USB path moved: `s_bulk_staging` (32 KB DTCM), the OCRAM
tensor, the EHCI descriptors in `m_ncache`, the lwIP/NCM pools (37 + 33 KB
DTCM) are exactly where the 2026-04 and 2026-06 investigations left them.
Verification on 58eb booted from flash: s249 15/15 (camera, TPU detection,
HTTP, FS, IMU).  Synthetic-frame ArUco bench numbers (`s111`) now read the
pattern from SDRAM; production detection always worked on SDRAM camera frames,
so its timing is unchanged.

## Rules for new allocations (embeded.md §B/§J discipline)

1. **Default is SDRAM.**  A new static buffer goes to `.sdram_bss`
   (`__attribute__((section(".sdram_bss")))`), new cold code to `.sdram_text`
   (`__attribute__((section(".sdram_text"), noinline))`).  ≥ 1 KB of DTCM or
   OCRAM, or any ITCM code, needs a one-line justification in the declaration
   comment and a `mem_budget.py` run in the commit message.
2. **DTCM** only for: ISR-touched state, per-frame hot scratch (< 4 KB), stacks
   of tasks with hard latency, the lwIP/NCM pools and the TPU bulk staging
   (history: do not move).  Never a DMA target except the `.ncache` window.
3. **OCRAM** only for DMA-heavy buffers that must avoid SEMC contention (TPU
   tensor/ring, USB host code) and per-frame hot buffers ≥ 4 KB (ArUco fill
   stack).  Keep ≥ 32 KB free for the instruction-caching option
   ([[reference_edgetpu_instruction_caching_256kb]] needs ~372 KB — a separate
   decision).
4. **ITCM** only for `.ramfunc` ISR code and verified hot loops (flow, ArUco
   threshold, PXP glue).  Whole-library code is routed by archive in the linker
   script (`.lwip`, `.tensorflow`, `.micropython`, `.libm`, `.cmsis_nn`,
   `.libc_cold`, `.sentai_slow`, …); add a new library there, not to `.text`.
5. **Guard:** keep ≥ 16 KB free in ITCM and DTCM and ≥ 32 KB in OCRAM; the
   budget tool fails below 4 KB.  When a region is tight, the candidates are
   listed by `mem_budget.py --top 20`; move cold data first, code second,
   never the items in rule 2/3.
6. **Stacks:** task stacks come from the SDRAM heap except the static ones that
   must survive heap trouble (`sd_init` 8 KB SDRAM static, `health_led` 2 KB,
   `cam_tune` 4 KB).  Watch `sentai.rtos.tasks()` high-water marks; `IDLE` 53
   words free is normal.

## Remaining headroom options (not done)

- lwIP `PBUF_POOL` (37 KB) and `g_cdc_ncm` (33 KB) could move DTCM → OCRAM now
  that OCRAM has 78 KB free; they are on the 5.2 MB/s NCM path — needs an A/B
  with `s250`-style throughput runs before and after.
- `.text` still holds the USB device stack, TPU driver init, boot clock setup
  and OV5640 init (~15 KB of cold code) — candidates for `.sdram_text` if ITCM
  gets tight again.
