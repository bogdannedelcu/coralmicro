---
name: project_sxga_tpu_sdram_contention_w24_t11_t12
description: "2026-10-31 — Edge TPU at 1280x960 YUV420 = 13 fps (build 1722); bulk OUT is a flat 35 MB/s = the TPU's USB2 intake; SDRAM contention root cause = EHCI TX FIFO underrun, fixed by SBUSCFG INCR16 + TXFIFOTHRES 16 (+ SEMC BMCR1 weights); ReLU not SiLU on Edge TPU; virtual camera out of board build"
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-11-01T08:37:39.074Z
---

**EXP-s258 / OP-S10-W24-T11 + T12 (builds 1711–1722, board 58eb).**

- **Edge TPU bulk OUT = 35 MB/s flat** on this USB2 host for every tag, chunk size
  (36..128 KB), memory source (OCRAM/SDRAM arena) and camera state; EHCI async
  park mode is already on (USBCMD 0xB31; turning it off → 81 ms, ITC=1 → 85 ms).
  Same ~33 MB/s on the Coral Dev Board Mini (ch569w exp024) → it is the TPU's
  USB2 intake, not the RT1176.  An invoke is ~99 % transfer; compute is hidden.
  SXGA YUV420 model: 297 KB instr + 1.87 MB input → 65 ms isolated → 15.4 fps cap.
  Instruction re-send per invoke (13–17 % of the time) is the biggest software
  lever left (TPU-side instruction caching wedged in 2026-06; retry post-W24 fixes).
- **SDRAM contention mechanism (solved):** EHCI TX FIFO underrun = qTD "Data
  Buffer Error" when CSI (147 MB/s at SXGA30) + PXP + EHCI share the 16-bit SEMC.
  SDK defaults SBUSCFG AHBBRST=INCR8, TXFILLTUNING TXFIFOTHRES=10.  **Fix:
  INCR16 + TXFIFOTHRES=16** (usb_host_task.cc after USB_HostInit): SXGA30 from
  total collapse (32 buffer errors/12 s) to 13.0 fps / 0 errors; VGA45 unchanged
  (42.5 fps).  Threshold 8 worse, 24/31 no gain, fixed BURSTSIZE w/o AHBBRST 3×
  slower.  SEMC BMCR1 WQOS/WAGE=F + WPH=FF: −5 ms/invoke (board_hardware.c);
  WBR/WRWS max made it worse.  USB_OTG2 base is **0x4042C000** (not 0x40438000 =
  USBPHY2).  SDK patch 0006 exposes qTD error bits: `tpu.urb_sync_stats()['ehci']`.
- **Ceilings at 1280x960:** USB 16–18 fps; SDRAM (64 MB, 16-bit, 166.7 MHz,
  ~200 MB/s practical; 4.9 MB XYUV CSI write + 4.9 MB PXP read per frame) ~20 fps;
  today 13.0 fps at camera 30 fps, 11.2 at camera 15 fps (camera-phase bound).
  PXP repack SXGA: 42 ms (cam 15) / 62 ms (cam 30).  Only >15 fps path: PXP
  downscale to the VGA model (SDRAM-bound 15–18).
- **Edge TPU compiler: SiLU is 4.5× slower than ReLU** at these shapes (replica
  87 vs 19 ms); the trained exports use ReLU.  Keras 3 UpSampling2D → TILE v3 which
  compiler 16.0 cannot parse → use fixed-size tf.image.resize nearest.
- Plumbing shipped: `camera.set_hw(w,h)` + runtime `g_runtime_cam_w/h`
  (camera_support.c), YUV dual path uses runtime geometry + SDRAM staging pair in
  NonCacheableCamera, m_ncamera 32 MB, PXP deadline 200 ms at SXGA, two frames in
  flight (semaphores sized to cap — old start() re-primed ONE token), virtual
  camera stubbed on the board build (`SENTAI_VIRTUAL_CAMERA=OFF`),
  `pipeline.trace()` GPT1 timeline ring, `pipeline.infer_stats()` bufs_cap/first_fail.
- **CORRECTION 2026-11-01 (s260):** the "SXGA 30 fps" row (fps key 30) really runs at
  **22.6 fps** (CSI CR3.FRMCNT; cloned from 720P/30, 984 lines) and "15" at ~11.3;
  `sensor_frames()` over-counts ~15 %.  All s258 "camera 30" numbers are at 22.6.
  True 30 fps = fps key 36 (PLL 0x70, SDK patch 0007); legacy XYUV hangs the board
  there (147 MB/s CSI), option B (packed YUV420, s260) runs.
- Lesson: profile before theorising — the PXP-yield and double-buffer hypotheses
  were both wrong; the trace + per-tag URB stats + EHCI error bits found the
  truth in three runs.
