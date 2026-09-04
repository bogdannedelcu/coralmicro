---
name: fr-scalars-arm-fix-and-drain-wedge
description: sentai.fr scalars/events were broken on ARM (fopen fails) — fixed to sentai_fs_append + batched drain; but per-frame fr drain during live detection wedges USB (platform limit)
metadata:
  type: project
  originSessionId: 218d6445-6c73-4c0f-bb51-4d13943aaeb1
---

**Two facts about `sentai.fr` on ARM (found + fixed build 1585-1586):**

**1. fopen write-glue is NOT wired to FileX on ARM.** `sentai.fr.open("scalars"/"events", path)` used raw `fopen(path,"ab")` which returns IO error (-5) on ARM — newlib stdio→FileX *write* is absent (reads via fopen work: aruco/calib model reads; but "ab"/"wb" fail for ALL path forms /x, x, ./x, /data/x). Only the DEBUG channel had been ported (uses `sentai_fs_append`). Fix: `sentai_fr.cc` open + drain for scalars/events now `#ifdef __arm__` → `sentai_fs_size`+`sentai_fs_append` (→ `LfsUserAppendFile` FileX compat shim, returns 1 ok / 0 fail). Header written on open when size<=0. SIM keeps fopen. This was a latent bug: fr file logging (calib peaks, mission events) was silently non-functional on ARM.

**2. Per-row NAND append drain WEDGES USB; batch it.** First ARM port did one `sentai_fs_append` per row in the drain task. A burst of ~60 scalar rows (each = FileX open+seek-to-end+write+close = a NAND program cycle) starved the USB host ISR → CDC-ACM dropped (transient re-enum, sometimes FileX SAFE_MODE). Fix: `drain_scalars_batch_()` — format all pending rows that fit into ONE static buffer under a single lock, issue ONE bounded (<=~3 KB) append per 20 ms drain round. Validated: isolated 60-scalar burst → wr_ok=60, drops=0, no wedge (was a hard wedge per-row). Events kept per-row (low-rate, 640 B text).

**3. HARD LIMIT — do NOT drain fr to NAND during live detection.** Even batched, draining ~600 scalar rows *after* (or *during*) a 15-20 s @45 fps detection run wedges USB → re-enum → SAFE_MODE. Root cause is [[tpu-usb-wedge-mechanism]]: sustained TPU-USB traffic makes USB fragile; NAND program cycles (which briefly hold buses / affect IRQ latency) tip it over. The fr *push* (ring enqueue) is free and safe in the hot path (no-op when channel closed); it's the NAND *drain* that's incompatible with concurrent detection. **For pipeline timing use `sentai.pipeline.infer_stats` (ms_sum/ok/last_rc, in-RAM, zero I/O) + `frame_count` — NOT fr-to-file.** fr-to-file is fine for low-rate logging on an idle/non-detection board.
