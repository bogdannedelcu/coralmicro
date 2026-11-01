# EXP-s262 — OCRAM input ring: PXP producer, USB consumer, one frame streamed (OP-S10-W24-T15 step 3) + the direct-tensor bug

> **fps keys in this README are the pre-build-1756 keys.**  Since 2026-11-01 (SDK patch 0007 rewritten, EXP-s264) the SXGA keys mean the measured frame rate: old key 36 (true 30 fps) = `camera.init(1, 30)`, old key 30 (22.6 fps) = `init(1, 22)`, old key 18 (true 15 fps) = `init(1, 15)`; the old 11.2 fps key 15 is gone.

**Board** 58eb, **builds** 1749–1752, **date** 2026-11-01, one camera, SXGA option B
(packed YUV420 capture, s259/s260), models from s258.  Operator request: "încearcă
inelul OCRAM umplut cu cel puțin 3×256 KB buffere pregătite pentru transferul pe USB".

## Design

* The 921 KB OCRAM `.tpu_input` buffer (idle at SXGA, where the staging pair is in
  SDRAM) becomes a ring of **3 slots × 245 760 B** (128 packed rows × 640 × 3).
  5 slots = exactly the 1 228 800-byte Y tensor, so the Y/UV boundary is a slot
  boundary; UV = 2.5 slots.
* **Producer** = PrepTask: grabs the frame, publishes it to InferTask immediately,
  then runs the PXP byte-drop slice by slice (128 rows) into the next free slot
  (`free` counting semaphore, initial 3), signalling `filled` per slice.
* **Consumer** = the Edge TPU driver's `SendInputs`: the executable keeps passing the
  logical (SDRAM staging) pointer; the driver recognises `[stream_base,
  stream_base + total)`, maps the stream offset to the ring slot, waits for the slice,
  sends it in URBs that never cross a slot boundary, releases the slot when its last
  byte is on the wire and gives `frame_done` after the last byte of the stream.
  `begin_frame` takes `frame_done` first → exactly one frame in flight (the
  pipeline's two-frame cap would otherwise reset the ring under a running transfer —
  that was the first failure mode, build 1751: prod/cons timeouts → desync →
  recovery).
* The model's input DMA hints (TPU trace): Y in three overlapping tiles
  (`off 0/416000`, `404480/421120`, `814080/414720`), then UV (`0/614400`).  Tiles
  overlap by 9 rows (stride-2 halo); a slot is only released when a URB ends exactly
  at its end, so re-reads inside a slot are safe.
* Everything is behind `sentai.pipeline.ocram_ring(1)` (default off);
  `sentai.pipeline.ring_stats()` = (frames, urbs, consumer_wait_us, consumer_waits,
  consumer_timeouts, producer_timeouts).

## Results (`run_option_b.py --phases sxga_b,sxga_b_ring`, 15 s, build 1751/1752)

| SXGA row | path | pipeline fps | invoke ms | cycle ms | PXP (+waits) ms | fail | ring stats |
|---|---|---|---|---|---|---|---|
| 22.6 fps (key 30) | option B, SDRAM staging | 13.0 | 74.2 | 68.3 | 33.2 | 0 | — |
| 22.6 fps | **option B + OCRAM ring** | **15.6** | **60.7** | 48.2 | 42.9 | 0 | 398 frames, 21 890 URBs, consumer waited 1.1 ms total, 0 timeouts |
| 30 fps (key 36) | option B + OCRAM ring | 15.3 | 63.1 | 50.3 | 49.2 | 0 | 390 frames, 0 timeouts |

* **+20 % fps, invoke 74 → 61 ms** — the EHCI now reads OCRAM only (no SDRAM
  arbitration against the CSI), and the USB transfer of a frame overlaps the PXP of
  the same frame.  61 ms ≈ the isolated invoke (65 ms in s258): the pipeline is at
  the Edge TPU's 35 MB/s intake, 1.84 MB + 0.3 MB instructions per invoke.
* "PXP" in ring mode includes the producer's waits for free slots (paced by USB).
* URB size 36 KB vs 120 KB: invoke 60.8 vs 59.7 ms, fps identical (`ring_check.py`);
  36 KB kept.
* Output identity (`ring_check.py`, colour bars): staging and ring produce
  byte-identical TPU outputs (fingerprints 3587543 / 494665 / 756860 on both);
  the gradient pattern differs by ±1–3 per value between runs (frame-to-frame ISP
  drift, as in s260).
* SDRAM traffic per SXGA frame: CSI 2.46 + PXP read 2.46 MB = **4.9 MB** (legacy
  13.5 MB, option B 8.6 MB); the EHCI's 1.84 MB and the PXP's 1.84 MB write moved to
  OCRAM.

## The bug found on the way: the direct-tensor path never fed the TPU

`input_path_check2.py` (VGA45, c2f YUV420 dual-input model, `pipeline.direct_tensor`
default 1): the TPU output fingerprint was **identical** for colour bars, the natural
scene and the gradient pattern, and equal to a bare `tpu.invoke()` on the untouched
arena; with `direct_tensor(0)` (legacy staging→arena memcpy) it changed per scene.
Cause: `sentai_tpu_invoke_with_input(s2)` patched `MicroInterpreter::input_tensor(k)->
data`, but TFLM copies the eval tensor's pointer into that persistent `TfLiteTensor`
at allocation (`micro_allocator.cc`), and `EdgeTpuExecutable::Invoke` reads the
EvalTensor → the arena address.  Fix (build 1751): the invoke wrappers publish the
caller's buffers in `g_sentai_tpu_input_override[k]` and `Invoke()` sends from them
(and applies the int8 transform to them).  After the fix the direct-mode output varies
with the scene (bars 1268663, natural 1270885, gradient 1279448 vs arena 1270111).
All direct-mode speed numbers of s257/s258/s260 stand (the transfer is the same size);
any detection quality observed on that path before 2026-11-01 was on stale data.
The RGB single-input production path uses the same wrapper and is covered by the fix.

## Lessons taken from the CH32H417 project (`~/work/ch569w`, operator request)

* The Edge TPU **compute/USB clocks**: coralmicro's M3 bring-up defaults are not
  kMax; SentAI already opens the device at `PerformanceMode::kMax` (GCB 500 / AXI 250
  / 8051 500 MHz, `g_tpu_mode`), so that lever is in place.
* Instructions are re-sent every inference by design (not cacheable through the
  executable's hint loop) — consistent with the June wedge when forcing it.
* Drain the completion event every inference (done), never zero-write a launched
  transfer control (EHCI equivalent: static URB slots), thermal: a warm Coral makes
  the M3 run-state poll stall — worth remembering for the wedge forensics.

## Files

`run_option_b.py` phases `sxga_b_ring`, `sxga36_b_ring` (in s260), `ring_check.py`
(output identity + chunk A/B), `input_path_check.py`/`input_path_check2.py` (the
direct-tensor bug), `hint_order.log` (TPU trace of the input hints),
`run_b1749..1752_ring.log`, `results_b17*_ring.txt`.  Firmware: `libs/tpu/
edgetpu_driver.cc` (ring + `SendInputs` consumer), `detection_task.cc` (producer,
`sentai_pipeline_ocram_ring_set`), `libs/tpu/edgetpu_executable.cc` + `sentai_runtime.cc`
(input override fix).
