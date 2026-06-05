---
name: iarna p3p4 on-board model bench
description: Build #1077 on-board comparative bench of 6 candidate models — p2p4 broken, 5 others work and switch live cleanly
type: project
originSessionId: f158eff9-f718-4f1f-82fc-48d9e0e77747
---
2026-04-28 build #1077 on-board EdgeTPU bench results (no camera, no
pipeline, pure tpu.invoke loop).  See
`examples/sentai_runtime/agent/experiment.md` "Session 2026-04-28 —
on-board comparative bench" for full table.

**Working models (live-switch in one boot, zero fails):**
- iarna p3p4 MSBlock — 11 ms / 90 FPS
- iarna p3p4 C2f      — 11 ms / 90 FPS
- iarna p3p4 GELAN    — 11 ms / 90 FPS
- yolo_1 inloc_P5 (CANON) — 12 ms / 83 FPS
- yolo_1_1up alt          — 14 ms / 71 FPS

The 3 iarna p3p4 candidates are statistically identical on-board
(host pycoral showed MSBlock advantage; M7 single_ep doesn't see it
because per-invoke is USB-bandwidth-bound on the 921 KB input tensor,
not compute or instruction stream).

**iarna p2p4_5ep_export_640x480 is BROKEN on Coral USB silicon.**
Signature: median 200 ms, 6/30 invokes rc<0, in_bytes_per_invoke=0
(input DMA never starts), libedgetpu prints
`E:0B62:0 Node edgetpu-custom-op (number 0) failed to invoke with status 1`.
SendInstructions fails before PrepareInputs.  Once touched, TPU is
wedged for the boot — no public reset path recovers it.  DROPPED from
candidate set.

**Why:** Iarna backbone selection.  All p3p4 variants supersede p2p4
on every metric.  v22 pipeline at 41.4 FPS used yolo_1 inloc_P5;
swapping to p3p4 should give ~60+ FPS in pipeline (90 pure-TPU − pipeline overhead).

**How to apply:** When proposing iarna model promotion, default to
p3p4 family.  Use MSBlock as primary candidate (smallest, fewest ops);
C2f and GELAN are interchangeable fallbacks.  Don't propose p2p4 —
it's a known wedger.  Drivers live at `examples/sentai_runtime/diag/_t_models_live.py`
(live-switch with WARM0_BAIL guard) and `_t_one_model_bench.py` (per-boot
isolated).  Helper `/tmp/pull_csv.py` reads CSV via REPL when HTTP is off.
