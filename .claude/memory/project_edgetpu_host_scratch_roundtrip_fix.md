---
name: project_edgetpu_host_scratch_roundtrip_fix
description: Wide EdgeTPU models (c2f_thick) crashed the board; root cause = host-scratch activation spill the coralmicro port ignored. Fixed 2026-06-26 (commit 8510b1b7).
metadata:
  type: project
---

EXP-s236 / TD-S10-A10. The cover_v1 revision champion `c2f_thick` (widest of
8 detectors) crashed the WHOLE board on every TPU invoke while the other 7 ran
~24 fps: USB CDC-ACM died, M7 watchdog-reset.

**Root cause** (found by offline-parsing the darwinn Executable flatbuffer —
`experiments/s236.../iter15_offline_exe/dump_exe.cc`, which prints per-executable
`dma_hints`): c2f_thick is the ONLY model with `scratch_size_bytes=153600`. It is
wide enough that its intermediate activations don't fit in the Apex on-chip SRAM,
so the edgetpu compiler split compute into two halves joined by a **host-mediated
activation spill**: hint sequence has `SCRATCH OUTFEED 153600` -> `FENCE` ->
`SCRATCH INFEED 153600`. The TPU spills its working set to host RAM, host parks
it, feeds it back. coralmicro's reduced libedgetpu port
(`libs/tpu/edgetpu_executable.cc::Invoke`) let `Description_BASE_ADDRESS_SCRATCH`
fall through `default:break` — the spill DMAs were ignored, so the TPU pushed
153600 bytes onto bulk-IN that nobody drained, the next `GetOutputs` read scratch
instead of the real output, the endpoints desynced and USB wedged. Shipped demo
models never spilled, so the omission was never hit. The full libedgetpu (the
Linux `libedgetpu.so` reference) handles this; the MCU port dropped it.

**Fix** (mirrors libedgetpu, parking the working set in M7 RAM):
- `edgetpu_driver`: `GetScratch` (OUTFEED drain, bulk-IN like GetOutputs) +
  `SendScratch` (INFEED refill, bulk-OUT on the `kInputActivations` tag — the USB
  protocol has NO dedicated scratch DescriptorTag).
- `edgetpu_executable`: `PrepareScratch` reserves the buffer via
  `RequestScratchBufferInArena` (TFLM arena, Prepare-only, auto-sized, no heap);
  new `BASE_ADDRESS_SCRATCH` case dispatches OUTFEED/INFEED against it using
  `hint->direction()`.
- `edgetpu_op` + `edgetpu_manager`: `CustomOpPrepare` ->
  `EdgeTpuManager::PrepareScratch`.

**Verified** board build 1560: trace shows scratch OUTFEED drain (153600 in) +
INFEED refill (153600 out); `invoke()` returns 34 (success), board stays alive,
msblock (scratch=0) unregressed. Across all 8 cover_v1 models, ONLY c2f_thick has
scratch>0 and a FENCE; none has exec_count>1.

**Residual libedgetpu divergences still in the MCU port** (verified none bite the
current 8 models): `FenceHint` + mid-sequence `InterruptHint` fall through
`default:break` (masked by synchronous-sequential transfer ordering);
`exec_count>1` output path untested. Next likely landmine for a future champion
model. See [[reference_edgetpu_instruction_caching_256kb]].
