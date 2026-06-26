# s236 iter16 — ROOT CAUSE + FIX: EdgeTPU host-scratch round-trip

Anchor: `TD-S10-A10/B10` (paper §4.2) · EXP-s236 · fixes the c2f_thick
board-crash found in iter03/iter04.

## Symptom (iter03/iter04)

7 of 8 cover_v1 detectors ran on the board at ~24 fps. The 8th —
`c2f_thick`, the widest architecture and the revision champion — **crashed
the entire board** the instant it was invoked: USB CDC-ACM died, the M7
watchdog-reset, build number reverted. Every other model was fine. Earlier
iters ruled out (empirically): multi-pass structure (2 other multi-pass
models work), output-buffer overflow (4× over-alloc didn't help, iter12),
the output memory write itself (discard test, iter13), TPU clock (kHigh vs
kMax, iter14), and HardFault (no SRC GPR breadcrumb, iter07/iter09).

## Root cause (iter15 offline executable parse → here)

`dump_exe` (iter15) parsed the darwinn executable straight out of each
`.tflite` and printed the per-executable `dma_hints` sequence. `c2f_thick`
is the ONLY model with `scratch_size_bytes = 153600`; all others are 0.
Its hint sequence:

```
[0] INSTR chunk=0
[1] DMA INPUT   INFEED  size=460800   off=0
[2] DMA INPUT   INFEED  size=466560   off=455040
[3] DMA SCRATCH OUTFEED size=153600   off=0     <-- TPU spills activations to host
[4] FENCE
[5] DMA SCRATCH INFEED  size=153600   off=0     <-- host feeds them back
[6] DMA OUTPUT  OUTFEED size=38400    off=0
[7] INSTR chunk=1
[8] DMA OUTPUT  OUTFEED size=9600     off=0
[9] INTERRUPT
```

`c2f_thick` is wide enough that its intermediate activations do not fit in
the Apex on-chip SRAM, so the edgetpu compiler split the compute into two
halves joined by a **host-mediated activation spill**: an OUTFEED drains
153600 bytes of working set to host RAM (device→host, bulk-IN), a FENCE
orders it, and an INFEED feeds the same bytes back (host→device, bulk-OUT)
before the second half runs.

coralmicro's reduced libedgetpu port (`libs/tpu/edgetpu_executable.cc`
`Invoke`) handled INSTR / PARAMETER / INPUT / OUTPUT hints but had
`Description_BASE_ADDRESS_SCRATCH` fall through `default: break` — the spill
DMAs were silently ignored. So the TPU pushed 153600 scratch bytes onto the
bulk-IN endpoint that nobody drained; the next `GetOutputs` then read scratch
bytes instead of the real output, the bulk endpoints desynced, and the USB
transport wedged → watchdog reset. The shipped demo models never spilled, so
the omission had never been exercised.

## Fix

Service the scratch round-trip the same way the full libedgetpu does, using
M7 RAM as the parking buffer for the TPU's working set:

- `libs/tpu/edgetpu_driver.{h,cc}`: `GetScratch` (OUTFEED drain, bulk-IN, same
  path as `GetOutputs`) + `SendScratch` (INFEED refill, bulk-OUT, routed on the
  input-activation tag — the USB protocol has no dedicated scratch tag).
- `libs/tpu/edgetpu_executable.{h,cc}`: `PrepareScratch` reserves the buffer via
  `RequestScratchBufferInArena` (TFLM arena, Prepare-only, auto-sized — no heap,
  fails cleanly in AllocateTensors if too big); the new
  `Description_BASE_ADDRESS_SCRATCH` case in the Invoke DMA loop dispatches
  OUTFEED→GetScratch / INFEED→SendScratch against that arena buffer.
- `libs/tpu/edgetpu_op.cc` + `edgetpu_manager.{h,cc}`: `CustomOpPrepare` →
  `EdgeTpuManager::PrepareScratch` so the arena request happens in Prepare.

## Verification (board build 1560)

Live REPL, `sentai.tpu.trace(1)`, `sentai.tpu.load("/c2f_thick.tflite")`,
`sentai.tpu.invoke()`:

```
[tpu] I ... bytes=460800 off=0
[tpu] I ... bytes=466560 off=455040
[tpu] S name=(null) bytes=153600     <-- GetScratch (OUTFEED)
  [bulkin] ... 36864*4 + 6144 = 153600   drained OK
[tpu] s name=(null) bytes=153600     <-- SendScratch (INFEED)
  [bulkout] ... 153600                    fed back OK
```

`invoke()` returns `34` (>=0 = success), board stays alive (`sentai.version()`
still build 1560, `lsusb` still `1fc9:c0a1`). Re-running back-to-back with
`msblock` (scratch=0) confirms the non-scratch path is unregressed. **The
widest revision champion now runs on the board.**
