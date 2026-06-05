---
name: no-heavy-data-through-mp
description: "HARD RULE 2026-05-17.  Never expose heavy data (camera frames, tensors, raw buffers >1 KB) through the MicroPython binding boundary.  MP heap ceiling is ~256 KB and any allocation from a C binding (mp_obj_new_bytes / mp_obj_new_list of large size) creates GC pressure, fragmentation, and slow paths.  Instead: pre-process in C/C++ on a fixed schedule (PrepTask on ARM, camera_bridge_recv on SIM) and expose ONLY pointer-via-extern-C-hook to consumers (zerocopy).  MP bindings return ONLY small scalars/dicts (descriptor bytes ≤ 64 B, scalar metrics, status enums).  Mirror the detection_task PrepTask/InferTask pattern for any new consumer."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Operator-stated 2026-05-17** when I proposed adding
`sentai.camera.grab_rgb(w, h) -> bytes(3*w*h)` for HSV consumption.

## Rule

1. **NO heavy buffers through MP heap.**  MP allocations from C
   bindings (`mp_obj_new_bytes`, `mp_obj_new_list` with large len)
   are limited by the MP heap (`~256 KB on ARM`).  Even small
   "occasional" allocations create GC pressure that destabilises
   long-running missions.  Documented in `agent/agent.md:877` —
   "use the camera/PrepTask path for real workloads".
2. **All image processing in C/C++** running at camera FPS in a
   dedicated task (PrepTask pattern).  Algorithms consume the
   *pre-prepared* buffers via extern-C zerocopy hooks — they do NOT
   trigger image processing on demand.
3. **MP bindings return only small scalars/dicts** — descriptor
   bytes ≤ 64 B (slot-native), marker detection tuples (id + 3-vec
   tvec + 8 corner floats), counter dicts.  Anything that scales
   with image size goes through the pointer path.

## Why

- **MP heap ceiling** (~256 KB total, smaller per-alloc in
  practice).  A 320×240 RGB frame is 230 KB — barely fits but kills
  the rest of MP runtime.
- **GC pressure**: even if it fits, every alloc churns the MP GC.
  Long-running missions (s146-s156 closed-loop flights) need GC
  predictability — heap alloc inside a 100 Hz loop is unacceptable.
- **Thread-safety**: PrepTask producing into a fixed slot ring with
  semaphore-gated rotation is far safer than per-call malloc.  The
  Crazyflie firmware does this; so does our InferTask.
- **Consistency with HW reality**: on ARM the CSI ISR + PXP already
  produce all the frame variants we need.  The pattern is the
  natural extension; ad-hoc per-call conversions are anti-pattern.

## How to apply

When adding a new algorithm that needs the camera frame:

1. **Decide which prep slot it needs** (format + resolution).
   Reuse existing slots when possible.
2. **Add the slot** to PrepTask's output set if missing.  PXP on
   ARM does scale + format conversion; scalar/CMSIS-DSP on SIM.
   Slot lives in `.sdram_bss` (ARM) / static buf (SIM).
3. **Expose a C accessor** `sentai_prep_get_<slot>(const uint8_t**,
   int*, int*, uint32_t* seq)` that returns the latest finalised
   snapshot.  Atomic-publish semantics (caller never sees a
   half-written frame).
4. **Algorithm calls the accessor** from its C entry, processes the
   pointer, and returns ONLY the result (scalars / small bytes).
5. **MP binding wraps the C entry** — never sees the image buffer.

Example for `sentai.places.compute_hsv` (OP-S10-W4, refactor pending):
- Current shape: `compute_hsv(rgb_bytes, w, h) -> bytes(64)` —
  WRONG, takes rgb_bytes through MP.
- Target shape: `compute_hsv_from_camera() -> bytes(64)` —
  RIGHT, reads via `sentai_prep_get_rgb_64()`.

## Anti-patterns to call out

- `mp_obj_new_bytes(buf, w*h*3)` for any RGB frame: NO.
- `sentai.camera.grab_rgb(w, h)`-style MP API: NO.
- Algorithm-specific image conversion at the binding entry: NO —
  move it to PrepTask so it runs once at camera FPS, not per MP call.
- Passing raw `bytearray` from MP into a binding to be processed:
  NO (heap allocation + copy).

The ONLY exception today is `sentai.camera.grab_gray(w, h)` returning
bytes — kept for legacy diag scripts that need to dump a frame to
disk via `sentai.fs.write`.  New consumers MUST go through the
PrepTask path.

## Related

- `[[arm-hw-primitives-first]]` — PXP / CMSIS-DSP / SIMD for the
  preparation itself (no scalar RGB→Y when PXP does it)
- `[[m7-aruco-bench-findings]]` — PXP+SIMD+DTCM gave 11× CPU savings
- `[[tpu-pipeline-aggressor]]` — PXP and USB share bus master ID;
  PrepTask scheduling is the contention-avoidance lever
- `agent/agent.md` PrepTask + InferTask description (lines 1185-1240)
