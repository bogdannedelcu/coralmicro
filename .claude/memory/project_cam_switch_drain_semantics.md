---
name: Camera switch drain — what it actually protects against
description: Drain semantics clarified 2026-04-25. MUX flip itself is glitch-free (VBLANK in CSI ISR); drain protects against queue staleness, not tearing.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
The dual-camera MUX flip on Coral Dev Board Micro is **glitch-free** —
`camera_support.c:78-85` flips the analogue MUX inside the CSI ISR
right after FB2-done (EOF, in VBLANK). The next DMA buffer is filled
100 % by the new sensor; **no mid-buffer seam**.

`sentai.camera.switch_drain(N)` does NOT protect against flip artifacts.
It protects against **queue staleness**: when PrepTask calls
`cam_grab_latest()` immediately after a flip, the CSI ring still holds
1-2 buffers that completed *before* the flip — clean pixels, but from
the *old* camera. Without the drain, "latest" could be the wrong camera.

`drain=1` (production default) waits for one post-flip sensor frame to
accumulate, then drains stale and keeps the latest — guaranteed
new-camera. Wait cost = ~22 ms at VGA45.

**Why:** I once described the drain as protecting against "tearing /
mid-buffer artifact" — that is wrong. The flip is in VBLANK so there
is no mid-buffer seam. The cost is queue-induced, not pixel-induced.

**How to apply:**
- When proposing optimizations to camera switching, do NOT reach for
  "make the flip non-blocking" — the flip is already glitch-free in
  ISR. The lever is queue-side: detect freshness via buffer-index, tag
  buffers with `cam_id_at_capture`, or pre-flush the queue in-ISR on
  flip. Until one of those ships, drain=1 is the right default.
- Failure mode at drain=0 would NOT be visible tearing — it would be
  occasionally returning a frame from the OTHER camera (clean image,
  wrong source). Different observable, different debug signature.
- The 30 FPS ceiling at 1:1 alternation is queue-staleness-bound, not
  bus-contention-bound. SEMC has nothing to do with it.

Sources:
- `libs/camera/camera_support.c:78-183` (CSI ISR + flip-on-EOF)
- `examples/sentai_runtime/sentai_runtime.cc:2173-2240` (drain logic)
- `examples/sentai_runtime/agent/experiment.md` (camera-switch table)
