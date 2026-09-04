---
name: flow-detection-semc-contention
description: Why flow+detection concurrent resets the M7 (memory-bandwidth saturation on BOTH buses) and why M4-offload is the only real fix
metadata: 
  node_type: memory
  type: project
  originSessionId: 218d6445-6c73-4c0f-bb51-4d13943aaeb1
---

Running optical-flow (`flow_phase_corr.cc`, 2D 64×64 phase-corr FFT) concurrently
with the detection pipeline on real HW intermittently watchdog-resets the M7 under
sustained load. Cause is **memory-bandwidth saturation**, NOT CPU starvation
(flow runs below pipeline prio; the CPU has spare cycles — it's the bus).

**On the M7, NEITHER memory bus is free during inference:**
- **OCRAM** (0x20240000, 1 MB) is owned by the TPU input tensor `.tpu_input`.
  `c2f_thick`'s input is a full **640×480×3 = 921 600 B** image the TPU **re-DMAs
  every invoke (~26 ms of input transfer, per s238)**. OCRAM is the *busiest* bus
  during inference, not free.
- **SEMC/SDRAM** is owned by the camera DMA + PXP scaling.

Detection **alone** saturates the M7 memory bandwidth; flow's per-frame FFT tips
it over regardless of which bus its working set sits on.

**Experiments (s240):**
- Flow FFT in SDRAM + `slot_div(12)` throttle (build 1565/1569): one clean 48 s
  run early, but a later run reset at `flow.start`. Throttle LOWERS reset
  probability (fewer FFT bursts) but does NOT reliably eliminate it — intermittent.
- Flow FFT moved to OCRAM (build 1568, freed by demoting unused ArUco's ~93 KB
  `.ocram_bss` — ArUco superseded by WhyCon): **NEGATIVE result, made it WORSE.**
  Reset at div=12 t≈13 s. Because OCRAM is where the 900 KB TPU-input DMA lives —
  flow collided with the TPU's largest memory op. Reverted (build 1569).

**Fix = move flow compute to the idle M4 core** (build 1572, DONE + validated).
M4 has its own DTCM, off BOTH the M7 OCRAM and SEMC paths → the SAD working set
runs there with zero contention. The prior M4 attempt (removed build 1130) failed
for 3 concrete bugs, ALL fixed (see [[s240 README]] m4_offload_review.md):
(1) it ran a FreeRTOS scheduler on a miscalibrated SysTick (custom main skips
BOARD_InitBootClocks to protect the M7 camera) → froze; fixed by running BARE-METAL
(no scheduler, DWT timing). (2) scalar SAD → USADA8 SIMD (4×). (3) FLOW_SHARED_ADDR
was a STALE pointer (0x202C1000) into .tpu_input after rpmsg moved; relocated to
0x2033D000 + rpmsg re-widened 8→16 KB both `.ld`. Also removed the m4_curr_gray
mirror (a 4800 B/frame non-cached-OCRAM memcpy that alone cratered M4 to 1 fps).
On-board (build 1572): M4 boots (`sentai.flow.m4_start()`→0), USADA8 SAD ≈4.3 ms/
frame (~230 fps capable), no freeze. Bindings: `sentai.flow.m4_start/m4_stop/
m4_result`. M4 poll loop MUST be DWT-throttled (not tight) or its OCRAM reads
saturate the TPU-input bus.

**Open last-mile:** the cross-core gray handoff still lives in OCRAM (0x2033D000,
non-cacheable) so the M7-write + M4-read contend with the TPU's 900 KB OCRAM DMA →
intermittent `edgetpu invoke failed` / occasional reset under active flow. Fix =
move flow_shared to SDRAM (off the TPU's OCRAM bus; SAD stays in M4 DTCM) — needs
M4+M7 MPU non-cacheable region on the SDRAM span. Tracked in m4_offload_review.md.

Interim: flow+detection concurrent on M7 is best-effort (throttle high via
`sentai.flow.slot_div`, expect occasional resets). Detection-alone and flow-alone
are both stable. Don't bother trying to place flow buffers in M7 fast RAM — both
regions are full/contended.

See [[s240 README]] `examples/sentai_runtime/experiments/s240_flow_plus_pipeline_parallel/`.
FileX corrupts on each involuntary reset this session — reformat+re-upload model
via MSC (see fx-reformat-dev skill). Model host copy:
`experiments/s238_relu_v1_board_speed_sweep/models/c2f_thick.tflite`.
