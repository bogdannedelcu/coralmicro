---
name: rpmsg-cross-core-alignment-hard-rule-2026-05-19
description: "HARD RULE 2026-05-19.  When using IpcM7/IpcM4 (or any other RT1176 multi-core IPC built on FreeRTOS-MessageBuffers + MCMGR events), the rpmsg_sh_mem MEMORY region MUST be at the SAME physical address AND size on both cores' linker scripts.  The framework reconstructs cross-core stream-buffer handles as `__RPMSG_SH_MEM_START | eventData`; if the cores' bases differ, M7 and M4 each see-and-write a DIFFERENT physical region for what they think is the shared queue.  alive=1 from M4IsAlive() can still work (separate MCMGR event), but the App-message reply path silently fails."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

Operator-instituted 2026-05-19 after a multi-day W16 debug pit:
*"sunt ok aliniate memoriile shared in memory map-urile celor 2 cores? Sa nu fie desincronizare pe surse..."*

## Why: How to apply

**Why**: the coralmicro IPC framework
(`libs/base/ipc.cc:FreeRtosMessageEventHandler`) reconstructs the
event recipient as:

```cpp
xStreamBufferSendCompletedFromISR(
    reinterpret_cast<StreamBufferHandle_t>(
        reinterpret_cast<uint32_t>(__RPMSG_SH_MEM_START) | eventData),
    ...);
```

If `__RPMSG_SH_MEM_START` is different on M7 vs M4, the
reconstructed handle on one core points to memory that the
other core never reads.  Messages disappear into a black hole.

In our specific tree the bug was:
- M7 sentai linker `MIMXRT1176xxxxx_cm7_ram_mp.ld` had rpmsg at
  0x2033E000 size 0x2000 (8 KB at top of OCRAM2) — done to free
  OCRAM2 for the V22 TPU 786 KB tensor staging.
- M4 SDK linker `MIMXRT1176xxxxx_cm4_ram.ld` had rpmsg at the
  standard 0x202C0000 size 0x4000 (16 KB at bottom of OCRAM2).

Different physical regions.  alive=1 worked (different MCMGR
event); kBenchDone replies vanished silently.

**How to apply**: when adding ANY M7 + M4 IPC, audit the linker
scripts on both sides.  ORIGIN AND LENGTH of rpmsg_sh_mem MUST
match.  If the sentai M7 linker is custom (it is), the M4 linker
must mirror it — single-line edit to the M4 .ld.

Diagnostic that surfaces the issue cleanly: a counter inside the
M7-side IPC RX handler.  If `handler_calls == 0` after a known
M4 send, the issue is delivery (linker), not compute.

## Concrete remediation

`libs/nxp/rt1176-sdk/MIMXRT1176xxxxx_cm4_ram.ld` was modified to
match the sentai M7 layout exactly:

```
RPMSG_SHMEM_SIZE = DEFINED(__use_shmem__) ? 0x2000 : 0;
rpmsg_sh_mem (RW) : ORIGIN = 0x20340000 - RPMSG_SHMEM_SIZE, LENGTH = ...
m_ocram      (RW) : ORIGIN = 0x202C0000, LENGTH = 0x80000 - RPMSG_SHMEM_SIZE
```

Side benefit: M4 m_ocram grew from 384 KB → ~504 KB.

Other coralmicro multi-core examples (`multi_core_*`,
`tflm_person_detection_m4`) need the standard layout; they're
currently disabled in `examples/CMakeLists.txt:41-44` so the
global change is safe.  If/when re-enabled they should get a
sentai-side linker copy instead.

## Cross-refs

- `[[op-s10-w16-multicore-2026-05-19]]` — parent WP.
- `[[op-s10-w14-t18-simd-threshold-2026-05-19]]` — the M7
  baseline that the M4 measurement compares against.
- Commit 692f8b95 — the fix + first working bench (M4 250 us
  for 80x60 Bradley threshold, 2x faster per pixel than M7 at
  320x240 due to L1-D-cache fit).
