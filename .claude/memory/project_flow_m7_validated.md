---
name: Flow stack M7-only architecture (validated 2026-05-05)
description: sentai.flow now runs entirely on M7 (M4 retired). Bit-perfect on-board vs offline replay. Camera fps regression (18 vs 30) is upstream of flow.
type: project
originSessionId: 23ce703b-532f-42c3-ac8e-f35fa410241e
---
**State:** `sentai.flow.*` runs entirely on M7 in C++ (M4 retired
2026-05-05).  Validated bit-perfect: firmware on-board cumsum
matches numpy offline replay within 1 milli-grid-px (rounding
only) on the same gray frames.

**Why:** Pipeline (build #1126 → #1139):
1. M4 had unreliable SysTick (no BOARD_InitBootClocks; ticks ran
   ~360x faster than configCPU_CLOCK_HZ expected).  Froze under
   load after ~5 sec.
2. M4 cross-core memcpy to non-cached OCRAM (publisher gray -> M4
   curr) was 4800 bytes via AHB byte writes = several ms per
   frame, collapsed M4 throughput from ~15 fps to ~1 fps.
3. M7 has 800 MHz + I-cache + ITCM + working WDOG + HardFault
   handlers + SERR plumbing.  Single-core debug.

**Architecture:**
- `flow_task.cc` (M7) hosts publisher_task + sad_match.
- `sentai_flow_publish_frame()` is called from BOTH PrepTask
  (when TPU pipeline runs) AND publisher_task (standalone, when
  no pipeline).
- Output: `dx, dy` in milli-grid-pixel units (Q*1000), 1 grid-px = 8 raw-px.
- API names dropped `m4_` prefix: `flow.enable / start / stop / read /
  body_read / gray_snap / gray_to_cache / detail_score / gray_stretch /
  pub_stats / perf`.

**Why:** The `m4_` prefix lied -- compute is M7.

**How to apply:** When extending flow algorithm, edit `flow_task.cc`
on M7.  `flow_task_m4.cc` is on disk for historical reference but
NOT in CMakeLists.txt (M4 build target removed).
