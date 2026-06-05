---
name: op-s10-w17-whycon-2026-05-19
description: "OP-S10-W17 WhyCon-lite circular-marker detection prototype shipped on M7 with full optimization stack (OCRAM placement + SIMD divide-elim Phase 2 rolling threshold + inline 2nd-order moments in flood-fill).  Final M7 end-to-end @ 4 markers = 6.00 ms (4.1× faster than ArUco rolling 24.8 ms, 5.3× than ArUco production 31.8 ms).  M4 ablation on Phase-A only kernel = 25.86 ms (~7× slower than M7 SIMD); reconfirms OP-S10-W16 verdict that M4 is not viable for compute-bound pixel workloads."
metadata:
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

## Goal

First WhyCon work on the codebase: prototype the circle-marker
detection pipeline (Krajník/Nitsche 2013 family) on M7, optimize
with HW primitives, and measure latency vs the established ArUco
baseline.  Documented as the alternative to ArUco for landing-pad
pose, which the SOTA review predicts to be ~3× faster.

## M7 optimization stack (build #1397, commit 8eccf31b)

Cumulative measurements at 320×240 synth-disks frame, anti-DCE
via n_det + _get_markers state consumption.  Median over 6 iter.

| Stack | N=1 | N=4 | N=8 |
|---|---:|---:|---:|
| Baseline (init impl) | 7.35 | 8.24 | 9.42 |
| + OCRAM placement | 7.35 | 7.54 | 8.72 |
| + SIMD rolling Phase 2 | 5.24 | 6.12 | 7.31 |
| + Inline moments | **5.19** | **6.00** | **7.05** |

Total saving vs baseline: **-29% @ N=1, -27% @ N=4, -25% @ N=8**.

Each step:

1. **OCRAM placement** (commit c694cee5): s_test_gray (75 KB)
   + s_fill_stack (16 KB) moved to .ocram_bss after reducing
   ARUCO_MAX_W/H from 640×480 → 320×240 (no external user of the
   max-rez buffers; freed 1.4 MB unused SDRAM and made 91 KB of
   hot buffers fit the 103 KB-free OCRAM region post .tpu_input).
   Saving: -0.7 ms consistent across N.
2. **SIMD divide-elim Phase 2** (commit c694cee5): same USUB8/SEL
   trick as the production aruco_adaptive_threshold, ported into
   the rolling-integral path's prefix_x lookup.  Math-identical
   vs scalar reference (0 mismatches across blocks 7/23/51/101).
   Saving: -1.4 ms consistent.  Surprised the operator who was
   skeptical — compiler does NOT autovectorize Phase 2 because of
   the divide + boundary clamps + prefix_x indirection.
3. **Inline moments** (commit 8eccf31b): extend aruco_comp_t with
   m20_sum/m02_sum/m11_sum (int64), accumulate during flood-fill
   pop, skip Phase W2 rescan.  Saving scales with marker count
   (-0.05 ms @ N=1, -0.26 ms @ N=8).

## ArUco vs WhyCon-lite comparison @ 4 markers

| Algorithm | Latency | Speedup vs ArUco prod |
|---|---:|:---:|
| ArUco production (SIMD integral SDRAM) | 31.8 ms | 1.0× |
| ArUco rolling (OCRAM scratch) | 24.8 ms | 1.28× |
| WhyCon-lite baseline | 8.24 ms | 3.86× |
| WhyCon-lite + OCRAM | 7.54 ms | 4.22× |
| WhyCon-lite + OCRAM + SIMD | 6.12 ms | 5.19× |
| **WhyCon-lite + OCRAM + SIMD + inline moments** | **6.00 ms** | **5.30×** |

At 30 Hz SafetyTask period (33 ms slot):
- ArUco prod: 96 % slot (marginal)
- ArUco rolling: 75 % slot
- **WhyCon-lite full opt: 18 % slot (huge headroom)**

## M4 ablation (commit e1b1f598)

Sentinel range 0xC100..0xC108 added to existing m4_aruco_bench.
M4 path: synth(N disks) + rolling threshold scalar.  PHASE A
ONLY — no flood fill / moments (port not done).

| N | M4 cycles | M4 @ 400 MHz |
|---:|---:|---:|
| 1 | 10.36 M | 25.90 ms |
| 4 | 10.34 M | 25.85 ms |
| 8 | 10.33 M | 25.83 ms |

Constant in N (O(W×H) threshold cost dominates).

M7 vs M4 on Phase-A kernel: ~3-4 ms M7 vs 25.86 ms M4 = **~7×
M7 win**.  Reconfirms OP-S10-W16 ArUco ablation — M7 dual-issue
+ D-cache + DSP-SIMD + 800 MHz beats M4 single-issue + OCRAM +
no-cache + 400 MHz by a large margin on bulk pixel workloads.
M4 stays an "idle backup" core for SentAI, not a perf-boost.

## What WhyCon-lite IS NOT (production gaps)

1. **No concentric inner-disk check (Phase W3)**: toggle stub
   exists (`sentai.whycon._set_concentric(1)`) but the actual
   inner-white-disc validation is not implemented.  This is the
   defining difference between "WhyCon-lite" (here) and full
   WhyCon (Krajník original).  Adds ~1 ms est. when wired.
2. **No PnP / 3D pose**: outputs centroid+axes pixel only.
3. **No WhyCode bit decode**: no per-marker IDs.
4. **No robust ellipse fit (Fitzgibbon LSQ)**: moments-only;
   degrades under perspective foreshortening.
5. **No multi-marker constellation pose**: needs 3-4 markers
   in known asymmetric pattern; FSM not built.

## API surface (MicroPython sentai.whycon.*)

- `._test_synth(n, radius=15) -> int n_detected`
- `._test_pgm(path)           -> int n_detected (or <0 err)`
- `._detect_cyc()             -> uint cycles for last call`
- `._set_concentric(on)       -> toggle (W3 stub)`
- `._get_markers()            -> list of dicts {cx, cy, a, b, ang}`

## Cross-refs

- `[[op-s10-w16-ablation-findings-2026-05-19]]` — sibling M7-vs-M4
  ablation on ArUco (same conclusion).
- `[[op-s10-w14-t18-simd-threshold-2026-05-19]]` — origin of the
  USUB8/SEL divide-elim trick reused for WhyCon Phase 2.
- `[[op-s10-w16-T3.8-rolling-integral]]` — origin of the rolling
  threshold kernel (lives in sentai_aruco.cc, reused by WhyCon).
- WhyCon SOTA papers: Nitsche-Krajník 2014 JINT (original),
  Lightbody-Krajník 2017 SAC best paper (WhyCode ID encoding),
  Blaha-Krajník 2023 6-DoF refresh.
- commits: bb0a40d1 (WhyCon-lite prototype), c694cee5 (OCRAM+SIMD),
  8eccf31b (inline moments), e1b1f598 (M4 ablation).

## Next steps (not done this session)

1. Concentric W3 validation (full WhyCon contract).
2. PnP / 3D pose computation from semi-axes.
3. Multi-marker constellation pose for 6-DoF landing pad.
4. Upload + bench on whycon_real.pgm (proper inner-disc pattern).
5. Wire WhyCon into SafetyTask as an alternative to ArUco
   detector, gated by sentai.safety mode flag.
