---
name: op-s10-w14-t18-simd-threshold-2026-05-19
description: "OP-S10-W14-T18-T+U SHIPPED 2026-05-19.  Bradley adaptive-threshold SIMD on real M7 @ 800 MHz.  End-to-end ArUco detect 24 ms → 22 ms (-8 %, 45 FPS equivalent, 66 % of 30 FPS slot) for a 320×240 synthetic frame, byte-identical to scalar reference across block ∈ {7, 23, 51, 101, 201}.  Stack: CMSIS-DSP usub8/sel inline-asm wrappers (toolchain arm_acle.h doesn't ship them on gcc 9.3), divide-elimination via (gray+C+1)*box_area<=box_sum trick, 4-wide compare on interior columns, 32B aligned hot buffers.  ARM viability for the thesis is VALIDATED."
metadata:
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

Shipped 2026-05-19 at build #1353, commit pending.  Spec file:
`examples/sentai_runtime/sentai_aruco.cc` → `aruco_adaptive_threshold`
+ `aruco_adaptive_threshold_scalar_ref` + the runtime byte-equivalence
check `sentai_aruco_adaptive_threshold_verify()`.

## Outcome

- End-to-end ArUco detect on synth 320×240 frame:
  **24 ms → 22 ms** (median, 30 iters, σ ≈ 0.4 ms).
- Frame budget at 30 FPS: 72 % → 66 %.
- Equivalent FPS: 41.7 → 45.5.
- Byte-identical to scalar reference: 0 mismatches at
  block ∈ {7, 23, 51, 101, 201}.

## Stack of changes (cumulative)

1. `usub8` / `sel` inline-asm wrappers — toolchain `arm_acle.h`
   on `arm-none-eabi-gcc 9.3` does NOT expose `__USUB8` / `__SEL`,
   only `__USAD8` / `__USADA8`.  Wrote 2-line `__asm volatile`
   wrappers verbatim from CMSIS `Core/Include/cmsis_gcc.h`.
2. Divide elimination via algebraic identity:
   `gray < mean - C` ⟺ `(gray + C + 1) * box_area <= box_sum`
   (exact integer floor proof in the source comment).
3. Interior 4-wide SIMD compare (only for x in [half, W-half-1]
   where x_factor = block, constant per row):
   - `raw = usub8(mean_pack, gray_pack)` → 4-byte signed diff
   - `clamped = sel(raw, 0)` → zero out lanes where mean < gray
   - `usub8(clamped, C+1)` → GE set iff diff ≥ C+1
   - `bin_pack = sel(0x01010101, 0)` → packed binary output
4. Border columns (62.5 % at block=201) stay scalar with full
   x clamping — varying `x_factor` blocks SIMD.
5. 32 B `aligned(32)` on all hot ArUco buffers (s_binary,
   s_labels, s_integral, s_fill_stack) — codifies OP-S10-W15-T5
   alignment policy.

## Verification infrastructure (KEEP for future refactors)

- `sentai.aruco._verify_threshold(block)` → runs scalar_ref vs
  production threshold on a deterministic synth gray frame
  (gradient + LFSR noise + central dark square); returns byte-
  mismatch count.  ANY future Phase-2 refactor MUST keep this at 0.
- `sentai.aruco._thresh_cycles()` → (old_cyc, new_cyc) tuple
  populated by the last verify call.  M7 @ 800 MHz → divide by
  800 for µs.

## Dead ends documented in-source

The `aruco_adaptive_threshold` comment block above the function
lists three measured-not-merged iterations:
- Iter 1 (LUT + division-elim in pure C): 24 % SLOWER.  gcc -O2
  already does this; LUT loads break pipelining.
- Iter 2 (ITCM `.ramfunc` placement): 16 % SLOWER.  Function-call
  overhead from breaking inlining > ITCM fetch win.
- Iter 3 (the one shipped): SIMD intrinsics + divide-elim, only
  on interior columns.

## Why not bigger win

Phase 1 (serial integral-image build, ~5 ms) is the next limiter.
Serial prefix-sum, intrinsically hard to vectorise on Cortex-M7
(no parallel prefix-sum primitive).  Further reductions would
need either:
- Frame downscaling (160×120 → ~6 ms but accuracy hit on small
  markers).
- Algorithm pivot (PXP path — 5.8 × on threshold but
  algorithmically distinct, rejected by operator on math-identity
  grounds).

## Cross-refs

- `examples/sentai_runtime/experiments/s179_hw_aruco_perf_bench/`
  — HW perf bench (kernel breakdown + end-to-end detect timing).
- `diary/2026-05-19.md` "Lane A follow-up #2" — operator
  conversation that drove the iteration.
- `[[op-s10-w15-arm-memory-budget]]` — cache-line alignment
  policy this work pre-applied (T5).
- `[[m7-aruco-bench-findings]]` — historical "12.1→1.1 ms"
  numbers were on a DIFFERENT kernel (naive 7×7 box, not the
  integral-image Bradley); my 22 ms is for the full
  `sentai_aruco_detect` pipeline including the multi-scale
  threshold + contour search + bit decode + PnP gates.
