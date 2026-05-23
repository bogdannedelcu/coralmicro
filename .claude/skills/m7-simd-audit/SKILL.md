---
name: m7-simd-audit
description: Audit a C/C++ hot loop on M7 for SIMD opportunities, libc-call regressions, and assembly emission. Use when writing or reviewing per-pixel / per-cycle code (camera, flow, ArUco, WhyCon, descriptor compute). Catches silent perf regressions like the flow 1 ms → 13 ms incident.
---

# /m7-simd-audit

The Cortex-M7 has SIMD instructions (`__USADA8`, `__USAD8`, `__UQADD8`, `__USUB8`, `__SEL`, `__SADD16`, `__SSAT`) that pack 4 byte-lane ops per cycle.  Falling back to scalar costs 4-16× more cycles.  Worse: small idioms like `memcpy(&u32, p, 4)` emit a libc call (30+ cycles overhead) instead of the obvious `LDR`.

## When to audit

- Any new `.ramfunc` or `.sentai_slow` C/C++ function in a per-frame path.
- Touching `libs/sentai/sentai_flow*.cc`, `sentai_aruco.cc`, `sentai_whycon*.cc`, `sentai_phog.cc`, `sentai_gist.cc`.
- Before declaring a perf number final.

## Audit recipe

1. **Identify the hot loop.**  Per-frame work that runs ≥ 30 Hz at ≥ 320×240 pixels.

2. **Look for SIMD opportunities:**
   - 4× byte SAD (Sum of Absolute Differences): `__USAD8` / `__USADA8` (the latter accumulates).
   - 4× byte threshold compare + select: `__USUB8` + `__SEL`.
   - 4× byte saturated add/sub: `__UQADD8` / `__UQSUB8`.
   - 2× 16-bit signed add/sub: `__SADD16` / `__SSUB16`.
   - Saturate-to-N-bits: `__SSAT` / `__USAT`.

3. **Check for hidden libc calls:**
   ```bash
   arm-none-eabi-objdump -d build/examples/sentai_runtime/sentai_runtime.elf \
     | awk '/<your_hot_function>:/,/^$/' \
     | grep -E 'bl\s+\w+'
   ```
   ANY `bl memcpy`, `bl memset`, `bl __aeabi_*`, `bl __div*` in a hot loop is a regression.
   - `memcpy(&u32, p, 4)` → emit `LDR` with `__attribute__((packed))` struct + cast, OR `*(uint32_t*)(p)` if you can prove alignment.
   - `memset(buf, 0, N)` in a per-frame path → use a static zero-fill or `arm_fill_q7`.
   - Integer division → check if it can be a shift or fixed-point reciprocal-multiply.

4. **Verify SIMD intrinsics emit the right instruction:**
   ```bash
   arm-none-eabi-objdump -d build/.../sentai_runtime.elf \
     | awk '/<your_hot_function>:/,/^$/' \
     | grep -E '\b(usad8|usada8|uqadd8|usub8|sel|ssat)\b'
   ```
   If you wrote `__USADA8(...)` but the assembly has no `usada8` instruction, the compiler optimized it away or didn't recognize the intrinsic.  Check headers (`arm_acle.h` vs CMSIS `core_cmInstr.h`) — our `sentai_aruco.cc` uses inline-asm wrappers because the toolchain `arm_acle.h` doesn't ship usub8/sel.

5. **Cycle-count via DWT** (see `pipeline-profiler` agent for full procedure).

## Anti-patterns (every one cost real debug time)

- `memcpy(&u32, p, 4)` — emits `bl memcpy` even when alignment is provable.  Replace with `*(const uint32_t *)p` (if aligned) or a packed-struct load.
- Per-pixel `?:` ternary where `__SEL` would handle 4 lanes.
- Per-pixel `if (x > t) ... else ...` where `__USUB8 + __SEL` would handle 4 lanes.
- EMA (exponential moving average) at the source: smoothing pushes the latency response one frame later.  Use a **deadband** instead — only update when change exceeds N.
- `-O3` on tight loops without checking the assembly — -O3 occasionally MAKES things slower via icache thrash; we hit this in `sentai_aruco_threshold` (12.1 → 1.1 ms after manual SIMD).

## Reference: real wins in this codebase

| Function | Before | After | Change |
|---|---|---|---|
| sentai_aruco threshold (320×240) | 12.1 ms | 1.1 ms | `__USADA8` + divide-elim + 4-wide compare |
| Flow SAD inner loop | 13 ms | 0.94 ms | `__USAD8` + remove hidden `memcpy(&u32...)` |
| WhyCon Phase 2 rolling | (pre-W17) | -1.4 ms saving | SIMD divide-elim |

## Buffer alignment

Hot buffers ≥ 256 B MUST be 32-byte aligned (one D-cache line) per `[[arm-mem-budget]]`:
```c
__attribute__((aligned(32))) static uint8_t hot_buf[SIZE];
```
SIMD loads on un-aligned addresses cost 2× more cycles than aligned ones.

## Reject patterns

- Submitting perf numbers without running the assembly audit.
- Adding new intrinsics without checking the toolchain emits them (the `arm_acle.h` gotcha).
- "It's faster on my x86 SIM build" — irrelevant; SIM uses gcc native, not arm-none-eabi.

## See also

- `agent.md` §M7 SIMD best practices (~lines 1854-1889)
- `[[m7-aruco-bench-findings]]` auto-memory — 11× speedup case study
- `[[arm-hw-primitives-first]]` rule — check PXP/CMSIS-DSP/SDK driver before scalar loops
- `[[pipeline-profiler]]` agent for cycle-count automation
