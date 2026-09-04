---
name: reference_edgetpu_instruction_caching_256kb
description: EdgeTPU instruction reuse — desc_cache already implements it but WEDGES the board (build 1560); instructions are opaque VLIW; c2f_thick exceeds the 256KB instruction memory.
metadata:
  type: reference
---

Operator idea (2026-06-26): instead of re-sending ~280KB of instructions every
invoke, send a tiny "jump to already-loaded instructions" so the TPU reuses them.

**Findings:**
- The mechanism ALREADY exists as `desc_cache` (`g_sentai_tpu_desc_cache_enabled`,
  default OFF, toggle `sentai.tpu.desc_cache(1)`). On a cache hit (same
  EdgeTpuExecutable instance + matching parameter_caching_token) it skips BOTH
  parameter and instruction uploads entirely — sends ZERO instruction bytes, not
  a 2-3KB jump — relying on the TPU instruction FIFO persisting the last stream.
- **BUT it WEDGES the board in build 1560**: enabling it and invoking hangs the
  invoke -> REPL dies -> board needs recovery (confirmed the hard way; see
  [[reference_board_usb_wedge_needs_physical_por]]). That's why it's default-OFF
  and the s236 driver `_t_s236_speed.py` calls `desc_cache(0)`. It is an
  INCOMPLETE experiment — do not enable without fixing it first.
- A hand-crafted jump is NOT feasible: the instruction bitstream is opaque
  DarwiNN VLIW (entropy ~3.5 bits/byte, ~60% zero padding, no documented
  CALL/JMP). Same-architecture models share only a 3-byte prefix -> no reusable
  preamble to extract.
- The 256KB intuition is REAL and is the hard limit: chunk0 is capped at ~256KB
  (the instruction memory size). Per-model inference instruction totals:
  c2f_thick 261984+17984 (TWO chunks, chunk1 streamed mid-execution AFTER the
  scratch round-trip), c2f_deep 261520+22112 (two), c2f 155168 (one), msblock
  142576 (one). The 6 smaller models fit entirely (single chunk) -> caching
  feasible IF desc_cache were fixed; c2f_thick/c2f_deep cannot keep all code
  resident, so "load once, jump forever" is impossible for them by construction.

**Recommendation:** the realistic win is to DEBUG desc_cache for the single-chunk
models (likely the FIFO-persistence assumption is wrong, or a run-control "go" is
missing after skipping instructions) — NOT to decompile the proprietary ISA.

Offline analysis tooling: `experiments/s236_model_speed_sweep_640x480/`
`iter15_offline_exe/dump_exe.cc` (dma_hints + instruction chunk sizes; set
`DUMP_BITS_DIR` to export raw bitstreams) and `iter17_instr_analysis/`.
Related: [[project_edgetpu_host_scratch_roundtrip_fix]].
