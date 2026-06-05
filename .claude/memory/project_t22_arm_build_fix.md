---
name: t22-arm-build-fix-2026-05-19
description: "OP-S10-W14-T22 (commit 3a89466f, 2026-05-19): reactive ARM build restoration after W12/W13/W14 silently broke 3 link regions.  Linker HEAP_SIZE 16→14 MB; FR pool ARM defaults shrunk to 8×320×240 (vs SIM 16×640×480); sentai_safety{,_task}, sentai_fr{,_task}, modsentai_hal/safety/fr routed to .sentai_slow; inline CRTP packing in modsentai_crazy + sentai_calib_task removes ARM-only send_extpos/extpose helpers; sentai_crazy_hl_stop alias added.  Post-fix: m_text/m_data/m_sdram all clean, ELF 26 MB.  Both ARM + SIM build."
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

ARM build broke silently across W12/W13/W14 — three regions
overflowed once T22 was attempted:

| Region    | Pre-T22 overflow | Root cause                                  |
|-----------|------------------|---------------------------------------------|
| `m_data`  | 340 KB           | FR pools (events 30 KB, scalars 32 KB, drain scratch 308 KB) — all defaulted to DTCM .bss |
| `m_sdram` | 4.5 MB           | FR frame pool 16×640×480 ≈ 5 MB + already-tight SDRAM (~15.5 MB pre-W11) |
| `m_text`  | ~7.5 KB          | cumulative cold-path code without `.sentai_slow` routing |

Fix shipped in commit `3a89466f`:

1. **Linker** `MIMXRT1176xxxxx_cm7_ram_mp.ld`:
   - `HEAP_SIZE = 0x01000000 → 0x00E00000` (16 → 14 MB).
   - Added `.sentai_slow` routing for `sentai_safety{,_task}.cc`,
     `sentai_fr{,_task}.cc`, `modsentai_hal.cc`,
     `modsentai_safety.c`, `modsentai_fr.c`.
2. **`sentai_fr.h`** — ARM-specific defaults:
   ```
   SENTAI_FR_FRAMES_SLOTS  = 8   (ARM) | 16  (SIM)
   SENTAI_FR_FRAMES_MAX_W  = 320 (ARM) | 640 (SIM)
   SENTAI_FR_FRAMES_MAX_H  = 240 (ARM) | 480 (SIM)
   ```
   Production camera is 320×240; full 640×480 only useful for
   SIM host debugging.  Total ARM `.sdram_bss` FR footprint
   shrinks 4.9 MB → ~745 KB.
3. **`sentai_fr.cc`** — `s_frame_pool` + `s_drain_snap_frame` +
   `s_event_pool` + `s_scalar_pool` all carry
   `__attribute__((section(".sdram_bss,\"aw\",%nobits @")))`.
   The explicit `%nobits` matches the NOLOAD linker section and
   silences the assembler warning.
4. **Cross-platform CRTP** (`bindings/modsentai_crazy.c` +
   `sentai_calib_task.cc`): inline 12-byte ExtPos / 29-byte
   ExtPose packing routed through the existing cross-platform
   `sentai_crazy_send_crtp`.  Removes ARM-only
   `sentai_crazy_send_extpos`/`send_extpose` helpers.
5. **`sentai_crazy.cc`**: added `sentai_crazy_hl_stop` alias
   (same CRTP packet as `sentai_crazy_stop_motors`).
6. NASA/JPL hygiene: dropped dead `s_hold_yaw_target_deg` (T20
   never shipped) and unused `t_start` in
   `sentai_cam_get_raw_with_recovery`.

## Post-T22 region budgets (use these for next subsystem)

- `m_data` (DTCM, 224 KB): ~30 KB free after T22.  Default for
  hot-path .bss (ISR-touched, freq-accessed).
- `m_text` (ITCM, 252 KB): ~7 KB free.  Hot-path .text only.
- `m_sdram` (16 MB after heap shrink): ~3 MB free.  Default for
  large static buffers — use `.sdram_bss` for NOLOAD,
  `.sentai_slow` for cold-path .text.
- `m_ncamera` (24 MB uncached SDRAM): ~19 MB free (used only by
  CSI ring + a few large debug buffers).  Reserved for DMA-
  reachable non-coherent allocations.
- `m_ocram` (1 MB minus RPMSG): ~120 KB free after `.tpu_input`
  900 KB.  Reserved for TPU input + USB EHCI structures.

## Lesson learned (drives W15)

Every W12/W13/W14 task added static buffers / cold-path code
without a global memory-budget view.  T22 was reactive cleanup;
without an explicit policy, the NEXT subsystem will repeat the
break.  **`OP-S10-W15` filed** as open ToDo to produce
`paper/arm_memory_budget.md` design doc + audit scripts + CI
soft-alarm assertions.  Promotion trigger: next build break OR
thesis embedded-engineering chapter starts.  See
[[op-s10-w15-arm-memory-budget]].

## Verification

- `cmake --build build --target sentai_runtime` → clean link,
  ELF 26 MB, no overflow warning.
- `cmake --build build-sim --target sentai_sim` → clean link.
- SIM smoke: `sentai.version()` returns build 289,
  `sentai.fr` module loads, no runtime regression.
