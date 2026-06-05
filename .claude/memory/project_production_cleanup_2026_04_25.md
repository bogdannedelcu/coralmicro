---
name: Production cleanup pass — dead-end paths purged (2026-04-25)
description: Build #878 — MoverTask, Cale 1 ring, save_raw_jpeg, copy_bench all removed. 43 FPS pipeline preserved. Fine-grained SendInputs sync kept.
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
NASA/JPL-style cleanup pass shipped on 2026-04-25, build #878.

**Removed (all empirically-confirmed dead-ends):**
- MoverTask + 4 semaphores + eDMA ch29 + 1.5 MB SDRAM ring (`s_prep_sdram_ring`)
- Cale 1 OCRAM ring buffer + eDMA ch30 + `.tpu_ring` linker section (72 KB OCRAM freed)
- `sentai_cam_save_raw_jpeg()` + `sentai.camera.save_raw_jpeg` binding
- `sentai.diag.copy_bench` retired stub comment
- Error codes 0x0B50..0x0B54 marked RETIRED (codes preserved in CSV per never-delete policy)

**Kept (documented production capability):**
- Fine-grained one-shot SendInputs sync via `g_sentai_tpu_input_done_sema` + atomic-exchange in `SendInputs()`. Default ON. Eliminates `.tpu_input` race without reducing parallelism.
- Direct zero-copy path (PrepTask writes OCRAM `.tpu_input` directly, InferTask hands pointer to invoke).
- Legacy memcpy + DMA fallback via `sentai.pipeline.direct_tensor(0)` for A/B.
- All real diagnostics: `infer_stats`, `prep_stage_stats`, `async_stats`, `tpu_perf`.

**Measured (fresh-flash `_t_yolo512.py`):**
- Pure TPU: **76.1 FPS** (13.1 ms/invoke), 0 fails
- Pipeline: **43.0 FPS** end-to-end (215 ok / 0 fail in 5 s, avg 21.1 ms invoke)
- Camera-bound ceiling (OV5640 VGA45 = ~45 FPS sensor cap)

**How to apply:**
- This is the new baseline. Next session starts here.
- Fine-grained SendInputs sync is load-bearing — do NOT remove `g_sentai_tpu_input_done_sema` / `sentai_tpu_set_input_done_sema()` / atomic-exchange give logic.
- Direct path is the ONLY active path; legacy is A/B-only.
- If pursuing more SEMC bandwidth: explore DTCM-source eDMA→OCRAM ring (224 KB DTCM = ~half of 372 KB instructions, ~50% reduction in SEMC instruction traffic). Untried.

**Build counter:** flashed at #878 on `feature/ov5640-camera-support`. Working tree dirty until commit.
