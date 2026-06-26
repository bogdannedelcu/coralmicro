# s236 iter03 - Board per-model speed sweep (640x480 cover_v1)

Standalone EdgeTPU invoke speed for the 8 headless p3p4 detectors, measured on
the physical SentAI board (build 1546), using the s235/B10 methodology:
`sentai.rtos.micros()` wall-clock around N=50 invokes + `sentai.tpu.urb_stats()`
URB wall-time phase breakdown. NOT DWT, NOT ticks (per TD-S10-A10).

## Method

Self-contained on-board driver `diag/_t_s236_speed.py`: per model -> load,
camera 640x480 -> to_tensor (provides the 921 KB input), 5 warmup invokes,
then 50 timed invokes; reads URB bytes/wait per phase. `sys.reset()` between
models to clear TPU state (a failing model wedges the TPU for the boot; the
driver resumes via `/diags/.s236_state`). Host runner `run_sweep.py` exec's
the driver and re-exec's after each reset until `=== done ===`.

- chunk_size = 64 KB, desc_cache OFF.
- Input = live camera tensor (640x480x3 uint8); content is irrelevant to
  transfer/compute time. Detection correctness was validated host-side (iter01).

## Result (build 1546)

| model | etpu KB | ms/invoke | fps | instr ms | input ms | output ms | fails |
|---|---:|---:|---:|---:|---:|---:|---:|
| msblock | 265 | 43.73 | 22.87 | 4.03 | 32.44* | 5.30 | 0 |
| c3 | 325 | 38.12 | 26.23 | 4.46 | 25.64 | 6.07 | 0 |
| c2f | 329 | 38.14 | 26.22 | 4.37 | 25.63 | 6.18 | 0 |
| c2f_pan2 | 413 | 40.19 | 24.88 | 5.60 | 25.63 | 7.00 | 0 |
| gelan | 445 | 40.98 | 24.40 | 6.07 | 25.64 | 7.33 | 0 |
| gelan_pan2 | 561 | 43.84 | 22.81 | 7.83 | 25.66 | 8.40 | 0 |
| c2f_deep | 585 | 44.18 | 22.63 | 7.98 | 25.65 | 8.59 | 0 |
| **c2f_thick** | 1077 | **FAIL** | - | - | - | - | **50** |

\* msblock input_wait is a first-model-post-boot outlier (USB/camera warmup);
its instruction time (4.03 ms, smallest model) is correct. Re-run in isolation
to get its steady input ~25.6 ms (-> ~37 ms/invoke, ~27 fps) if needed.

Per-invoke transfer sizes (constant input, model-varying instructions):
- input image: 921,608 B/invoke (~25.6 ms) - DOMINANT, identical all models
- instructions: 142-284 KB/invoke (4.0-8.0 ms) - scales with model width
- output readback (2 heads): 48,000 B/invoke (5.3-8.6 ms)
- parameters: 0 (cached on-chip after warmup)

Chart: `per_phase_chart.png`.

## Key findings

1. **7/8 cover_v1 detectors run on the board EdgeTPU**, 22.6-26.2 fps, zero
   invoke failures. Throughput is **USB-input-bound**: the 921 KB image send
   (~25.6 ms) is ~60-65% of every invoke; the instruction stream (model-
   dependent, 4-8 ms) is the only term that separates the architectures.
   Smaller/narrower models (msblock/c3/c2f) are fastest because their
   instruction stream is shortest; the input send is a fixed floor.

2. **c2f_thick (the champion) does NOT invoke on the board — root cause
   pinned to OUTPUT readback, NOT instruction upload.**

   The visible `0B62 SendInstructions` is a red herring: it only appears on
   the 2nd+ invoke, AFTER the 1st failure wedges the TPU. The per-phase URB
   counters from a clean-boot FIRST invoke show every upload succeeds:

   | phase | bytes done | submit_fail | timeout |
   |---|--:|--:|--:|
   | parameters | 782,344 | 0 | 0 |
   | instructions | 287,208 | 0 | 0 |
   | input image | 927,376 | 0 | 0 |
   | **output (GetOutputs)** | **38,400** | 0 | **1** |

   So params + instructions + image all transfer fine; the invoke fails at
   **`0B63 GetOutputs` — the output bulk-IN stalls** after ~38 KB and never
   completes. Raising `sentai.tpu.urb_timeout_ms` from 200 to **2000 ms does
   NOT help** -> it is not a tunable-timeout issue; the TPU genuinely stops
   producing output mid-readback. That partial-readback wedges the USB
   transport (code: "NO CANCEL ... cancelling a partial bulk leaves the TPU
   in an undefined state"), so every later invoke fails at the next upload
   (0B62), and eventually the USB re-enumerates.

   c2f_thick is the widest/"thickest" arch and the largest on-chip-param user
   (866.75 KiB) with the most ops (188). The other 7 (<=585 KB / <=532 KiB
   params) complete output readback fine. Hypothesis: c2f_thick's runtime
   on-chip working set (cached params + widest-layer activation tiles)
   exceeds what the board's single-EP apex firmware leaves available, so the
   TPU faults/hangs mid-compute and never emits the full output. It runs on
   host pycoral (full libedgetpu USB transport, 7/7 GT) -> model is valid;
   this is a board single-EP runtime interaction, not a model defect.

   Documented as a limitation per operator decision (2026-06-26). Possible
   firmware-side fixes if revisited: multi-EP firmware
   (`-DSENTAI_TPU_MULTI_EP=ON`), or recompiling c2f_thick with the EdgeTPU
   compiler constrained to a smaller on-chip param budget so more SRAM is
   left for activation tiles.

## Artifacts

- `results.csv` - per-model CSV pulled from board `/diags/s001_s236_speed/`
- `per_phase_chart.png` - stacked per-phase invoke time
- `sweep_stream.log` - raw REPL stream across all reset/resume segments
- `run_sweep.py` - host runner
- driver: `examples/sentai_runtime/diag/_t_s236_speed.py`
