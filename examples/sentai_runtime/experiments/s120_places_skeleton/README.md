# s120 — sentai.places skeleton (Stage 11.B)

**Goal.** Verify the freshly-added `sentai.places` MicroPython module
(H3-indexed world-model gallery) works end-to-end on SIM.

This is Stage 11.B of `ideas/objects_plan.md`. The skeleton stores a
fixed-size array of `places_entry_t` keyed by H3 index, with per-cell
class histogram (top-K=8 by count) and a stub embedding slot for
Stage 11.D (HSV histogram from `sentai_tracker`).

## API surface verified

| Function | Behavior |
|----------|----------|
| `init(lat, lng [, scale=1, res=9])` | set local→geo origin + default res |
| `cell(x_m, y_m [, res])`            | local coords → H3 index |
| `observe(cell, class_id)`           | bump visit + class histogram (returns visits) |
| `classes(cell)`                     | `[(class_id, count), ...]` sorted desc |
| `visits(cell)`                      | observation count for cell |
| `neighbors(cell, k)`                | k-ring as H3 index list (incl. origin) |
| `cells()`                           | all known cells |
| `clear()`                           | wipe gallery (origin/scale preserved) |
| `info()`                            | dict: initialized/n/max/default_res/scale/origin |

## Run

```bash
cmake --build build-sim --target sentai_sim
python3 examples/sentai_runtime/experiments/s120_places_skeleton/test_places_basic.py
```

## Verified 2026-05-13

```
  OK: init returns 0
  OK: first observe returns 1
  OK: fourth observe returns 4
  OK: classes = [(56,3),(17,1)]
  OK: visits returns 4
  OK: neighbors length 7
  OK: neighbors include origin
  OK: cells list length 1
  OK: far cell differs from origin
  OK: post-clear visits is 0
  OK: post-clear info.n is 0
  OK: post-clear observe returns 1

[test] PASS — sentai.places skeleton works
```

12/12 checks PASS.

## What's next (Stage 11.D)

The `embedding[PLACES_EMB_DIM]` field is allocated but unused. Stage 11.D
wires it to `sentai_tracker.cc`'s HSV 4×4×8 histogram (128-D, 8-bit
normalized) — adds `places.set_embedding(cell, bytes)` + `places.match(bytes)`
for query-by-appearance.
