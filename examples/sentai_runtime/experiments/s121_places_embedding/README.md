# s121 — sentai.places HSV embedding + match (Stage 11.D)

**Goal.** Verify the per-cell embedding storage and Bhattacharyya match
against `sentai.places` work end-to-end on SIM.

This is Stage 11.D of `ideas/objects_plan.md`. The 128-byte embedding format
is the same one `sentai_tracker.cc` uses for re-id: 4×4 spatial grid × 8 hue
bins, each spatial cell normalized to sum ≈ 255. Match cost is per-cell
Bhattacharyya averaged over the 16 spatial cells (returned as a percentage
similarity, 100 = identical, 0 = completely different).

## API added in this stage

| Function | Behavior |
|----------|----------|
| `places.set_embedding(cell, bytes)`   | store 128-byte HSV histogram |
| `places.embedding(cell)`              | return bytes(128) or None |
| `places.match(bytes [, k=1])`         | best k cells by similarity, desc |

## Run

```bash
cmake --build build-sim --target sentai_sim
python3 examples/sentai_runtime/experiments/s121_places_embedding/test_places_match.py
```

## Verified 2026-05-13

```
  OK: 3 cells distinct
  OK: set_embedding A returns 0
  OK: set_embedding B returns 0
  OK: set_embedding C returns 0
  OK: embedding len is 128
  OK: embedding roundtrip exact
  OK: match returned 3 rows
  OK: match top is cell A
  OK: match A self-similarity 100%
  OK: noisy match still ranks A first
  OK: noisy match sim within 50-100
  OK: wrong-length rejected (-2)
  OK: invalid cell rejected (-1)

[test] PASS — places embedding + match work
```

13/13 checks PASS.

## What's still missing (deferred)

  - **Compute-from-frame helper.** A `places.compute_hsv(rgb_buf, w, h)` —
    or better, wiring this directly into the pipeline so each captured
    frame writes its HSV histogram into the current cell — is needed
    before `sentai.explore` can use places.match for re-localization.
    Deferred until the explore mission needs it.

  - **Cross-scale composition (§17 sum-pool).** Aggregate child-cell
    embeddings into parent-cell embeddings on demand — deferred until
    we have multiple resolutions populated.

  - **Persistence (FileX).** `places.save(path)` / `places.load(path)`
    mirroring `sentai.slam.save/load`. Easy to add when needed; the
    per-cell entry is fixed-size and self-contained.
