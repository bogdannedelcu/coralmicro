# s119 — H3 submodule smoke test

**Goal.** Prove the freshly-vendored `third_party/h3` (Uber H3 v4.4.1)
compiles cleanly against the SIM toolchain and the canonical API surface
we need for `sentai.places` works:

  * `latLngToCell(latLng, res, &cell)` — H3 index from coordinates
  * `cellToLatLng(cell, &latLng)` — coordinates from H3 index
  * `cellToBoundary(cell, &boundary)` — hex boundary polygon
  * `gridDisk(origin, k, out)` — k-ring neighborhood

If any of these fail or the link errors, we know before wiring `sentai.places`.

## Run

```bash
cd examples/sentai_runtime/experiments/s119_h3_smoke
./build_and_run.sh
```

## Actual output (verified 2026-05-13)

For Empire State Building (40.689167°N, 74.044444°W) at res=9:

```
[h3] latLngToCell(40.689167, -74.044444, res=9) -> 892a1072b5bffff
[h3] cellToLatLng(892a1072b5bffff) -> (40.690210, -74.043242)
[h3] gridDisk k=1 -> 7 cells (origin + 6 ring)
[h3] boundary -> 6 vertices
[h3] PASS
```

Round-trip error is ~0.001° (≈ 111 m), consistent with res=9 cell radius
~150 m. The four API surfaces (`latLngToCell`, `cellToLatLng`, `gridDisk`,
`cellToBoundary`) are confirmed working — `sentai.places` (Stage 11.B) can
proceed with H3 wired into its build.

## Why this experiment

Per CLAUDE.md and `ideas/objects_plan.md` §11, no new code accumulates in
SentAI without SIM verification. This is the verification step for
Stage 11.A (H3 submodule port). Once it passes we proceed to Stage 11.B
(`sentai.places` skeleton).
