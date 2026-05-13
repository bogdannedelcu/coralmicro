# s119 — H3 smoke test

**ObjectsPlan Layer 1.** Proves the freshly-vendored `third_party/h3`
(Uber H3 v4.4.1, commit `383ecdb3`) compiles cleanly against the host
SIM toolchain and that the four API entry points `sentai.places` (L3)
will lean on return sane values.

This test was written from scratch for the layered re-implementation;
it intentionally does **not** reuse the test code from the broken
`feature/ov5640-camera-support` branch (see memory
`feedback_no_broken_branch_test_reuse.md`).

## What we check

| API | Invariant |
|---|---|
| `latLngToCell` | non-zero index returned for Bucharest at res=8 |
| `cellToLatLng` | round-trip drift < ~700 m (res-8 cell radius bound) |
| `gridDisk` | `k=1` -> exactly 7 cells incl. origin once; `k=2` -> 19 cells |
| `cellToBoundary` | 6 vertices (origin is a hex, not a pentagon) |
| `isPentagon` | returns false for the origin |

The origin is the Bucharest city centre (44.4268°N, 26.1025°E) — chosen
deliberately distinct from any of the documented H3 examples so the test
exercises the API rather than reproducing a memorised result.

## Run

```bash
cd examples/sentai_runtime/experiments/s119_h3_smoke
./build_and_run.sh
```

Expected last line: `s119 H3 smoke: PASS`. Any assertion failure prints
`FAIL: <why> (<file>:<line>)` and exits non-zero.

## Why this experiment exists

Per `CLAUDE.md` and `ideas/objects_plan.md`, no new code should land in
production before its dependencies have been validated on the target
toolchains. H3 is a brand-new vendor dep; this test confirms it links
against a vanilla gcc/clang host build with no special flags before we
wire it into the SIM and ARM SentAI builds (planned at L3
`sentai.places`).
