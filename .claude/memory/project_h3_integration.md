---
name: H3 (Uber) submodule integration recipe
description: third_party/h3 v4.4.1 wired in via 18 lib/*.c + sed for h3api.h.in version macros; SIM smoke test in s119_h3_smoke
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
Uber H3 v4.4.1 vendored at `third_party/h3` (commit 383ecdb3). Stage 11.A
of objects_plan.md (hexagonal world model) — used by `sentai.places`.

**Why:** Hexagonal hierarchical spatial indexing for the world model.
Same library Uber uses internally. 16 resolutions, neighbor lookup,
boundary polygons, all free of trig at query time.

**How to apply:**

1. Build needs just the 18 C files under `third_party/h3/src/h3lib/lib/*.c`
   + headers under `third_party/h3/src/h3lib/include/`. The CMake top-level
   pulls in tests/examples we don't need — skip add_subdirectory(third_party/h3).

2. `h3api.h.in` → `h3api.h` requires ONLY 3 substitutions (no other @VAR@
   markers in the template). A `sed -e "s/@H3_VERSION_MAJOR@/4/g" ...`
   pipeline suffices — CMake `configure_file()` not required. See
   `examples/sentai_runtime/experiments/s119_h3_smoke/build_and_run.sh`
   for the exact incantation.

3. libh3.a is 188 KB on x86 with -O2. ARM equivalent will be similar
   (no SIMD intrinsics; pure C + libm). Code is ROM-safe.

4. API surface confirmed working on x86 (s119 PASS 2026-05-13):
   - `latLngToCell(&p, res, &cell)` — geographic → H3 index
   - `cellToLatLng(cell, &p)` — H3 index → geographic
   - `gridDisk(origin, k, out)` — k-ring neighbors (need `maxGridDiskSize`)
   - `cellToBoundary(cell, &b)` — 6 vertices per hex
   - `isValidCell(cell)` — fault gate

5. LatLng struct uses **radians**, not degrees. Convert before/after with
   M_PI/180.0 (libm). Roundtrip accuracy: ~0.001° at res=9 (cell radius
   ~150 m), consistent with H3 spec.
