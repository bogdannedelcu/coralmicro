---
name: objectsplan-l3-handoff
description: HISTORICAL handoff (superseded 2026-05-14 by [[places-l3-shipped]] + [[objectsplan-l4-handoff]]). L3 sentai.places landed at commit 19c40d88. Kept for chronology only.
metadata: 
  node_type: memory
  type: project
  originSessionId: cfa3374f-a472-4f14-be65-3858146f8b62
---

> **STATUS:** SUPERSEDED 2026-05-14.  L3 sentai.places shipped at commit
> `19c40d88`; FlowBaseline gate PASS dist=10.0 cm; driver 58/58 PASS in
> SIM.  See [[places-l3-shipped]] for the frozen API + ITCM placement
> contract + descriptor format that L4+ consume.  See
> [[objectsplan-l4-handoff]] for the next-layer entry point.  This file
> is kept only so prior session links resolve.

**State at handoff** (2026-05-13 22:35, branch `integration/from-180bbb5f`):

- L0 baseline: commit `a4454163` (FlowBaseline canonical no-wind).
- L1 shipped: commit `2717bb27` (docs + H3 + s119 smoke). Gate PASS dist=5.4 cm.
- L2 shipped: commit `eaf67e75` — sentai.objects data layer.
  Gate PASS dist=9.3 cm / all4=1.0 / n=14 / z=2.9 cm.
- L3 **not started**. Next layer: `sentai.places` (Stage 11.B/D in
  the broken branch, see [[objectsplan]]).

**L2 deliverables on disk** (so L3 can build on them, not re-discover):

- `examples/sentai_runtime/sentai_objects.{h,cc}` — public C API + impl.
  `.sentai_slow` SDRAM section (linker script, like `sentai_tracker.cc`).
  State arrays in `.sdram_bss`.
- `examples/sentai_runtime/modsentai_objects.c` — MP binding,
  `#include`'d from BOTH `modsentai.c` (ARM) and `sim/modsentai_sim.c`
  (SIM).  Single source of truth for the binding.
- `examples/sentai_runtime/diag/_t_01_objects.py` — fresh driver test.
- `examples/sentai_runtime/sentai_error.h` — block 0x10xx
  (SERR_MOD_OBJ + SERR_OBJ_*) registered for L4+ ingestion logging.
- `Sim.md` §10w — REPL test-execution recipe (drop file in
  `build-sim/sentai_fs_root/` and `import` from REPL).

**API surface available to L3** (no changes expected):
```python
sentai.objects.add(class_id, x, y, z, cov6=None) -> id|-1|-2|-3
sentai.objects.get(id) -> dict|None
sentai.objects.list() -> [dict, ...]
sentai.objects.mark_visited(id) -> 0|-1
sentai.objects.set_status(id, status) -> 0|-1|-2
sentai.objects.remove(id) -> 0|-1
sentai.objects.clear() -> int
sentai.objects.count() -> int
sentai.objects.stats() -> dict
sentai.objects.{FREE, TENTATIVE, CONFIRMED, COASTING, STALE, LOST}
```
Plus the C API in `sentai_objects.h` for direct calls from future C tasks.

**Resume recipe for L3 (`sentai.places` per ideas/objects.md):**

1. Read `ideas/objects.md` §11.B + §11.D (the two L3-relevant sections).
   Cross-reference `ideas/objects_plan.md` to see if L3 has a Stage
   number there too (objects_plan.md focuses on Stages 1-10, not the
   later place layer; L3 may be entirely from objects.md).
2. L3 deps: H3 (already vendored at L1) + sentai.objects (now shipped
   at L2).  Should be self-contained — no dependency on slam (L5),
   servo (L4), or explore (L6).
3. Mirror the L2 file layout: `sentai_places.{h,cc}` +
   `modsentai_places.c` (#include'd from both modsentai.c and
   modsentai_sim.c), .sentai_slow SDRAM placement.
4. Fresh driver test `diag/_t_02_places.py`.  Test recipe per
   Sim.md §10w (no leading underscore on the SIM copy — `import`
   needs a valid identifier).
5. Audit against agent/embeded.md before commit (operator request,
   2026-05-13).  See L2's commit body for the audit checklist.
6. FlowBaseline gate after commit ([[gate-every-layer-no-exceptions]]).
7. Atomic commit subject: `ObjectsPlan L3: sentai.places (HSV
   embedding + H3 gallery + match)`.

**Open question for the next session start:**
L3 (`sentai.places`) involves an HSV color-histogram embedding + H3
cell indexing — the broken branch may have done HSV computation in a
way that's tightly coupled to the OV5640 capture path.  Read the
broken-branch commits for `sentai_places` to understand the *concept*
but **re-implement** the HSV computation against the current camera
abstraction (see [[no-broken-branch-test-reuse]]).  Propose the
embedding shape (16 hue bins × 4 sat bins?) and capacity (places
table size) before coding, then pause for redirect.

Related: [[objectsplan]], [[flowbaseline-canonical-config]],
[[gate-every-layer-no-exceptions]], [[no-broken-branch-test-reuse]],
[[itcm-budget]] (placement rules), [[h3-integration]] (H3 vendoring),
[[flow-anchor-platform-shim]] (ARM/SIM platform-abstraction pattern).
