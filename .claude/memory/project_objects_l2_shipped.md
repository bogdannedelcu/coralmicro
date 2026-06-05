---
name: objects-l2-shipped
description: "ObjectsPlan L2 (sentai.objects data layer) shipped 2026-05-13 commit eaf67e75 — API surface, file layout, NASA/JPL hardening notes for L4+ ingestion to consume."
metadata: 
  node_type: memory
  type: project
  originSessionId: cfa3374f-a472-4f14-be65-3858146f8b62
---

L2 of [[objectsplan]] shipped 2026-05-13 on `integration/from-180bbb5f`,
commit `eaf67e75`.  FlowBaseline gate PASS (dist=9.3 cm vs 15 cm threshold).

**Why this exists**: future layers (L4 servo, L5 slam, L6 explore) will
ingest into and read from `sentai.objects`.  This memory documents the
contract those layers consume, so they don't have to re-derive it from
the source.

**API surface** (frozen — change only via a new memory):
```python
# Storage (returns id >0 / -1 NaN / -2 class>=80 / -3 full+no-evict)
sentai.objects.add(class_id, x, y, z, cov6=None) -> int

# Read (None on miss)
sentai.objects.get(id) -> dict | None
sentai.objects.list() -> [dict, ...]
sentai.objects.count() -> int

# Mutate (0 ok / negative on bad id or status)
sentai.objects.mark_visited(id) -> int
sentai.objects.set_status(id, status) -> int
sentai.objects.remove(id) -> int

# Lifecycle / introspection
sentai.objects.clear() -> int        # returns # cleared
sentai.objects.stats() -> dict {used, hwm, capacity, confirmed, coasting,
                                stale, adds, gets, removes, evictions,
                                oob_rejected, bad_ids, cov_clamped}

# Constants
sentai.objects.{FREE=0, TENTATIVE=1, CONFIRMED=2, COASTING=3, STALE=4, LOST=5}
```

**C API** (for L4+ ingestion from C tasks): see `sentai_objects.h`.
Same shape, same semantics.

**Concurrency contract**: L2 callers are MP-task-only.  When L4 wires
ingestion from sentai_tracker (different task), the caller MUST
serialize via an external mutex — internal `g_objects` array and the
file-static `s_obj_list_snap` snapshot buffer in `modsentai_objects.c`
both rely on this.

**File layout** (pattern for L3+ to mirror):
- `examples/sentai_runtime/sentai_objects.{h,cc}` — pure data + math, no HW.
- `examples/sentai_runtime/modsentai_objects.c` — MP binding,
  `#include`'d from BOTH `modsentai.c` and `sim/modsentai_sim.c`.
- ARM linker placement: `*sentai_objects.cc.obj(.text/.data/.rodata)`
  added to the `.sentai_slow` section in
  `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`.
- State arrays tagged `__attribute__((section(".sdram_bss")))` on ARM
  via `SENTAI_OBJ_SDRAM_BSS` macro (no-op on SIM).
- Error code block: `0x10xx` (SERR_MOD_OBJ + SERR_OBJ_*) in
  `sentai_error.h`.  L4+ should `SERR_LOG(SERR_OBJ_*, val)` when
  ingestion-from-C-task hits a fault.

**Hardened per agent/embeded.md** (operator audit-on-write request,
2026-05-13).  System model documented inline at the top of
`sentai_objects.h` (fault / execution / recovery / concurrency / memory
sections).  Mirror that doc style for L3+ headers.

**Driver test**: `diag/_t_01_objects.py` — 38 PASS / 0 FAIL on SIM.
Recipe for SIM REPL test execution: drop file (without leading `_`)
into `build-sim/sentai_fs_root/`, then `import t01_objects` from REPL.
See Sim.md §10w.

Related: [[objectsplan]], [[objectsplan-l3-handoff]],
[[flowbaseline-canonical-config]], [[itcm-budget]],
[[no-broken-branch-test-reuse]].
