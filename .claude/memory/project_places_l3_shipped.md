---
name: places-l3-shipped
description: ObjectsPlan L3 sentai.places shipped 2026-05-14 commit 19c40d88. Frozen API + H3 wiring + ITCM placement contract for L4+ to consume.
metadata: 
  node_type: memory
  type: project
  originSessionId: ef599a0d-1126-4214-964d-31fac8efaa48
---

**Commit:** `19c40d88` on `integration/from-180bbb5f` (2026-05-14).

**Files (mirror layout for L4 sentai.servo):**
- `examples/sentai_runtime/sentai_places.{h,cc}` — 64 slots × ~96 B in
  `.sdram_bss`, statuses FREE/TENTATIVE/CONFIRMED.  Pure data + math,
  one .cc compiles for ARM and SIM via `SENTAI_PLATFORM_SIM` ifdef.
- `examples/sentai_runtime/modsentai_places.c` — MP binding,
  `#include`'d from BOTH `modsentai.c` (ARM) and `sim/modsentai_sim.c`
  (SIM).
- `examples/sentai_runtime/diag/_t_02_places.py` — fresh driver test,
  58/58 PASS in SIM.
- `examples/sentai_runtime/h3_gen/h3api.h` — pre-generated for the
  QSTR pre-pass (CMake also configures it independently into the build
  dir; this static copy unblocks the Makefile-driven QSTR step).
- `examples/sentai_runtime/sentai_error.h` — block 0x11xx
  (SERR_MOD_PLR + SERR_PLR_*) registered for L5+ ingestion logging.

**API surface (frozen — L4+ consume this):**
```python
sentai.places.add(h3_cell, desc_bytes_or_None, x=0, y=0, z=0) -> id|<0
sentai.places.get(id)                                          -> dict|None
sentai.places.observe(id)                                      -> 0|-1
sentai.places.set_status(id, status)                           -> 0|-1|-2
sentai.places.remove(id)                                       -> 0|-1
sentai.places.clear()                                          -> int
sentai.places.count()                                          -> int
sentai.places.list()                                           -> [dict, ...]
sentai.places.query(desc, h3_cell=0, k_disk=1, thresh=0)       -> dict
sentai.places.stats()                                          -> dict
sentai.places.cell_at(lat, lng, res)                           -> int (H3)
sentai.places.cell_to_latlng(h3_cell)                          -> (lat, lng)
sentai.places.neighbors(h3_cell, k=1)                          -> [int, ...]
sentai.places.{FREE, TENTATIVE, CONFIRMED}
```

**H3 wiring (L1 only vendored; L3 made it production):**
- `sim/CMakeLists.txt` adds `libh3_sim` (18 lib/*.c + configure_file
  h3api.h.in -> build-sim/sim/h3_gen/h3api.h, version macros 4/4/1).
- `examples/sentai_runtime/CMakeLists.txt` adds mirror `libh3_arm`
  target.
- `modules/sentai/micropython.mk` gets two `-I` paths (h3_gen + H3
  internal headers) so the QSTR pre-pass can resolve `#include "h3api.h"`.

**ITCM budget (per [[itcm-budget]]):**
- H3 lib `.text` is ~14 KB.  Default linker rule routed it to
  m_text/ITCM and overflowed by 14168 B.  Fix: linker script
  `MIMXRT1176xxxxx_cm7_ram_mp.ld` routes `*liblibh3_arm.a:*(.text)`
  AND `*sentai_places.cc.obj(.text)` into `.sentai_slow` SDRAM section
  alongside L2 sentai_objects + tracker.  Both are cold-path
  (queried at ≤ 1 Hz from the future explore SM, never on per-frame
  TPU path).
- Final ARM placement: `.text` 247/256 KB ITCM, `.sentai_slow` 92 KB
  SDRAM, `.sdram_text` 4 KB SDRAM.

**Concurrency contract:** single-writer single-reader (MP task) at L3.
When a future C task ingests descriptors from camera+tracker (L5+),
callers MUST serialize via an external mutex (TBD).  The static
snapshot buffer used by `sentai_places_list` and the file-scope
`s_plr_list_snap` in modsentai_places.c are part of this contract.

**Descriptor:** opaque uint8_t[64] bytes (16 hue × 4 sat bins
externally computed).  L3 doesn't touch the camera; the upper layer
(MicroPython on cf2 SIM, future C task on ARM) computes the histogram
and passes it in.

**Match:** L1 byte distance, bounded loop, no float.  Score returned
as 0..100% (100 = identical bytes).  Optional H3 cell-ring spatial
prefilter via `gridDisk(cell, k_disk)` before the L1 sweep.

**Verification at commit:**
- diag/_t_02_places.py: 58/58 PASS in SIM build #109.
- FlowBaseline gate (s127): PASS dist=10.0 cm, all4=1.0, n=18.
- ARM build PASS (link OK, ITCM healthy).

**Next layer (L4):** `sentai.servo` skeleton (Stage 4 in
[[objectsplan]]).  Action layer: intent → velocity dispatch.  No
closed loop yet.  Will mirror L2/L3 file layout.

Related: [[objectsplan]], [[objectsplan-l3-handoff]] (now historical),
[[objects-l2-shipped]], [[itcm-budget]], [[h3-integration]],
[[gate-every-layer-no-exceptions]], [[gazebo-gui-required]].
