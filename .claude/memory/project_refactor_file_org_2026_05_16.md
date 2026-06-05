---
name: refactor-file-org-2026-05-16
description: "T0 + T1 refactor of MicroPython binding files. SIM modsentai_sim.c (1684 LoC, 14 modules) split into 12 modsentai_sim_<name>.c fragments + 164-line dispatcher. ARM 31 modsentai_*.c moved from examples/sentai_runtime/ flat root into examples/sentai_runtime/bindings/. Both bindings stay #include'd into their dispatcher (single TU, static linkage preserved). Required -I path updates: micropython.mk + sim/CMakeLists.txt. ARM build + SIM smoke both PASS post-refactor. T2 (split sentai_crazy.cc + sentai_runtime.cc monoliths) follow-up."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

## What changed

### T0: SIM split (`sim/modsentai_sim.c`)
- Was: 1684-line monolith, 14 modules in one file.
- Now: 164-line dispatcher (`modsentai_sim.c`) + 12 fragments named
  `modsentai_sim_<name>.c` (io, rtos, diag, sys, fs, journal, camera,
  flow, tpu, pipeline, link, crazy).  All `#include`d into the
  dispatcher → still ONE translation unit, no header files needed, all
  `static` linkage preserved.

### T1: ARM bindings into `bindings/`
- Was: 31× `modsentai_*.c` flat at `examples/sentai_runtime/` top
  level (alongside 41× `sentai_*.{cc,h}` impls = 92 total files at
  one dir).
- Now: 31 in `examples/sentai_runtime/bindings/`.  Dispatcher
  `modsentai.c` updated to `#include "bindings/modsentai_X.c"`.
- Cross-tree SIM includes updated in `sim/modsentai_sim.c`:
  `#include "../examples/sentai_runtime/bindings/modsentai_X.c"`.

## Build-system changes (LOAD-BEARING)

| File | Change | Why |
|---|---|---|
| `examples/sentai_runtime/modules/sentai/micropython.mk` | `+CFLAGS_USERMOD += -I$(SENTAI_MOD_DIR)/../..` | bindings/modsentai_X.c unqualified `#include "sentai_X.h"` needs parent dir on QSTR pre-pass include path |
| `sim/CMakeLists.txt` | `+${CMAKE_SOURCE_DIR}/examples/sentai_runtime` in `target_include_directories(sentai_sim PRIVATE)` | same problem, SIM cmake side |
| `examples/sentai_runtime/CMakeLists.txt` | NO CHANGE — already had `${CMAKE_CURRENT_SOURCE_DIR}` covering this | ARM cmake already correct |

## Verification

- SIM build #131 PASS, smoke `t_crazy_smoke.py` PASS.
- ARM `cmake --build build --target sentai_runtime` PASS (1018 KB text,
  25 MB ELF — no size regression vs pre-refactor).
- QSTR regen runs clean (no errors, no missing symbols).

## File-naming convention (codified)

| Pattern | Where | Owner |
|---|---|---|
| `modsentai_<name>.c` | `examples/sentai_runtime/bindings/` | ARM MicroPython binding |
| `modsentai_sim_<name>.c` | `sim/` | SIM MicroPython binding |
| `sentai_<name>.cc` + `.h` | `examples/sentai_runtime/` | shared impl (ARM + SIM via #ifdef) |
| `sentai_<name>_sim.cc` | `sim/` | SIM-only impl |
| `<feature>_task.cc` | `examples/sentai_runtime/` | FreeRTOS task body (ARM) |

## The two ironclad rules

1. **Bindings are `#include`d, NEVER compiled stand-alone.**  Single
   TU per dispatcher keeps `static` linkage intact across fragments.
2. **One impl file owns each subsystem; bindings only call into it.**
   MP binding is a thin facade.  Hot code in `sentai_<name>.{cc,h}`.

## T2 (still pending) — split monoliths

Two large `.cc` files left unsplit:
- `sentai_runtime.cc` (3321 LoC) — boot + tasks + USB + pipeline glue.
  HIGH RISK to split (shared `static` state set up in init); defer
  until Stage 9 ARM bring-up debugging forces it.
- `sentai_crazy.cc` (2007 LoC) — 5 zones: CPX framing, CRTP TX, CRTP
  RX dispatcher, log subscription, UART backend.  MEDIUM RISK; clean
  internal boundaries make this the next candidate.

## Anti-patterns documented in Sim.md §10y

- Cross-file `static`: promote to `sentai_<utility>.{cc,h}` instead.
- Reordering fragment `#include`s without verification (later
  fragments may reference earlier `static` symbols).
- Linker section moves: `MIMXRT1176xxxxx_cm7_ram_mp.ld` references
  specific `<file>.cc.obj` patterns for `.sentai_slow` placement
  (`sentai_phog.cc`, `sentai_gist.cc`, `sentai_object_lifter.cc`).
  Update the ld script when moving these.

## Related

- `[[sim-repl-test-recipe]]` — drop `.py` in `build-sim/sentai_fs_root`, single `import`
- `[[missions-run-in-sentai-only]]` — bindings facade for mission code
- `[[s142-hex-patrol-shipped]]` — example test exercising places binding
- CLAUDE.md §QSTR regen — recipe untouched by refactor
- Sim.md §10y — full convention doc
