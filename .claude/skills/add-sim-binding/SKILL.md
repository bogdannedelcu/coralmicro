---
name: add-sim-binding
description: Scaffold a new sentai.X MicroPython binding on the SIM side (FreeRTOS POSIX build). Mirrors the ARM modsentai_<subsys>.c with a SIM-only stub or shim. Use when adding a new subsystem after the ARM binding is in place (e.g. OP-S10-W12-T5, W13-T4, W14-T5 followed this pattern).
---

# /add-sim-binding

The SIM target uses a dispatcher pattern: `sim/modsentai_sim.c` includes one fragment per subsystem (`modsentai_sim_<subsys>.c`).  Each fragment exposes the same `sentai.X.*` API as the ARM build but with a SIM implementation (UDS / file / stub).

## Existing fragments (do not rewrite)

```
sim/modsentai_sim_camera.c    — UDS reads from gz_to_uds_bridge
sim/modsentai_sim_crazy.c     — TCP/UDS to CrazySim
sim/modsentai_sim_diag.c      — host-side diag printf
sim/modsentai_sim_flow.c      — calls the same sentai_flow.cc; PXP shim
sim/modsentai_sim_fs.c        — POSIX-file-backed sentai.fs
sim/modsentai_sim_io.c        — stdin/stdout REPL
sim/modsentai_sim_journal.c   — append-mode log (sentai.sim.journal_*)
sim/modsentai_sim_link.c      — stub
sim/modsentai_sim_pipeline.c  — stub
sim/modsentai_sim_rtos.c      — pthread-backed
sim/modsentai_sim_sys.c       — exit + sleep stubs
sim/modsentai_sim_tpu.c       — libedgetpu Linux (Phase 5)
```

## Steps to add a new `sentai.<subsys>` binding

### 1. Confirm the ARM binding already lands

The convention is ARM-first, SIM mirrors.  Verify:
```bash
ls /home/bogdan/work/coralmicro/examples/sentai_runtime/bindings/modsentai_<subsys>.c
ls /home/bogdan/work/coralmicro/libs/sentai/sentai_<subsys>.{h,cc}
```

If ARM binding does not exist yet, STOP — add ARM first.

### 2. Create the SIM fragment

Create `sim/modsentai_sim_<subsys>.c`.  Copy the structure from an existing minimal fragment (e.g. `modsentai_sim_link.c` for pure stubs, `modsentai_sim_flow.c` if it can share the same `.cc`).

Three patterns by complexity:

**A — Pure stub** (function returns sentinel value):
```c
// sim/modsentai_sim_<subsys>.c — included by modsentai_sim.c
static mp_obj_t sentai_<subsys>_func(mp_obj_t arg) {
    (void)arg;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sentai_<subsys>_func_obj, sentai_<subsys>_func);
```

**B — Shared C++ impl** (calls the same `libs/sentai/sentai_<subsys>.cc` that ARM uses, when no HW dependency):
```c
extern "C" int sentai_<subsys>_do(int x);  // from sentai_<subsys>.cc
static mp_obj_t sentai_<subsys>_func(mp_obj_t arg) {
    int x = mp_obj_get_int(arg);
    return mp_obj_new_int(sentai_<subsys>_do(x));
}
```

**C — SIM-specific impl** (real impl uses UDS / file / OS feature, e.g. camera):
- Write the SIM-only `.c` body fully here.
- DO NOT use `#ifdef SENTAI_PLATFORM_SIM` in `libs/sentai/sentai_<subsys>.cc` if the fork can be done at binding level — keeps libs/ clean.

### 3. Register in the dispatcher

Edit `sim/modsentai_sim.c`:
- Add `#include "modsentai_sim_<subsys>.c"` (textual include, not separate compile unit).
- Add the qstr table entries for the new functions.
- Add the new `sentai.<subsys>` module attribute in the parent module table.

### 4. Update CMake

Edit `sim/CMakeLists.txt` — `modsentai_sim.c` is the single source file (fragments are textual include), so usually no change needed.  But if the SIM-specific impl requires linking a new library (e.g. libfftw3 for an FFT shim), add the target_link_libraries entry.

### 5. Regenerate QSTRs

Use `/qstr-regen` skill.  Without this, the new `MP_QSTR_<name>` references won't resolve.

### 6. Rebuild SIM

```bash
cd /home/bogdan/work/coralmicro
cmake --build build-sim --target sentai_sim
```

### 7. Smoke test from SIM REPL

```bash
./build-sim/sentai_sim
>>> import sentai
>>> sentai.<subsys>.func(42)
# Should not crash; should return sentinel or computed value
```

### 8. Add to ARM build dispatcher for parity (if `bindings/modsentai.c` lists modules)

```bash
grep -n MP_REGISTER_MODULE /home/bogdan/work/coralmicro/examples/sentai_runtime/bindings/modsentai.c
```
Verify the new `sentai_<subsys>` module appears.

## Hard rules

- **Same API surface ARM ↔ SIM.**  If `sentai.X.foo()` exists on ARM, it must exist on SIM (even as stub returning `mp_const_none`).  Otherwise mission scripts pass on SIM and fail on ARM (or vice versa).
- **No `#ifdef SENTAI_PLATFORM_SIM` in `libs/sentai/`** unless absolutely necessary — fork at binding layer per `[[platform-shim-pattern]]`.
- **No heavy data through MP** — the binding returns scalars / small dicts / opaque pointer-zerocopy, never frames or large buffers (`[[no-heavy-data-through-mp]]`).
- **English everywhere** on disk.

## Examples to copy from

| Pattern | Reference |
|---|---|
| Pure stub | `modsentai_sim_link.c` |
| Shared C++ impl | `modsentai_sim_flow.c` (calls `sentai_flow.cc`) |
| SIM-specific (UDS) | `modsentai_sim_camera.c` |
| SIM-specific (file) | `modsentai_sim_fs.c` (POSIX-backed FS) |
| Recent additions | OP-S10-W12-T5 (safety), W13-T4 (fr), W14-T5 (calib) |

## Reject patterns

- Skipping QSTR regen — silent missing symbol on next build.
- Forgetting CMake target_link_libraries for a new system library — link fails late.
- Adding the SIM binding before ARM — diverges API surface, hard to retro.
- `#ifdef SENTAI_PLATFORM_SIM` inside `libs/sentai/sentai_<subsys>.cc` when fork-at-binding works.

## See also

- `Sim.md` §10y "file org + modsentai_sim.c dispatcher"
- `[[qstr-regen]]` skill
- OP-S10-W12-T5 / W13-T4 / W14-T5 — recent applications of this pattern
