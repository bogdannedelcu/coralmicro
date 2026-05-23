---
name: qstr-regen
description: Regenerate MicroPython embed QSTRs after touching modsentai_*.c or MP_REGISTER_MODULE. Use after ANY change to a binding file that adds a new MP_QSTR_* reference. Failure mode is silent missing symbol on SIM build or runtime AttributeError on ARM.
---

# /qstr-regen

After any change in `examples/sentai_runtime/bindings/modsentai_*.c` that adds a new `MP_QSTR_<name>` reference, the embed-generated QSTR table is stale.  Symptoms:
- SIM build: `error: 'MP_QSTR_<name>' undeclared`
- ARM runtime: `AttributeError: 'module' object has no attribute '<name>'`

## Recipe (canonical, idempotent)

```bash
cd /home/bogdan/work/coralmicro/examples/sentai_runtime
rm -rf build-embed
make -f ../../third_party/micropython/ports/embed/embed.mk \
    MICROPYTHON_TOP=../../third_party/micropython \
    USER_C_MODULES=$(pwd)/modules \
    micropython-embed-package
```

Then rebuild SIM + ARM normally (`bash build.sh` / sim cmake).

## Verify

```bash
grep -r 'MP_QSTR_<your_new_name>' /home/bogdan/work/coralmicro/examples/sentai_runtime/micropython_embed/
```

Should find it in `genhdr/qstrdefs.collected.h`.

## Triggers (run /qstr-regen if you did any of these)

- Added a new `MP_DEFINE_CONST_FUN_OBJ_*` macro in `modsentai_*.c`.
- Added a new `{ MP_ROM_QSTR(MP_QSTR_<name>), ... }` table entry.
- Added a new `MP_REGISTER_MODULE(MP_QSTR_<name>, ...)`.
- Renamed an existing binding.

## Reject patterns

- Editing the generated files in `micropython_embed/` by hand.
- Skipping the regen because "the build seemed fine" — the symbol resolution failure can lurk until first use.

## See also

- `examples/sentai_runtime/agent/agent.md` §6 (authoritative recipe)
- `embeded.md` §6.3 "Tool qualification" — generated code regen must be reproducible
