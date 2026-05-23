---
name: mp-binding-audit
description: Diff the ARM and SIM MicroPython binding files for a sentai.X module to detect divergence. Use AFTER adding/modifying a binding function — the same function must exist in both bindings/modsentai_<subsys>.c (ARM) and sim/modsentai_sim_<subsys>.c (SIM), or one platform silently lacks the symbol.
---

# /mp-binding-audit

The SentAI MicroPython API has two implementations per module — ARM (`examples/sentai_runtime/bindings/modsentai_<subsys>.c`) and SIM (`sim/modsentai_sim_<subsys>.c`).  When you add a function on one side and forget the other, mission scripts pass on the platform where it exists and silently fail (`AttributeError`) on the other.

Pattern recurred twice in 2026-05-18 sessions: `hl_stop` and `send_extpos` were added on ARM (where the operator was working at the time), discovered missing on SIM when the next experiment ran on cf2 SITL.  Same class of bug bit the W12-T5 / W13-T4 / W14-T5 binding work.

## Usage

`/mp-binding-audit <subsys>`

Examples:
- `/mp-binding-audit crazy`
- `/mp-binding-audit markers`
- `/mp-binding-audit fs`

## Recipe

For a given `<subsys>`, list every MP-exposed function on each side and diff.

```bash
SUBSYS=<subsys>
ARM=/home/bogdan/work/coralmicro/examples/sentai_runtime/bindings/modsentai_${SUBSYS}.c
SIM=/home/bogdan/work/coralmicro/sim/modsentai_sim_${SUBSYS}.c

# 1. Extract MP_DEFINE_CONST_FUN_OBJ_* names from each side
arm_funcs=$(grep -hE 'MP_DEFINE_CONST_FUN_OBJ_[0-9NV]+\s*\(\s*sentai_' "$ARM" 2>/dev/null \
    | sed -E 's/.*MP_DEFINE_CONST_FUN_OBJ_[0-9NV]+\(\s*([a-zA-Z0-9_]+).*/\1/' \
    | sed -E 's/_obj$//' | sort -u)

sim_funcs=$(grep -hE 'MP_DEFINE_CONST_FUN_OBJ_[0-9NV]+\s*\(\s*sentai_' "$SIM" 2>/dev/null \
    | sed -E 's/.*MP_DEFINE_CONST_FUN_OBJ_[0-9NV]+\(\s*([a-zA-Z0-9_]+).*/\1/' \
    | sed -E 's/_obj$//' | sort -u)

# 2. Diff
diff <(echo "$arm_funcs") <(echo "$sim_funcs")
```

Or use the MP_QSTR table directly:
```bash
# Compare exported attribute names
grep -hE 'MP_ROM_QSTR\(MP_QSTR_' "$ARM" | sort -u > /tmp/arm_qstr.txt
grep -hE 'MP_ROM_QSTR\(MP_QSTR_' "$SIM" | sort -u > /tmp/sim_qstr.txt
diff /tmp/arm_qstr.txt /tmp/sim_qstr.txt
```

Empty diff = parity.  Any line = divergence to fix.

## Decision when divergence found

| Case | Action |
|---|---|
| Function in ARM only, real impl | Add to SIM (often as stub returning `mp_const_none` if SIM has no HW backing) |
| Function in SIM only | Decide: was this SIM-only on purpose (e.g. `sentai.sim.journal_*`)?  If yes, document why and exempt.  If no, port to ARM. |
| Function exists on both but signature mismatch | Worse than missing — silent runtime arg-count error.  Reconcile. |
| Function only in ARM but operator confirms SIM-only NA | Add to SIM as a stub that returns a sensible "not available in SIM" sentinel (e.g. `mp_obj_new_int(-ENOTSUP)`) instead of leaving it missing. |

## Exempted SIM-only modules (no ARM counterpart by design)

- `sentai.sim.journal_*` (host-side post-mortem log; ARM uses `sentai.fr` instead)

If you're adding a new SIM-only module, document the exemption in the module's header comment.

## When to run

- After EVERY add to a `bindings/modsentai_*.c` or `sim/modsentai_sim_*.c` file.
- Before committing a PR that touches MP bindings.
- As part of the `add-sim-binding` skill workflow (final step).
- When chasing a mission script that "works on SIM, AttributeError on ARM" (or vice versa).

## Reject patterns

- Adding to ARM "and I'll do SIM later" — `[[no-half-finished-implementations]]` rule.  Add both or neither.
- Stubbing out SIM with `assert(false)` instead of a return value — silent crash on import.
- Skipping the audit because "I only touched one function" — that's exactly when the second side gets forgotten.

## See also

- `[[add-sim-binding]]` skill — the workflow this audit completes
- `[[qstr-regen]]` skill — required after the missing side is added
- 2026-05-18 diary entries on `hl_stop` + `send_extpos` divergence
