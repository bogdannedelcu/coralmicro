---
name: fault-model
description: HARA-backward fault enumeration for a module per embeded.md §1.1. Use when starting a new WP that introduces a subsystem, or before a non-trivial refactor of an existing one. Produces a fault table, not code.
---

# /fault-model

Per `embeded.md` §1.1 (ISO 26262 HARA backward reasoning): "what's the worst real-world consequence?" → "where does mitigation live?"

## Usage

`/fault-model <module-or-subsystem>`

## Steps

1. **Enumerate credible faults across all layers** for the module:
   - **HW**: peripheral failure, supply glitch, clock loss, EMC, thermal
   - **Peripheral**: I2C NACK, SPI underrun, CSI sync loss, USB enumeration fail, UART framing error
   - **Comms**: dropped/corrupted UART byte, radio link loss, USB stall, lwIP HTTP timeout
   - **Timing**: missed deadline, jitter, ISR storm, priority inversion, watchdog near-miss
   - **Memory**: stack overflow, heap exhaustion, DMA into unmapped region, buffer overrun, cache coherency
   - **Lockup**: deadlock, livelock, infinite loop
   - **Invalid state transition**: FSM forced into illegal state, race in event delivery
   - **Runtime / scripting**: MP exception, MP heap OOM, untrusted radio input, garbage qstr lookup

2. **Fill the fault table:**

```markdown
| Fault | Detection mechanism | Worst consequence | Mitigation location | Recovery level (§1.3) |
|---|---|---|---|---|
| <name> | <counter / WDOG / return code / supervisor> | <unmitigated outcome> | <which module owns the fix> | <1: retry / 2: re-init / 3: restart / 4: degraded / 5: safe / 6: reset> |
```

3. **Mark RESIDUAL RISKS** — credible faults with no mitigation today.  File each as a `RISK-{NN}` entry per WBS axis (append-only).

4. **Validate FFI** (§4.4) — does any new fault path violate Freedom From Interference between criticality levels?  E.g. SafetyTask must NOT be blocked by lower-priority FR drain.

## Output destination

Append to:
- `paper/<module>_safety.md` (if module already has a design doc), or
- `ideas/objects_plan/OP-S{N}-W{M}_<slug>.md` (in a `## Fault model` section), or
- The WP's section in `ideas/wbs.md` (for short modules).

## Reject patterns

- "It can't fail" — every module CAN fail; if you can't enumerate any fault, the model is incomplete.
- Mitigation = "log and continue" without a recovery level — that's silent fault swallowing (§2 rule 5).
- Skipping memory/lockup classes because "we're a small module" — RTOS shared state catches small modules too.

## See also

- `embeded.md` §1.1 (fault model definition)
- `embeded.md` §1.3 (recovery escalation hierarchy)
- `embeded.md` §4.3 (diagnostic coverage)
- `embeded.md` §4.4 (FFI)
