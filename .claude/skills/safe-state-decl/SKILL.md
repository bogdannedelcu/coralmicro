---
name: safe-state-decl
description: Generate a Safe State declaration for a subsystem per embeded.md §1.4 (ISO 26262). Use when introducing a new subsystem (task, driver, state machine) or refactoring an existing one to add explicit safe state.
---

# /safe-state-decl

Per `examples/sentai_runtime/agent/embeded.md` §1.4 (ISO 26262 Safe State per subsystem).

**Not "stop"** — specifically *which outputs, buffers, timers, and downstream effects are quiesced*.  Camera safe ≠ flow safe ≠ motor safe ≠ FR drain safe.  Safe state must itself be reachable from any healthy state within FTTI (§4.2).

## Usage

`/safe-state-decl <subsystem-name>`

## Output template (paste into the subsystem's header or `paper/<name>_safety.md`)

```markdown
## Safe state — <subsystem>

| Aspect | Specification |
|---|---|
| **Outputs quiesced** | <every output sink, named: motor cmd, USB packet, FR channel, GPIO state, ...> |
| **Buffers** | <flushed? frozen? overwritten with safe sentinel?> |
| **Timers** | <stopped? re-armed to recovery interval? left running for heartbeat?> |
| **Downstream effects** | <consumers notified via health-state update? events emitted?> |
| **Entry triggers** | <conditions that force entry: fault counter ≥ N, deadline miss, supervisor decision, operator command> |
| **FTTI** | <max ms from trigger detection to fully-quiesced state — quantitative, not "fast">  |
| **Recovery path** | <how does subsystem leave safe state? operator command? boot? timer re-arm?> |
| **Diagnostic coverage** | <% of credible failures that lead here vs slip past undetected> |
```

## Examples in this codebase

- `sentai.safety` (OP-S10-W12): safe state = `STATE_ABORT` → cancels mission, motors disarmed via servo backend, FR captures the trigger frame.
- `sentai.fr` (OP-S10-W13): safe state = drain pending queue → close sinks atomically → leave breadcrumb.
- `sentai_calib_autotune` (OP-S10-W14): safe state = stop the relay, freeze last good Kp, surface failure cause via `is_done()` API.

## Checklist before committing the declaration

- [ ] Outputs explicit, not "stops" — name every sink.
- [ ] FTTI quantified (ms or cycles, not "fast" / "soon").
- [ ] Safe state is reachable from EVERY healthy state (not just nominal).
- [ ] No silent failure: any failure not covered is listed as a residual risk (file as `RISK-{NN}`).
- [ ] Recovery escalation per §1.3 (6 levels: retry / re-init / restart / degraded / safe / reset).
- [ ] USB CDC stays alive in the safe state (anti-brick per §10).
- [ ] Doc lands in `paper/<subsystem>_safety.md` OR in the subsystem header.

## See also

- `embeded.md` §1.4 (canonical definition)
- `embeded.md` §4.2 (FTTI quantification)
- `embeded.md` §5 (fault management — module health states)
- Safety.md (project Safety architecture, sentai.safety reference)
