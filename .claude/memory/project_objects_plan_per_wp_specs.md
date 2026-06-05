---
name: objects-plan-per-wp-specs-2026-05-19
description: "Per-WP spec file convention adopted 2026-05-19: ideas/objects_plan/OP-S{N}-W{M}_<slug>.md.  wbs.md becomes a table of contents (≤ ~250 lines, one-line entries + links).  Detail (motivation, scope, task contract, design invariants, MP examples) lives in the per-WP file.  First applied: ideas/objects_plan/OP-S10-W11_sentai_prep.md.  Future passes rename legacy numeric-prefix files (00_methodology.md..16_sentai_calib_autotune.md) to match the OP-S{N}-W{M}_<slug>.md convention; meta files (methodology, bibliography, namespace audit) under _meta/ prefix."
metadata: 
  node_type: memory
  type: project
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

Operator-requested 2026-05-19.  Captured as **Lane E** in the
`diary/2026-05-19.md` plan.

## Why: How to apply

**Why**: `ideas/wbs.md` grew to ~668 lines as W14 added inline
prose for each new task.  Hard to scan as a project index.

**How to apply**: when adding non-trivial scope to a WP, create
or update its dedicated spec file under `ideas/objects_plan/`.
Keep `wbs.md` entries to one-line summaries + relative links.

## Naming

```
ideas/objects_plan/OP-S{N}-W{M}_<slug>.md
```

Examples (mapping from legacy numeric-prefix names — apply when
the file is next touched):

| Legacy                              | Canonical                              |
|-------------------------------------|----------------------------------------|
| `14_sentai_safety.md`               | `OP-S10-W12_sentai_safety.md`          |
| `15_sentai_fr.md`                   | `OP-S10-W13_sentai_fr.md`              |
| `16_sentai_calib_autotune.md`       | `OP-S10-W14_sentai_calib_autotune.md`  |
| `11_camera_calib.md`                | `OP-S6-W1_camera_calib.md`             |
| `12_places_two_track.md`            | `OP-S10-W4_places_two_track.md`        |

Non-WP files (cross-cutting): under `objects_plan/_meta/`:
- `00_methodology.md` → `_meta/methodology.md`
- `10_bibliography.md` → `_meta/bibliography.md`
- `09_namespace_audit.md` → `_meta/namespace_audit.md`

## Acceptance criteria (for the full Lane E refactor)

1. `ideas/wbs.md` ≤ ~250 lines (currently 668).
2. Every non-trivial WP has a `OP-S{N}-W{M}_<slug>.md` spec.
3. `git mv` for renames so history follows.
4. Broken-link grep clean:
   `grep -rn "objects_plan/[0-9][0-9]_" ideas/ docs/` returns
   nothing.
5. `CLAUDE.md` / `agent.md` cross-refs updated.

## First applied (2026-05-19)

`ideas/objects_plan/OP-S10-W11_sentai_prep.md` — PrepTask spec
covering Path A (TPU staging OCRAM) + Path B (aux slots SDRAM),
cadence, refcount, ARM vs SIM, MP init example, design
invariants.  Linked from `wbs.md` OP-S10-W11 entry.
