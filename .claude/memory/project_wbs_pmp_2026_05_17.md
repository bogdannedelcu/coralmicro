---
name: wbs-pmp-2026-05-17
description: "Canonical WBS for ObjectsPlan thesis. OP / OP-S{N} (stage 1..10 FROZEN per §3) / OP-S{N}-W{M} (work pkg) / OP-S{N}-W{M}-T{K} (task) / OP-M{N} (milestone). Orthogonal: ARCH-L{n}, EXP-s{NNN}, FW-{NN}, RISK-{NN}, F-AC-{NN}, §N.M. Append-only IDs. No ad-hoc letters (Stage X.A / A9 / W5). Commit subject prefix MUST be the WBS code. See ideas/wbs.md for full spec + legacy mapping table."
metadata:
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Approved by operator 2026-05-17**.  Replaces the prior numbering
chaos (Stage 1-10 + Stage 4.A + L1-L7 + A1-A8 + W1-W4 + Track A/B all
intermixed without hierarchy) with a single PMP-aligned scheme.

## Hierarchy (primary WBS axis)

- `OP` — Project (ObjectsPlan thesis, just one)
- `OP-S{1..10}` — Stage (matches `objects_plan.md` §3 roadmap, FROZEN)
- `OP-S{N}-W{M}` — Work Package (~ 1 week of owner-hours)
- `OP-S{N}-W{M}-T{K}` — Task (atomic, ≤ 1 day, trackable in TaskCreate)
- `OP-M{1..5}` — Project milestone (cross-stage gate)

## Orthogonal axes (separate dimensions)

- `ARCH-L{1..7}` — Architecture code-layer (different from Stage)
- `EXP-s{NNN}` — Experiment number (append-only, sNNN folders kept)
- `FW-{NN}` — FutureWork item (FW1..FW17 in FutureWork.md, append-only)
- `RISK-{NN}` — Risk register entry
- `F-AC-{NN}` — Anti-cheat audit feature
- `§{N.M}` — objects_plan.md doc-section reference (stable across split)

## Rules (hard)

1. **Append-only**: never renumber / rename / reorder.  Codes are
   ticket-IDs; renames break cross-refs across memory + docs + commits.
2. **No ad-hoc letters**: do NOT invent `Stage X.A`, `A9`, `W5`.
   Append the next `OP-S{N}-W{M}-T{K}`.
3. **Stage 1-10 FROZEN** per `objects_plan.md` §3.  Adding `OP-S11+`
   requires explicit operator approval.
4. **Commit subject prefix MUST be the WBS code**, e.g.
   `OP-S6-W1-T4: EXP-s157 calib smoke — perturb ±5° → recover 0.3°`.
5. **Memory entries**: front-load the WBS code in the description.
6. **Experiment READMEs**: header lists which `OP-S{N}-W{M}` they
   validate.  The `sNNN_<name>/` folder convention is preserved.
7. **Calendar `W1..W4`** is OK as a scheduling tag only — must be
   paired with a WBS code in any Gantt-style table.

## Most relevant active codes (2026-05-17 snapshot)

- `OP-S6-W1` — sentai.calib MP binding (DEFERRED until docs done)
- `OP-S10-W4` — HSV descriptor (week 2 calendar)
- `OP-S10-W5` — FFT-mag log-polar (week 3 calendar)
- `OP-S10-W6` — Opposite-direction recall (week 3 calendar)
- `OP-S10-W7` — L1 tracker minimal (week 4 calendar)
- `OP-S9-W1..W5` — ARM bring-up (post-calib track)
- `OP-S10-W8` (=`OP-M3`) — Indoor L7 demo
- `OP-S10-W9` (=`OP-M4`) — Outdoor PX4
- `OP-S10-W10` (=`OP-M5`) — Evaluation chapter

## Why this matters

Without canonical codes you cannot do cross-doc search reliably:
"where is A4 mentioned" hits prose `A4`, paper sizes, equation labels.
With `OP-S6-W1` grep-ability becomes deterministic, commits link to
plan items, memory entries link to commits, etc.  By-the-book PMP
artifact discipline, mirroring NASA/JPL discipline we already apply on
the code side.

## Cross-references

- Spec: `ideas/wbs.md` (canonical doc with full mapping table)
- Hard rule in CLAUDE.md (top of file, near anti-cheat rule)
- Stage roadmap: `objects_plan.md` §3 (post-split: chapter 02)
- Thesis scope: `objects_plan.md` §23 (post-split: hoisted into index)
- FutureWork: `FutureWork.md` (FW-{NN} entries)
- Related: [[short-term-plan-2026-05-17]], [[objectsplan-vs-futurework]]
