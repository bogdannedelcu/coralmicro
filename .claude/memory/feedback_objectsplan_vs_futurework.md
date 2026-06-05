---
name: objectsplan-vs-futurework
description: "ObjectsPlan = what we're actively executing for PhD thesis. FutureWork.md = deferred items with explicit promotion criteria. Items move bidirectionally and dated. Implementation/design content moves; bibliography stays in ObjectsPlan for lit-review value. New ideas during work go directly to FutureWork.md to keep ObjectsPlan clean."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Operator-decided 2026-05-15** as part of PhD thesis scope freeze.

## Rule

`ideas/objects_plan.md` is the **execution plan** — what is actively
being built for the thesis demo + defense. Frozen 2026-05-15 at §23
(thesis-MVP scope).

`ideas/FutureWork.md` is the **parking lot** — items deferred beyond
thesis scope. Each item has:
- Source pointer (which `objects_plan.md` § it came from)
- Move date
- Scope summary
- "Why deferred" rationale
- "Promotion trigger" (what would need to be true to bring it back)

**Why**: Build-up over time. The plan grew to 5700+ lines covering 20
research addendums. Single-developer thesis bandwidth cannot execute
all of it. Without explicit scope freeze, planning will continue to
outrun implementation.

## How to apply

**When tempted to add a new idea / section / addendum to
`objects_plan.md`**:
1. Ask: is this on the critical path for thesis demo §23.1?
2. If YES: add to `objects_plan.md` and update §23.2 in-scope table.
3. If NO: add to `FutureWork.md` with the standard 5-field structure.
   Don't pollute the execution plan.

**When implementing items from `objects_plan.md`**:
1. If the work reveals an obvious sub-feature that's not on the
   critical path, move that sub-feature to `FutureWork.md` *before*
   you start coding it — don't drift into building extra scope.
2. If the work reveals a *missing* sub-feature that IS critical, add
   to §23.2 in-scope table with a date and update the relevant
   stage section.

**When reading `FutureWork.md` and noticing a promotion trigger
fires**:
1. Move the item OUT of `FutureWork.md`.
2. Add to `objects_plan.md` §23.2 in-scope.
3. Date the move in both files.

## What got moved on 2026-05-15 (initial cleanup)

`objects_plan.md` shrank from **5706 → ~4683 lines** (-18%) by moving:
- §16 *Multi-level honeycomb storage* (full ~540 lines) → FW8
- §17 *Cross-scale embedding composition* (full ~400 lines) → FW9
- §15.12.1 auto-merge → FW15
- §15.12.2 4D hex (temporal) → FW16
- §15.12.3 multi-drone mesh → FW7
- §14.3 Strategia C ORB+VLAD → FW3
- §14.3 Strategia D equivariant CNN → FW4
- §14.4 Mec1 SeqSLAM → FW2
- §22.3 Track B DNN places (sumar stays as reference) → FW1
- Stage 11.D multi-altitude → FW6
- Stage 7 full COAST/ALIGN → FW12 (minimal SM3 stays)
- Stage 8 standalone → FW13 (absorbed into L7)
- §13.3 Approach A macro-category detection → FW14

Removed implementation/design content. Bibliography stays in
`objects_plan.md` (lit-review value for thesis). Original sections
available in git history (`git show <pre-cleanup-commit>:ideas/objects_plan.md`).

## Related

[[gate-every-layer-no-exceptions]] — discipline complement (regression
gate after every layer commit)
[[no-broken-branch-test-reuse]] — design docs reusable but tests fresh
[[places-two-track-decision]] — Track A/B split decision §22
[[no-safety-logic-in-explore]] — separate concerns rule
[[s132-lifter-gazebo-shipped]] — last milestone before scope freeze
[[experiment-run-cadence]] — verbose(1) single trial before
multi-trial runs
