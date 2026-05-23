---
name: wbs-task
description: Resolve a WBS code from ideas/wbs.md and format a commit subject or check that a planned task exists. Use when staging a commit, opening a new task, referencing a WBS code in docs, or before claiming a new task ID.
---

# /wbs-task

WBS hard rule (CLAUDE.md + `ideas/wbs.md` §4): every commit subject, plan artifact, experiment README, memory entry MUST reference its canonical WBS code from `ideas/wbs.md`.

## Usage

`/wbs-task <code> [: <one-line summary>]`

Examples:
- `/wbs-task OP-S10-W20` — verify code exists, show what it covers
- `/wbs-task OP-S10-W20-T1: extract jacobi_sym3+svd3 from sentai_calib.cc` — format commit subject

## Steps

1. **Look up the code** in `ideas/wbs.md`:
   ```bash
   grep -n "^[│ ]*├── OP-S10-W20\|OP-S10-W20-T1" /home/bogdan/work/coralmicro/ideas/wbs.md
   ```
2. **If the code does NOT exist:**
   - Verify the parent WP/Stage exists.
   - Propose the next free `-T{K}` (append-only — never reuse retired numbers).
   - STOP and ask the operator to confirm before claiming a new code.
3. **If the code exists:**
   - Print the line where it appears + its status (✅ / 🟡 / ⬜).
   - If a summary was given, output the commit subject: `<code>: <summary>` (target ≤ 70 chars).
4. **Surface hard-rule reminders** if the user is about to commit:
   - Append-only (never renumber existing codes — they are stable IDs like JIRA tickets).
   - No ad-hoc letters (`Stage X.A`, `A9`, `W5`).
   - Multi-WBS commits format: `<primary>: <one-line> + <secondary>`.
   - Adding a new Stage (`OP-S11+`) requires explicit operator approval (10-stage roadmap is frozen).

## Reject patterns

- Claiming a code not in `wbs.md` without operator confirmation.
- Renumbering — even "to clean up the order".
- Using `A1..A8` / `Stage X.A` / bare `W{N}` codes (those are legacy; see `wbs.md` §3 mapping table).
- Skipping the WBS prefix on a commit subject that touches firmware or plans.

## See also

- `ideas/wbs.md` §1 (canonical axes) + §2 (current tree) + §4 (hard rule)
- `CLAUDE.md` "Project plan numbering"
