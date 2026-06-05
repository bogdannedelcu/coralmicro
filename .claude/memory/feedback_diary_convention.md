---
name: diary-convention-2026-05-18
description: "Daily Captain's-log convention started 2026-05-18 (operator: 'as vrea sa notam sa persistem istoria').  One file per session day at diary/YYYY-MM-DD.md, English only, honest accounting of attempted/worked/failed/queued.  Pre-existing handoff docs (e.g. op_s8_w1_handoff.md from 2026-05-17) moved into the diary retroactively.  Tone: complete sentences a cold reader can pick up, no shorthand."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

Operator-instituted 2026-05-18 with: *"as vrea sa notam sa
persistem istoria chiar daca uneori ajungem in blocaje"*.

## Format

- Path: `diary/YYYY-MM-DD.md` (one entry per session day).
- Title: `# Captain's log — YYYY-MM-DD`.
- Body sections (loose, adapt to the day):
  - Starting point (state at session open).
  - Plan for today (lanes / priorities).
  - Done this session (what shipped, links to commits).
  - Open / next session.
  - Honest "what was hard" / "what was good" if relevant.
- Always English (per [[english-docs-only]]).
- Reference WBS codes (`OP-S{N}-W{M}-T{K}`) when discussing work.

## Why: How to apply

**Why**: operator wants the project's narrative to survive
context compaction + multi-day breaks.  Captain's-log style
> traditional changelog because it captures **reasoning** and
**dead-ends**, not just shipped artefacts.  Anyone (operator or
future Claude) reading entries a month later can reconstruct the
decision tree.

**How to apply**: at session end, write that day's `diary/`
entry before context compacts.  Pre-compaction is the highest-
leverage time — the lessons are still vivid.  Don't wait until
the next session to draft retroactively.

## Retroactive migration

Files that pre-date the convention but read as Captain's logs
should be moved with a header note acknowledging the
retro-classification.  Already done: `ideas/op_s8_w1_handoff.md`
→ `diary/2026-05-17.md` (2026-05-19).  Pattern: keep body
verbatim, add a `> Note: this entry pre-dates the diary/
convention...` blockquote at the top.

## Current state (2026-05-19)

- `diary/2026-05-17.md` — cf2 SITL anti-cheat crisis (OP-S8-W1).
- `diary/2026-05-18.md` — W12 + W13 + W14 binge + T22 ARM build
  late-night addendum.
- `diary/2026-05-19.md` — today's plan (lanes A-E), big picture
  to defense, W15 ARM memory budget filed.
