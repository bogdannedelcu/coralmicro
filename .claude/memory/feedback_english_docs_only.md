---
name: english-docs-only
description: "HARD RULE 2026-05-17. All committed artefacts (docs, plans, code, comments, commit msgs, READMEs, PRs, memory entries) MUST be in English. Romanian allowed ONLY in live chat with operator. Final thesis is in English; project must match. Translate Romanian-on-disk on next pass through any file; never write new Romanian to disk."
metadata:
  node_type: memory
  type: feedback
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Operator-stated 2026-05-17**.

**Rule**: all documentation, plans, code, comments, commit messages,
READMEs, PR descriptions, memory entries — every artefact that gets
persisted on disk — MUST be in English.

**Why**: the final PhD thesis paper is in English; the project
artefacts must match.  Mixing languages in committed code/docs
creates friction for reviewers, harms publishability, and contradicts
the by-the-book PM discipline the operator wants applied.

**How to apply**:
- New artefacts written today: English.
- Existing Romanian text encountered while editing a file: translate
  in-place on that pass (don't accumulate translation debt).
- Live chat with operator: Romanian remains the default.  This rule
  is for *committed* artefacts, not conversation.
- Memory entries (these files in `/home/bogdan/.claude/projects/...`):
  English from now on, even though they're "private."  They influence
  future agent behavior and may be read alongside committed docs.

**Backfill status (2026-05-17)**:
- `ideas/objects_plan.md` (master index after split) — DONE.
- `ideas/objects_plan/*.md` chapter files — Romanian content still
  present in most chapters; translate on next edit pass per chapter.
- `ideas/FutureWork.md` — mostly English already; verify on next read.
- `ideas/research_review_2026_05_16.md` — likely mixed; verify.
- `ideas/objects.md` (original conceptual) — likely Romanian; defer.
- Comments in C/C++/Python source files — verify on next file edit;
  no proactive sweep.

**Cross-references**:
- Hard rule in `CLAUDE.md` (top of file)
- Related: [[wbs-pmp-2026-05-17]] (the PM-formalization push)
