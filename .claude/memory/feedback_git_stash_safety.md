---
name: Git stash without --include-untracked loses new files
description: Plain `git stash` only saves tracked-modified files. Untracked NEW files survive in working tree but stash drop wipes nothing extra. Stash apply then drop = irreversible loss for tracked work.
type: feedback
originSessionId: 23ce703b-532f-42c3-ac8e-f35fa410241e
---
**Rule:** before `git stash drop`, ALWAYS verify the working
tree has the changes restored.  `git stash apply` followed by
`git stash drop` looks safe but if the apply silently failed
to apply some files (no explicit conflict in the output), the
drop is irreversible loss of work.

**Why:** During 2026-05-05 session I lost ~1500 lines of new
flow_task.cc / modsentai_flow.c / agent.md / experiment.md
content because `git stash apply` reported success but the
working tree was actually still on the baseline version.
`git stash drop` then deleted the only copy.

**Recovery:** `git fsck --lost-found` shows dangling commits;
`git ls-tree <sha>` finds blob hashes; `git cat-file -p <blob>
> file` recovers content of TRACKED files.  Unfortunately
plain `git stash` does NOT include untracked files, so any new
files (not previously committed) are lost forever -- they were
never in any commit.

**How to apply:**
- Use `git stash -u` (or `--include-untracked`) if any of your
  current uncommitted work is in NEW files.
- After `stash apply`, verify a representative file with `head`
  before doing `stash drop`.
- For long sessions with significant uncommitted work,
  `git stash` (single command) is dangerous.  Prefer creating
  an explicit WIP commit instead.
