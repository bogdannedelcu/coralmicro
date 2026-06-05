---
name: experiments-in-own-folder-log-dead-ends
description: "HARD RULE — every experiment lives in its own folder under examples/sentai_runtime/experiments/sNNN_<name>/ with README + script(s) + captured outputs.  Reference results from there in WP docs / memory entries.  Keep logs of WRONG results / dead-ends too — they're thesis evidence of the engineering process.  Operator-stated 2026-05-20."
metadata:
  node_type: memory
  type: feedback
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

## The rule

**Every experiment — successful, failed, or methodology-corrected
later — lives in its own `examples/sentai_runtime/experiments/sNNN_<name>/`
folder.  README + script + captured output stay together.  WP docs
and memory entries REFERENCE the experiment folder, not the other
way around.**

Three reasons:

1. **Thesis evidence**.  The PhD defense argument is "structured
   engineering process produces verifiable results."  Dead-ends and
   methodology-corrected measurements ARE evidence of that process.
   The W17 → W18 clamp-bug discovery sequence is exactly the kind
   of story the defense needs — but only if the wrong numbers are
   PRESERVED alongside the corrected ones.

2. **Reproducibility**.  An on-disk experiment folder with a
   pinned bench script (host-side `.py` and any board `.py`) +
   the exact build counter + the captured stdout means future-me
   can re-run the same measurement and see if a change broke
   something.  /tmp scripts vanish on reboot.

3. **Source of truth for documentation**.  WP docs cite cycle
   counts, ratios, deltas.  Reviewers (and operator) need to be
   able to click through to the bench harness that produced
   those numbers.  Inline tables in WPs are summary; the folder
   is the archive.

## Why dead-ends matter

The W17/W18 chain showed three measurement-quality bugs in a
row, each invalidating prior numbers:

- W16 round 3 (940611e2): "M7 OCRAM 11× faster than M4 OCRAM"
  on 160×120 frame — refuted by W16 round 4 (576fc1d5) when
  the `s_binary` DCE artefact was found.
- W17-T4 (02660094): "M7 wins 27× on Flow SAD" — refuted by
  W18-T1 (5f34a8dc) when the IPC-host-clamp bug was found.
- W17-T2 (8eccf31b): "M4 WhyCon Phase A 25.86 ms" — same
  clamp bug; the real number is 20.50 ms (post-fix, build
  #1408).

Each wrong number was published as a commit message + a memory
entry + a workpackage table.  After correction, the prior write-
ups MUST be left in place (with a "SUPERSEDED" note linking to
the corrected entry) — never deleted.  The defense slides for
the embedded chapter will literally show "what we measured, what
turned out to be wrong, why, and what the corrected number was."

## How to apply

When adding a measurement experiment:

1. Choose the next free `sNNN_<name>` (current high water: s179).
2. Create the folder.  Write `README.md` first — state the
   claim, the bench method, the pass criterion, before writing
   the script.
3. Write the script(s).  Host-side Python under that folder.
4. Run the experiment.  Capture stdout to a `run_<timestamp>.log`
   in the same folder.  Capture any PGM / CSV / image artefacts.
5. After completion, write a `results.txt` summary block at
   the bottom of the README that quotes the numbers + cites
   the log filename.
6. WP docs reference `examples/sentai_runtime/experiments/sNNN_<name>/`
   for the canonical version.

When a previous experiment's result is invalidated:

1. Do NOT delete the original folder.
2. Add a `SUPERSEDED_BY_sNNN.md` note at the top of the README
   linking to the corrected experiment.
3. Keep the original log + scripts.  The dead-end is now
   thesis-evidence material.
4. The new experiment folder cites the superseded one in its
   own README intro.

## Counter-examples to avoid

- Inline cycle counts in commit messages are a SUMMARY but
  they are not the experiment.  The folder must exist
  separately.
- `/tmp/*.py` bench scripts are forbidden — they don't survive
  reboot, aren't in git, aren't reproducible.
- Re-running an experiment with a different bench harness
  silently is forbidden — bump the folder number, write a
  fresh README that cites the prior folder, capture the
  delta explicitly.

## Cross-refs

- `[[ipc-clamp-in-worker-not-host]]` — the bug that invalidated
  W17 measurements; classic example of why dead-ends must stay
  on disk.
- `[[op-s10-w16-ablation-findings-2026-05-19]]` — 4 rounds of
  M4 ablation; each round's commit message survives as the
  on-disk trail.
- experiments convention: `examples/sentai_runtime/experiments/README.md`.
