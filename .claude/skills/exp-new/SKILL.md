---
name: exp-new
description: Create a new experiment folder under examples/sentai_runtime/experiments/sNNN_<slug>/ with README + WBS code + pass criteria + anti-cheat note. Use when starting a new SIM or HW experiment.
---

# /exp-new

Codifies `[[experiments-in-own-folder-log-dead-ends]]`.  Every experiment is a self-contained folder with README + script + captured outputs — never `/tmp`.

## Usage

`/exp-new <slug> <OP-Sx-Wy[-Tz]> [sim|hw]`

## Steps

1. **Find next free `sNNN`**:
   ```bash
   ls /home/bogdan/work/coralmicro/examples/sentai_runtime/experiments/ | grep -E '^s[0-9]+_' | sort | tail -3
   ```
   Latest at time of writing: s183+.  Increment.

2. **Create folder** `examples/sentai_runtime/experiments/sNNN_<slug>/`.

3. **Generate top-level `README.md`** with this skeleton:

```markdown
# sNNN — <human-readable title>

**WBS**: <OP-Sx-Wy-Tz>
**Started**: <YYYY-MM-DD>
**Status**: ⬜ planning

## Claim

<What this experiment proves.  Be specific — what hypothesis, what number, what mechanism.>

## Pass criteria

- <quantitative threshold 1>
- <quantitative threshold 2>
- (SIM only) drone lands ≤ 10 cm of PHYSICAL takeoff origin — see [[sim-test-must-return-home]]
- (SIM only) anti-cheat: SentAI sensors fed ONLY by camera frames + CRTP LOG — see [[sentai-sim-air-gapped-from-truth]]
- (HW only) build counter verified post-flash + USB enumerated as 1fc9:c0a1

## How to run

```bash
bash run.sh <iter-tag>     # e.g. bash run.sh iter1
```

## Top-level files (shared across all iters)

- `mission_sNNN.py` — mission script (runs in sentai_sim per [[missions-run-in-sentai-only]])
- `verdict.py` — host-side verdict (post-mortem GT comparison allowed)
- `run.sh` — launcher; takes an iter tag argument; writes outputs into `<iter-tag>/`
- `SOTA_CITATIONS.md` — literature anchors (if applicable)

## Iter sub-folders

Each iteration goes into its OWN sub-folder.  This keeps a clean record of what each iter changed and what it produced — no more intermixed `_iter1`, `_iter5_yawXY` suffixes scattered in root.

```
sNNN_<slug>/
├── README.md            (this file — experiment-level)
├── mission_sNNN.py      (shared mission)
├── verdict.py           (shared verdict)
├── run.sh               (takes iter tag arg)
├── iter1/
│   ├── README.md        (iter-specific: hypothesis, what changed, result)
│   ├── cf2_gt.jsonl
│   ├── journal.txt
│   ├── kabsch.csv
│   ├── xyz.png
│   └── summary.json
├── iter2/
│   └── ...
└── iter5_yawXY/
    └── ...
```

## Iter results table (updated after each iter)

| Iter | Hypothesis / change | Result | Pass? |
|---|---|---|---|
| iter1 | <baseline> | <key number> | ✅/❌ |
| iter2 | <one-line what changed vs iter1> | <key number> | ✅/❌ |

## Result (final, after experiment completes)

(Filled in after the LAST iter ships.  Include: final numbers, root-cause if FAIL, link to follow-up sNNN if iter.)
```

4. **Generate `iter1/README.md` skeleton** in the first sub-folder:

```markdown
# sNNN iter1 — <one-line>

**Hypothesis**: <what this iter tests>
**Change vs prior iter**: <baseline | one-line>
**Date**: <YYYY-MM-DD>

## Files captured in this folder

- `cf2_gt.jsonl` — host-side GT recorder output
- `journal.txt` — mission log (sentai.sim.journal)
- `kabsch.csv` — paired GT vs detector samples
- `xyz.png`, `z_timeseries.png` — verdict plots
- `summary.json` — key numbers (MAE/max/RMS/residuals)

## Result

(Filled in after running.)
```

5. **Do NOT stage / commit** — wait until experiment has shipped a result.

## Iter-folder rule (operator-stated 2026-05-21)

Every iteration of an experiment MUST live in its OWN sub-folder (`iter1/`, `iter2/`, `iter5_yawXY/` ...).  Outputs (CSV, JSONL, PNG, journal, summary.json) go INSIDE that folder.  NEVER scatter `_iter<N>` suffixed files in the experiment root — operator finds it confusing.  Top-level holds only files SHARED across iters: mission, verdict, run.sh, README, citations.

When adding a NEW iter to an existing experiment:
- Create `sNNN_<slug>/iter<tag>/` (next free tag).
- Drop a fresh `README.md` in it with hypothesis + change-from-prior + date.
- Modify `run.sh` so it accepts the iter tag and writes into the sub-folder (use it as `OUT_DIR=$1` early).
- After the iter ships, update the top-level README iter table.

## Reject patterns

- `/tmp/test_*.py` (CLAUDE.md "Where experiments live")
- Missing WBS code in README header
- Missing pass criteria (vague "see if it works" is not a criterion)
- Missing anti-cheat note for SIM experiments
- Deleting a superseded experiment folder instead of adding `SUPERSEDED_BY_sNNN.md`
- Scattered `_iter<N>` suffixed files in experiment root — must be in sub-folders

## See also

- `examples/sentai_runtime/experiments/README.md` (convention origin)
- `[[experiments-in-own-folder-log-dead-ends]]` (auto-memory)
