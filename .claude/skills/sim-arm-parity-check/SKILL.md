---
name: sim-arm-parity-check
description: Verify that an algorithm/feature targeted at SIM also builds, links, and behaves identically on ARM. SIM never reports m_text/m_data overflow, linker routing gaps, ITCM placement violations, or QSTR table staleness — all of which are common ARM-only failure modes after a SIM-driven sprint. Use after any C/C++ change that adds new code paths in examples/sentai_runtime/, BEFORE declaring a feature shipped or committing to main.
---

# /sim-arm-parity-check

Bridges the "works on SIM, fails on ARM" gap.  SIM build = x86 hosted,
no ROM/RAM region budgets, no linker placement rules, no QSTR table
regen, no ITCM hot/cold separation.  ARM = strict regions, hard
ITCM/m_text/m_data budgets, mandatory linker routing for cold-path
modules, QSTR table must be regenerated.

If you just made a SIM-side change work, run this BEFORE saying
"done" — it catches:

1. **ITCM (`m_text`) overflow** — new cold-path code lands in ITCM by
   default; must be routed to `.sentai_slow` (SDRAM) via the linker
   script.  Memory: [[itcm-budget]].
2. **`.bss` / `m_data` overflow** — new static buffers ≥ 1 KB must be
   tagged `__attribute__((section(".sdram_bss")))` (see also skill
   `section-decision`).
3. **Stale QSTR table** — new `MP_QSTR_<name>` references in
   `modsentai_*.c` need the embed regen (skill `qstr-regen`).
4. **MP binding divergence between ARM and SIM** — if the binding has
   a SIM-only mirror (`sim/modsentai_sim_<subsys>.c`), the function
   list must match (skill `mp-binding-audit`).
5. **SIM/ARM CMakeLists divergence** — new `.cc` files must be added
   to BOTH `examples/sentai_runtime/CMakeLists.txt` (ARM) and
   `sim/CMakeLists.txt` (SIM).  Forgetting either side gives "works
   on one, missing symbol on the other".
6. **Float-precision drift** — SIM is x86 with double FPU; ARM M7 is
   single-precision (Cortex-M4 has FPU too but slower).  Algorithms
   that converged tightly in SIM may show wider residuals on ARM if
   any `double` slipped in.  Grep for `double` / non-`f`-suffixed
   literals / `sqrt` / `sin` / etc.
7. **`#ifdef SENTAI_SIM` leaks** — per Sim.md §2 rule 2, core SentAI
   source must NOT contain `#ifdef SENTAI_SIM`.  Feature-detect via
   compile-time flags or runtime backend selection.

## Recipe

```bash
cd /home/bogdan/work/coralmicro

# Step 1 — SIM build (validate algorithm-level correctness).
cmake --build build-sim --target sentai_sim -j$(nproc) 2>&1 | tail -5
# Run the SIM smoke / regression that exercises the new code.
# bash examples/sentai_runtime/experiments/sNNN_<name>/run.sh

# Step 2 — ARM build (catches m_text/m_data overflow, missing routing).
cmake --build build --target sentai_runtime -j$(nproc) 2>&1 | tail -10
# Failure modes to watch:
#   "region m_text overflowed" -> route the file to .sentai_slow in
#       examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld
#   "region m_data overflowed" -> add __attribute__((section(".sdram_bss")))
#       to the offending static (see skill section-decision)
#   "undefined reference to MP_QSTR_<name>" -> run skill qstr-regen
#   "undefined reference to sentai_<func>" -> file missing from
#       examples/sentai_runtime/CMakeLists.txt

# Step 3 — CMakeLists parity (no symbol drift between targets).
diff <(grep -oE 'sentai_[a-z_0-9]+\.cc' \
        /home/bogdan/work/coralmicro/examples/sentai_runtime/CMakeLists.txt | sort -u) \
     <(grep -oE 'sentai_[a-z_0-9]+\.cc' \
        /home/bogdan/work/coralmicro/sim/CMakeLists.txt | sort -u)
# Empty diff = good.  Any line = file present in only one target.

# Step 4 — MP binding parity (if a SIM mirror exists).
# Skill: mp-binding-audit  (diffs ARM bindings/ vs sim/modsentai_sim_*).

# Step 5 — Float-precision sweep on new code.
git diff --name-only HEAD~1 -- 'examples/sentai_runtime/sentai_*.cc' | xargs -r \
  grep -nE '\b(double|sqrt|sin|cos|tan|atan2|fabs|exp|log|pow)\(' | \
  grep -v 'sqrtf\|sinf\|cosf\|tanf\|atan2f\|fabsf\|expf\|logf\|powf'
# Each hit is a potential double-promotion on ARM.

# Step 6 — Section-attribute sweep on new statics.
git diff HEAD~1 -- 'examples/sentai_runtime/sentai_*.cc' | \
  grep -E '^\+.*static.*\[' | grep -v 'attribute__'
# Static arrays without explicit section: candidate for sdram_bss
# review per skill section-decision.

# Step 7 — Anti-cheat audit (SIM-only feature creep into ARM scope).
bash sim/scripts/audit_anti_cheat.sh
```

## Pass criteria

- SIM smoke: PASS (algorithm-level correctness).
- ARM build: clean, no overflow, no undefined refs.
- CMakeLists diff: empty.
- Float-precision sweep: zero non-`f`-suffix math calls in new code.
- Section sweep: every new static ≥ 1 KB has an explicit section
  attribute (or has been reviewed via `section-decision`).
- Anti-cheat audit: PASS.

## When to invoke

- After any sprint that added or changed C/C++ code in
  `examples/sentai_runtime/`.
- BEFORE pushing a SIM-validated commit to main / integration.
- After resolving an ARM-only build failure to confirm no other gaps
  remain.

## See also

- Sim.md §2 (the "ARM never regresses" hard rule).
- [[itcm-budget]] memory entry — m_text discipline.
- [[no-heavy-data-through-mp]] — binding-layer hard rule.
- Skill `section-decision` — placement decision flowchart.
- Skill `qstr-regen` — stale QSTR table recipe.
- Skill `mp-binding-audit` — ARM vs SIM binding divergence.

## TODO (filed 2026-05-21)

This skill is a stub written after the OP-S10-W19-T6b ARM build
overflowed `m_text` because `sentai_markers.cc` + the new
`sentai_svd3.cc` weren't routed to `.sentai_slow` — SIM build had
been green for 30 minutes before the ARM rebuild caught it.
Operator-requested durable note + automation hook.

Phase 2 (not yet implemented): wrap the recipe above in a single
script `scripts/sim_arm_parity_check.sh` that returns 0/non-zero
so this skill becomes a one-liner invocation.  Until then, the
steps run by hand.
