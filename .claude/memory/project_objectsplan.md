---
name: objectsplan
description: "ObjectsPlan — operator-named (2026-05-13). Layer-by-layer plan bringing the 31 commits from feature/ov5640-camera-support (broken advanced branch) back into integration/from-180bbb5f. Each layer gated by FlowBaseline PASS. Re-implement following ideas/objects_plan.md, NOT cherry-pick."
metadata: 
  node_type: memory
  type: project
  originSessionId: 3ff7dffd-7518-4967-97ba-fe951cc1baf6
---

**Source**: branch `feature/ov5640-camera-support` (commit a5f5a568 + 30 more)
is the "advanced broken" branch. Diverged from `integration/from-180bbb5f`
at merge-base `180bbb5f`. Has 31 commits we want; the implementation
turned out brittle (s125 iteration hit dead-ends), so we re-implement
following the design docs `ideas/objects.md` + `ideas/objects_plan.md`
(both authored in a5f5a568).

**Why:** the broken branch tried to do too much in one go and ended up
with cf2 SITL "DESTABILIZED on purpose for sentai.flow testing"
(commit 37f80bb1) — work was discarded. Layered approach with
FlowBaseline gate per layer prevents that.

**How to apply:** For each Layer L, do the work, run
`bash examples/sentai_runtime/experiments/s127_flowbaseline/run.sh`,
verify PASS, commit L atomically. If FAIL, revert L and diagnose
BEFORE moving to L+1.

### Layer breakdown

**Layer 0 — Baseline established (2026-05-13)**: see
[[flowbaseline-canonical-config]]. FlowBaseline canonic e fixat:
dist_mean=7.4 cm, all4=100%.

**Layer 1 — Foundation static infra** — SHIPPED 2026-05-13 commit
`2717bb27` on `integration/from-180bbb5f`:
- ideas/objects.md + ideas/objects_plan.md (verbatim from a5f5a568)
- third_party/h3 v4.4.1 (`383ecdb3`) registered in .gitmodules
- examples/sentai_runtime/experiments/s119_h3_smoke/ — fresh smoke test
  (NOT cherry-picked, see [[no-broken-branch-test-reuse]]), PASS
  on x86 gcc: round-trip drift 412 m, gridDisk k=1→7, hex 6v.
- Sim.md §10v dual-scale world pattern — already in branch (82f2a267)
- World kept as-is per operator: harmonic floor texture dropped, the
  FlowBaseline cf2 world IS the gate substrate.
- FlowBaseline gate PASS post-commit: dist_mean=5.4 cm, all4=1.00,
  n_samples=18, flow_n=400 @ 26.67 Hz. Better than canonical 7.4 cm.

**Layer 2 — sentai.objects data layer** — SHIPPED 2026-05-13 commit
`eaf67e75` on `integration/from-180bbb5f`. See [[objects-l2-shipped]]
for API surface + file layout + hardening notes.  Gate PASS:
dist_mean=9.3 cm (canonical 7.4 cm, threshold 15 cm), all4=1.0, n=14.

**Layer 3 — sentai.places** — SHIPPED 2026-05-14 commit `19c40d88`,
pushed to origin.  See [[places-l3-shipped]] for frozen API + ITCM
placement contract.  H3 lib production-wired into both build trees
(libh3_sim + libh3_arm, ~14 KB routed to .sentai_slow SDRAM).
Driver 58/58 PASS, FlowBaseline gate PASS dist=10.0 cm.

**Layer 4 — sentai.servo skeleton** (Stage 4):
- Action layer: intent → velocity (no closed loop yet, just dispatch)

**Layer 5 — sentai.slam** (Stage 1.A/B/C from broken branch + plan §5):
- monocular class-prior depth lifting
- update_3d() bearing → inverse-depth point estimate

**Layer 6 — sentai.explore skeleton** (Stage 3.A-D from broken branch):
- Mission SM (SEARCH → APPROACH → FINAL → DONE)
- Guard conditions over real inputs
- NO safety logic (see [[no-safety-logic-in-explore]])

**Layer 7 — s125 integrated demo** (final wire-up):
- End-to-end mission demo
- Where broken branch died iteratively; now incrementally validated

### Order rationale

- L1 first because zero runtime impact — safest gate test
- L2 (objects) before L3-5 because all other layers depend on object storage
- L3 (places) early because it depends only on H3 (Layer 1) — independent track
- L4 (servo) before L5/L6 because action layer is the spine the mission uses
- L5 (slam) before L6 (explore) — explore consumes 3D estimates from slam
- L7 last — integration depends on everything above

### Out of scope for now

- Stage 2 (TPU model task-specific) — needs synthetic dataset gen +
  training pipeline; orthogonal track, scheduled separately
- Stages 5-10 from objects_plan.md (loop closure, full SM3, ARM
  bring-up, hardening) — after L1-L7 land

Related: [[flowbaseline-canonical-config]] (the gate), [[h3-integration]]
(H3 already vendored — verify in L1), [[no-safety-logic-in-explore]]
(constraint for L6), [[flow-anchor-platform-shim]] (sentai.flow.mode
shim from broken branch — may need to bring back in L4/L6).
