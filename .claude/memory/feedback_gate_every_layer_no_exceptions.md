---
name: gate-every-layer-no-exceptions
description: "For ObjectsPlan, run the FlowBaseline gate after EVERY layer commit, no exceptions — including layers that look like \"zero runtime impact\". Discipline > rationalization."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f735b0d3-8915-45fb-9396-880ffde2553d
---

When working through ObjectsPlan (L1..L7), run
`examples/sentai_runtime/experiments/s127_flowbaseline/run.sh`
after **every** layer commit, including layers that appear to have zero
runtime impact (e.g. L1: docs + submodule registration + a standalone
host test).

**Why:** operator (2026-05-13) — after I justified skipping the gate
for L1 because "nothing here links into sentai_sim, no regression
possible", operator pushed back: *"Paihai sa verificam FlowBaseline,
nu? De ce ai skip?"* The reasoning to skip looked airtight (zero
linkage, no source changes) but missed two points: (1) running the
gate also validates the SITL substrate is still healthy — world
patches, cf2 firmware, distrobox env — which can drift independently
of our code, and (2) the value of the layered approach is the
*reflex* of always gating; skipping once teaches the wrong reflex.

**How to apply:**
- Before committing any ObjectsPlan layer (L1 onwards), run the gate.
- If a layer truly has no risk of regression, the gate still costs
  only the SITL cold-start time (≈45 s for stack up + 15 s hover +
  verdict). Pay it.
- Record the PASS metrics in the commit message body (dist_mean,
  all4_rate, n_samples) so the layer's gate result is auditable from
  `git log` alone.
- The gate passes when: `dist_mean_m < 0.15` AND `all4_rate >= 0.5`
  AND `n_samples >= 5`.

Related: [[objectsplan]], [[flowbaseline-canonical-config]].
