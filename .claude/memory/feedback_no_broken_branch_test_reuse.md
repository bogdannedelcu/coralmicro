---
name: no-broken-branch-test-reuse
description: "For ObjectsPlan L1-L7, tests must be written from scratch, NOT cherry-picked from feature/ov5640-camera-support — the broken branch's tests may carry the same assumptions that made the implementation brittle."
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f735b0d3-8915-45fb-9396-880ffde2553d
---

When implementing an ObjectsPlan layer, the *design docs* (`ideas/objects.md`,
`ideas/objects_plan.md`) and the *gate* ([[flowbaseline-canonical-config]])
are reusable from the broken branch — they are spec, not implementation.

**Tests are NOT reusable**. Even small smoke tests (e.g. the s119 H3
compile-and-call) must be re-written from scratch for the layer we're
landing now. Reason: the broken branch's tests were authored alongside
the brittle implementation; they may encode the same misunderstandings
that led to the rework. A fresh test exercises the API surface *we*
are committing to and provides honest signal.

**Why:** operator (2026-05-13) — "referitor la alte teste facute cum ar
fi s119 nu prea vreau sa refolosim codul testelor, poate nu e bun. Mai
bine facem testele de la zero pentru ce cod scriem".

**How to apply:**
- L1 H3 smoke: write fresh C source under
  `examples/sentai_runtime/experiments/s119_h3_smoke/h3_smoke.c`,
  even though a previous version exists in commit `fa541a4b`.
- L2-L7 each get fresh `tNN_*` driver tests on board / SIM,
  named per the layer being validated.
- Pull *design docs* and the *FlowBaseline gate* freely — those
  describe intent, not implementation.

Related: [[objectsplan]], [[flowbaseline-canonical-config]].
