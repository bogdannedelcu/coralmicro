---
name: airrepl-paper-lit-review
description: "Mini-paper AirREPL — canonical plan lives at ideas/AirREPL_paper.md (created 2026-05-15). Lit review confirmed gap vs MAVLink+LLM, MCP+drone, Code-as-Policies, TypeFly, SwarmGPT — none run a Turing-complete REPL on bare MCU over radio. Parked at FW17 in ideas/FutureWork.md, promote post-defense."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Decision recorded 2026-05-15**: AirREPL is a separate publishable
artifact from the thesis. Park, promote post-defense.

## Canonical location

**`ideas/AirREPL_paper.md`** — full paper plan with:
- Contribution claims (C1–C5)
- What we already have (firmware + s091–s132 proofs)
- **Experiments E1–E10** with setup + metrics + pass criteria + folder
- Paper section structure
- §6.bis AirREPL vs MCP positioning (substrate / interface layer)
- Venues + timeline + reproducibility plan + open questions

**`ideas/FutureWork.md`** FW17 — parking-lot pointer (brief).

## The one-liner

AirREPL: Turing-complete Python REPL on Cortex-M7 over ~30 B CRTP
radio, designed for LLM-agent consumption. Distinct from existing
MAVLink+LLM / MCP+drone work which treat the drone as a dumb endpoint
with fixed vocabulary.

## Why deferred

Thesis claim = drift-bounded autonomous perception+nav on a wireless
sensor. AirREPL is the **mechanism**, separable paper.

## Promotion triggers

- Thesis defended → arxiv preprint + workshop submission
- Industry / collab interest in SentAI platform
- Workshop CFP fit (ICRA LLM+Robotics, NeurIPS agent workshops,
  HotMobile, SenSys, MLSys)

## Effort (post-defense)

~32 days experiments + ~10 days writing ≈ 6 weeks of focused work.

## Related

[[radio-no-file-transfer]] — the 30 B MTU constraint that defines
AirREPL semantics
[[crazyflie-radio-bridge]] — hardware substrate (firmware auto-init)
[[s132-lifter-gazebo-shipped]] — best existing proof artifact
[[sentai-sim-journal]] — REPL log infra
[[objectsplan-vs-futurework]] — scope-management rule
