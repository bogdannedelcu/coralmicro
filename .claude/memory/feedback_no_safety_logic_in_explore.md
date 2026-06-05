---
name: No safety/battery/emergency logic in sentai.explore
description: Keep mission FSM (sentai.explore) and safety FSM separate — battery, EMERGENCY_HOVER, abort-on-critical belong in sentai.safety
type: feedback
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
Don't push battery thresholds, EMERGENCY_HOVER, or emergency-landing
abort triggers into `sentai.explore`. Those belong in a **separate
safety state machine** (future `sentai.safety` namespace).

**Why:** Concerns are different. Mission FSM transitions across
exploration phases driven by progress signals (alt, marker, cells
visited, distance home). Safety FSM watches hardware-side risks
(battery, link loss, IMU faults, geofence) and can override the
mission FSM with a forced state. Mixing the two makes both harder
to reason about and breaks NASA/JPL discipline (one fault model per
component).

**How to apply:** When extending `sentai.explore`, keep guards
focused on mission progress. If a battery / IMU / link sensor appears
in a guard expression, stop and split it out — that signal belongs
to `sentai.safety`. The two FSMs communicate via a thin handshake
(safety can hold mission via a "halted" flag; mission queries
`safety.ok()` before forward transitions if needed).

Confirmed 2026-05-13 after operator caught battery_crit + EMERGENCY_HOVER
logic in the first cut of Stage 3.B.
