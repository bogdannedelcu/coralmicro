# s149 — HexPatrol MP-only port of s142

**Date**: 2026-05-16

## What this migrates

s142 (host-orchestrated) → **MP-only**.  Drone flies 3-waypoint patrol;
at each waypoint computes synthetic PHOG+GIST descriptor and stores in
L3 places gallery with H3-indexed cell.  After patrol: self-query each
pid → must return same pid as best match.

Original s142 ran `mission_l4`-style orchestration on host; this MP
port keeps the SAME descriptor pipeline (`hex_helpers.py`) but moves
the decision-making + REPL-driving into sentai firmware.

## Files

```
s149_hex_patrol_mp/
├── README.md
├── mission_s149.py        # MP mission
└── verdict.py             # 9 hard gates
```

Reuses:
- `crtp_log.py` from s146
- `hex_helpers.py` from s142_hex_descriptor_patrol — provides
  `capture_and_store(seed, x, y, z)` + `self_query(pid)`.

## Hard gates

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 9 fired |
| G3 places_stored | ≥ 3 |
| G4 all pids ≥ 0 (add succeeded) |  |
| G5 self_queries | ≥ 3 |
| G6 self_query.ok all true | exact-match retrieval |
| G7 closure_xy | < 10 cm |
| G8 errors empty |  |
| G9 journal events all present |  |
