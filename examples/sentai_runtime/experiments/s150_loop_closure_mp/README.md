# s150 — Loop closure MP-only port of s143

**Date**: 2026-05-16

## What this migrates

s143 (host-orchestrated) → **MP-only**.  Two-lap mission:
- lap-1: store 3 synthetic-image PHOG+GIST descriptors at 3 waypoints
- lap-2: revisit each, query → expect MATCH id=stored_pid

Demonstrates "drone CONSUMES memory" — first mission where the drone
both writes to L3 AND reads from L3 in the same flight.

## Files

```
s150_loop_closure_mp/
├── README.md
├── mission_s150.py    # lap1 store + lap2 query
└── verdict.py         # 8 hard gates including matches_ok ≥ 2/3
```

Reuses: `crtp_log.py` (s146), `hex_helpers.py` (s142).

## Hard gates

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 9 fired |
| G3 lap1_stores | ≥ 3 |
| G4 lap2_matches | ≥ 3 |
| G5 matches_ok | ≥ 2/3 (allows 1 noisy match) |
| G6 closure_xy | < 10 cm |
| G7 errors empty |  |
| G8 journal events |  |
