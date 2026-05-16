# s143 — Loop closure (lap-2 revisit + query)

**Date**: 2026-05-16
**Predecessor**: s142 HexPatrol (descriptors stored per H3 cell)
**Purpose**: first proof drone CONSUMES the memory it stored.

## Why this exists

s142 proved L3 places + descriptors + H3 indexing work IN mission
flow.  Gallery ends with 3 populated cells, self-match works.  **But**
the drone never USES the gallery during flight — it only writes to it.

s143 adds a second lap: drone returns to a previously-visited target,
re-computes descriptor, **queries L3 gallery** (instead of storing),
verifies the best match is the place stored in lap-1.

This is the semantic "I've been here before" — first time end-to-end.

## Mission profile

```
=== LAP 1 (build memory) ===
phase 1-3   cf2 setup + capture PHYSICAL_ORIGIN
phase 4     Store HOME descriptor (seed=0) at h3(origin)
phase 5     Inject 2 synthetic L5 targets
phase 6     explore.start + takeoff → HOVERING
phase 7     goto(t1) + INSPECT → store seed=1 at h3(t1) → pid_t1
phase 8     goto(t2) + INSPECT → store seed=2 at h3(t2) → pid_t2
phase 9     explore.return_home → HOVERING (back at origin)
            [Gallery state: 3 places, ids 1/2/3]

=== LAP 2 (consume memory) ===
phase 10    goto(t1) AGAIN — same target as lap-1
phase 11    at INSPECT: re-compute descriptor with SAME seed=1
            (re-use same synthetic image; only deterministic).
            CALL: query(desc, h3=h3(t1), k_disk=1, thresh=0)
            DO NOT store again — gallery stays at 3 places.
phase 12    Verify: query returns place_id == pid_t1 (memory hit!)
            Verify: score_pct >= 95 (high confidence — same desc)

phase 13    explore.return_home → land at PHYSICAL_ORIGIN
phase 14    Final verdict: gallery count unchanged (3), match correct,
            closure < 15 cm.
```

Drone path: ~4.69 (lap-1) + ~3.0 (lap-2 detour to t1 and back) ≈ **~7.7 m total**.
Mission time: ~55-65 s estimated.

## Pass criteria — HARD gates

```
state_final == "DONE"
aborts == 0
land_err_xy_vs_origin < 0.15

# Memory hit gate (the key new claim)
match_id_lap2 == pid_t1_lap1     # query returns lap-1's place
match_score_pct >= 95            # high confidence (identical desc)

# Storage discipline gate
gallery_count_final == 3         # lap-2 did NOT add new place

# Closure (universal)
mission_duration < 90 s
```

## What this proves vs s142

| Aspect | s142 | **s143** |
|---|---|---|
| FSM transitions | ✓ | ✓ |
| Drone navigates | ✓ (4.7m) | ✓ (7.7m, 2 laps) |
| Descriptors computed in mission | ✓ | ✓ |
| Gallery populated by drone | ✓ | ✓ |
| Closure < 15 cm | ✓ | ✓ |
| **Drone QUERIES gallery during flight** | ✗ | **✓** |
| **Match (place recognition) works** | ✗ | **✓** |
| **"Drone remembers" demonstrated** | ✗ | **✓** |

This is the first test that demonstrates the semantic SentAI demo
§23.1 promises: drone vizitează locuri → memorize → revizitează →
recunoaște.

## Why deterministic re-capture (vs natural re-capture)

For the MVP, lap-2 re-uses the SAME synthetic seed (seed=1) at the
re-visit, producing a byte-identical descriptor to lap-1.  This
guarantees the query → match path works under ideal conditions.

For real-camera (s144+):
- Lap-1 captures real Gazebo frame at t1 → desc_A → store
- Lap-2 captures real Gazebo frame at t1 (slightly different pose,
  noise, lighting) → desc_B (NOT identical to desc_A)
- Query desc_B against gallery → match desc_A (with score < 100)
- Robustness threshold: score_pct >= 60 (looser, accounts for noise)

s143 thus has a HARD gate at score 95% (deterministic), s144+ will
relax to 60% (realistic frame variation).

## Files

```
s143_loop_closure/
├── README.md            # this file
├── mission_explore.py   # lap-1 store + lap-2 revisit/query
├── verdict.py           # match-correctness HARD gates
└── run.sh               # bootstrap + run + verdict
```

## Related

- `[[s142-hex-patrol-shipped]]` — lap-1 logic reused
- `[[s139-phog-shipped]]`, `[[s140-gist-shipped]]` — descriptors
- `[[s141-descriptor-baseline-shipped]]` — anti-regression
- `[[l3-shipped]]` — query path now exercised
- `[[sim-test-must-return-home]]` — closure preserved
- `[[test-must-be-relevant-to-claim]]` — match-correctness HARD-gated
- Next: s144 — real Gazebo PPM frame instead of synthetic seed (true
  cross-time-noise robustness test)
