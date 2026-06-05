---
name: s142-hex-patrol-shipped
description: "s142 HexPatrol PASS 2026-05-16. First mission using L3 places gallery + PHOG+GIST+H3 in-mission. Drone 4.69 m tour, stores 3 places at 3 distinct H3 cells, all self-match correctly, closure 1.8 cm. New API sentai.places.get_desc(id). Hex_helpers MP module: hex_image/quantize/xy_to_h3/capture_and_store/self_query."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16.** Operator-prompted: "să începem să creăm
descriptori pentru hexagoane".  First mission that USES L3 places +
descriptors as designed (until now, L3 storage was UNUSED in flight).

## Mission profile (Gazebo, cf2 real)

```
takeoff origin → store HOME descriptor at h3(0,0)
inject 2 synthetic targets (s137-style)
goto(t1 @+1.5,0) → INSPECT → store seed=1 descriptor at h3(1.5,0)
goto(t2 @-0.5,+1.0) → INSPECT → store seed=2 descriptor at h3(-0.5,1.0)
return_home → land at origin
Verify: gallery has 3 places, distinct cells, self-match all correct
```

## Results (single run, PASS first try)

| Gate | Result | Threshold |
|---|---|---|
| state_final | DONE | == DONE |
| aborts | 0 | == 0 |
| **land_err vs PHYSICAL_ORIGIN** | **1.8 cm** | < 15 cm |
| gallery count | 3 | ≥ 3 |
| distinct H3 cells | 3 | ≥ 3 |
| all desc_set | True | True |
| self-match correctness | 3/3 | all |
| mission_duration | 39.4 s | < 120 s |

Trace: ARMING → HOVERING → APPROACH → INSPECT → HOVERING → APPROACH →
INSPECT → HOVERING → RETURNING → HOVERING → LANDING → DONE.

Drone path **4.69 m** (vs s137's 4.80 m — essentially identical trajectory).

FlowBaseline post-commit: dist_mean = 9.8 cm canonical = 7.4 — PASS.

## What this proves (cinstit)

**Drone behavior**: same as s137 (multi-target tour, 4.69m, 39s, closure
1.8cm).  s142 does NOT test new flight dynamics — that's s137 territory.

**Software stack**: first mission proving end-to-end integration of:
- PHOG + GIST called IN mission flow (not just SIM smoke isolation)
- L3 places gallery populated by mission (was UNUSED storage until now)
- xy_to_h3 mapping at each INSPECT
- 168 PHOG + 64 GIST floats → 64 uint8 bytes quantized
- places.add(h3_cell, desc, x, y, z) stores correctly
- places.get_desc(id) retrieves the stored bytes
- places.query(desc, h3_cell, ...) self-matches each stored place

## What this does NOT prove (deferred to s143)

- **Loop closure utility** — drone doesn't yet CONSUME the memory it
  stored.  s143 adds lap-2 where drone revisits + queries (no store) +
  verifies "I've been here before" match.

## New API shipped in this commit

```c
sentai.places.get_desc(id) → bytes(64) | None
   // Returns the descriptor bytes for a place; complements
   // sentai.places.get() which returns pose+status dict only.
   // None if id unknown OR desc_set == 0.
```

Plus `hex_helpers.py` MP module (in `sentai_fs_root/` + experiment folder):
- `hex_image(seed)` — deterministic 64×64 synthetic image per seed
- `quantize(phog, gist)` — pack 32+32 floats → 64 uint8 bytes
- `xy_to_h3(x, y, res=15)` — ENU meters → fake lat/lng → H3 cell
- `capture_and_store(seed, x, y, z)` — end-to-end pipeline
- `self_query(pid)` — verify desc finds itself in gallery

## Synthetic vs real frame (s143+ wires real)

s142 uses **synthetic deterministic images** per seed (each location
gets a unique descriptor).  This proves the **flow** works.  Real
Gazebo frame capture (read latest PPM → grayscale → compute desc)
is straightforward extension (read_ppm already exists in
aruco_detector.py) but s142 leaves it for s143+ to keep the focus on
"L3 + descriptors + H3 integration" without PPM I/O complexity.

## Related

- `[[s137-explore-long-shipped]]` — same trajectory baseline
- `[[s139-phog-shipped]]` — PHOG used here
- `[[s140-gist-shipped]]` — GIST used here
- `[[s141-descriptor-baseline-shipped]]` — anti-regression gate
- `[[l3-shipped]]` — L3 places (finally populated!)
- `[[sim-test-must-return-home]]` — closure preserved (1.8 cm)
- `[[test-must-be-relevant-to-claim]]` — claim "integration" gated
- Next: s143 — loop closure (drone revisits, queries, verifies match)
