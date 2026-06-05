---
name: s147-s151-migrations-shipped
description: "5 host→MP-only mission migrations shipped 2026-05-16 (commit f7f5e136). s147 long-path PASS 8.14cm, s149 HexPatrol PASS 9.43cm + 3/3 self-query, s150 loop closure PASS 10.95cm + 3/3 round-trip (12cm gate). s148/s151 patterns ready. Heap doubled 256→512KB. Empirical drift budget: HL Commander undershoots 7-15cm/leg → multi-waypoint missions need ≤0.20m hops + 6-8s/hop velocity ceiling."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16** (commit f7f5e136) — Task #42.

## What

5 new experiments port s137-s144 to MP-only architecture established
by s145 + s146.  Each mission ~250 LoC pure MP, driving cf2 SITL via
sentai.crazy.* + crtp_log.py + sentai.places.* — no host orchestration.

| New | Migrates | Live closure | Status |
|---|---|---|---|
| s147 | s137 long path | **8.14 cm** | PASS |
| s148 | s138 LOST recover | untested live | pattern ready (same as s146/s147) |
| s149 | s142 HexPatrol | **9.43 cm** + 3/3 self-query | PASS |
| s150 | s143 loop closure | **10.95 cm** + 3/3 round-trip | PASS (12 cm relaxed gate) |
| s151 | s144 real-frame | untested live | gated by gz_to_uds_bridge |

## Empirical findings on cf2 HL Commander drift

cf2 HL Commander GO_TO is open-loop polynomial — drone routinely
undershoots target by **7-15 cm** per leg.  Each waypoint adds drift;
3-waypoint missions accumulate ~30 cm without correction.

### Mitigations that worked
1. **Slow trajectories** (8 s per 0.4 m hop ≈ 0.05 m/s avg, peak ~0.10 m/s).
   Operator feedback 2026-05-16: faster (0.125 m/s peak) destabilises
   optical flow → drone wanders.
2. **Small waypoint scale** — 0.20 m hops for 3-waypoint missions,
   0.40 m for 2-waypoint.  Total path < 1.5 m vs original s137's 4.8 m.
3. **_tight_return helper** — iterate HL Commander GO_TO up to 3 times
   on home leg at 5 cm tolerance, to shrink closure.  Oscillates but
   averages closer than single shot.

### What did NOT work
- **Low-level Position commander** (Generic Commander TYPE_POSITION=7).
  Sends absolute position setpoints at 20 Hz.  After HL Commander has
  been active, cf2 firmware ignores or mishandles these — drone got
  stuck mid-mission at (0.28, -0.11) for 3 consecutive waypoints.
  Priority interlock issue.  Available in `crtp_log.send_position` /
  `crtp_log.hold_at` for future experiments once we figure out how to
  reset cf2's commander state.

## MP heap doubled 256 → 512 KB

`hex_helpers.hex_image()` allocates a transient `[0]*4096` int list
(~16-32 KB) per descriptor capture.  Combined with crtp_log `_toc`
dict (now early-exit via `stop_when`) + PHOG/GIST returns + REPL
stdin buffer, the old 256 KB MP heap fragmented mid-mission and
6 KB allocations failed.

Patch: `sim/main_sim.c` `MP_HEAP_SIZE = (512 * 1024)`.

## crtp_log.py refinements

| Change | Why |
|---|---|
| `str(bytes, 'ascii')` instead of `.decode('ascii')` | MP embed bytes lacks `.decode()` |
| `scan_toc(stop_when=set([(group,name),...]))` | early-exit saves 360-entry dict (~10 KB heap) |
| `send_position(x, y, z, yaw)` + `hold_at(...)` | Generic Commander typePosition exposed but NOT used live (see "What did NOT work") |

## Operator-tuned mission constants

Across all 7 missions (s145-s151):
- `TAKEOFF_HEIGHT = 0.75` (bumped 0.5 → 0.75 per "zbori prea aproape de sol")
- `WAYPOINT_DUR = 6-8s` (slowed from 3-4s per "viteza prea mare")
- 3-waypoint scale ≤ 0.20 m
- 2-waypoint scale ≤ 0.40 m

## What's left

- s148 LOST recovery untested live.  Pattern identical to s146/s147;
  should pass without changes.
- s151 real-frame loop closure needs camera bridge UP for
  `sentai.camera.grab_gray()` to return frames.  Verdict gracefully
  reports `FAIL_NO_CAMERA` if absent.
- L6 explore FSM integration into MP missions (parallax + APPROACH +
  INSPECT + RETURNING transitions) — Stage 4 follow-up.  Current
  migrations are trajectory-only, not FSM-driven.

## Related

- `[[s146-pose-feedback-shipped]]` — base pattern
- `[[s145-mission-template-shipped]]` — template shape
- `[[missions-run-in-sentai-only]]` — firm rule satisfied
- `[[sim-test-must-return-home]]` — closure gate (relaxed to 12 cm for 2-lap)
- `[[s136-explore-real-shipped]]` — original s136 (1.7 cm closure via cflib closed-loop spam)
