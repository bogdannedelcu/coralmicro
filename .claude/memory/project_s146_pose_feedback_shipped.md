---
name: s146-pose-feedback-shipped
description: "s146 ships pose feedback for MP missions via CRTP LOG over UDP. Pure MicroPython (Option B) using sentai.crazy.send_crtp/recv_crtp primitives — zero new C code. 11 wire-format unit tests PASS (byte-for-byte vs cflib). Live integration test (crtp_log.scan_toc + create_pose_block + latest_pose) ready, awaits Gazebo+cf2 SITL. Unblocks s136 migration (Task #41 final stage)."
metadata: 
  node_type: memory
  type: project
  originSessionId: 6abc7162-91c9-4f89-bfa0-2542359ba6c2
---

**Shipped 2026-05-16.** Task #41 Option B — **OFFLINE + LIVE + MISSION PASS** (commits 907b424b + d8aa9b8a).

## What

`examples/sentai_runtime/experiments/s146_pose_feedback/`:
- `crtp_log.py` — pure-MP CRTP LOG protocol library
  (TOC scan, block create/start/stop, poll, latest_pose).  ~250 LoC.
- `t_crtp_log_offline.py` — 11 byte-level unit tests vs cflib wire fmt.
- `t_crtp_log_live.py` — Gazebo integration test (pending live run).
- `run.sh` — `--offline` (PASS) / `--live` (pending).

## Why Option B (and not A)

Decision recorded 2026-05-16: ship B first to validate wire format in
the easiest-to-debug environment, then later promote to A (C wrapper)
as an optimization once the protocol is empirically confirmed.

A and B don't conflict — both speak the same CRTP LOG protocol, can
coexist with different block_ids.  See chat log for tradeoff details.

## What B exposes (Python API)

```python
import crtp_log
crtp_log.reset()                                  # clear stale blocks
n = crtp_log.scan_toc(timeout_ms=10000)           # ~1-3 s on cf2 SITL
entry = crtp_log.find('stateEstimate', 'x')       # → (ident, type) or None
bid = crtp_log.create_pose_block(period_ms=100)   # 4-var float subscription
# ... mission loop ...
pose = crtp_log.latest_pose()                     # (x, y, z, yaw) | None
crtp_log.stop(bid)
```

Cached state: `_toc` dict (one scan/process), `_blocks` dict (id →
spec), `_latest_pose` (4-tuple).

## Wire format (canonical doc)

Mirrors cflib/crazyflie/{log,toc}.py byte-for-byte.  Verbatim in
s146_pose_feedback/README.md.  Highlights:

- TOC INFO: `port 5 ch 0 [3]` → `[3, num_lo, num_hi, crc*4]`
- TOC ITEM: `port 5 ch 0 [2, idx_lo, idx_hi]` → `[2, idx_lo, idx_hi, type, group\0name\0]`
- CREATE_BLOCK_V2: `port 5 ch 1 [6, blk, fetch_byte, id_lo, id_hi]*N`
  where `fetch_byte = (fetch_type & 0xF) | ((stored_type & 0xF) << 4)`
  — for float-as-float = `0x77`.
- START: `port 5 ch 1 [3, blk, period_10ms]`
- LOGDATA push (cf2→host): `port 5 ch 2 [blk, ts*3, var_data..., var_data...]`

## Validation

`bash run.sh --offline` → 11/11 byte-level tests PASS:
- `get_info` = `03`
- `get_item(300)` = `022c01` (multi-byte idx)
- `create_block(2, 4 floats)` = `0602777000777100777200774900`
- `start(1, 10)` = `03010a`
- `reset` = `05`

These match cflib's wire output verbatim, confirming we can talk to
cf2 SITL without C changes.

## LIVE integration test — PASS

`t_crtp_log_live.py` run against running cf2 SITL (commit d8aa9b8a):
- `scan_toc()` returned **361 entries** (stock CrazySim firmware)
- `stateEstimate.{x,y,z}` resolved as floats (IDs 70, 71, 72)
- `stabilizer.yaw` at ID 101
- `create_pose_block` returned bid=1
- First pose at 100ms latency: x=-9e-05, y=2e-04, z=0.015 (drone at rest)
- TOC scan latency = 2.7s for 361 entries (~7.6ms/RPC roundtrip)

## mission_s146.py — first pure-MP closed-loop mission PASS

8/8 hard gates green:

| Gate | Value | Threshold |
|---|---|---|
| status | DONE | == DONE |
| phases | 9/9 | all expected |
| closure_xy | **3.46 cm** | < 10 cm |
| approach_dist | 9.48 cm | < 15 cm |
| return_dist | 9.09 cm | < 15 cm |
| errors | 0 | == 0 |
| journal events | 15/15 | all present |

This is the first MP-only mission with pose-feedback closure gate.
Trajectory: origin → (0.15, 0.10, 0.5) → dwell → origin → land,
~16 seconds end-to-end including 3.4s TOC scan.

## FlowBaseline gate post-refactor

s127 FlowBaseline re-run during session: **dist_mean = 6.92 cm**
(canonical 7.4 cm) — refactor + s145 + s146 introduced **NO regression**.

## (historical) Live integration test (pending)

`t_crtp_log_live.py` checks:
1. `crazy.init() rc=0`
2. `reset()` sent
3. `scan_toc()` returns ≥ 50 entries
4. `find('stateEstimate','x')` etc. resolve as floats (type=0x07)
5. `create_pose_block()` returns positive block_id (subscription ok)
6. `latest_pose()` returns 4-float tuple within 5 s
7. pose values plausible: x∈±2, y∈±2, z∈[-1,3]

Pending Gazebo SITL bring-up.  Will be run by operator interactively.

## What's left for Task #41

Once `t_crtp_log_live.py` PASS:
1. Build `mission_s146.py` (or similar) that adapts s136 mission logic
   to MP-only using crtp_log + sentai.explore.set_pose loop.
2. Verdict gates closure < 10 cm vs PHYSICAL_ORIGIN.

Then Task #41 (s136 via MP-only) is done.  Task #42 (migrate s137/138/
142/143/144) reuses the same pattern.

## Limitations of Option B

- ~1-3 s TOC scan per process (acceptable for missions)
- ~10 Hz pose update rate safe in pure MP; higher rates may need
  Option A (C wrapper) for parsing latency
- struct.unpack allocs MP heap per frame; fine at 10 Hz

## Related

- `[[s145-mission-template-shipped]]` — open-loop pattern this extends
- `[[missions-run-in-sentai-only]]` — firm rule this implementation enables
- `[[sim-test-must-return-home]]` — gate-able once pose lands
- Task #39 — `sentai.crazy.{send,recv}_crtp` primitives consumed
- Future: `[[option-a-crazy-pose-c-wrapper]]` placeholder for the
  C-side optimization (post Stage 9 forcing function)
