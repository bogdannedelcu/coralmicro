# s146 — Pose feedback (CRTP LOG over UDP, pure MP)

**Date**: 2026-05-16
**Status**:
- ✅ Wire-format unit tests PASS (11/11)
- ⏳ Live integration test pending (needs Gazebo + cf2 SITL up)

## Why this exists

Task #41 needs `cf2.stateEstimate → sentai.explore.set_pose(x,y,z,yaw)`
so the L6 mission FSM can run closed-loop in pure MP.  Pose feedback
is the **last gap** between the s145 template (open-loop waypoints)
and a real migrated mission (s136-style closed-loop).

Per the design decision recorded in chat 2026-05-16 (Option B first,
Option A later), this experiment implements pose feedback in **pure
MicroPython** using only the `sentai.crazy.send_crtp` /
`sentai.crazy.recv_crtp` primitives shipped in Task #39.

No new C code.  Zero build risk.  Future-portable: when ARM build
needs the same telemetry (Stage 9), the same MP code works there too,
unless latency forces an Option A C-side rewrite.

## What this commit ships

| File | Role | Status |
|---|---|---|
| `crtp_log.py` | pure-MP CRTP LOG protocol library (TOC scan, block create/start/stop, poll, latest_pose) | offline-verified |
| `t_crtp_log_offline.py` | 11 wire-format unit tests — byte-level comparison vs cflib | ✅ PASS |
| `t_crtp_log_live.py` | end-to-end integration test, requires cf2 SITL | ⏳ pending Gazebo |
| `run.sh` | `--offline` (now) / `--live` (Gazebo) modes | ready |

## Protocol summary (verbatim from cflib/crazyflie/{log,toc}.py)

```
port = 0x05 (CRTP_PORT_LOGGING)
  channel 0 (CH_TOC)      — TOC discovery
    OUT: [CMD_GET_INFO_V2=3]
    IN:  [3, num_lo, num_hi, crc[4]]
    OUT: [CMD_GET_ITEM_V2=2, idx_lo, idx_hi]
    IN:  [2, idx_lo, idx_hi, type_byte, group\0name\0]

  channel 1 (CH_SETTINGS) — block create / control
    OUT: [CMD_CREATE_BLOCK_V2=6, blk_id,
          fetch_byte, id_lo, id_hi,  /* per variable */
          ...]
    IN:  [6, blk_id, err_code]      (err_code 0 = ok, 17 = EEXIST)

    OUT: [CMD_START_LOGGING=3, blk_id, period_10ms]
    IN:  [3, blk_id, err_code]

    OUT: [CMD_STOP_LOGGING=4, blk_id]    / [CMD_DELETE_BLOCK=2, blk_id]
    OUT: [CMD_RESET_LOGGING=5]

  channel 2 (CH_LOGDATA)  — data frames pushed by cf2 at period
    IN:  [blk_id, ts_lo, ts_mid, ts_hi, var_data..., var_data...]
```

`fetch_byte = (fetch_type & 0x0F) | ((stored_type & 0x0F) << 4)`.  For
float-stored-as-float, that's `0x07 | 0x70 = 0x77`.

Type IDs (`LogTocElement.types`):
```
0x01 uint8_t   0x02 uint16_t   0x03 uint32_t
0x04 int8_t    0x05 int16_t    0x06 int32_t
0x07 float     0x08 FP16
```

## Test the offline path

```
$ bash run.sh --offline
[s146] mode = OFFLINE (wire-format unit tests)
[s146] staging crtp_log.py + t_crtp_log_offline.py
get_info: 03
get_item(42): 022a00
...
---- t_crtp_log_offline PASS ----
```

11 byte-level tests confirm every packet builder matches cflib byte-
for-byte.

## Test the live path (Gazebo + cf2 SITL)

```
$ bash run.sh --live
```

Brings up the SITL stack, copies test files to `sentai_fs_root/`,
runs `t_crtp_log_live.py` via the REPL, then runs verdict.  Expects:
- TOC scan returns ≥ 50 entries
- `stateEstimate.{x,y,z}` + `stabilizer.yaw` resolvable as floats
- LOG block subscribes successfully (err_code 0)
- First pose frame arrives within 5 s of subscription
- x/y within ±2 m of origin, z between -1 and 3 m

## Next steps (after live PASS)

1. Build `mission_s146.py` (in this dir or a sibling) that:
   ```python
   crtp_log.scan_toc()
   bid = crtp_log.create_pose_block(period_ms=100)
   while not done:
       p = crtp_log.latest_pose()
       if p:
           sentai.explore.set_pose(*p)   # closed-loop into L6 FSM
       sentai.rtos.sleep_ms(20)
   ```
2. Adapt s136's mission logic (parallax + APPROACH+INSPECT+RETURNING+
   LANDING) into the MP file.
3. Verdict gates closure-vs-physical-origin (`[[sim-test-must-return-home]]`).

Once `mission_s146.py` runs end-to-end with closure < 10 cm, Task #41
is complete.

## Known limitations of Option B

- TOC scan adds ~1-3 s to mission init (~250 RPC roundtrips at ~5 ms
  each).  Cached in `_toc` module-global; one scan per process.
- Pure MP parsing of LOG frames is fine at 10 Hz (period=100 ms) but
  may struggle past 50 Hz.  If we need 100 Hz pose updates, move
  to **Option A** (C-side `sentai.crazy.pose()`).
- `struct.unpack` consumes some MP heap per call.  No issue at 10 Hz
  but worth re-measuring if upgrading rate.

Option A (C wrapper) can be added later WITHOUT removing Option B —
both can coexist with different block IDs.  See chat 2026-05-16
"A vs B" analysis.

## Related

- `[[s145-mission-template-shipped]]` — open-loop template (no pose)
- `[[missions-run-in-sentai-only]]` — the firm rule this enables
- `[[sim-test-must-return-home]]` — closure gate consumer
- Task #39 — `sentai.crazy.send_crtp`/`recv_crtp` consumed here
- `sim/sentai_crazy_sim.cc` — UDP CRTP transport
- cflib `crazyflie/log.py` + `crazyflie/toc.py` — wire-format reference
