---
name: sentai.flow.anchor_forward — continuous C++ auto-VPE task (s113 P2)
description: REPL-toggled FreeRTOS task that forwards anchor pose to PX4 MAVLink VPE + cf2 CRTP ext_position at N Hz. 98/290 packets sent in live PX4 test, 2026-05-12.
type: project
originSessionId: 12c47722-7968-4e7d-a740-c42574354aaa
---
REPL-controlled continuous C++ task. Same API on ARM + SIM:

```python
sentai.flow.anchor_forward(rate_hz, target)   # target: "auto"|"px4"|"cf2"|"both"|"off"
sentai.flow.anchor_forward(0)                 # stop
sentai.flow.anchor_forward_stats()            # dict
```

Stats: `sent_px4`, `sent_cf2`, `skipped`, `last_seq`, `send_failed`,
`running`, `rate_hz`, `target`, `iters`.

**Bring-up gotcha (load-bearing)**: FreeRTOS POSIX scheduler will
NOT schedule a task at `tskIDLE_PRIORITY + 1` if other tasks are at
`+2` (REPL + camera_bridge_recv).  Use `+2` to share via round-robin.
First cut silently never ran the task body (`iters=0` despite
`running=True`).  Added 500 ms liveness check in `_start()` that
returns -3 if the task hasn't entered its loop body — fails loud.

**Live PX4 validation (sim build #115, 2026-05-12)**: 38 s OFFBOARD
hover @ 1.5 m, `anchor_forward(10, "px4")`, final stats:
- iters=384 (10 Hz × 38 s)
- sent_px4=98 (frame_seq-deduplicated; bridge saw 290 detected, ~1/3
  forwarded because forwarder is rate-limited to 10 Hz vs 30 fps bridge)
- send_failed=0
- Independent pull-mode `anchor_pose()` still works (46/60 detected).

**Transports added**:
- ARM: `sentai_link_send_vpe(x,y,z,yaw)` in `sentai_link.cc` (MAVLink
  #102 over UART, ENU→NED on wire); `sentai_crazy_send_ext_position(x,y,z)`
  in `sentai_crazy.cc` (CRTP port 6 / ch 1, 12-byte LE float payload).
- SIM: same `send_vpe` in `sentai_link_sim.cc` (UDP variant); cf2
  stubbed in `sentai_crazy_stub_sim.c` (no on-board cf2 bridge on x86).

**Why a separate task vs piggybacking on flow_task**: anchor pose is
published asynchronously by the detector (M7 in s113 P1+; Python
sidecar in SIM).  Dedicated task fires at fixed rate regardless of
camera FPS, keeps running even if flow is stopped, and dedup-skips
on `frame_seq` so the FCU isn't spammed with duplicates.

**NASA/JPL hardening per `embeded.md`** (audit shipped after the
live PX4 PASS):
- F1: NaN/Inf rejection via IEEE-754 bit-pattern (no libgcc helper)
- F2: per-axis bounds check (`±50 m XY, [-2, +20] m Z, ±π yaw`)
- F3: stale-pose gate (drop if `src_ts_ms` > 500 ms old)
- F6: 500 ms liveness check in `_start()` returns -3 if scheduler
  starves the task (caught a real bug — priority +1 on POSIX never
  ran; fixed to +2)
- Stack high-water-mark exposed via `stack_hwm_words` in stats
- Unit test `test_fault_gates.py` proves all 3 gates fire (each
  counter incremented when poisoned packet sent)
- All m_text-bloating debug printfs removed — caller checks return
  codes instead (ITCM is tight, no spare bytes)

**Stats dict shape** (identical ARM ↔ SIM):
```python
{'sent_px4': int, 'sent_cf2': int, 'skipped': int, 'last_seq': int,
 'send_failed': int, 'running': bool, 'rate_hz': int, 'target': str,
 'iters': int,
 'health': (dropped_nonfinite, dropped_oob, dropped_stale, stack_hwm_words)}
```

Reference: `experiments/s112_x86_anchor_shim/run_px4_live.sh` for
the canonical live bring-up.
