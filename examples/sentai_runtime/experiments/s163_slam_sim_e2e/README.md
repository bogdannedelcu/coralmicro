# s163 — SlamTask SIM end-to-end (OP-S10-W11-T4)

Validates the SIM `camera_bridge_recv` SLOT_RGB_64 producer wired in
T4.  Unlike s162 (lifecycle only), this exercises the **full
producer → consumer chain** without Gazebo:

```
inject_frames.py  →  /tmp/sentai_cam.sock  →  camera_bridge_task
   (host)             (UDS, SCM1 header)        sentai_prep_publish_slot_rgb_64
                                                  ↓ pxp_scale (SIM shim)
                                                  ↓ slot commit + DMB
                                                  ↓ sem.give → SlamTask wakes
                                                            ↓ hsv compute
                                                            ↓ places query
                                                            ↓ atomic publish
                                       MP reads via sentai.places.slam_*
```

## What it proves

| Gate | Check |
|---|---|
| 1 | `start_slam()` returns 0 |
| 2 | `frames_processed >= 5` after 3 s while injector pushes 15 frames @ 20 Hz |
| 3 | `result_seq >= 5` (atomic publish path live) |
| 4 | `t_compute_us > 0` (real HSV+query compute happened) |
| 5 | Clean `stop_slam()` |

## How to run

```bash
cmake --build build-sim --target sentai_sim
bash examples/sentai_runtime/experiments/s163_slam_sim_e2e/run.sh
```

Expected tail:

```
[PASS] frames_processed >= 5 (got 15; dropped=0, last_us=20)
[PASS] result_seq >= 5 (got 15)
[PASS] t_compute_us > 0 (got 20)
[PASS] stop clean (rc=0, running=0)
[VERDICT] s163 slam_task SIM e2e: PASS
[s163] PASS
```

## Notes

- Driver inject uses synthetic per-frame-varying RGB gradients so
  the bridge's duplicate-frame detector doesn't drop them.
- SlamTask priority was bumped to `tskIDLE_PRIORITY + 2` in T4 to
  match the producer (camera_bridge on SIM, PrepTask on ARM).  With
  `configUSE_TIME_SLICING=1` on the POSIX port, equal-priority
  pairs interleave fairly; at `+1` SlamTask was starved by the
  REPL + bridge tasks during a sustained frame burst.
- Match IDs are 0 (gallery empty) — that's expected; the test
  validates the **pipeline**, not the match accuracy.  Use s161
  (HSV anti-regression) and a future EXP-s163b for match-quality
  testing.
