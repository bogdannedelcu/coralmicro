# s162 — SlamTask lifecycle smoke (OP-S10-W11-T3)

Validates the InferTask-style perception loop (`slam_task.cc`) added in
Phase 1c.  Phase 1c ships the consumer + the ARM PrepTask producer; the
SIM producer (camera_bridge_recv) lands in Phase 1d.  This smoke
therefore tests **lifecycle plumbing only**: start, status, current()
shape, idle survival across one sem-timeout cycle, clean stop.

## What it proves

| Gate | Check |
|---|---|
| 1 | `sentai.places.start_slam()` returns 0 |
| 2 | `slam_stats().is_running == 1` |
| 3 | `slam_current()` returns dict with all 6 keys |
| 4 | Task survives 700 ms idle (one full 500 ms sem timeout); `frames_processed == 0` (no SIM producer yet — expected) |
| 5 | `stop_slam()` returns 0; `is_running` flips to 0 |

## How to run

```bash
cmake --build build-sim --target sentai_sim
cp examples/sentai_runtime/experiments/s162_slam_task_smoke/t_slam_smoke.py \
   build-sim/sentai_fs_root/
echo 'import t_slam_smoke' | ./build-sim/sim/sentai_sim
```

Expected last line:

```
[VERDICT] s162 slam_task lifecycle smoke: PASS
```

## Coverage gap (intentional)

- No real frames cross SLOT_RGB_64 on SIM yet.  Phase 1d wires
  `camera_bridge_recv` as the producer; Phase 1e (EXP-s163) will exercise
  the full pipeline end-to-end via a multi-pose Gazebo run.
- ARM-side validation deferred to a separate live test once the rest
  of the OP-S9 ARM bring-up is wrapped up (the path is identical to
  the existing 30 FPS PrepTask cadence).
