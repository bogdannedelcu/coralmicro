# s100 — PX4 + Gazebo + x500_sentai smoke

Phase 6d step 0: verify the new `x500_sentai` PX4 model spawns into
our `sentai_crazysim_world.sdf` and the downward camera publishes
frames on the gz topic `/downward_cam/image`.

NO flow, NO EKF tuning, NO takeoff — just "drone is in the world and
its camera works".  Once this passes we wire EKF flow-only nav (s101)
and run the hover-stability bench (s102/s103/s104 — no-wind, half,
full wind).

## Pass criteria

1. `gz topic -l` lists `/world/sentai_crazysim_world/...` topics.
2. `gz topic -l` lists `/downward_cam/image`.
3. `gz topic -e -t /downward_cam/image -n 3` returns 3 non-empty frames
   in under 5 s (640×480 RGB888 ≈ 920 KB each).
4. The PX4 log shows `Startup script returned successfully` and at
   least one `EKF2` task running (sim sensors hooked up).

## How to run

```bash
bash examples/sentai_runtime/experiments/s100_px4_gz_smoke/run.sh
```

Output → `/tmp/sentai_s100_<stamp>/{gz.log,px4.log,frame_dump.txt}`.
