# s151 — Real-frame loop closure MP-only port of s144

**Date**: 2026-05-16

## What this migrates

s144 (was marked SUPERSEDED, host-orchestrated) → **MP-only**.
Same loop-closure pattern as s150 but descriptors come from **real
Gazebo camera frames** via `sentai.camera.grab_gray()` instead of
synthetic hex_image.

Tests whether the PHOG+GIST descriptor pipeline is robust to natural
variation across two laps (lighting, pose drift, texture aliasing).

## Files

```
s151_realframe_loop_closure_mp/
├── README.md
├── mission_s151.py
└── verdict.py
```

Reuses: `crtp_log.py` (s146), `hex_helpers.py` (s142 — provides
`real_capture_and_store` + `real_query_at`).

## Prerequisites

Camera bridge must be UP — i.e. `gz_to_uds_bridge` running inside
crazysim-garden distrobox, feeding `/downward_cam/image` from Gazebo
into `/tmp/sentai_cam.sock`.  Without it, `sentai.camera.grab_gray()`
returns `None` and the mission reports `FAIL_NO_CAMERA`.

The `s127_flowbaseline/run.sh` launches the bridge as a side-effect;
once you've run that or s146, you can run this mission.

## Hard gates

| Gate | Threshold |
|---|---|
| G1 status | == DONE |
| G2 phases | all 10 fired (extra `camera_probe` vs s150) |
| G3 lap1_real_stores | ≥ 3 |
| G4 all pids ≥ 0 |  |
| G5 lap2_real_matches | ≥ 3 |
| G6 matches_ok | ≥ 1/3 (relaxed vs s150 — real frames are noisier) |
| G7 closure_xy | < 10 cm |
| G8 errors empty |  |
| G9 journal events |  |

## Why G6 is relaxed

s144 README documented empirically: synthetic descriptors round-trip
3/3 deterministically; real Gazebo frames round-trip 1-2/3 typically
because frame_seq differs across laps (different bytes → different
descriptor → match score < 100%).

If you want stricter gates, use s150 (synthetic) for descriptor-pipeline
regression and s151 only for "real frame integration sanity".
