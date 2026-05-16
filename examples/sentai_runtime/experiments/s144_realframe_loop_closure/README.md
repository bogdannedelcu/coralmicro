# s144 — Real Gazebo frame loop closure + telemetry observability

**Date**: 2026-05-16
**Predecessor**: s143 (loop closure with SYNTHETIC descriptors)
**Purpose**: Replace synthetic seeded images with REAL Gazebo camera
frames + add telemetry observability via journal for post-mortem.

> ⚠️ **STATUS: SUPERSEDED — migration required.**  This mission uses
> host-side Python orchestration (cflib + mission_explore.py drives
> the FSM via per-command REPL calls).  That architecture violates
> Sim.md §10 rule 0 + `[[missions-run-in-sentai-only]]` (operator rule
> 2026-05-16): missions must run inside sentai firmware, not on host.
> Drone control via host Python defeats the thesis claim "autonomous
> drone on MCU".  The infrastructure pieces shipped in this commit
> (`sentai.camera.grab_gray`, `sentai.places.get_desc`, `hex_helpers`
> real-frame variants) are kept because they are reusable by the
> migrated MP-side mission.  Mission script itself will be rewritten
> in `build-sim/sentai_fs_root/mission_s144.py` once
> `sentai.crazy.*` SIM transport (UDP CRTP) is in place.  See task
> migration roadmap.

## Why this exists

s143 proved drone can read from L3 gallery — but with deterministic
synthetic images, match score was 100% trivially.  s144 captures
**real Gazebo camera frames** at each place (different bytes per
visit, lighting/pose noise) and verifies the descriptor pipeline is
robust enough to recognize the place under natural variation.

Plus s143 surfaced a "telemetry goes stale mid-lap-2" issue with no
debug info.  s144 adds **journal-driven event capture** so any
flakiness leaves a parseable trail.

## What this commit adds

### SIM-side (40 LoC, refolosesc existing)

```c
sentai.camera.grab_gray(w=320, h=240) → dict | None
  // Pulls the latest Gazebo frame from sim_camera_latest_rgb,
  // resizes with sim_resize_rgb888_nearest (existing pipeline helper),
  // converts RGB→Y via BT.601 fixed-point luma.
  // Returns { 'data': bytes(w*h), 'w': w, 'h': h, 'seq': frame_seq } or None.
  //
  // Per embeded.md §2 (NASA/JPL Power of Ten) + §4.1 (resource discipline):
  //   F1 invalid w/h (< 8 or > 640/480)    → None
  //   F2 no frame (Gazebo bridge offline)  → None
  //   F3 resize/conversion fail            → None
  // Bounded loops only; static scratch buffers (~1.5 MB BSS in SIM);
  // single-writer convention (REPL is sequential).
```

ARM port deferred to Stage 9 — will use `sentai_cam_get_raw` +
`sentai_pxp_scale` (PXP DMA) + BT.601 luma.  Same return type, same
contract.

### hex_helpers.py — new variants using real frames

```python
real_capture_and_store(x, y, z=0.0, w=80, h=60)  -> pid | -200..-202
real_query_at(x, y, w=80, h=60)                  -> dict | None
```

### Mission flow + telemetry observability

```
LAP 1 (build memory with real frames):
  cf2 takeoff at origin → CAPTURE PHYSICAL_ORIGIN
  journal "phys_origin", {x, y, z, tel_age_ms, n_cb}
  real_capture_and_store(0, 0, z) → pid_home (real Gazebo frame!)
  journal "store_home", {pid, frame_seq}
  inject 2 targets, explore.start, takeoff, foreach target:
    goto + INSPECT
    journal "pre_capture_t10", tel + state
    real_capture_and_store(tel.x, tel.y, tel.z) → pid
    journal "store_t10", {pid, frame_seq}
  return home

LAP 2 (consume memory with FRESH real frame):
  cf2 → target1 directly (s143-style)
  journal "lap2_pre_query", tel + state
  real_query_at(target1.x, target1.y) → match dict
  journal "match_result", {id, score, l1, frame_seq}
  # Crucially: this is a NEW Gazebo frame, NOT byte-identical to lap-1's
  # frame.  Match score will reflect descriptor's tolerance to
  # natural variation (lighting, pose drift, marker occlusion).

Land at origin.  Throughout: every phase boundary journals tel state.
```

## Pass criteria — HARD gates

```
state_final == "DONE"
aborts == 0
land_err_xy_vs_origin < 0.15

# REAL match claim (the substance):
match.id == pid_t1                       # correct place
match.score_pct >= 60                    # RELAXED from 95% (deterministic)
match.l1_dist > 0                        # PROVES new frame != stored (genuine matching, not bit-equality)

# Storage discipline (same as s143):
gallery_count_after_lap2 == gallery_count_after_lap1

# Telemetry health (NEW):
tel_max_gap_ms < 5000                    # callback never silent > 5s

# Frame freshness proof (NEW):
frame_seq_lap1 != frame_seq_lap2         # genuinely different captures
```

## Telemetry observability via journal

Mission script journals on every phase boundary:
- `phys_origin` after takeoff
- `pre_capture_<label>` before grab_gray
- `store_<label>` after places.add (with frame_seq)
- `tel_check_<n>` periodic tel state snapshots
- `lap2_pre_query` immediately before query
- `match_result` with full match dict + frame_seq

After mission, `verdict.py` parses journal:
- Detects "no telemetry callback in N ms" gaps
- Reports `tel_max_gap_ms` from callback timestamps
- Cross-references match outcome with frame_seq evolution

## What this proves vs s143

| Aspect | s143 | **s144** |
|---|---|---|
| Drone navigates 2-lap mission | ✓ | ✓ |
| L3 gallery populated + queried | ✓ | ✓ |
| Match score | 100% (deterministic) | **60-95%** (natural noise) |
| Descriptor tested with REAL variation | ✗ | **✓** |
| Telemetry health observable | ✗ | **✓ (journal)** |
| Frame seq verifies genuine recapture | ✗ | **✓** |

## What this does NOT cover (deferred)

- **PXP on ARM**: s144 is SIM-only.  ARM grab_gray (PXP DMA path)
  comes at Stage 9 bring-up.
- **Rotation invariance**: PHOG/GIST are orientation-sensitive.  If
  drone revisits target1 with significantly different yaw, match
  may degrade.  FFT-mag in s145 fixes.
- **Telemetry hardening**: s144 OBSERVES the issue but doesn't fix
  it.  Next iteration adds tel-freshness gate in `goto_xy_abs`.

## Files

```
s144_realframe_loop_closure/
├── README.md            # this file
├── mission_explore.py   # 2-lap with REAL frames + telemetry journal
├── verdict.py           # HARD gates including frame_seq + tel_max_gap
└── run.sh               # bootstrap + run + verdict
```

## Related

- `[[s143-loop-closure-shipped]]` — synthetic-descriptor predecessor
- `[[s142-hex-patrol-shipped]]` — first L3 mission population
- `[[sentai-sim-journal]]` — journal API used for telemetry log
- `[[sim-test-must-return-home]]` — closure rule preserved
- `[[test-must-be-relevant-to-claim]]` — match claim hardened with
  frame_seq + L1>0 gates
- ARM PXP path: `sentai_pxp_shim.h` (cross-platform shim, SIM = pure-C,
  ARM = PXP DMA) — reused on ARM port at Stage 9
