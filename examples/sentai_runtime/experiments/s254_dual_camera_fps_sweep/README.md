# EXP-s254 — dual-camera alternation at VGA30 and VGA60: stability and drift

**WBS:** OP-S10-W23-T13 (operator request 2026-10-31: "test at 30 fps and at
60 fps, see whether the system holds and how much it drifts; all constants
must scale with the camera fps").  **Board:** rev 2026-10 (id 58eb), build
1678 RAM-loaded.  **Follows:** [s251](../s251_camera_genlock/) (VGA45 genlock,
deferred MUX flip, pre-trim + on-the-fly autotune).

## fps-derived constants (build 1676+)

Nothing in the phase path is hard-wired to VGA45 any more:

| quantity | source |
|---|---|
| sensor period T | measured from the FB-done log (`period_us`), fallback 1e6/fps |
| VTS (frame length, lines) | read from `TIMING_VTS` whenever the trim code touches it (`vts_lines`) |
| VBLANK | `(VTS − 960 active rows) · T / VTS` → `vblank_us` in `camera.stats()` |
| drift warn / clear | VBLANK/2 and VBLANK/4 (was fixed 270/150 µs) |
| pre-trim lines | `410 µs SCCB gap / (T/VTS)` → 12 @30, 18 @45, 24 @60 |
| trim timing | 0.25 T after the sensor's frame edge, restore 1.25 T later |
| MUX flip delay | 40 µs absolute (≥ 10× the Frame-End race margin, < VBLANK/2 even at VGA60) |

## Method

`soak_phase.py --fps N --minutes 2.5`: cold boot, `camera.init(1, N)` with
`genlock_auto` mode 2 (pre-trim, then measured verify), `ratio(1,1)` with a
consumer for five 30 s windows, `stats()` per window, `phase()` at the end.

## Result (build 1678, `run_summary_b1678.txt`, `results_soak_vga*_b1678.csv`)

| fps | T (µs) | VTS | VBLANK (µs) | pre-trim | verify | consumed / sensor per 30 s | fps | phase over 2.5 min | drift / stall |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 30 | 33 436 | 984 | 815 | 12 lines | −23 µs, 0 steps | 897 / 898 | 29 | −23 … −22 µs | 0 / 0 |
| 45 (s251) | 22 288 | 984 | 543 | 18 lines | −3 … −35 µs, 0–1 steps | 1343 / 1345 | 44 | −13 … −12 µs (5 min) | 0 / 0 |
| 60 | 16 718 | 984 | 407 | 24 lines | −14 µs, 0 steps | 1794 / 1795 | 59 | −14 … −13 µs | 0 / 0 |

- Full sensor rate at every fps (one frame per window lost to the host's
  REPL round-trip, counted in `stale_skips`).
- The phase does not move at any fps: ±1 µs over 2.5 minutes, consistent
  with the shared 24 MHz XCLK.  The on-the-fly autotune never had to act.
- VTS is 984 lines at all three rates (the OV5640 tables change the PLL,
  not the frame length), so VBLANK scales exactly with T.

## Files

`results_soak_vga30_b1678.csv`, `run_soak_vga30_b1678.log`,
`results_soak_vga60_b1678.csv`, `run_soak_vga60_b1678.log`,
`run_summary_b1678.txt` (also holds the s249 15/15 run of the same build).
Script: `../s251_camera_genlock/soak_phase.py`.
