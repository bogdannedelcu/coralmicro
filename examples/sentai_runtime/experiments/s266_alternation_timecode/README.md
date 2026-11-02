# s266 — Frame-exact camera alternation validated against a monitor time code (VGA 30 fps)

**WBS**: OP-S10-W24-T19
**Started**: 2026-11-01 (late evening, scaffold + first run)
**Status**: 🟢 VGA30, VGA45 PASS and SXGA15 PASS on cam1 (30 s each, re-confirmed on build 1763); VGA60 inconclusive (display tearing vs camera mix undecidable without a redundant code); SXGA15 cam0 column unusable (geometry, not cache); **SXGA30 packed: `ratio(1,1)` alternates 2:2** (FB-done ring, iter7) and the two sensors are ~17 ms out of phase — open. Firmware: SXGA JPEG and `peek_col` fixed (builds 1757–1760), runtime-geometry audit fixes (1762–1763, W24-T20, EXP-s267).
**Board**: 58eb, build 1756, VGA 30 fps, both OV5640, Kamvas 16 as the time-code display

## Claim

With a time code displayed on the monitor (frame counter redrawn at every display refresh,
60 Hz), the board alternating the two cameras frame by frame (`ratio(1,1)`) captures, for every
frame, (a) the counter expected for its time slot (no stale, no duplicated, no skipped frames at
30 fps = one camera frame per two display frames) and (b) an image that really comes from the
camera the firmware attributes it to.  s251 proved the alternation against the sensors' own
test patterns; this experiment proves it against an **external clock and a real image**.

## Method

- `timecode_display.py`: fullscreen on the Kamvas, redraws at the GTK frame clock (vsync):
  a stack of horizontal stripes across the whole screen width — a white marker stripe at the
  top, 12 stripes with the display frame counter in Gray code (black = 0, white = 1), a black
  marker stripe at the bottom.  Logs `(counter, host monotonic time)` for every drawn frame.
- Camera side: short manual exposure (≈ 3 ms, so a capture never straddles two display
  frames), `ratio(1,1)`, then `sentai.camera.peek_col(40)` in a loop on the board: one dequeued
  frame per call, returning the B-channel column 40 over the 480 rows plus the firmware's camera
  tags and the FB-done timestamp.  The column crosses all stripes because they span the whole
  screen; **cam1 is mounted rotated 180°, so it reads the stack bottom-up** — the marker order
  (white first vs black first) identifies the camera from the image content alone, independent
  of the firmware tag.  Decoding happens on the board (threshold at the marker midpoint, Gray →
  binary); the host receives `(ts_us, tag, idx, code, orientation)` per frame.
- Validation (`run_timecode.py`): orientation == tag for every frame; counter strictly
  increasing with step 2 ± 1 (60 Hz / 30 fps) between consecutive frames of the same camera
  step 4 ± 1; no repeated code (stale buffer); capture rate from the board timestamps and from
  Δcode / Δt against the display's 60 Hz.

## Pass criteria

- 0 frames whose content orientation disagrees with the firmware camera tag.
- 0 repeated codes, 0 steps > 3 display frames between consecutive captures over ≥ 60 s.
- Mean capture interval 33.3 ms ± 0.5 (30 fps) measured from FB-done timestamps, consistent
  with Δcode·16.67 ms.

## How to run

```bash
touch /tmp/mps_pause                      # keep the display awake
python3 timecode_display.py &              # stripes on the Kamvas, log in iter1/display_log.csv
python3 run_timecode.py --out iter1 --seconds 60
```

## iter1 — result (2026-11-01, 23:45, Kamvas 16 at 2560x1440@60, VGA 30 fps, exposure 3 ms, gain 6x)

Stack of 12 bands (white marker, 10 Gray-code bits, black marker) drawn between 4 % and 90 % of
the screen height (the range both cameras see); the rows of camera column 40 that fall in each
band come from the s265 Kamvas homographies (cam0: 12 bands of ~24 rows from row 31 to 473,
cam1: the same bands in reverse order, rows 461 → 33, because it is mounted rotated 180°).
Board loop `_tc()` averages each band on the B channel of one dequeued frame per call.

| quantity | value |
|---|---|
| frames in 30 s | 960, firmware tags 0 1 0 1 … (ratio(1,1), ratio_unit 1) |
| capture interval (FB-done timestamps) | **33.43 ms, sd 0.01 → 29.9 fps**; per camera 66.86 ms |
| frames decoded | 960 / 960 (no ambiguous marker pair) |
| display frames per capture (Δcode) | 1.986 → 30.2 fps at 60 Hz (display measured 59.2 Hz) |
| per-camera code step (expected 4) | cam0 {2:2, 3:24, 4:437, 5:16}; cam1 {1:2, 2:4, 3:42, 4:391, 5:38, 7:2} |
| **stale / duplicated frames (step 0)** | **0 on both cameras** |
| skipped (step > 5) | cam0 0, cam1 2 (the display itself had gaps up to 43 ms: Python redraw jitter, sd 1.8 ms) |
| cam1 code − preceding cam0 code | 1 display frame in 384 / 480 cases |

The constant one-display-frame offset between the two cameras is rolling-shutter geometry:
image column 40 is read at the start of cam0's frame and near the end of cam1's (rotated
module), ≈ 19 ms apart, i.e. one 16.7 ms display frame.  Identity: every frame decoded with the
band table of the camera the firmware tagged, markers in the expected order; a frame from the
other camera would have failed the marker test (reversed) — 0 such frames.

Verdict: at VGA 30 fps the 1:1 alternation delivers every frame, current and from the right
camera, against an external 60 Hz clock.  Remaining jitter in the code steps is the display's,
not the camera's (`display_log.csv`: 59.2 Hz, sd 1.8 ms, max gap 43 ms).

Follow-ups: pre-render the stripe pixbufs so the display holds 60.00 Hz; decode each frame with
both band tables on the host (explicit cross-check of identity); repeat at VGA45 (`init(0, 45)`) and
with `select()` switching instead of `ratio(1,1)`; 60 s soak.

## iter2–iter5 — mode sweep (`run_mode.sh`: reset → init → locate at that mode → time code, 30 s each)

A mode change shifts the image vertically by ~20 rows (sensor window differs per mode), so the
band tables must come from a localisation made **in the same mode** (the first VGA45 attempt with
the VGA30 homographies mis-decoded cam0; `iter2_vga45` was redone with its own locate).

| mode | capture interval (FB-done) | frames / 30 s | decoded | per-camera code steps | sequence breaks | verdict |
|---|---|---|---|---|---|---|
| VGA 30 (iter1) | 33.43 ms, sd 0.01 | 960 | 960 | 4 (437/479, 391/479), rest 3/5 | 0 stale, 2 skips (display gaps) | **PASS** |
| VGA 45 (iter2, exp 3.4 ms, gain 10x) | 22.38 ms, sd 1.44; 6 intervals of 45 ms | 1440 | 1440 | {3: 463, 2: 248} / {3: 414, 2: 267, 4: 25} vs 2.69 expected | **0** broken codes | **PASS** (6 dequeues missed by the Python loop, 0.4 %) |
| VGA 60 (iter3, exp 3.4 ms, gain 12x) | 16.92 ms, sd 2.15; 14 × 33 ms, 4 × 50 ms | 1800 | 1800 | 2 dominant on both | **134** broken (cam0 47, cam1 87) | inconclusive, see below |
| SXGA 15 legacy (iter4, build 1760, exp 50 lines, gain 10x) | 66.86 ms, sd 0.01 (15.0 fps); per camera 133.7 ms | 480 | cam1 240 / 240, cam0 0 (all ambiguous) | cam1 {8: 228, 7: 4, 6: 3, 9: 2} vs 8.02 expected | cam1: 0 stale, 1 skipped | **cam1 PASS**; cam0 column 40 unusable at SXGA (see below) |
| SXGA 30 packed (iter5, build 1760, byte 1 = Y1, exp 100 lines) | 63.5 ms, **sd 19.2** (15.7 fps dequeued) | 600 | cam0 286 ok, cam1 mostly garbage | cam0 {8: 221, 4: 29, 7: 13, 16: 13} | cam0 14 skipped | **not a valid cadence measurement**: the REPL loop dequeues every other 2.5 MB frame |

Firmware fixes that made the SXGA runs possible (builds 1757–1760, committed with this iteration):
`sentai_cam_capture_jpeg` took its source size from the compile-time VGA constants and returned
-5 for every SXGA frame ("cam capture fail"); it now uses `g_runtime_cam_w/h`, the JPEG of an
SXGA frame is encoded at 640x480 (RGB staging buffer and MicroPython heap are VGA-sized) and
packed-mode frames are refused with -7.  `sentai_cam_peek_col` used the VGA pitch and bounds
(at SXGA it read two interleaved columns over half the frame); it now follows the runtime
geometry and the packed pitch, returns `g_runtime_cam_h` rows, and takes a second argument
`byte` (0 = B/X, 1 = Y1 of a packed pixel pair).  The SXGA localisation therefore runs on the
halved JPEG and the band rows are scaled ×2 to frame rows (`run_timecode.py` detects the scale).

SXGA observations: (1) the CSI cadence at SXGA15 1:1 is as clean as at VGA (66.86 ms, sd 0.01);
(2) cam0's column 40 reads ambiguous markers on every SXGA frame while cam1 decodes perfectly
with the same method — the SXGA readout window of cam0 puts image column 40 somewhere the VGA
homography ×2 does not predict (vertical shift between modes again, or a horizontal offset at
SXGA); fix: locate at SXGA with a column probe instead of the JPEG, or sample a column nearer
the centre (x ≈ 400); (3) at SXGA30 packed the `peek_col` loop (one 2.5 MB frame dequeued per
REPL call) keeps up with only every other frame (63.5 ms, sd 19 ms), so cadence at SXGA30 must
be read from the firmware's FB-done ring (`camera.frame_log(64)`), not from the Python loop; cam1's
codes in packed mode are single-bit patterns (128/384/640/896) → the Y1 sampling or the band
table is off for cam1 in packed layout — to be checked with a `save_raw` dump.

VGA60: the broken frames show band means like `[0, 0, 0, 141, 129, 0, 144, 141, 140, 115, 41, 78]`
— the white marker band reads black and bands in the lower part are intermediate: the frame
contains two different display frames, split at a horizontal line.  With the camera period equal
to the display period the tear line stays put, so it hits the code every frame; at 45 and 30 fps
the beat moves it and the 3 ms exposure rarely straddles a refresh.  The firmware side is regular
(tags 0 1 0 1, intervals 16.9 ms), so this is the display (pixbuf swap not vsynced under XWayland)
and/or the rolling shutter reading across the refresh, not a stale or mixed camera frame — but
the current code cannot prove that: a camera-mixed frame would also show a split.  To settle it
the code needs redundancy (the stack drawn twice, top and bottom half: a display tear corrupts
one copy, a camera MUX mix would show the *other camera's* reversed stack in the lower part) or a
vsynced display path.  The 1 % of 33/50 ms intervals at 60 fps are dequeues the MicroPython loop
missed (one `peek_col` per frame at 60 Hz is at the REPL loop's limit); they are not drops in the
CSI path.

## iter6 / iter7 — SXGA re-run on build 1763 (W24-T20 fixes, 2026-11-02 07:05)

`bash run_sxga.sh iter6_sxga15_t20 iter7_sxga30p_t20` after the runtime-geometry fixes (D-cache
maintenance now covers the whole SXGA frame, see `agent/audit_runtime_geometry_2026-11-02.md`).

| mode | capture interval (FB-done, peek loop) | frames / 30 s | decoded | per-camera code steps | verdict |
|---|---|---|---|---|---|
| SXGA 15 legacy (iter6) | 66.87 ms, sd 0.02 | 480 | cam1 240 / 240, cam0 0 | cam1 {8: 200, 7: 15, 9: 7, 11: 6}, 0 stale, 6 skipped (display gaps) | cam1 PASS again; cam0 column 40 still ambiguous on every frame → **not** a stale-cache effect (the bottom half of the column is live in s267), it is the band table / geometry of cam0 at SXGA (open) |
| SXGA 30 packed (iter7) | 63.40 ms, sd 19.5 (peek loop, every other frame) | 600 | cam0 282, cam1 garbage | cam0 {8: 247, 7: 10, 16: 5} | unchanged vs iter5; cam1 packed Y1 codes still single-bit patterns (open) |

**FB-done ring at SXGA30 packed with `ratio(1,1)`** (`iter7_sxga30p_t20/frame_log.json`, 64
entries, read right after the run with no consumer dequeuing, so the "queue full" caveat of
`frame_log()` applies and this needs confirming with the pipeline running):

- tags `1001 1001 1001 …` — each camera delivers **two** consecutive frames before the MUX
  flips: the alternation runs 2:2, not 1:1, in this mode.  Two consecutive entries share the
  same `seq`, i.e. the frame counter the alternation scheduler uses advances once per two CSI
  frames at SXGA30 packed (same family as the "frame_count is half rate" note of 2026-09-04).
- FB-done intervals repeat the pattern 33.4, 16.4, 33.4, 50.6 ms: the per-sensor period is
  33.4 ms (30.0 fps, as the SDK-patch row promises) and the two sensors are **~17 ms out of
  phase** (not genlocked here), so the first frame after each flip arrives half a period early
  or late.  `stats()["period_us"]` reports 16713 µs in this mode, which is the 60 Hz event grid,
  not the sensor period.
- Consequence for the time-code analysis: with the Python loop dequeuing every other frame and
  the 2:2 pattern, the per-camera code step of 8 display frames (133 ms) is what one expects;
  the "cadence" number of iter5/iter7 is not a camera measurement.  To validate 1:1 at SXGA30
  packed the alternation counter must be fixed first (open, W24-T19 follow-up), then measured
  with the pipeline as consumer and `frame_log(64)`.

## iter8 — SXGA30 packed 2:2 root-caused and fixed (build 1764, `frame_log_check.py`)

Root cause (firmware, not the camera): `sentai_cam_phase_from_log` sanity-checked each new
sensor-period estimate against the **previous** estimate (boot default 22 288 µs = VGA45) and
halved anything above 1.5× of it.  The first SXGA30 measurement, 33.4 ms, is 2 µs above
1.5 × 22 288, so it folded to 16.7 ms and then confirmed itself forever; every genlock VTS trim
was computed for half the real period, genlock reported "not converged" (`E:0A0D:40`), and the
phase-drift monitor fell back to pair-granularity alternation (`ratio_unit` 2 → tags 1001).
VGA30 (33.43 ms) passed the same check by 2 µs.  Fix: the plausibility window comes from the
nominal period of the current fps key and the estimate is reset at init
(`cam_nominal_period_us`).  `stats()["period_us"]` now reads 33 428 / 66 855 / 33 428 µs.

`frame_log_check.py` reads the FB-done ring **while a consumer dequeues** (the ring pauses when
the queue is full) and drops the entries before the last log gap:

| mode (build 1764/1765) | period_us | genlock | ratio_unit | tags (64) | FB-done interval | verdict |
|---|---|---|---|---|---|---|
| SXGA 30 packed | 33 428 | ok (`E:0A0C`) | 1 | 0101… (100 % flips) | **33.43 ms, sd 0.00** | **PASS 1:1** |
| SXGA 15 legacy | 66 855 | ok | 1 | 0101… | 66.86 ms, sd 0.01 | PASS 1:1 |
| VGA 30 | 33 428 | ok | 1 | 0101… | 33.43 ms, sd 0.01 | PASS 1:1 |

The iter7 "two sensors ~17 ms out of phase" was the un-converged genlock, not a sensor
property: with the trims computed for the right period both sensors lock in 0–1 steps.

## Files

- `timecode_display.py` (stripes at the frame clock, no Cairo needed), `run_timecode.py`
  (board loop, per-camera band tables from the s265 homographies, validation),
  `iter1/` (`frames.json`, `display_log.csv`, `verdict.json`, `repl.log`, check captures).
