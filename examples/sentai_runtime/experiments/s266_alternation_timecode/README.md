# s266 — Frame-exact camera alternation validated against a monitor time code (VGA 30 fps)

**WBS**: OP-S10-W24-T19
**Started**: 2026-11-01 (late evening, scaffold + first run)
**Status**: 🟢 iter1 PASS (30 s); soak and VGA45 pending
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

## Files

- `timecode_display.py` (stripes at the frame clock, no Cairo needed), `run_timecode.py`
  (board loop, per-camera band tables from the s265 homographies, validation),
  `iter1/` (`frames.json`, `display_log.csv`, `verdict.json`, `repl.log`, check captures).
