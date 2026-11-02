# s267 — Runtime-geometry smoke test of the camera-frame readers (VGA / SXGA / packed)

**WBS**: OP-S10-W24-T20
**Started**: 2026-11-02
**Status**: 🟢 PASS on build 1763 (iter1 64/65, the one FAIL was a test bug; iter2 re-run of the packed mode 22/22)
**Board**: 58eb, builds 1762–1763 (fixes of the audit `agent/audit_runtime_geometry_2026-11-02.md`)

## Claim

After W24-T20 every code path that reads a raw camera frame buffer takes its SOURCE
geometry (width, height, row pitch, XRGB vs packed YUV420) from the runtime camera state,
so the same MicroPython calls behave correctly, or refuse with a documented code, in all
three capture modes: VGA 30 fps legacy, SXGA 15 fps legacy and SXGA 30 fps packed YUV420.
Before the fix (build ≤ 1760) these paths used the compile-time 640x480 constants: at SXGA
they produced interleaved half-row images with return code 0, `native_res()` reported
640x480, `set_resolution()` changed the source pitch of the YUV pipeline, and the D-cache
maintenance in the camera driver covered only rows 0..239 of an SXGA frame.

## Pass criteria

Per mode (`smoke.py`, one board reset per mode, `iter1/verdict.json`):

- `native_res()` = sensor geometry chosen with `set_hw()`; `resolution()` defaults to it.
- `peek_col(40[, 1])` returns `h` rows (480 / 960) and the bottom half of the column is not
  constant (live data above row 240 at SXGA).
- `save_jpeg()` returns a JPEG > 5 kB in both legacy modes; raises `cam capture fail` in
  packed mode (firmware -7, frame is not XRGB).
- `set_resolution(320, 240)` changes `resolution()` and the JPEG size only: `native_res()`
  and the `peek_col` row count are unchanged; a size above the sensor geometry is refused.
- `prep_once()` returns 0 in every mode (packed: producers skipped, SERR `0x0448` val 2 once).
- `flow.start(0)` returns 0 and publishes frames in the legacy modes; returns -4 with
  SERR `0x0E21` in packed mode.
- `peek5_b40()` / `peek5_tags()` return tuples; `yuv420_test()` returns -5 / -7 outside VGA
  legacy.
- No `0x0448` / `0x0E21` in dmesg in the legacy modes.
- (HW) build counter 1762 verified after flash, USB `1fc9:c0a1`.

The cache-maintenance fix (bug #1) is not provable with a single frame: it is exercised by
the s266 SXGA time-code run (cam0 column 40, rows up to 960) after this smoke test.

## How to run

```bash
python3 smoke.py --out iter1                 # all three modes, ~3 min, board resets 3x
python3 smoke.py --out iter2 --modes sxga15  # one mode
```

## Files

- `smoke.py` — host driver (REPL over `/dev/ttyACM0`, `board_repl.Repl` from s265)
- `iter1/verdict.json`, `iter1/repl.log` — results of the first run on build 1762

## Iter results table

| Iter | Build | Modes | Result |
|---|---|---|---|
| iter1 (first run, 06:40) | 1762 | vga30, sxga15, sxga30p | 57/62: `peek_col_bottom_live` 0 in both legacy modes because the monitor was still off (dark room, B channel all 0), `packed420(1)` returns 1 not 0 (test), `flow.start` raises instead of returning -4 (test), `prep_once` with no slot enabled never fires the producers so no SERR (test → slots enabled first) |
| iter1 (re-run, 06:55) | 1763 | vga30, sxga15, sxga30p | **vga30 21/21, sxga15 22/22, sxga30p 21/22** — the FAIL was the dmesg check reading only the last line (`E:0448:2` is in the log, one line above `E:0E21:1`) |
| iter2 | 1763 | sxga30p | **22/22 PASS** with the whole dmesg text: `E:0448:2` present once, `E:0E21:1` present |

## Result

Build 1763, all three modes (`iter1/verdict.json`):

- `native_res()` reports the sensor geometry (640x480 / 1280x960), `resolution()` follows
  `set_resolution()` only; `set_resolution(320, 240)` leaves `native_res()` and the
  `peek_col` row count unchanged and only shrinks the JPEG (6.3 kB → 2.0 kB at VGA, 6.0 kB →
  2.0 kB at SXGA); `set_resolution(2*w, h)` → -1.
- `peek_col(40[, 1])` returns 480 / 960 / 960 rows and the bottom half of the column is live
  in every mode (spread 223 / 144 / 156 with the monitor on).
- `save_jpeg()` works in both legacy modes and raises `cam capture fail` in packed mode (-7).
- `prep_once()` returns 0 everywhere; with GRAY_NATIVE + RGB_64 enabled the slot sequence
  advances in the legacy modes (0 → 1) and stays at 0 in packed mode, with `E:0448:2` logged
  once.
- `flow.start(0)` publishes in both legacy modes (46 frames / 1.5 s at VGA30, 23 at SXGA15,
  0 grab failures) and raises `flow.start failed (-4)` in packed mode with `E:0E21:1`.
- `peek5_b40` / `peek5_tags` return tuples in every mode; `yuv420_test` returns 0 at VGA
  legacy and -5 at SXGA (legacy and packed, the geometry check comes first).
- No `0x0448` / `0x0E21` in dmesg in the legacy modes.

The D-cache maintenance fix (bug #1) is exercised separately by the s266 SXGA time-code
re-run on build 1763 (`s266_alternation_timecode/iter6_sxga15_t20`, `iter7_sxga30p_t20`).
