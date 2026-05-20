# s180 — ArUco vs WhyCon apples-to-apples bench (real M7 @ 800 MHz)

WBS: **OP-S10-W17-T7**.  Closes the question raised against W17 §4:
"6.00 ms vs 31.8 ms — comparison is not mere-cu-mere; ArUco emits
3D pose, WhyCon-lite emits only 2D ellipse."

## Claim under test

WhyCon brought up to **pose-emitting parity** with ArUco (Phase A
+ B + W1 + W2 + W3 + closed-form PnP-z) has a per-stage cost on
real M7 hardware that we can compare side-by-side with ArUco's
per-stage cost.  The comparison answers two questions:

1. With ID-decode + IPPE PnP stripped from ArUco, how close is the
   raw-detector cost vs WhyCon?
2. With W3 concentric + closed-form PnP-z added to WhyCon-lite, how
   much does it grow?

## What's measured

For each detector, the firmware emits per-stage DWT cycle counts
collected during one `detect()` call on a synthetic 4-marker frame
(320×240, 8 bpp).  Stages:

| ArUco | WhyCon |
|---|---|
| `t_thresh` — Bradley adaptive (rolling) | `t_a` — same |
| `t_flood`  — 8-conn label_components    | `t_b` — same |
| `t_quad`   — bbox/aspect/fill filter + extract_quad + convexity / minDistanceToBorder / minCornerDistance / CW winding | `t_w` — bbox + aspect + fill + eigenvalue axes |
| `t_decode` — perspective warp + Otsu + bit extract + dict lookup + hamming  (ID) | (no equivalent — WhyCon has no per-marker ID) |
| `t_pnp`    — IPPE_SQUARE PnP | `t_w3` + `t_pnp` — concentric inner-disc + closed-form `z = fx·d/(2·a)` |

## How to run

Board must be flashed with sentai_runtime build ≥ #1410 (the build
that introduces `sentai.aruco._stage_cyc()` and the WhyCon W3 + PnP
stages).  Then:

```
$ python3 bench_runner.py
```

Defaults to `/dev/ttyACM0`.  Writes `results.txt` to this folder.

## Pass criterion

This is an **instrumentation** experiment, not an algorithm gate.
Pass = the bench completes and emits a parsable per-stage table for
both detectors.  Numbers feed back into W17 §4 (the "fair table").

## Folder layout

- `README.md`           — this file (what's measured + how to run)
- `bench_runner.py`     — host-side REPL driver
- `results.txt`         — captured numbers (written by bench_runner)
- `run_*.log`           — per-run stdout capture

## Cross-refs

- W17 §4 — the table this experiment is going to update.
- W17 §6 — defines what WhyCon-lite IS NOT (W3, PnP, WhyCode, multi-
  marker pose); W17-T5 + W19-T2 close the first two of those gaps.
- W19-T1 plan — once the unified `sentai.markers` API lands, the
  bench will use it instead of two separate modules.
- HARD-RULE `[[experiments-in-own-folder-log-dead-ends]]` — bench
  output stays here even if a subsequent measurement supersedes it.

## Status

`#1410` initial commit, bench not yet run.
