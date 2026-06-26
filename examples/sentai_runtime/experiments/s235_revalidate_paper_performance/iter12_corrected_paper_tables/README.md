# Iter12 - Corrected Paper Tables

This iteration consolidates A10/B10 into paper-ready replacement tables for
the runtime-performance chapter.  No new board measurements were taken here;
all values trace back to earlier `s235/iterNNN_*` artifacts.

## Table Replacement Policy

The old optimization-history table should not be reconstructed from historical
firmware states for the paper.  Those rows were mixed with invalid timing
domains.  Replace it with a corrected current-firmware table that reports what
was actually revalidated with board wall-clock timing.

## Corrected Overall Progression

| Capability | Old paper value | Corrected value | Status | Source |
| --- | ---: | ---: | --- | --- |
| Standalone TPU, 512x512 YOLO | 73-75 FPS | 24.50 FPS | superseded | iter03 |
| Single-camera 512x512 pipeline | 40-42 FPS | 23.44 FPS | superseded | iter06 |
| Multi-patch 2 invokes/frame | 48.5 TPU FPS | 24.02 invoke FPS, 12.01 frame FPS | superseded | iter07 |
| Multi-patch 4 invokes/frame | 56.0 TPU FPS | 24.25 invoke FPS, 6.06 frame FPS | superseded | iter07 |
| Continuous dual-camera `ratio(1,1)` | 19.5 FPS / 21 FPS historical | 14.39 FPS combined, 7.20 FPS/cam | corrected | iter09 |
| Manual switch `select -> to_tensor`, drain=1 | n/a in final table | 93.89 ms | corrected | iter08 |

## Corrected TPU Chunk-Size Table

| Chunk size | Wall ms/invoke | FPS | Input wait ms/invoke | Instruction wait ms/invoke |
| ---: | ---: | ---: | ---: | ---: |
| 33 KB | 41.547539 | 24.068813 | 24.603979 | 14.598960 |
| 36 KB | 41.428139 | 24.138182 | 24.508600 | 14.572780 |
| 64 KB | 41.001480 | 24.389364 | 24.244680 | 14.435160 |
| 128 KB | 40.813641 | 24.501614 | 24.151621 | 14.361740 |

Conclusion: the old 33 KB optimum and 73-75 FPS value were not reproduced under
wall-clock measurement.  The current matrix is tightly clustered around 24 FPS,
with 128 KB slightly fastest on build 1543.

## Corrected Standalone Invoke Breakdown

| Component | Corrected wall time |
| --- | ---: |
| USB bulk OUT: input tensor | 24.151621 ms |
| USB parameters header | 0.137 ms historical carry-over / 0.09 ms DWT row superseded |
| USB bulk OUT: instructions | 14.361740 ms |
| USB bulk IN: output | 1.704 ms carry-over |
| USB event readback | 0.038 ms carry-over |
| Invoke total | 40.813641 ms |

Use the full URB table from iter03/iter02 in the appendix if exact per-phase
rows are needed.  Do not use the old `14.19 ms` DWT table as wall time.

## Corrected Single-Camera Pipeline Table

| Run | Wall us / 100 outputs | FPS | Timeouts | Infer failures |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 4390419 | 22.776869 | 0 | 0 |
| 1 | 4235336 | 23.610878 | 0 | 0 |
| 2 | 4235444 | 23.610275 | 0 | 0 |
| 3 | 4235558 | 23.609640 | 0 | 0 |
| 4 | 4234778 | 23.613989 | 0 | 0 |

Mean: `23.444330 FPS`, sample sigma: `0.373126 FPS`.

## Corrected Multi-Patch Table

| prep_fps | invokes/frame | mode | Frame FPS | Invoke FPS | Failures |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 1 | 0 | 23.610554 | 23.610554 | 0 |
| 30 | 1 | 0 | 23.614239 | 23.614239 | 0 |
| 30 | 2 | 1 | 12.008011 | 24.016022 | 0 |
| 20 | 3 | 1 | 8.058186 | 24.174555 | 0 |
| 15 | 4 | 1 | 6.062988 | 24.251953 | 0 |

Conclusion: multi-invoke preserves the requested invokes/frame ratio, but total
TPU invoke throughput remains near 24 invokes/s.  The previous 48.5-56.0 TPU
FPS claims are superseded.

## Corrected Dual-Camera Table

| Mode | Drain | Combined FPS | Per-camera FPS | Correctness verdict |
| --- | ---: | ---: | ---: | --- |
| Continuous `ratio(1,1)` | 1 | 14.392763 | 7.196382 | valid |
| Continuous `ratio(1,1)` | 2 | 4.981067 | 2.490534 | invalid for continuous alternation |

Visual/content validation:

| Drain | Frames | Correct | Mismatch | Mixed | Verdict |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 2 | 2 | 0 | 0 | valid for continuous alternation |
| 2 | 2 | 1 | 1 | 0 | invalid for continuous alternation |

`drain=2` should not be presented as the conservative continuous-alternation
setting.  It can slip phase because the ISR ratio scheduler flips again before
the consumer's two-frame drain completes.

## Corrected Latency-Facing Values

| Quantity | Value | Status |
| --- | ---: | --- |
| Single-camera output period | 42.654 ms | derived from measured FPS |
| Standalone TPU invoke | 40.813641 ms | measured |
| Continuous 1:1 combined output period | 69.479 ms | derived |
| Continuous 1:1 per-camera update period | 138.959 ms | derived |
| Manual `select -> to_tensor`, drain=1 | 93.891 ms | measured |

Use these as update-period bounds unless a future same-frame timestamp trace is
added through capture, prep, invoke, and publication.

