# Iter09 - Dual-Camera Pipeline Alternation

Build: `SentAI v1.0 build 1544 (2026-06-24 11:07:59)`

Scope:

- 512x512 YOLO model;
- `sentai.camera.ratio(1,1)`;
- direct tensor pipeline;
- 100 measured detection results after warmup;
- fresh reset between drain conditions.

Result:

| ratio | drain | observed | cam0 | cam1 | frame FPS | cam0 FPS | cam1 FPS | invoke FPS | failures |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1:1 | 1 | 100 | 50 | 50 | 14.392763 | 7.196382 | 7.196382 | 14.392763 | 0 |
| 1:1 | 2 | 100 | 50 | 50 | 4.981067 | 2.490534 | 2.490534 | 4.981067 | 0 |

Notes:

- The run script currently leaves the pipeline active after writing the CSV.
  For drain=1 this produced post-result `0B62` logs until the board was reset.
- `pipeline_alternation_summary.csv` is generated from the host logs. The full
  board CSVs remain on board under `/diags/s235_b10_alternation/`.
