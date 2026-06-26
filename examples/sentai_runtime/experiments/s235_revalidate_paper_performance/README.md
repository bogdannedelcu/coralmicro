# s235 - Revalidate Paper Performance Chapter

Mission for `TD-S10-A10/B10`.

Goal: rerun the paper Section 4.2 performance experiments with valid
wall-clock timing and table-level traceability.

Authoritative todo documents:

- `todo/TD-S10-A10_revalidate_experiments.md`
- `todo/TD-S10-B10_revalidate_performance_chapter.md`

Iteration convention:

- `iter01_table_inventory` maps every paper row to old evidence and timing
  source.
- Later iterations rerun each table or table family.
- `iter12_corrected_paper_tables` consolidates the paper-ready replacement
  tables and superseded-claim list.

Carry-over result from `s234_tpu512_timing_recheck`:

- 512x512 yolo_1 standalone EdgeTPU wall time is about `41.514 ms/invoke`
  (`24.09 FPS`) on build 1542, not the old DWT-derived `14.19 ms/invoke`.

Current corrected headline values:

- standalone 512x512 TPU: `24.50 FPS` on build 1543 (`128 KB` chunk row);
- single-camera 512x512 pipeline: `23.44 FPS` on build 1544;
- multi-patch total invoke throughput: about `24 invokes/s`;
- continuous dual-camera `ratio(1,1)`: `14.39 FPS` combined with validated
  `drain=1`;
- `drain=2` is invalid for continuous `ratio(1,1)` on build 1544 because it
  can produce tag/content phase errors.
