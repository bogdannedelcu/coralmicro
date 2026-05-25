# TD-S10-B2 100-Frame WhyCon Ablation Summary

Source dataset:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_114246/
```

Sentai output:

```text
sentai_sim_20260523_114846/
sentai_sim build #558 (2026-05-23 11:27:12)
```

## Standard Metric

Standard metrics exclude `partial_crop` manifest markers from expected counts
and ignore detector outputs that match those cropped markers.

| Backend / variant | Matched | Expected | Recall | False positives | Ignored cropped detections | Complete frames |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| OpenCV paper | 444 | 444 | 1.000000 | 0 | 39 | 1.00 |
| OpenCV edge_partial | 444 | 444 | 1.000000 | 12 | 61 | 0.88 |
| sentai_sim | 444 | 444 | 1.000000 | 3 | 39 | 0.97 |

## Legacy Stress Metric

Legacy metrics include `partial_crop` manifest markers as expected.  These are
kept only for cropped-marker stress analysis.

| Backend / variant | Matched | Expected | Recall | False positives | Complete frames |
| --- | ---: | ---: | ---: | ---: | ---: |
| OpenCV paper | 483 | 526 | 0.918251 | 0 | 0.68 |
| OpenCV edge_partial | 505 | 526 | 0.960076 | 12 | 0.75 |
| sentai_sim | 483 | 526 | 0.918251 | 3 | 0.65 |

Standard comparison outputs:

| Comparison | Validation dir |
| --- | --- |
| OpenCV paper vs sentai_sim | `validation_20260523_120842/` |
| OpenCV edge_partial vs sentai_sim | `validation_20260523_120958/` |
| OpenCV paper vs sentai_sim + pose ambiguity diagnostic | `validation_20260523_121953/` |

Pose ambiguity diagnostic:

| Backend | Pose frames | Near-ambiguous frames | Notes |
| --- | ---: | ---: | --- |
| OpenCV paper | 75 | 45 | Mostly symmetric correspondence alternatives; many are 180-degree yaw variants with near-identical reprojection. |
| sentai_sim | 75 | 45 | Same ambiguity profile as OpenCV, which suggests layout/marker symmetry rather than backend-specific detector behavior. |

Detailed ambiguous candidates are in:

```text
validation_20260523_121953/report_tables/pose_ambiguity_candidates.md
```

Legacy comparison outputs:

| Comparison | Validation dir | Count disagreement frames | Centroid p95 px | Axis A p95 px | Axis B p95 px | Stable angle p95 deg | Pose frames | Translation p95 m | Yaw p95 deg |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| OpenCV paper vs sentai_sim | `validation_20260523_114854/` | 3 / 100 | 0.271884 | 0.000362 | 0.163386 | 3.76007 | 87 | 0.0174059 | 0.108553 |
| OpenCV edge_partial vs sentai_sim | `validation_20260523_114856/` | 33 / 100 | 0.271884 | 0.000362 | 0.163386 | 3.76007 | 87 | 0.0172099 | 0.167010 |

Grouped OpenCV recall:

| Variant | Interior | Near edge | Cropped |
| --- | ---: | ---: | ---: |
| paper | 232 / 232 | 12 / 12 | 239 / 282 |
| edge_partial | 232 / 232 | 12 / 12 | 261 / 282 |

Interpretation:

- `paper` remains the parity baseline. It matches `sentai_sim` recall on this
  dataset and has zero OpenCV false positives.
- `edge_partial` improves cropped-marker recall by 22 markers but introduces
  12 false positives, so it should remain a named experimental variant until
  ported to `sentai_sim` and tuned under the same ablation protocol.
