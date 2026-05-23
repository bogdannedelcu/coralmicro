# TD-S10-B2 Ablation Summary

Source dataset:

```text
dataset/TD-S10-B1/whycon_gazebo_synth_20260523_133537/
```

Validation output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/validation_20260523_141003/
```

Sentai output:

```text
dataset/TD-S10-B2/whycon_gazebo_synth_20260523_133537/sentai_sim_20260523_135732/
```

## Dataset

```text
frames              368
marker_distribution {4: 73, 5: 56, 6: 56, 7: 183}
z_distribution      {0.30: 16, 0.50: 32, 0.75: 112, 1.00: 208}
roll_pitch          {(0.0, 0.0): 200, (4.0, -3.0): 168}
```

The requested target was 500 images.  The current visibility policy generated
368 valid frames with at least 4 evaluation-visible markers and no counted
out-of-frame/cropped markers.

## Detection

| backend | expected | matched | recall | false_positives_per_frame | complete_frame_success | centroid_p95_px |
| --- | --- | --- | --- | --- | --- | --- |
| opencv | 2189 | 2189 | 1.000000 | 0.005435 | 0.994565 | 1.51118 |
| sentai_sim | 2189 | 2185 | 0.998173 | 0.051630 | 0.937500 | 1.50771 |

Detection parity is strong.  Matched centroids are nearly identical between the
two implementations; sentai_sim has slightly more false positives and missed 4
markers in this batch.

## Pose

| backend | pose_valid_frames | translation_rmse_m | translation_p95_m | x_mae_m | y_mae_m | z_mae_m | yaw_mae_deg | yaw_p95_deg |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| opencv | 364 | 0.128351 | 0.0410935 | 0.0219133 | 0.0114690 | 0.0071681 | 0.197608 | 0.221727 |
| sentai_sim | 367 | 0.0155885 | 0.0285586 | 0.0065462 | 0.0063278 | 0.0059818 | 0.089908 | 0.221193 |

OpenCV pose has a good median and p95, but its RMSE is dominated by several
approximately 1 m mirrored-branch outliers in 4-marker weak-geometry frames at
`z=1.0`.  sentai_sim is more robust on this batch, with max translation error
0.0672 m.

## OpenCV Versus sentai_sim

| comparable_frames | count_disagreement_frames | marker_delta_count | centroid_delta_p95_px | pose_delta_count | translation_delta_p95_m | yaw_delta_p95_deg |
| --- | --- | --- | --- | --- | --- | --- |
| 368 | 21 | 2185 | 0.363643 | 363 | 0.0396604 | 0.20013 |

## Conclusion

The main remaining issue is not marker centroid quality.  It is pose branch
selection under weak 4-marker configurations.  For the next pass, use targeted
ambiguity diagnostics on the outlier frames and report pose metrics separately
for 4-marker edge/near-edge frames versus 5-7 marker frames.

