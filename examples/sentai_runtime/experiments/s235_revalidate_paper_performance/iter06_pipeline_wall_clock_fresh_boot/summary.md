# iter06 Summary - B10.3 Pipeline Wall Clock Fresh Boot

Five fresh-boot runs were executed on SentAI build 1544. Each run measured 100 pipeline detection events after 5 warmup events using `sentai.rtos.micros()` around `sentai.pipeline.frame_count(after, timeout_ms)`.

## Corrected FPS

| run | wall_us | FPS | timeouts | infer_ok | infer_fail | prep_frames | latest_inv_ms | latest_total_ms |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 4390419 | 22.776869 | 0 | 105 | 0 | 106 | 39 | 39 |
| 1 | 4235336 | 23.610878 | 0 | 105 | 0 | 106 | 37 | 37 |
| 2 | 4235444 | 23.610275 | 0 | 105 | 0 | 106 | 40 | 40 |
| 3 | 4235558 | 23.609640 | 0 | 105 | 0 | 106 | 40 | 40 |
| 4 | 4234778 | 23.613989 | 0 | 105 | 0 | 106 | 39 | 39 |

Mean FPS: **23.444330**

Sample sigma: **0.373126 FPS**

Mean wall time for 100 measured events: **4266.307 ms** (sigma **69.381 ms**).

All five fresh-boot runs produced 100/100 measured events with zero timeouts and zero `infer_fail`.

## Important Iter05 Carryover

Iter05 showed that repeated `pipeline.stop()`/`pipeline.start()` in the same boot is not a valid way to collect independent repeats: run 0 succeeded, then runs 1-4 entered repeated `0B62` TPU invoke failures. Therefore the valid repeated-run protocol is one measured pipeline run per fresh firmware boot.
