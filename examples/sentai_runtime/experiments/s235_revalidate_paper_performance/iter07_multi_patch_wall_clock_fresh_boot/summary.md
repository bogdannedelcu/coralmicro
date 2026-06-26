# iter07 Summary - B10.4 Multi-Patch Wall Clock Fresh Boot

Five fresh-boot matrix rows were executed on SentAI build 1544. Each row measured 100 pipeline frame events after 5 warmup events. Frame FPS uses `pipeline.frame_count()` over `sentai.rtos.micros()`. Invoke FPS uses `infer_stats().ok` delta over the same wall interval.

| run | prep_fps | ipf | mode | frame FPS | invoke FPS | invokes/frame | invoke fail | timeouts | wall_us |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 0 | 1 | 0 | 23.610554 | 23.610554 | 1.000 | 0 | 0 | 4235394 |
| 1 | 30 | 1 | 0 | 23.614239 | 23.614239 | 1.000 | 0 | 0 | 4234733 |
| 2 | 30 | 2 | 1 | 12.008011 | 24.016022 | 2.000 | 0 | 0 | 8327774 |
| 3 | 20 | 3 | 1 | 8.058186 | 24.174555 | 3.000 | 0 | 0 | 12409742 |
| 4 | 15 | 4 | 1 | 6.062988 | 24.251953 | 4.000 | 0 | 0 | 16493517 |

All rows completed with zero invoke failures and zero timeouts.

Corrected conclusion: the old multi-patch claims of `48.5`, `54.8`, and `56.0` TPU FPS are not reproduced with wall-clock timing on build 1544. Multi-invoke rows preserve the requested invokes/frame ratio, but total invoke throughput remains near `24 FPS`, close to the corrected standalone/pipeline ceiling.
