# Manuscript Rewrite Notes

Suggested replacement framing for Section 4.2:

1. State that the original timing audit found DWT/tick-derived values were not
   valid wall-clock measurements for the USB EdgeTPU path.
2. Present the current-firmware wall-clock results instead of a historical
   optimization progression.
3. Keep the optimization story qualitative: chunk sizing and zero-copy-style
   cleanup reduced software overhead, but the corrected 512x512 throughput is
   limited by USB transfer wall time at about 24 invokes/s.
4. Separate frame FPS from TPU invoke FPS in all multi-patch claims.
5. Separate manual camera switching from continuous ISR-ratio alternation.
6. Present `drain=1` as the valid continuous `ratio(1,1)` setting for build
   1544.  Present `drain=2` only for manual/batched switching unless separately
   validated.
7. Replace old reality-to-detection latency decompositions with update-period
   bounds, or add a future same-frame timestamp trace before making stronger
   latency claims.

Suggested concise paragraph:

> After revalidating the runtime performance chapter with board wall-clock
> timing, the 512x512 EdgeTPU path sustains about 24 standalone invokes/s and
> about 23.4 end-to-end single-camera detections/s.  Multi-patch operation
> preserves the requested invokes-per-frame ratio, but does not increase total
> TPU invoke throughput beyond approximately 24 invokes/s.  Continuous
> dual-camera alternation at `ratio(1,1)` sustains 14.4 combined detections/s,
> or 7.2 detections/s per camera, when using the validated one-frame drain
> setting.  The previously reported 73-75 FPS standalone, 40-42 FPS pipeline,
> and 48-56 FPS multi-patch values are therefore superseded by the corrected
> wall-clock measurements.

