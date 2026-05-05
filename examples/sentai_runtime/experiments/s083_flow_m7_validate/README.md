# s002 motion — sentai.flow VALIDATION (M7-only, build #1139)

Build: 1139 (M7-only, USAD8 SIMD, no-M4)
Date:  2026-05-05

## HEADLINE
Firmware and offline-replay produce **bit-identical** SAD outputs
on the same gray frames (per-frame |fw - off| max = 1 milli-gp).

| Metric         | FIRMWARE | OFFLINE | DIFF   |
|----------------|---------:|--------:|-------:|
| cum_dx (gp)    |   +11.15 |  +11.04 |  0.11  |
| cum_dy (gp)    |   -23.55 |  -23.66 |  0.11  |
| closure (gp)   |    26.06 |   26.11 |  0.05  |
| single-shot GT | dx=+10, dy=-7 (integer only), conf=84 |

## Files
- bulk_gray.bin (1.16 MB)         -- 240 frames text header + 4800 B gray
- offline_per_frame.csv (full)    -- per-frame fw vs offline + conf + phase
- frames/f0000.pgm ... f0239.pgm  -- individual 80x60 grayscale
- frames/animation.gif (~ )       -- 16 sec playback at 15 fps
- replay_full.py                  -- numpy SAD replay
