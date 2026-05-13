# Harmonic World — top-down floor texture tiles

Pre-rendered top-down (nadir) views of `OpenRobotics/worlds/harmonic world`,
saved here as floor textures for the SIM-side world model experiments. Same
texture is used in both PX4 (natural-scale) and cf2 (1/10 scale) worlds —
see `Sim.md` §10v for the dual-scale convention.

## Files

| File | Size | Resolution | Purpose |
|------|------|------------|---------|
| `harmonic_alt200_4k.png`   | 24 MB | 4096×4096 | Master texture (commit to git; binary OK at this size) |
| `harmonic_alt200_1k.jpg`   | 177 KB | 1024×1024 | Viewable preview (open in browser/preview) |
| `harmonic_alt200_thumb.jpg`| 37 KB | 512×512   | Thumbnail for docs |

The `_alt200_` suffix marks the rendering altitude (200 m, FOV 100°), which
covers ~480 m × 480 m of the Harmonic World scene at nadir. The Harmonic
terrain heightmap spans ~1200 m × 1200 m, so this tile shows the central
~40% — the lake, lake house, and surrounding terrain features.

## How it was made

```bash
python3 sim/scripts/render_topdown.py \
  --src ~/.gz/fuel/fuel.gazebosim.org/openrobotics/worlds/"harmonic world"/3/harmonic.sdf \
  --out sim/gazebo/worlds/assets/harmonic_tiles/harmonic_alt200_4k.png \
  --altitude 200 \
  --fov-deg 100 \
  --run-seconds 18
```

The render script auto-strips problematic includes (Fidget Spinner, Pendulum,
Tethys, CartPole, wide-angle-camera-lensflare — none renderable under
ogre2/Garden 7.9) and injects a static nadir camera with `<save enabled="true">`.

Multi-altitude variants (100 m / 500 m) require additional scene tuning —
deferred until needed. The 200 m tile is sufficient as a backdrop texture.
