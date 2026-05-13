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

This is a **frozen artifact** — the render pipeline (`sim/scripts/render_topdown.py`
plus a minimal scene SDF that `<include>`d four Fuel-hosted Harmonic World
models) was removed 2026-05-13. We use Gazebo **Garden 7.9** (per Sim.md
§10c+§10d), and Harmonic-side infrastructure isn't needed once the texture
is generated.

If you ever need to regenerate it, the recipe is:

  1. Install a Gazebo 8/Harmonic environment (NOT on the dev host — use a
     separate distrobox or VM; the dev host MUST stay on Garden 7.9 or
     run no gz at all).
  2. Set up a minimal scene SDF that includes the 4 Harmonic World
     `<include>` URIs (Terrain, TerrainObjects, Lake House, Coast Waves 2)
     plus a static nadir camera with `<save enabled="true">`.
  3. Run `gz sim -r --headless-rendering` for 15–20 s; grab the PNG that
     the camera writes.

The 4K master here is sufficient as a backdrop texture for SIM demos
(see `examples/sentai_runtime/experiments/s125_integrated_demo/`).
