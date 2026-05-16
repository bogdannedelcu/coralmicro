# s142 — HexPatrol: descriptors per H3 cell

**Date**: 2026-05-16
**Predecessor**: s141 (DescriptorBaseline gate locked)
**Plan**: Pasul 2 §23.5 — first mission using L3 places as designed.

## Why this exists

Operator question after s141: *"can't we start creating descriptors per
hexagon?"*  L3 places gallery has had H3-indexed spatial storage since
the layer shipped, but NO mission has yet **populated** the gallery
with descriptors tied to drone position.

s142 is the first mission that uses L3 places as L3 was designed:
- Drone visits a location (X, Y in world frame)
- xy → H3 cell index (high-res grid)
- Compute visual descriptor for that location
- `places.add(h3_cell, descriptor, x, y, z)` — gallery slot per cell

After mission: gallery has N slots, each tied to a unique H3 cell,
with a populated descriptor.  This is **spatial-indexed visual memory**.

## Behavior under test (claim)

> "Drone takes off, visits 2 synthetic targets, computes and stores a
> PHOG+GIST-derived 64-byte descriptor at the H3 cell of each visited
> location (plus origin), returns home, lands.  L3 gallery ends with
> ≥ 3 places, each at a distinct H3 cell, each with a populated
> descriptor.  Querying the gallery with any stored descriptor returns
> that place (self-match sanity).  Closure < 15 cm of physical origin."

## MVP scope vs production

**MVP (this commit)**: synthetic image per spot — `_hex_image(seed)`
generates a deterministic seeded image, so each visited location gets
a unique descriptor.  Proves the **pipeline flow**: compute → quantize →
H3-index → store → query.

**Production (s143+)**: real Gazebo camera frame at each INSPECT.
Same flow but the input bytes come from `latest_ppm()` of the
camera_bridge stream, converted to grayscale.  s142 leaves a clear
extension point.

## Mission profile

```
phase 1-3   cf2 setup + telemetry + flow forwarder
phase 4     cf2.take_off(1.5) → CAPTURE PHYSICAL_ORIGIN
phase 5     L3 + descriptor module init (clear gallery, import helpers)
phase 6     Inject 2 synthetic L5 targets (long distance, s137-style)
phase 7     Store HOME place:
              seed = 0, xy = PHYSICAL_ORIGIN
              compute_phog + compute_gist on synthetic image
              quantize 168+64 floats → 64 bytes uint8
              cell = xy_to_h3(0, 0, res=15)
              places.add(cell, desc, x, y, 0)
phase 8     explore.start + explore.takeoff → HOVERING
phase 9     explore.goto(target_1) → INSPECT
              At HOVERING (post-INSPECT): seed=1 → store at target_1's cell
phase 10    explore.goto(target_2) → INSPECT → store at target_2's cell
phase 11    explore.return_home → land
phase 12    VERIFY gallery:
              count == 3
              3 distinct H3 cells
              each desc_set == 1
              pairwise descriptor distance > 0 (sanity: not all same)
              query(home_desc, h3=home_cell) returns home place_id
              closure < 15 cm
```

## Pass criteria — HARD gates

```
state_final == "DONE"
aborts == 0
land_err_xy_vs_origin < 0.15
gallery_count == 3                  # home + 2 targets
distinct_h3_cells == 3              # each at unique location
all_descriptors_set                 # desc_set == 1 for all 3
descriptors_unique                  # min pairwise L1 dist > 0
self_match_works                    # query returns the place
mission_duration < 90s
```

Plus existing universal rules:
- `[[sim-test-must-return-home]]` — closure preserved
- `[[test-must-be-relevant-to-claim]]` — claim is "spatial memory",
  verdict tests for it (not just "places.add doesn't error")

## New helpers (MP-side, in `hex_helpers.py`)

```python
# Maps ENU meters → fake lat/lng → H3 cell.  1 m ≈ 1/111111 deg.
def xy_to_h3(x, y, res=15):
    return sentai.places.cell_at(x * 9e-6, y * 9e-6, res)

# Generates a unique synthetic 64×64 image per seed.
def hex_image(seed):
    ...

# Quantize PHOG[:32] + GIST[:32] floats → 64 uint8 bytes.
def quantize(phog, gist):
    ...

# End-to-end: synthetic image → descriptors → quantize → places.add.
def capture_and_store(seed, x, y):
    ...
```

## Why synthetic image now, not Gazebo frame

PPM transfer from host to SIM REPL adds infrastructure complexity
(write large bytes to FS, MP reads, parses) that's orthogonal to the
core claim of s142 ("spatial memory works").  Synthetic-per-spot
proves the pipeline; real-frame integration is its own step in s143.

## Related

- `[[s139-phog-shipped]]`, `[[s140-gist-shipped]]` — descriptors used
- `[[s141-descriptor-baseline-shipped]]` — anti-regression gate
- `[[s137-explore-long-shipped]]` — multi-goto pattern reused
- `[[l3-shipped]]` — L3 places gallery (finally populated by a mission)
- `[[sim-test-must-return-home]]` — universal closure rule
- `[[test-must-be-relevant-to-claim]]` — relevance gates
- Next: s143 — real Gazebo frame at INSPECT (replaces synthetic) +
  loop closure (drone revisits, queries, matches)
