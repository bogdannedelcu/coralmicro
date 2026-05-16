# hex_helpers — MP-side helpers for s142 HexPatrol mission.
# Pre-copied to sentai_fs_root; imported by host script before use.

import sentai

W, H = 64, 64

def hex_image(seed):
    """Generate a deterministic synthetic image keyed by `seed`.
    Different seeds → different gradient + texture patterns, so each
    location's descriptor is unique."""
    out = [0] * (W * H)
    for y in range(H):
        for x in range(W):
            v = (x * (seed + 3) + y * (seed * 5 + 7) + seed * 11) & 0xff
            out[y * W + x] = v
    return bytes(out)

def quantize(phog, gist):
    """Pack first 32 PHOG + 32 GIST floats into 64 uint8 bytes."""
    raw = [0] * 64
    for i in range(32):
        v = phog[i] if phog else 0.0
        if v < 0.0: v = 0.0
        elif v > 1.0: v = 1.0
        raw[i] = int(v * 255)
    for i in range(32):
        v = gist[i] if gist else 0.0
        if v < 0.0: v = 0.0
        elif v > 1.0: v = 1.0
        raw[32 + i] = int(v * 255)
    return bytes(raw)

def xy_to_h3(x, y, res=15):
    """ENU meters → fake lat/lng → H3 cell.
    1 m ≈ 1/111111 deg lat; tiny epsilon avoids x=0,y=0 degenerate cell."""
    return sentai.places.cell_at(x * 9e-6 + 1e-9, y * 9e-6 + 1e-9, res)

def capture_and_store(seed, x, y, z=0.0):
    """End-to-end: synthetic image → descriptors → quantize → places.add.
    Returns: ≥0 assigned place id, or negative on error."""
    img = hex_image(seed)
    phog = sentai.places.compute_phog(img, W, H)
    gist = sentai.places.compute_gist(img, W, H)
    if phog is None or gist is None:
        return -101
    desc = quantize(phog, gist)
    cell = xy_to_h3(x, y, 15)
    pid = sentai.places.add(cell, desc, x, y, z)
    return pid

def self_query(pid):
    """Verify: query(desc of place pid) returns pid as best match."""
    p = sentai.places.get(pid)
    if p is None or not p['desc_set']:
        return -1
    desc = sentai.places.get_desc(pid)
    if desc is None:
        return -2
    cell = p['h3_cell']
    r = sentai.places.query(desc, cell, 1, 0)
    if r is None:
        return -3
    return r['id']
