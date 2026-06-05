---
name: OV5640 AEC dominates ISP presets
description: E38 showed preset sweep produces identical brightness because AEC auto-compensates gamma/SDE; fix is post-capture stretch in publisher
type: project
originSessionId: 243924ea-3818-4ef8-bf1a-9ba567b33988
---
E38 (ISP preset sweep: nxp_stock / bright_indoor / daylight / low_light)
produced visually identical frames with gray_mean within a handful of
DN of each other across all four presets.

**Why:** OV5640's AEC measures the post-gamma output histogram and
drives gain/exposure to a fixed target mean (0x50/255 by default).
Any gamma / SDE / AEC-target / gain-ceiling preset gets immediately
compensated by the AEC feedback loop — output mean luminance is
pinned regardless of preset.  Group-write protocol (0x3212 =
0x03/0x13/0xA3) IS correct and the writes DO stick; the problem is
the AEC negates them.

**How to apply:** If the user asks to brighten the image by tweaking
OV5640 registers, push back — only two levers actually work:
1. freeze AEC and set manual exposure+gain (brittle across scenes)
2. post-capture software remap (histogram stretch / auto-level)

We went with option 2: `sentai.flow.gray_stretch(True)` applies min-max
linear stretch to the 80×60 gray buffer in the publisher (cost ~120 µs,
BEFORE frame_seq bump).  The M4 SAD consumer and `detail_score` both
see the stretched buffer.  Visual JPEG is unchanged — stretch only
affects the flow path, not the full-res sensor→JPEG or the TPU tensor.
