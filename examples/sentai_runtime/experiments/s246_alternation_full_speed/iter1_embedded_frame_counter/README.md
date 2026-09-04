# s246 iter1 — embedded data lines (ABANDONED, no result)

**Date**: 2026-09-04
**Status**: ❌ abandoned before producing a result — superseded by
[iter2](../iter2_pattern_row_oracle/)

## What it was going to test

Whether OV5640 "embedded data lines" (per-frame counter / AGC / exposure placed
in the first bytes of the image, gated by `0x501F` bit 7 per the comment at
`modsentai_camera.c:346`) could serve as a per-frame provenance oracle for s246.

## Why it was abandoned

Operator direction: the house method for this is already established and
better — inject a **synthetic test pattern from the sensor itself** (BARS on one
camera, HBAND on the other) and check a horizontal row. The pattern signature is
known in advance, so it does not depend on scene content, and reading a whole
row tests line integrity directly rather than inferring it.

That is the right call. The embedded-line route also had an unresolved risk
noted in the probe's own header: the XRGB8888 ISP/CSI path carries the embedded
line as if it were pixels and may mangle it.

## What actually ran

Nothing conclusive. The probe aborted at its own baseline guard because
`bytes.hex()` does not exist in this MicroPython build, so `peek_row()` could
not be parsed. The guard behaved correctly — it refused to report a verdict from
an instrument that was not reading (the lesson carried over from s245 iter3).

`probe_embedded.py` is kept as the record of the attempt. It is NOT a result.
