# SentAI patches for `coralmicro-rt1176-sdk` submodule

These patches apply on top of the upstream
`https://github.com/google-coral/coralmicro-rt1176-sdk` submodule pinned
by `third_party/nxp/rt1176-sdk` in the parent `coralmicro` repo.  They
are **small, reversible fixes** that we cannot commit upstream (NXP SDK
is read-only for us) but are required for the SentAI runtime to hit the
performance + reliability envelope documented in the paper.

Apply automatically at build-time via
[`scripts/apply_sdk_patches.sh`](../../scripts/apply_sdk_patches.sh),
which CMake invokes as part of the top-level project configure step.
Running the script a second time is a no-op if the patches are already
applied (checked via `git apply --check --reverse`).

## Patch index

### `0001-ov5640-vga-30fps-pclkperiod.patch`

OV5640 VGA @ 30 fps `pclkPeriod` register was cloned verbatim from the
15 fps row in `s_ov5640MipiClockConfigs` (`pclkPeriod = 0x0a`).  At the
actual 30 fps pixel rate, that under-estimates the MIPI T-HSSETTLE
window → the D-PHY receiver on the RT1176 side loses sync on the second
and subsequent frames → `CAMERA_RECEIVER_GetFullBuffer` times out.
First frame gets through clean because the D-PHY starts idle; later
frames are mis-sampled.

Doubling `pclkPeriod` to `0x14` matches Linux's `drivers/media/i2c/ov5640.c`
VGA/30 table and the guidance in OV5640 Application Notes §1.3.  The
fix delivers:
- stable continuous VGA streaming (0/10 dropped frames over 60 s)
- E15 pipeline: 17.0 → 24.4 fps (+45 %)
- PXP per-frame: 28 ms → 12 ms (3× at 5.6× smaller input)

See `paper/camera.md` — section "Update 2026-04-21" for the full story.

### `0002-ehci-queue-depth-16.patch`

EHCI queue head + queue transfer descriptor pool sizes bumped from 8 to
16.  At 8, a single EdgeTPU YOLO 512×512 inference (≈38 chunked bulk-out
URBs) forces the USB host into strict serialisation.  Raising the pool
lets more URBs be in flight concurrently so the TPU's DMA side doesn't
block on the host's URB descriptor shortage.  Extra RAM cost: ≈640 B in
SRAM, negligible on this target.

## Adding a new patch

1. Edit the submodule tree under `third_party/nxp/rt1176-sdk/` directly.
2. Validate the fix (build + empirical test).
3. Generate the patch:
   ```bash
   cd third_party/nxp/rt1176-sdk
   git diff -- <path/to/file> > ../../../patches/coralmicro-rt1176-sdk/NNNN-short-name.patch
   ```
4. Add a section in this README explaining the patch.
5. Re-run `scripts/apply_sdk_patches.sh` to confirm it applies cleanly
   on a fresh submodule checkout.
