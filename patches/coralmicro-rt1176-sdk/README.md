# Local patches for the `coralmicro-rt1176-sdk` submodule

Small, reversible fixes to the NXP SDK pinned at `third_party/nxp/rt1176-sdk`.
They are applied **at CMake configure time** by `scripts/apply_sdk_patches.sh`
(idempotent: a patch already present is skipped, a patch that no longer applies
fails the configure step).  After `git submodule update` re-run the configure
step or the script.  `git status` shows the submodule as dirty (` m`) while the
patches are live — that is expected.  Bypass with `-DSKIP_SDK_PATCHES=ON`.

Each patch touches its own file(s), so the apply order does not matter and the
"already applied" check (`git apply --reverse --check`) stays exact.

## `0001-ov5640-vga30-pclkperiod-sxga-true-fps-rows.patch`

Two fixes to the OV5640 MIPI clock table `s_ov5640MipiClockConfigs`
(`components/video/camera/device/ov5640/fsl_ov5640.c`); they edit the same
rows, hence one patch.

### VGA @ 30 fps `pclkPeriod`

The VGA @ 30 fps row had `pclkPeriod` cloned from the 15 fps row (0x0a).  At
the real 30 fps pixel rate that under-estimates the MIPI T-HSSETTLE window, the
RT1176 D-PHY receiver loses sync from the second frame on and
`CAMERA_RECEIVER_GetFullBuffer` times out.  `pclkPeriod` = 0x14 matches the
Linux `ov5640.c` VGA/30 table.  Result: continuous VGA/30 streaming with no
dropped frames.

### SXGA rows with true frame rates

The 1280x960 rows were cloned from the 720p/30 PLL settings; a 720p frame has
~740 lines and an SXGA frame 984, so "SXGA 30" streamed 22.6 frames/s and
"SXGA 15" 11.2 frames/s.  The rows now mean the measured rate: key 15 = 15.0
fps (PLL 112x, sysdiv 4), key 30 = 30.0 fps (PLL 112x, sysdiv 2), key 22 = the
old 84x row (22.6 fps).  Note: SXGA at 30 fps in the 4 B/px XRGB capture is
147 MB/s and starves the SDRAM; use it only with a 2 B/px capture format.

## `0005-fsl-csi-empty-list-side-array-f15.patch`

The stock CSI driver (`devices/MIMXRT1176/drivers/fsl_csi.{c,h}`) keeps its
empty-buffer list as a singly linked list threaded **through the frame
buffers** (the first word of each empty buffer holds the next pointer).  A DMA
write into a buffer that is still linked turns the list head into pixel data
and the ISR dereferences it (BusFault in `CSI_TransferGetEmptyBuffer`, BFAR
inside SDRAM) — seen after a sensor restart or when the queue fills.  The list
becomes a LIFO side array inside `csi_handle_t`, never inside pixel memory.
See `docs/board_rev_1_1/init_audit.md` finding F15.

## Adding a patch

1. Edit the submodule tree under `third_party/nxp/rt1176-sdk/` directly.
2. Build and test.
3. `cd third_party/nxp/rt1176-sdk && git diff -- <files> > ../../../patches/coralmicro-rt1176-sdk/NNNN-short-name.patch`
   (one patch per file set, so the reverse check stays exact).
4. Describe it here; re-run `scripts/apply_sdk_patches.sh` on a clean checkout.
