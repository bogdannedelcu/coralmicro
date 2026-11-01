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

### `0003-fsl-csi-coralmicro-irq-hooks.patch`

Two coralmicro-side hook calls injected into `CSI_TransferHandleIRQ`
(`devices/MIMXRT1176/drivers/fsl_csi.c`):

1. `coralmicro_csi_on_frame_complete(bufferAddr)` — called at the
   exact point the NXP driver moves a freshly-filled DMA buffer into
   the user-visible queue.  This is the only place where a
   non-spurious frame is confirmed to have landed; the coralmicro
   side records the active `cam_id` per buffer slot here so the
   downstream consumer can validate which sensor produced the
   frame (used by the camera-id correctness work, build #953+).
2. `coralmicro_csi_on_buffer_arm(bufferAddr)` — called when an
   empty buffer is freshly submitted to CSI hardware, i.e. the slot
   that will be filled NEXT.  At this exact moment
   `g_cam_current_id` reflects the camera that will fill it (no
   MUX flip racing in between since CSI ISR runs in VBLANK).
   The hook writes that cam_id into a per-slot tag array, providing
   a synthetic per-buffer fingerprint independent of scene content.

Both hooks have weak default no-op implementations defined on the
SDK side (so non-coralmicro consumers of this submodule still build),
and strong overrides in [`libs/camera/camera_support.c`](../../libs/camera/camera_support.c)
that drive the dual-camera tagging machinery.

The patch is INTENTIONALLY narrow — only adds two `extern` calls
plus the `uint32_t emptyBuf` local needed to pass the address to
the second hook (the upstream driver passes
`CSI_TransferGetEmptyBuffer(handle)` directly to
`CSI_SetRxBufferAddr`; we hoist it to a local so we can also pass
it to the hook).  No control-flow changes, no new error paths, no
new branching — minimal surface for upstream merges to drift over.

### `0006-ehci-transfer-error-hook.patch`

**Why (OP-S10-W24-T9, 2026-10-31):** the natural Edge TPU wedge starts with
one bulk-OUT chunk completing `kStatus_USB_TransferFailed` (seen on the last
12 KB of a Y plane after 401 µs).  The NXP host stack maps every qTD error
bit except Halted-alone to that single status, so the application cannot
tell a **Transaction Error** (bit 3: wire / device not responding, 3 retries
exhausted) from a **Data Buffer Error** (bit 5: the EHCI could not fetch the
OUT data from memory in time — SDRAM contention with CSI/PXP DMA) or Babble
(bit 4) / Missed micro-frame (bit 2).  Those call for different fixes
(signal / device power vs. buffer placement / bus arbitration).

**What:** before each of the two `kStatus_USB_TransferFailed` callbacks in
`USB_HostEhciTransactionDone` the stack now calls
`sentai_ehci_transfer_error_hook(qtd_status, transfer_sofar, transfer_length)`.
The SDK side defines it `__attribute__((weak))` as a no-op; SentAI overrides
it in `libs/tpu/edgetpu_driver.cc` with per-bit counters and a last-error
record exposed through `sentai.tpu.urb_sync_stats()['ehci']`.  No control-flow
change, no new error path.

### `0007-ov5640-sxga-true-fps-rows.patch`

**Why (OP-S10-W24-T16/T17, 2026-11-01, EXP-s260 + EXP-s264):** the 1280x960
clock rows of patch 0001 were cloned from the SDK's 720P/30 row (PLL 84x,
sysdiv 2, MIPI div 1).  A 720p frame has ~740 lines, the SXGA frame 984, so
the "SXGA 30 fps" row streamed **22.6 frames/s** and the "SXGA 15 fps" row
**11.2 frames/s** (CSI `CR3.FRMCNT`; the firmware's `sensor_frames()` counter
over-counts by ~15 %, hence the belief).  Frame rate scales with PLL
multiplier / (sysdiv x mipi_div).  A key that does not mean the frame rate
is confusing (operator, 2026-11-01), so the keys now mean the measured rate.

**What:** the SXGA rows of `s_ov5640MipiClockConfigs` become
15 → PLL 0x70 (112x, 896 MHz, the PLL the VGA/60 row uses) with sysdiv 4
(`pllCtrl1` 0x41), `pclkPeriod` 0x10, measured 15.0 frames/s;
30 → PLL 0x70 with sysdiv 2, `pclkPeriod` 0x08, measured 30.0 frames/s;
22 → the old 84x row, measured 22.6 frames/s, kept because the legacy
XYUV8888 capture (4 B/px, 147 MB/s at 30 fps) starves the SDRAM and hung the
board under the full pipeline — `sentai_cam_init_full` refuses legacy SXGA at
key 30 with -12; the packed YUV420 capture (`camera.packed420(1)`, 2 B/px)
runs it.  The 11.2 fps row is gone.  D-PHY settle rows in `camera_support.c`:
15 → 0x13, 22 → 0x12, 30 → 0x12; `sentai.camera.csi_settle()` overrides.
History: between builds 1742 and 1755 the true rows were keyed 36 and 18
(files 0007-ov5640-sxga-true30-row / 0008-ov5640-sxga-true15-row); the s260,
s261, s263 and s264 logs of those builds use the old keys.

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
