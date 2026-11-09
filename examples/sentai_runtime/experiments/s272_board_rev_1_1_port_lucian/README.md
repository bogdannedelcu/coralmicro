# s272 — Board rev 1.1 hardware support ported to a SentAI-free branch (for the camera colleague)

**WBS**: OP-S10-W22-T11
**Started**: 2026-11-09
**Status**: ✅ iter1 PASS on the bench (SD block path, LIS2DUX12, both cameras detected, camera-1 frames over USB HTTP); FatFs file path still to be run on a FAT32 card

## Claim

The board rev 1.1 (OWL-R2, 2026-10) hardware deltas can be delivered on a branch
that starts from the camera colleague's last commit (`6448b25d`, 2026-03-23, no
SentAI runtime, no MicroPython) without pulling any `sentai_*` code in, and the
vendor examples built from that branch run on the rev 1.1 hardware: microSD on
USDHC1, camera 1 on LPI2C1 + camera 2 on LPI2C6, LIS2DUX12 on I2C5.

Branch: `lucian/board-rev-1.1` (worktree `/home/bogdan/work/coralmicro-lucian`),
plan in `ideas/OP-S10-W22_rev1_1_port_plan_for_lucian.md`.  Nine commits:
docs, SDK patches (configure-time), SD/USDHC1 (+ FatFs API kept, drive "2:"),
camera 1 on LPI2C1 + pad guard + per-bus detect, LIS2DUX12 + test example,
T5838/audio/PMIC fixes, tools (flashtool VIDs, eFuse tool), sdcard_test
non-destructive, USB console reliability.

## Pass criteria

- `sdcard_test` (vendor example, FatFs API) initialises the card on USDHC1, reads
  the CID, writes/reads single and multi blocks and **restores** the original
  blocks (the card on 58eb carries the SentAI FileX volume, which must survive).
- `lis2dux12_test`: WHO_AM_I 0x47, 10 acceleration samples, |Z| ≈ 1 g.
- `camera_streaming_http_usb`: `HandlePowerRequest` reports both cameras on
  their own buses; `/camera_stream` over the USB network returns a JPEG that
  shows the monitor.
- HW: load with `flashtool.py --ram` only (board 58eb keeps the SentAI image in
  NAND; a reset returns to it), build counter N/A (vendor examples).
- No `sentai` identifier in any file of the branch (`git grep -i sentai` →
  only the provenance notes in `docs/board_rev_1_1/*.md` and the patches README).

## How to run

```bash
# build (branch lucian/board-rev-1.1, Make generator; toolchain symlinked from the main checkout)
cd /home/bogdan/work/coralmicro-lucian
cmake -B build -S . && cmake --build build --target blink_led sdcard_test lis2dux12_test camera_streaming_http_usb elf_loader flashloader -j$(nproc)

# load into RAM from the main checkout's flashtool (it knows the SentAI 1fc9:c0a1 reset path)
cd /home/bogdan/work/coralmicro
venv-flash/bin/python scripts/flashtool.py --build_dir /home/bogdan/work/coralmicro-lucian/build --example sdcard_test --ram
python3 examples/sentai_runtime/experiments/s272_board_rev_1_1_port_lucian/capture_console.py --out iter1/sdcard_test.log --seconds 45
```

The vendor examples enumerate as `18d1:9308`; the installed udev rule did not
cover that PID (fixed in `scripts/99-coral-micro.rules`, re-install with
`sudo cp scripts/99-coral-micro.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger`).
To get back from a RAM-loaded vendor example to the ROM: open `/dev/ttyACM0` at
1200 baud with DTR low (the vendor CDC resets into the Serial Downloader), then
`flashtool.py` finds SDP.

## Top-level files

- `capture_console.py` — host-side; raw-mode `cat` of the CDC console for N s
  (pyserial and a non-blocking reader both lost whole lines of a printf burst).

## Iter results table

| Iter | Hypothesis / change | Result | Pass? |
|---|---|---|---|
| iter1 | port as planned, tested on 58eb with `--ram` | SD init 4-bit 49.5 MHz, block tests pass + restore; LIS2DUX12 0x47 @0x19, Z −977 mg; both cameras detected, cam0 JPEG of the monitor over `10.0.0.1/camera_stream` | ✅ (FatFs file path pending: card is FileX) |

## iter1 — findings

- **SD** (`iter1/sdcard_test_cat.log`): 64 GB card (SD64G, MID 0x03), 124735488
  blocks, 4-bit at 49.5 MHz; single and multi-block write/read verified, blocks
  restored; `f_mount("2:")` → FR_NO_FILESYSTEM (13) as expected on the FileX
  card, format left OFF (`kFormatIfMountFails`).  The FatFs file/benchmark part
  needs a FAT32 card (or a card you allow to be formatted).
- **LIS2DUX12** (`iter1/lis2dux12_test.log`): on this unit the part answered at
  **0x19** (0x18 NAK, WHO_AM_I 0x00) — the opposite of the cold power-up seen on
  2026-10-31 (0x18); both addresses stay in the probe.  Z ≈ −977 mg (board
  mounted lens-down over the monitor), T ≈ 30 °C.  No taps (nobody tapped).
- **Cameras** (`iter1/camera_http_log_early.txt`, read through the example's
  `/log` endpoint right after boot): `HandlePowerRequest: cam0 (LPI2C1): OK,
  cam1 (LPI2C6): OK`; every OV5640 register write ACKed on both buses;
  `CAMERA_RECEIVER_Start = 0`.  `iter1/camera_stream_cam0_monitor_on_3.jpg` =
  320x240 JPEG of the monitor's test chart (monitor woken for the shot, then
  put back to sleep); `*_monitor_off.jpg` is black as expected.  Camera 2 is
  selected by the USER button in this example — not pressed remotely, so its
  image path is not shown here (its I2C presence is).
- **Console**: the vendor CDC console drops output while no host is attached and
  lost whole lines in bursts; fixed on the branch (in-flight latch, 3 retries,
  queue 64) and the test examples wait 3 s for the terminal.  pyserial on the
  host side lost data too; `cat` on a raw-mode port received everything.
- `/index.shtml` is 404 on a `--ram` load: the HTML lives in the LFS data image
  that `--ram` does not program — not a port issue.

## Dead ends (logged)

- flashtool "Unable to find device in SDP mode" / "Permission denied" on
  `/dev/ttyACM0`: udev rule missing for `18d1:9308` (vendor app PID).
- First `cmake` configure with the SDK-patch hook failed: patch 0007 edits the
  rows patch 0001 adds, so the "already applied" reverse check of 0001 broke
  once 0007 was in.  Merged both into one patch per file set.
- Patch 0005 references `coralmicro_csi_on_queue_fault()` (defined in the SentAI
  camera code): the branch's copy of the patch carries a weak no-op default.
