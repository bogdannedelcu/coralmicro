# microSD on board rev 1.1 — notes from the bench (2026-11-09)

- Socket on **USDHC1** (GPIO_SD_B1_00..05), card supply through `SD_PWREN_B`
  (GPIO_AD_12, active low), IO rail select `SD1_VSELECT` (GPIO_AD_16, 0 = 3.3 V),
  no card-detect pin.  `libs/sdcard` handles the GPIOs; `apps/elf_loader` parks
  them while the loader runs.
- Measured with `examples/sdcard_test` on a 64 GB SDXC (SD64G): 4-bit at
  49.5 MHz, FatFs 2 MiB sequential write 8.0 MB/s, read 30.3 MB/s.
- `SDCard::Init()` is bounded (5 s): `SD_Init()` never returns on an empty
  socket, so it runs in a worker task; `SDCard::InitStage()` tells where it
  stopped.
- FatFs: the card is logical drive **"2:"** (SDDISK); `ffconf.h` has
  `FF_VOLUMES 3` for that.  `SDCard::Format()` runs `f_mkfs` with FM_FAT32,
  which writes an MBR with one 0x0C partition — a host sees the volume as
  `/dev/sdX1`.
- A card formatted by Eclipse ThreadX FileX (another firmware on the same
  hardware) is plain FAT32 without a partition table **and without the
  `BS_FilSysType32` "FAT32   " string** at offset 0x52 of the boot sector; this
  FatFs version's `check_fs()` reports it as "not FAT" → `f_mount` returns
  FR_NO_FILESYSTEM (13) although Linux mounts it.  A FatFs-formatted card is
  mounted by FileX unchanged, so if both firmwares must share a card, format it
  with FatFs (or on a PC).
- `examples/sdcard_test` never formats unless built with
  `-DSDCARD_TEST_FORMAT=ON`; its block tests restore the blocks they write.
