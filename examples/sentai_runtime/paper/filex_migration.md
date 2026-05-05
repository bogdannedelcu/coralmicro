# FileX/LevelX migration plan

Author: bogdan (with agent), 2026-04-27.
Status: PHASE 0 — vendoring + scoping. No production code touched yet.

Purpose: replace LittleFS on the **user** partition (NAND blocks 76..523, ~56 MB)
with **FileX over LevelX**, in order to fix the O(n) directory-listing slowdown
that occurs once the user partition accumulates many files (diag sessions,
JPEGs, model dumps).  System partition (blocks 12..75, 8 MB, holds
`default.elf` + boot artefacts) stays on LittleFS for now — too risky to
migrate two partitions in one shot, and the system partition has very few
files so it doesn't suffer.

This document follows the embeded.md K output format.

---

## 1. Architectural assessment

### Why we are doing this

LittleFS 2.4 on this NAND geometry (block_size=128 KB, 448 blocks on the user
partition) has known scaling problems:

- **Directory listing is O(n)** in metadata-pair walks.  Each `lfs_dir_read`
  walks every metadata pair in the directory; with 100s of files the
  worst-case latency reaches multiple seconds.
- **`lookahead_size = kPageSize` (2048)** is below the recommended ~32 KB for
  large filesystems — every block-allocator pass re-scans.
- **`block_cycles = 250`** spreads wear but adds metadata churn under heavy
  write loops.

This shows up in production as:
- `/api/ls/diags` taking seconds even on cold filesystems (the build #633
  fix forces it through the `lfs_task` queue, but the underlying scan is
  still slow).
- `sentai_lfs_task` blocking serial-style on every dir read.
- Watchdog near-misses during `_t_iarna_inspect` or `_t_pattern_31` runs
  that write 50+ JPEGs per session.

### Why FileX + LevelX

- **FileX** is the FAT FS implementation from Eclipse ThreadX (formerly
  Azure RTOS, MIT license).  Cluster-based directory tables make `ls` O(N)
  in the number of cluster-table entries, but with **fixed cost per entry**
  and a cache-friendly access pattern that LittleFS lacks.
- **LevelX** is the NAND/NOR wear-leveling translation layer from the
  same project.  It exposes a logical-sector API to FileX while internally
  managing wear-leveling, bad-block sparing, and crash-safe sector
  remap on top of raw NAND pages.
- Together (LevelX-NAND under FileX) is the textbook way to put FAT on
  raw MLC/SLC NAND.  Microsoft / Eclipse maintain it as a single
  validated stack.
- Both libraries support `*_STANDALONE_ENABLE` build modes that drop the
  ThreadX-kernel dependency — they call host shims for sleep/mutex.  We
  bind those shims to FreeRTOS.  No ThreadX kernel needed.

### What FileX does NOT give us (be honest)

- **FAT is not power-fail atomic by design.**  A yank during a directory
  update can corrupt the FAT.  FileX has `FX_ENABLE_FAULT_TOLERANT` (a
  journaling extension) — we MUST enable it.  Cost: ~1 extra sector
  written per file-system-mutating operation, plus one reserved cluster
  for the log.
- **GC pauses.**  LevelX runs erase-block GC when free-block count drops
  below a threshold.  Worst-case GC latency is hundreds of ms — must be
  bounded and measured.  LittleFS distributed this cost; FileX/LevelX
  concentrates it.
- **Code size**: ~60-100 KB flash for FileX + LevelX vs ~20 KB for
  LittleFS.  Acceptable on this MCU but not free.
- **MicroPython VFS**: there is no upstream MP `vfs_filex.c`.  We have
  to write our own VFS adapter that bridges MP's `mp_vfs_proxy_call`
  surface to `fx_*` calls.  Estimate: ~600 lines, mirroring `vfs_lfs.c`.

### Severity classification of weaknesses to address

| Weakness                                          | Severity | Phase |
|---------------------------------------------------|----------|-------|
| LittleFS dir-listing latency on user partition    | **Major**    | 1-3 |
| MSC exposes raw NAND blocks (host can't navigate) | Minor    | (later — could expose FileX via MSC reading LevelX logical sectors) |
| No power-fail journal                             | **Critical** | 2 (must enable FX_ENABLE_FAULT_TOLERANT) |
| LevelX GC latency unmeasured                      | **Major**    | 3 (must add bench + bounded supervision) |
| MicroPython VFS coupling                          | Major    | 3 |

---

## 2. Fault and timing model

### Credible faults (new ones introduced by FileX/LevelX)

| Fault | Detection | Recovery | Error code |
|-------|-----------|----------|------------|
| LevelX BD read failure (NAND ECC unrecoverable) | `lx_nand_*` returns LX_ERROR | Retry up to 3, then mark block bad, escalate to FX | `0x0C00` LFX_BD_READ |
| LevelX BD program failure | `lx_nand_*` returns LX_ERROR | Retry, then mark bad, FX returns FX_IO_ERROR | `0x0C01` LFX_BD_PROG |
| LevelX BD erase failure | Same | Same | `0x0C02` LFX_BD_ERASE |
| FileX FAT corruption (without FT enabled) | `fx_directory_*` returns FX_FAT_READ_ERROR | Mount-time fsck OR re-format user partition | `0x0C10` LFX_FAT_CORRUPT |
| FileX fault-tolerant log replay needed | `fx_fault_tolerant_enable` reports REPLAY | Automatic (FX runs replay), log breadcrumb | `0x0C11` LFX_FT_REPLAY |
| Mount failure (un-formatted volume) | `fx_media_open` returns FX_BOOT_ERROR | One-time format on first boot only (gated by SRC_GPR flag) | `0x0C12` LFX_MOUNT |
| LevelX GC starvation (no free blocks) | LX_NO_BLOCKS | Refuse writes, log, set FS health to DEGRADED | `0x0C20` LFX_NO_BLOCKS |
| LevelX GC pause exceeds budget | Time around `lx_nand_flash_*` calls | Bounded — caller observes high latency, no corruption | `0x0C21` LFX_GC_LATE |
| MicroPython VFS adapter contract break | MP raises OSError | Don't crash MP REPL; map FX errors to errno | none (MP-internal) |

### Timing-sensitive paths

| Path | Period / Trigger | Budget | Consequence of miss |
|------|-------------------|--------|----------------------|
| `LfsUserAppendFile` (REPL chunked upload) | every chunk arrival, ~3 KB driver in 11 s | ≤ 200 ms / chunk | CDC RX backpressure, host-side abort |
| `lfs_task` `LFS_REQ_LS` | per HTTP `/api/ls/...` | ≤ 2 s | Watchdog near-miss, host curl timeout |
| `_t_visual_ab` JPEG saves | 50+ writes per session | ≤ 100 ms / write | Drains from CSI ring, FPS drop |
| MSC LUN read (storage mode) | host-driven, bursty | host-tolerated | Slow FAT format on host |
| LevelX background GC | triggered when free-block-count < threshold | hundreds of ms isolated | Spurious latency spike on the next FS op — must be observable |

### Supervised entities and checkpoints

- `LfsUserInit()` (renamed `FxUserInit()`) — must complete in < 2 s on cold
  mount, < 100 ms on warm mount.  Failure → boot record breadcrumb +
  re-format gated by SRC_GPR retry counter (do not infinite-format).
- `sentai_lfs_task` (renamed `sentai_fs_task` later — but during dual-stack
  phase keep the name to avoid touching every call site) — heartbeat to
  CombinedWatchdogTask every 5 s.  Bounded queue depth.
- LevelX GC monitoring task — new lightweight task that polls
  `_lx_nand_flash_diagnostic_info_get` every 10 s, exposes
  `sentai.diag.fs_stats()` with: free-blocks, GC count, bad-block count,
  avg/peak op latency.  This is the post-mortem breadcrumb that lets us
  detect creeping degradation in the field.

---

## 3. Target architecture

### Layering

```
+---------------------------------------------+
|  MicroPython (sentai.fs.*, os.*)            |
|    -> VFS adapter (vfs_filex.c, NEW)        |
+---------------------------------------------+
|  Coralmicro public API (libs/base/         |
|    filesystem.h + .cc)                      |
|    -> dual-backend during migration:        |
|       Lfs*()       -> LittleFS (system)     |
|       LfsUser*()   -> dispatcher:           |
|         #if FX_USER_FS  -> FileX            |
|         #else           -> LittleFS         |
+---------------------------------------------+
|  FileX (third_party/eclipse-threadx/filex)  |
|    fx_media_*, fx_directory_*, fx_file_*    |
|  with FX_STANDALONE_ENABLE                  |
|       FX_ENABLE_FAULT_TOLERANT              |
+---------------------------------------------+
|  LevelX (third_party/eclipse-threadx/levelx)|
|    lx_nand_flash_*                          |
|  with LX_STANDALONE_ENABLE                  |
+---------------------------------------------+
|  NAND BD adapter (libs/base/                |
|    fx_nand_driver.cc, NEW)                  |
|    -> calls existing Nand_Flash_Read_Page,  |
|       Nand_Flash_Page_Program,              |
|       Nand_Flash_Erase_Block                |
+---------------------------------------------+
|  NXP SDK NAND driver (existing)             |
+---------------------------------------------+
```

The dual-backend layer means we can ship the FileX user-partition under a
build-time switch (`SENTAI_FS_FILEX_USER`) that is **OFF by default** until
validated.  When ON, every `LfsUser*` wrapper dispatches to a `FxUser*`
implementation that exposes the same C++ signatures.  This way the existing
60+ call sites in `sentai_httpd*`, `sentai_lfs_task`, `modsentai_*.c`, and
`examples/sentai_runtime/*` change ZERO during phase 2.

### What stays unchanged

- **System partition (LittleFS).**  `Lfs()` and all `Lfs*` calls keep
  working.  This is firmware-load-bearing — we don't touch it in this
  migration.
- **USB MSC raw block path.**  MSC keeps reading NAND pages directly via
  `Nand_Flash_Read_Page`.  After migration, this means the host sees raw
  NAND with FileX/LevelX metadata — *not* a navigable FAT.  Re-targeting
  MSC to expose LevelX logical sectors is a follow-up, not part of this
  migration (would let host mount as FAT natively but adds complexity to
  storage-mode boot).
- **Watchdog architecture, mode-switch logic, USB stack.**  All untouched.

### What changes

- `LfsUserInit` becomes a thin wrapper that, when `SENTAI_FS_FILEX_USER` is
  set, calls `FxUserInit()` instead of LittleFS init.
- `LfsUser*` wrappers in `filesystem.cc` get a dispatch on a single
  compile-time constant.
- New file `libs/base/fx_nand_driver.cc` — LevelX NAND BD callbacks
  bound to the same `BOARD_GetNANDHandle()` + `Nand_Flash_*` already used
  by LittleFS.
- New file `libs/base/fx_user_fs.cc` — FileX-backed implementations of the
  `LfsUser*` API surface (`FxUserReadFile`, `FxUserWriteFile`,
  `FxUserAppendFile`, `FxUserDirRead`, etc.).
- New file `third_party/micropython/extmod/vfs_filex.c` — MicroPython VFS
  adapter (only when `SENTAI_FS_FILEX_USER` and the user partition is
  exposed via VFS — currently it isn't; only system partition is mounted
  as MP root).
- New error codes `0x0C00..0x0CFF` reserved for FileX/LevelX module.

---

## 4. Refactoring plan (phased, deployable per phase)

Each phase is a separately-flashable, separately-revertable step.  The
board boots and works at every phase boundary.

### Phase 0 — Vendor + scoping (THIS DOCUMENT + submodules) ✅ in progress

- [x] Survey existing LittleFS integration (done earlier this session).
- [x] Architectural assessment + fault model (this document).
- [ ] Add `third_party/eclipse-threadx/filex` submodule.
- [ ] Add `third_party/eclipse-threadx/levelx` submodule.
- [ ] Verify both pull cleanly, tags pinned to a known release (NOT master
      — we want reproducible builds).

**Exit criterion**: submodules vendored, this doc reviewed, no production
file changed.

### Phase 1 — Build the libraries in isolation

- [ ] `libs/filex/CMakeLists.txt` — static library `libs_filex` for M7
      (and `_m4` if needed later).  Compile flags:
      `FX_STANDALONE_ENABLE`, `FX_ENABLE_FAULT_TOLERANT`,
      `FX_MAX_LONG_NAME_LEN=64`, `FX_MAX_SECTOR_CACHE=8`.
- [ ] `libs/levelx/CMakeLists.txt` — static library `libs_levelx`.
      Flags: `LX_STANDALONE_ENABLE`, `LX_DIRECT_READ`,
      sector size = 2048 (= NAND page size on this NAND).
- [ ] Wire both into the main CMake but DO NOT link them into
      `sentai_runtime` yet.
- [ ] Build the libs alone (`cmake --build build --target libs_filex`).
      Confirm zero warnings, sizes reasonable.

**Exit criterion**: libraries compile, zero warnings, no integration.

### Phase 2 — NAND BD adapter + standalone unit test

- [ ] Write `libs/base/fx_nand_driver.cc` — LevelX NAND driver callbacks
      (read_page, write_page, erase_block, get_block_status,
      set_block_status, system_error).  Each call delegates to the same
      `Nand_Flash_*` functions LittleFS uses, **on a separate block
      range** that does NOT overlap the LittleFS user partition (until
      Phase 4 cutover).  Reserve blocks 600..1023 for the FileX
      experimental volume during validation.
- [ ] Bounded-loop assertions + retry budgets in every BD callback per
      embeded.md (no `portMAX_DELAY`, no unbounded retries).
- [ ] Add a diag-only example (`examples/sentai_runtime/diag/_t_filex_smoke.py`
      + a C-side `sentai.diag.fx_smoke()` binding) that:
      1. Calls `FxNandFormat()` (one-shot) on the experimental block range.
      2. Mounts via `fx_media_open`.
      3. Creates 100 files, writes 4 KB each, verifies sizes.
      4. Lists the directory, measures latency.
      5. Reads each file back, verifies CRC.
      6. Unmounts.
- [ ] Compare `diag/_t_filex_smoke.py` numbers against equivalent
      `_t_lfs_smoke.py` (write a parallel for LittleFS).

**Exit criterion**: FileX volume mounts, smoke test passes, dir-listing
latency on 100 files measurably better than LittleFS.

### Phase 3 — `FxUser*` wrappers behind compile flag

- [ ] Implement `FxUserInit`, `FxUserReadFile`, `FxUserWriteFile`,
      `FxUserAppendFile`, `FxUserDirOpen`, `FxUserDirRead`,
      `FxUserDirClose`, `FxUserMakeDirs`, `FxUserFileExists`,
      `FxUserDirExists`, `FxUserSize`, `FxUserRemove`, `FxUserRemount`.
- [ ] Add a single compile-time switch `#define SENTAI_FS_FILEX_USER 0` in
      a header.  Default OFF.  When 1, `LfsUser*` wrappers delegate to
      `FxUser*`.
- [ ] Map FileX/LevelX error codes to `bool` / `ssize_t` semantics that
      existing callers already use.
- [ ] Error codes `0x0C00..0x0C2F` registered in `error_codes.csv`.

**Exit criterion**: with switch OFF, build is bit-identical to today.
With switch ON, board boots on FileX user partition, all REPL fs ops work,
HTTP `/api/ls/`, `/api/raw/` work, `_host_upload_repl.py` works.

### Phase 4 — Enable `SENTAI_FS_FILEX_USER` by default + bench

- [ ] Run the full diag suite (`_t_pattern_31`, `_t_visual_ab`,
      `_t_iarna_pipeline`, `_t_fps_pipeline`) on FileX-backed user
      partition, confirm zero regressions in pipeline FPS,
      cam_id correctness, etc.
- [ ] Bench `fx_directory_*` latency at 50, 100, 500, 1000 files and
      record in `paper/filex_bench.md`.
- [ ] Add LevelX GC supervision task (per §2 above).
- [ ] Flip the default to `SENTAI_FS_FILEX_USER 1`.

**Exit criterion**: shipped firmware uses FileX on user partition;
LittleFS code stays on system partition.  Old user-partition LittleFS
volume is no longer mounted, but the code paths remain compiled (small
overhead, but easy revert if a regression surfaces in the field).

### Phase 5 — Cleanup (post-stabilization, weeks later)

- [ ] Remove `LfsUser*` LittleFS-backed code path (compile-out via
      `SENTAI_FS_FILEX_USER` permanent).
- [ ] Drop `LfsUserInit`'s LittleFS branch.
- [ ] Update SENTAI_API.md.
- [ ] Optionally: re-target USB MSC to read LevelX logical sectors so
      host sees a real FAT.

**Exit criterion**: LittleFS is only on the system (8 MB) partition.

### What we are NOT doing

- We are NOT migrating the system partition to FileX.  Too risky, no
  pain there (few files, infrequent updates).  System stays LittleFS.
- We are NOT changing the MSC LUN backing during this migration.  Host
  still sees raw NAND in storage mode.  Re-targeting MSC is a follow-up.
- We are NOT changing the M4-core build (M4 doesn't use the user
  partition).

---

## 5. Refactored code

NONE in this commit.  Phase 0 only adds the submodules and this document.

---

## 6. Robustness review (preliminary, will be re-done at end of each phase)

### Blocking risks

- LevelX GC can take hundreds of ms.  Every call site that currently has
  a 200-ms budget (CDC append loop, lfs_task LS) needs explicit
  measurement before flipping the default.  PHASE 3 must add latency
  histograms to `sentai.diag.fs_stats()`.

### Watchdog strategy

- `sentai_lfs_task` already heartbeats via `g_repl_last_activity`-style
  proof-of-life.  After migration, same task heartbeats — the BACKEND
  changes, the supervisor doesn't.

### Memory risks

- `FX_MAX_SECTOR_CACHE=8` * 2048 bytes = 16 KB FileX RAM cache.  Live in
  `.sdram_bss` (do NOT take from OCRAM — TPU pipeline owns that budget;
  see `experiment.md` and `project_arena_in_ocram_done.md`).
- LevelX RAM map (logical→physical sector table) for 56 MB user
  partition at 2 KB sectors = ~14 KB.  Also `.sdram_bss`.
- Stack: FileX `fx_directory_*` calls are recursion-free per its design;
  worst-case stack ~2 KB.  Within current task stack budgets.

### Concurrency

- FileX requires external mutexing in standalone mode.  Same model as
  LittleFS today: a single mutex around `fx_*` calls in the wrappers.
- LevelX in standalone mode is single-threaded by design — all calls
  serialized through the same mutex as FileX (one-mutex-per-volume,
  not per-layer).

### Recovery readiness

- Mount-time fault-tolerant log replay is automatic (FX runs it during
  `fx_fault_tolerant_enable`).
- If `fx_media_open` fails after replay, escalate per the SRC_GPR
  boot-attempts rule (embeded.md §M): on the THIRD consecutive failure,
  re-format the user partition and post-log a `0x0C12` breadcrumb.
  Never re-format the system partition.

---

## 7. Remaining risks and assumptions

1. **LevelX wear-leveling on this exact NAND part** — datasheet says MLC
   with 5000 erase cycles per block.  LevelX assumes uniform wear; if
   the part has factory-bad blocks, LevelX must mark them.  Need to
   verify `lx_nand_flash_diagnostic_info_get` reports them correctly on
   first format.
2. **GC pause distribution unknown until measured.**  If 99th-percentile
   GC pauses approach 1 s, we will need to schedule GC explicitly during
   idle and disable opportunistic GC.  PHASE 4 measures this.
3. **MicroPython VFS coupling.**  Today only the SYSTEM partition is
   exposed to MP `os`.  We do not need an MP VFS for user partition in
   this migration.  But `sentai.fs.*` (custom MP module) calls
   `LfsUser*` directly — that's the surface that switches via
   `SENTAI_FS_FILEX_USER`.  No upstream MP changes required.
4. **MSC + FileX coexistence in storage mode.**  Today MSC is
   write-protected by default and the user partition is unmounted before
   storage mode.  After migration, the unmount path becomes
   `fx_media_close` instead of `lfs_unmount` — straightforward.
5. **Power-fail behavior at format time.**  A power yank during the
   first-boot format would brick the user partition.  Mitigation: gate
   the format on a SRC_GPR flag that's set by a host-driven REPL command
   (`sentai.fs.format_user()`), not auto-trigger on first mount failure.
   This is more conservative than LittleFS's auto-format.

---

## Appendix — Reference

- FileX upstream: https://github.com/eclipse-threadx/filex
- LevelX upstream: https://github.com/eclipse-threadx/levelx
- License: MIT (Eclipse ThreadX, donated by Microsoft to Eclipse Foundation
  early 2024).
- Standalone mode docs: `docs/filex_user_guide.md`, sections on
  `FX_STANDALONE_ENABLE` and `FX_ENABLE_FAULT_TOLERANT`.
- LevelX standalone: `docs/levelx_user_guide.md`,
  `LX_STANDALONE_ENABLE`.
