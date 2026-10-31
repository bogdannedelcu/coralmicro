# EXP-s250 — SD card I/O benchmark + HTTP-over-NCM transfer rate

**WBS:** `OP-S10-W23-T7` (firmware hardening audit, same home as EXP-s249).
**Board:** new rev 2026-10, 64 GB card (`SD64G`, 124 735 488 × 512 B
blocks), FileX FAT32 32 KB clusters, fault-tolerant journal ON, USDHC1 at
50 MHz / 4-bit.  RAM-loaded images, builds 1634 → 1640 (2026-10-31).
Operator ask (2026-10-30, paraphrased): measure the read and write speed
we get from the SD card.

## How to run

```bash
python3 examples/sentai_runtime/experiments/s250_sd_io_benchmark/sd_bench.py \
        [--reps 3] [--sizes 65536,262144] [--no-http]
```

Timing is taken on the board (`sentai.rtos.ticks_ms()` around each call,
median of `--reps`); B5 is timed by `curl` on the host.  Data pattern:
`bytes(range(256))*16` (4 KB) replicated.

## Results — build 1640 (`run_build1640.log`)

| op | size | time | rate |
|---|---|---|---|
| B1 `fs.write` (create+write+close) | 64 KB | 41 ms | 1.56 MB/s |
| B1 `fs.write` | 256 KB | 149 ms | 1.72 MB/s |
| B2 `fs.read` (whole file) | 64 KB | 5 ms | 12.8 MB/s |
| B2 `fs.read` | 256 KB | 16 ms | 16.0 MB/s |
| B3 `fs.append` 4 KB × 64 (log pattern) | 256 KB | 338 ms | 5.3 ms/append, 0.76 MB/s |
| B4 `fs.sync()` after B3 | — | 0–5 ms | (journal already committed per write) |
| B5 HTTP `GET /api/raw/<256 KB>` over CDC-NCM | 256 KB | **49 ms** | **5.2 MB/s** |
| — `GET /` (13.8 KB, in-memory page) | 13.8 KB | 1.5 ms | 9.4 MB/s |

Read ≈ 16 MB/s is the SD bus (4-bit @ 50 MHz ≈ 25 MB/s raw) minus FileX
overhead; write ≈ 1.6 MB/s is dominated by the card's program time plus the
fault-tolerant journal (every `fs.write` is open → write → close → journal
commit).  Appends of 4 KB cost ≈ 5 ms each — fine for the flight recorder
and `/log` at tens of Hz, not for per-frame dumps (use `sentai.fr` pools
and drain between flights).  Reads are capped at 512 KB by `fs.read`
(1 MB MP heap); bulk data moves over HTTP at ~5 MB/s.

## What the benchmark found on the way (all fixed)

1. **Build 1634 — FS lock leak.**  `fs.read` of a 256 KB file with a
   fragmented MP heap raised `MemoryError` *while holding the FS mutex*;
   every later `sentai.fs.*`, `fs.sync`, `/api/raw` and `crash_log_write`
   returned `lfs busy` until reboot.  Fixes (1637): `fs_read_whole()`
   allocates outside the lock (two short lock windows), and
   `sentai_fs_lock()` reclaims a mutex already owned by the calling task
   (`SERR_LFX_LOCK_RECLAIMED 0x0D38`) instead of timing out forever.
2. **Build 1637 — HTTP 256 KB at 17 KB/s (15–28 s per file).**
   Three independent defects in `libs/cdc_ncm/cdc_ncm.cc`, found with
   `tcpdump` on the host side (`raw16k_1639.pcap` pattern: segments 1-5
   sent, 6-7 lost, 8-9 sent, 10-12 lost, 1.3 s RTO):
   - `TransmitNtb()` wrote the next NTB into `tx_buffer_` **before** checking
     that the previous asynchronous `USB_DeviceCdcAcmSend()` from the same
     buffer had completed → corrupted back-to-back NTBs (host `RX errors`
     +31 per 256 KB).  Fixed with an in-flight flag + completion semaphore
     (`TxWaitIdle()`), ZLP moved to the completion event (1638; host RX
     errors 0).
   - lwIP → `cdc_ncm_task` hand-off queue was 10 deep with zero wait while
     `TCP_SND_BUF` lets lwIP burst 16 segments → segments 11-16 of every
     burst dropped → RTO stalls.  Depth 64 + 5 ms bounded wait + drop
     counter (1639; `/` went from 1.2 s to 1.8 ms).
   - `TxWaitIdle()` first version trusted a binary semaphore that kept a
     stale give → every second NTB dropped.  Loop on the flag (1640).
   Pre-existing (the 10-deep queue and the buffer race are from the
   original port); only visible once transfers exceed ~5 segments.
3. The over-long REPL line test and the whole-file read cap were tuned
   while debugging here; see EXP-s249.

## Earlier runs (for the record)

- `run_build1634.log` — first run: 64 KB ok, 256 KB read failed (MemoryError
  → lock leak), then everything `lfs busy`; HTTP no complete transfer.
- `run_build1636.log` / `run_build1637.log` — lock fixed; HTTP 15.4 s.
- `run_build1638.log` — NTB race fixed, RX errors 0, HTTP still 18.6 s.
- `run_build1639.log` — queue 64: `/` fast, `/api/raw` still 18–28 s
  (alternating NTB drops).
- `run_build1640.log` — all fixed, numbers above.
