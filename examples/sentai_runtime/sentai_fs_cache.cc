// sentai_fs_cache.cc -- generic SDRAM-buffered write cache for the
// user filesystem.  ONE active session at a time.
//
// Why this exists
// ---------------
// MicroPython diag drivers want to write large amounts of data
// (per-frame samples, gray frames, traces) to FileX in tight loops.
// Two routes from MP are slow for this:
//   * `sentai.fs.append(path, data)` -- one fx_file_open + write +
//     close per call (~30-60 ms).  Death by a thousand cuts at 50 Hz.
//   * Python-side accumulator (bytearray / list of bytes + b''.join) --
//     this MP build was compiled WITHOUT bytearray to save flash, and
//     the alternatives churn the small MP heap badly.
//
// Per agent.md "Python doar layer de comanda": move the buffering
// AND the FileX interaction into a static SDRAM buffer with three
// minimal entry points.  Driver becomes:
//
//     sentai.diag.cache_open(path)
//     for ... :
//         row = "%d,%d,..." % (...)            # str alloc only
//         sentai.diag.cache_write(row + "\n")  # one C copy, no fs
//     sentai.diag.cache_save()                 # one FileX write
//     sentai.diag.cache_close()
//
// The buffer lives in SDRAM (.sdram_bss), 4 MB.  Two counters:
// `s_len` (bytes accumulated) and `s_dropped` (bytes that overflowed).
// Caller can poll cache_len() to decide whether to save+reopen mid-run.
//
// Single-writer model.  No locks.  Concurrent sessions on the same
// API would corrupt each other's state -- enforce one-active-at-a-time
// at the driver level (cache_open returns -4 if already open).

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "libs/base/fx_user_fs.h"   // self-manages C / C++ linkage

namespace {

// 4 MB SDRAM cache buffer.  Sized for bulk-frame capture
// (80x60 gray @ 25 fps for 30 s = ~3.6 MB) plus typical CSV traces.
constexpr size_t kCacheBufSize = 4 * 1024 * 1024;
constexpr size_t kCachePathMax = 96;

static uint8_t s_buf[kCacheBufSize] __attribute__((section(".sdram_bss")));
static size_t  s_len     = 0;
static size_t  s_dropped = 0;
static char    s_path[kCachePathMax] = {0};
static int     s_open    = 0;

}  // namespace

extern "C" int sentai_fs_cache_open(const char* path) {
    if (!path || !path[0]) return -1;
    size_t plen = strnlen(path, kCachePathMax);
    if (plen >= kCachePathMax) return -2;
    if (s_open) return -4;             // single-session
    memcpy(s_path, path, plen + 1);
    s_len = 0;
    s_dropped = 0;
    s_open = 1;
    return 0;
}

// Append `data` (size bytes) to the buffer.  Returns bytes written,
// or -1 on overflow (bytes that didn't fit are tracked in s_dropped).
extern "C" int sentai_fs_cache_write(const uint8_t* data, int size) {
    if (!s_open) return -3;
    if (size < 0) return -1;
    size_t want = (size_t)size;
    size_t free_room = (s_len < kCacheBufSize) ? (kCacheBufSize - s_len) : 0;
    if (want > free_room) {
        s_dropped += (want - free_room);
        want = free_room;
        if (want == 0) return -1;
    }
    memcpy(s_buf + s_len, data, want);
    s_len += want;
    return (int)want;
}

// Flush the buffered bytes to FileX in one fx write call.  Caller
// decides cadence: at end of run, or mid-run when len() grows past
// a threshold.  After flush the cache buffer remains valid (s_len
// NOT reset) -- pair with cache_open() to start a new path or
// cache_close()+open() to reset.
extern "C" int sentai_fs_cache_save(void) {
    if (!s_open) return -3;
    if (s_len == 0) return 0;
    int rc = FxUserWriteFile(s_path, s_buf, s_len);
    return (rc < 0) ? rc : (int)s_len;
}

extern "C" int sentai_fs_cache_len(void) {
    return (int)s_len;
}

extern "C" int sentai_fs_cache_dropped(void) {
    return (int)s_dropped;
}

extern "C" int sentai_fs_cache_close(void) {
    s_open = 0;
    return 0;
}
