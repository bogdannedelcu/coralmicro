// SentAI in-RAM dmesg-like ring buffer.
//
// Static SDRAM ring (no heap, no dynamic allocation), captures error /
// warning / info messages from any context (task or ISR).  Survives REPL
// restarts, USB reconnects and command failures; cleared only by reboot.
//
// Read via `sentai.diag.dmesg()` from MicroPython.

#ifndef SENTAI_DMESG_H
#define SENTAI_DMESG_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DMESG_DEBUG = 0,
    DMESG_INFO  = 1,
    DMESG_WARN  = 2,
    DMESG_ERROR = 3,
} dmesg_level_t;

// Append a formatted line.  Safe from any context (task or ISR).
// The full formatted line, including timestamp + level prefix and a
// trailing '\n', is bounded to ~256 bytes; longer lines are truncated.
void sentai_dmesg(dmesg_level_t level, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

void sentai_dmesg_v(dmesg_level_t level, const char *fmt, va_list ap);

// Convenience macros — each captures __FILE__ basename + line.
#define DMESG_E(fmt, ...) sentai_dmesg(DMESG_ERROR, "[%s:%d] " fmt, \
                                       sentai_dmesg_basename(__FILE__), __LINE__, ##__VA_ARGS__)
#define DMESG_W(fmt, ...) sentai_dmesg(DMESG_WARN,  "[%s:%d] " fmt, \
                                       sentai_dmesg_basename(__FILE__), __LINE__, ##__VA_ARGS__)
#define DMESG_I(fmt, ...) sentai_dmesg(DMESG_INFO,  fmt, ##__VA_ARGS__)
#define DMESG_D(fmt, ...) sentai_dmesg(DMESG_DEBUG, fmt, ##__VA_ARGS__)

// Helper for the macros above — strips the path prefix from __FILE__.
const char *sentai_dmesg_basename(const char *path);

// Copy the buffer contents into `out` (oldest-first).
// Returns the number of bytes copied (≤ out_size-1).  The output is always
// NUL-terminated when out_size > 0.
size_t sentai_dmesg_read(char *out, size_t out_size);

// Clear the ring buffer.  Use sparingly.
void sentai_dmesg_clear(void);

// Total number of bytes currently in the buffer.
size_t sentai_dmesg_used(void);

// Total bytes ever written (wrapped or not).  Useful to detect overruns.
uint32_t sentai_dmesg_dropped(void);

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_DMESG_H
