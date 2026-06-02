// SentAI in-RAM dmesg ring buffer — implementation.
//
// Design choices:
//   * Static SDRAM buffer, no heap, no malloc, no FreeRTOS objects required
//     for storage (the FS object is not allocated dynamically).
//   * Lock-free single-byte ring index manipulation.  A short critical
//     section (taskENTER_CRITICAL_FROM_ISR / EXIT) guards the head/tail
//     update so the buffer is consistent under preemption and from ISR
//     context.  Critical section is bounded to a memcpy of at most a few
//     hundred bytes — well under any RTOS interrupt-latency budget.
//   * Power-of-two size so wrap-around is a cheap mask.
//   * Format is line-oriented: each entry is a single line ending with
//     '\n', so the output is directly readable on the REPL.
//   * Truncating overflow: when the ring is full the oldest bytes are
//     overwritten.  A 32-bit dropped counter records how many bytes were
//     thrown away — accessible from MicroPython for diagnostics.

#include "sentai_dmesg.h"

#include <stdio.h>
#include <string.h>

#include "third_party/freertos_kernel/include/FreeRTOS.h"
#include "third_party/freertos_kernel/include/task.h"

// ---------------------------------------------------------------------------
// Storage — static, in SDRAM, zero-initialised in BSS.
// 16 KiB ≅ ~150 average-sized log lines, plenty for post-mortem of a single
// USB transaction or a recent crash window.
// ---------------------------------------------------------------------------
#define DMESG_BUF_SIZE   (16U * 1024U)
#define DMESG_BUF_MASK   (DMESG_BUF_SIZE - 1U)
static_assert((DMESG_BUF_SIZE & DMESG_BUF_MASK) == 0,
              "DMESG_BUF_SIZE must be a power of two");

#define DMESG_LINE_MAX   256U

static char     g_dmesg_buf[DMESG_BUF_SIZE] __attribute__((section(".sdram_bss")));
// Producer (write) index — monotonic 32-bit, masked when used.
static volatile uint32_t g_dmesg_head = 0;
// Bytes currently held in the buffer (≤ DMESG_BUF_SIZE).
static volatile uint32_t g_dmesg_count = 0;
// Total bytes silently dropped by overwrite (informational).
static volatile uint32_t g_dmesg_dropped = 0;

// Tick → ms helper that works in either task or ISR context.
extern volatile uint32_t g_sentai_uptime_ms;  // updated by health task

static inline uint32_t dmesg_now_ms(void) {
    // Prefer the periodically-updated uptime hint (works in ISR).
    // Fallback to xTaskGetTickCount (task context) if the hint is zero.
    uint32_t hint = g_sentai_uptime_ms;
    if (hint != 0U) return hint;
#if defined(SENTAI_PLATFORM_SIM)
    if (true) {
#else
    if (xPortIsInsideInterrupt() == pdFALSE) {
#endif
        return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    }
    return 0U;
}

const char *sentai_dmesg_basename(const char *path) {
    if (path == NULL) return "?";
    const char *p = path;
    const char *last = path;
    while (*p) {
        if (*p == '/' || *p == '\\') last = p + 1;
        p++;
    }
    return last;
}

// Append `n` raw bytes to the ring.  Must be called with the critical
// section already held by the caller.  Overwrites the oldest bytes if the
// ring is full.
static void dmesg_append_locked(const char *src, uint32_t n) {
    if (n == 0U) return;
    if (n > DMESG_BUF_SIZE) {
        // The single message is larger than the entire buffer: keep only
        // the trailing DMESG_BUF_SIZE bytes (preserve the recent end).
        src += (n - DMESG_BUF_SIZE);
        n   = DMESG_BUF_SIZE;
    }

    uint32_t head = g_dmesg_head & DMESG_BUF_MASK;
    uint32_t first = DMESG_BUF_SIZE - head;
    if (first > n) first = n;
    memcpy(&g_dmesg_buf[head], src, first);
    if (n > first) {
        memcpy(&g_dmesg_buf[0], src + first, n - first);
    }

    g_dmesg_head += n;
    uint32_t new_count = g_dmesg_count + n;
    if (new_count > DMESG_BUF_SIZE) {
        g_dmesg_dropped += (new_count - DMESG_BUF_SIZE);
        new_count = DMESG_BUF_SIZE;
    }
    g_dmesg_count = new_count;
}

void sentai_dmesg_v(dmesg_level_t level, const char *fmt, va_list ap) {
    static const char level_chars[] = "DIWE";
    char line[DMESG_LINE_MAX];
    uint32_t ts = dmesg_now_ms();

    int hdr = snprintf(line, sizeof(line), "[%10lu] %c ",
                       (unsigned long)ts,
                       level_chars[level <= DMESG_ERROR ? (int)level : (int)DMESG_ERROR]);
    if (hdr < 0) hdr = 0;
    if ((unsigned)hdr >= sizeof(line)) hdr = (int)sizeof(line) - 1;

    int body = vsnprintf(line + hdr, sizeof(line) - (size_t)hdr, fmt, ap);
    if (body < 0) body = 0;

    int total = hdr + body;
    if ((unsigned)total >= sizeof(line) - 1) {
        total = (int)sizeof(line) - 2;  // leave room for '\n' and '\0'
    }
    line[total]     = '\n';
    line[total + 1] = '\0';
    uint32_t n = (uint32_t)total + 1U;  // include the newline

    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    dmesg_append_locked(line, n);
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

void sentai_dmesg(dmesg_level_t level, const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    sentai_dmesg_v(level, fmt, ap);
    va_end(ap);
}

size_t sentai_dmesg_read(char *out, size_t out_size) {
    if (out == NULL || out_size == 0U) return 0U;

    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    uint32_t count = g_dmesg_count;
    uint32_t head  = g_dmesg_head & DMESG_BUF_MASK;
    // Tail = head - count (modulo buffer size).
    uint32_t tail  = (head + DMESG_BUF_SIZE - count) & DMESG_BUF_MASK;

    size_t to_copy = (count <= out_size - 1U) ? (size_t)count
                                              : (size_t)(out_size - 1U);
    // Skip the oldest bytes if the caller's buffer is smaller than count.
    if ((size_t)count > to_copy) {
        uint32_t skip = (uint32_t)((size_t)count - to_copy);
        tail = (tail + skip) & DMESG_BUF_MASK;
    }

    size_t first = (size_t)(DMESG_BUF_SIZE - tail);
    if (first > to_copy) first = to_copy;
    memcpy(out, &g_dmesg_buf[tail], first);
    if (to_copy > first) {
        memcpy(out + first, &g_dmesg_buf[0], to_copy - first);
    }
    out[to_copy] = '\0';
    taskEXIT_CRITICAL_FROM_ISR(saved);
    return to_copy;
}

void sentai_dmesg_clear(void) {
    UBaseType_t saved = taskENTER_CRITICAL_FROM_ISR();
    g_dmesg_count = 0U;
    g_dmesg_head  = 0U;
    g_dmesg_dropped = 0U;
    taskEXIT_CRITICAL_FROM_ISR(saved);
}

size_t sentai_dmesg_used(void) {
    return (size_t)g_dmesg_count;
}

uint32_t sentai_dmesg_dropped(void) {
    return g_dmesg_dropped;
}
