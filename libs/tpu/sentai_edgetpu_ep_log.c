/*
 * sentai_edgetpu_ep_log.c — records the USB endpoint descriptor table
 * observed during EdgeTPU enumeration so it can be printed from a safe
 * task context later (printf inside the enumeration callback has caused
 * LOCKUP#17 on this platform — agent.md §2.7).
 *
 * No dynamic allocation, bounded storage (16 entries), no mutexes — the
 * recorder runs single-threaded during enumeration and readers either
 * read after enumeration completes or accept a torn snapshot (each slot
 * is only 8 bytes, so reads stay consistent via natural alignment).
 */
#include <stdint.h>
#include <stdio.h>

#define SENTAI_EP_LOG_CAP 16

typedef struct {
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
    uint8_t  _pad[3];
} sentai_ep_rec_t;

static sentai_ep_rec_t g_ep_log[SENTAI_EP_LOG_CAP];
static uint8_t         g_ep_log_count    = 0;
static uint8_t         g_ep_log_total_ep = 0;

void sentai_usb_edgetpu_begin_enum(uint8_t total) {
    g_ep_log_count    = 0;
    g_ep_log_total_ep = total;
    for (int i = 0; i < SENTAI_EP_LOG_CAP; ++i) {
        g_ep_log[i].bEndpointAddress = 0;
        g_ep_log[i].bmAttributes     = 0;
        g_ep_log[i].wMaxPacketSize   = 0;
        g_ep_log[i].bInterval        = 0;
    }
}

void sentai_usb_edgetpu_record_ep(uint8_t addr, uint8_t attrs,
                                  uint16_t maxp, uint8_t interval) {
    if (g_ep_log_count >= SENTAI_EP_LOG_CAP) return;
    sentai_ep_rec_t *r = &g_ep_log[g_ep_log_count++];
    r->bEndpointAddress = addr;
    r->bmAttributes     = attrs;
    r->wMaxPacketSize   = maxp;
    r->bInterval        = interval;
}

/* Called from a task context (edgetpu_task.cc after kConnected, or
 * Python via sentai.tpu.dump_eps()).  Prints one line per endpoint. */
void sentai_usb_edgetpu_dump_eps(void) {
    const char *type_name[4] = { "CTRL", "ISO", "BULK", "INTR" };
    printf("[EdgeTPU ep-map] %u endpoints observed (firmware reported %u)\r\n",
           (unsigned)g_ep_log_count, (unsigned)g_ep_log_total_ep);
    for (uint8_t i = 0; i < g_ep_log_count; ++i) {
        sentai_ep_rec_t r = g_ep_log[i];
        uint8_t  ep_num = r.bEndpointAddress & 0x0Fu;
        const char *dir = (r.bEndpointAddress & 0x80u) ? "IN " : "OUT";
        const char *t   = type_name[r.bmAttributes & 0x03u];
        uint16_t maxp   = (uint16_t)(r.wMaxPacketSize & 0x07FFu);
        uint8_t  mult   = (uint8_t)((r.wMaxPacketSize >> 11) & 0x03u) + 1;
        printf("  [%u] ep=0x%02x (num=%u dir=%s) type=%s maxP=%u mult=%u interval=%u\r\n",
               (unsigned)i, (unsigned)r.bEndpointAddress,
               (unsigned)ep_num, dir, t, (unsigned)maxp,
               (unsigned)mult, (unsigned)r.bInterval);
    }
}

uint8_t sentai_usb_edgetpu_get_ep_count(void) { return g_ep_log_count; }
