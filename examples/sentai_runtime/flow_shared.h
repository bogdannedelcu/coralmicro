// flow_shared.h — M7↔M4 shared-memory layout for sentai.flow offload.
//
// Placed at a HARD-CODED physical address inside the RPMsg shared
// region (rpmsg_sh_mem @ 0x202C0000, 8 KB total, configured by the
// board MPU as non-cacheable device memory).  The fixed address is
// immune to link-order drift between the two independent binaries.
//
// Layout (96 bytes header + 4800 byte gray frame):
//   M4 → M7 (M4 writes, M7 reads)   : 0x00..0x2F
//   M7 → M4 command channel         : 0x30..0x3F
//   M7 → M4 frame publish           : 0x40..0x5F (header)
//                                     0x60..0x126F (80×60 gray)
//
// Synchronisation is all lock-free: each slot has ONE writer and
// everyone else only reads.  The `frame_valid` and `cmd_seq` fields
// act as one-shot handshakes — the producer increments, the consumer
// observes the increment, acts, and mirrors the new value into its
// own ack slot.

#ifndef FLOW_SHARED_H_
#define FLOW_SHARED_H_

#include <stdint.h>

#define FLOW_SHARED_MAGIC    0x53464C57u  // 'SFLW'
#define FLOW_SHARED_VERSION  4u           // v4: + m4_curr_gray (the
                                          //     EXACT gray frame M4 used
                                          //     for its last SAD; lets
                                          //     host replay match
                                          //     firmware bit-for-bit).
                                          // v3: dx/dy carry milli-grid-px
                                          //     after parabolic sub-pixel
                                          //     fit (1000 = 1 grid-px).

// 80×60 gray is the "PX4FLOW-class" resolution for optical flow —
// 4× more spatial detail than our earlier 40×30, 4800-byte frame
// that fits comfortably below the 12 KB rpmsg-window budget after
// the ~96-byte header (rpmsg_sh_mem was expanded from 8 KB to 16 KB
// on 2026-04-21 in the 5 RT1176 linker scripts to accommodate).
// At camera native 640×480 that's a step-8 decimation on both axes,
// so 1 grid-px ≈ 8 raw-px — good match for hand-motion detection
// and modest drone-speed flow.  SAD loop: block 32×32 + search ±12
// ≈ 625 candidates × 1024 pixels = 640 k ops, ~3 ms on M4 @ 400 MHz.
#define FLOW_GRAY_W 80
#define FLOW_GRAY_H 60
#define FLOW_GRAY_PIXELS (FLOW_GRAY_W * FLOW_GRAY_H)

// Commands written by M7 to flow_shared_t.cmd.
#define FLOW_CMD_NOP    0u
#define FLOW_CMD_START  1u
#define FLOW_CMD_STOP   2u

// States published by M4 in flow_shared_t.m4_state.
#define FLOW_STATE_IDLE     0u
#define FLOW_STATE_RUNNING  1u

typedef struct {
    // ── M4 → M7: status/diagnostics ────────────────────────────── 0x00
    volatile uint32_t magic;            // FLOW_SHARED_MAGIC
    volatile uint32_t version;          // FLOW_SHARED_VERSION
    volatile uint32_t m4_heartbeat;     // ++ every 100 ms
    volatile uint32_t m4_tick_ms;       // xTaskGetTickCount*ms at last beat
    volatile uint32_t m4_boot_ts_ms;    // boot timestamp
    volatile uint32_t m4_state;         // FLOW_STATE_*

    // ── M4 → M7: last block-match result ────────────────────────  0x18
    // dx/dy in milli-grid-pixel units (1000 = 1 grid-px).  Sub-pixel
    // refinement uses parabolic-fit on the SAD surface around the
    // integer minimum (Honegger ICRA 2013).  Range typically:
    //   integer  : -12000 .. +12000 (search bound x 1000)
    //   fraction : within +/- 500 of the nearest 1000-multiple.
    volatile int32_t  last_dx;
    volatile int32_t  last_dy;
    volatile uint32_t last_sad;
    volatile uint32_t last_frame_seq;   // M7 frame_seq this result came from
    volatile uint8_t  last_confidence;  // 0..255; 255 = strong match
    volatile uint8_t  _pad0[3];

    // ── M4 → M7: stats ──────────────────────────────────────────  0x2C
    volatile uint32_t frames_processed;
    volatile uint32_t frames_dropped;   // published by M7 saw frame_valid==1
    volatile uint32_t last_compute_us;
    volatile uint32_t avg_compute_us;

    // ── M7 → M4: command channel ─────────────────────────────────  0x3C
    volatile uint32_t cmd;              // FLOW_CMD_*
    volatile uint32_t cmd_seq;          // ++ each time M7 writes a new cmd
    volatile uint32_t cmd_ack_seq;      // M4 mirrors cmd_seq after apply

    // ── M7 → M4: frame publish header ───────────────────────────  0x48
    volatile uint32_t frame_seq;        // monotonic, set by M7 AFTER gray fill
    volatile uint32_t frame_valid;      // 1 = unread, 0 = M4 consumed
    volatile uint8_t  frame_cam_id;     // 0 or 1 (for observability)
    volatile uint8_t  _pad1[3];

    volatile uint32_t _reserved[5];     // pad to 0x60

    // ── M7 → M4: 80×60 gray frame (4800 bytes) ───────────────────  0x60
    volatile uint8_t  gray[FLOW_GRAY_PIXELS];

    // ── M4 → M7: the EXACT gray M4 used as 'curr' in its last
    //              sad_match (4800 bytes).  M4 copies its local
    //              s_gray[curr_slot] here AFTER sad completes,
    //              before clearing frame_valid for the next round.
    //              Host's bulk capture should grab this (NOT the
    //              shared-publish gray) so offline replay sees the
    //              same input bytes M4 saw and produces bit-identical
    //              SAD results.  Useful for algorithm validation;
    //              ignore in production builds that don't bulk-cap.
    volatile uint8_t  m4_curr_gray[FLOW_GRAY_PIXELS];
} flow_shared_t;

// Physical address on RT1176.  Inside the 16 KB rpmsg_sh_mem window
// (2026-04-21 the region was bumped from 8 KB to 16 KB in all 5
// RT1176 linker scripts to host the 80×60 gray buffer; MPU region 15
// re-sizes automatically via BOARD_ConfigMPU's log2-of-symbol
// calculation).  Window is 0x202C0000..0x202C3FFF; offset 0x1000 leaves
// 4 KB of head-room below for the RPMsg queues and 12 KB above for the
// flow struct.
//
// SIM cannot dereference a board physical address, so the simulator
// provides a static flow_shared_t symbol and keeps the same macro shape.
// The bindings and consumers still read FLOW_SHARED().
#if defined(SENTAI_PLATFORM_SIM)
#ifdef __cplusplus
extern "C" {
#endif
extern volatile flow_shared_t g_sentai_flow_shared;
#ifdef __cplusplus
}
#endif
#define FLOW_SHARED() (g_sentai_flow_shared)
#else
#define FLOW_SHARED_ADDR 0x202C1000u
#define FLOW_SHARED() (*(volatile flow_shared_t*)FLOW_SHARED_ADDR)
#endif

// Sanity: the layout must fit into the 12 KB that the RPMsg window
// gives us past offset 0x1000 (16 KB total − 4 KB queue head-room).
// With 80×60 gray, header + frame = 0x60 + 4800 ≈ 4.8 KB; with
// the v4 m4_curr_gray addition, total ~9.7 KB; leaves ~2 KB margin
// before we have to widen the rpmsg_sh_mem region in the linker
// scripts again.  Keep this assert tight so a future field add
// fails loudly at compile time, not at runtime cross-core corruption.
#ifdef __cplusplus
static_assert(sizeof(flow_shared_t) <= 0x3000,
              "flow_shared_t exceeds the 12 KB allocation past FLOW_SHARED_ADDR");
#if !defined(SENTAI_PLATFORM_SIM)
static_assert((FLOW_SHARED_ADDR & 0x3) == 0,
              "FLOW_SHARED_ADDR must be 4-byte aligned for atomic 32-bit volatile fields");
#endif
#else
_Static_assert(sizeof(flow_shared_t) <= 0x3000,
               "flow_shared_t exceeds the 12 KB allocation past FLOW_SHARED_ADDR");
#if !defined(SENTAI_PLATFORM_SIM)
_Static_assert((FLOW_SHARED_ADDR & 0x3) == 0,
               "FLOW_SHARED_ADDR must be 4-byte aligned for atomic 32-bit volatile fields");
#endif
#endif

#endif  // FLOW_SHARED_H_
