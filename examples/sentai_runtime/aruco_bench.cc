// aruco_bench.cc — feasibility benchmark for ArUco detection on M7.
//
// Goal: measure how fast adaptive thresholding + simple traversal
// runs on a 320×240 grayscale image on the i.MX RT1176 Cortex-M7 @
// 800 MHz.  Result drives the decision whether to implement a
// hand-rolled ArUco detector in C++ on ARM HW (vs companion-computer
// fallback).
//
// Output: dmesg lines printed at boot time.  Read from REPL:
//   sentai.diag.dmesg()
// Look for lines starting with "ARUCO_BENCH:".
//
// Numbers to interpret (DWT cycle counts, M7 @ 800 MHz: 800 cyc = 1 µs):
//   - scan_cyc  = raw memory traversal (read every pixel)
//   - thresh_cyc = adaptive threshold (7×7 mean - 5 offset)
//   - edge_cyc  = simple Sobel-like edge filter (3×3 |dx|+|dy|)
// Sum gives a lower bound for the ArUco preprocessing pipeline.

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

// 320×240 grayscale test pattern.  Buffers in SDRAM via section attr.
// (Existing flow_phase_corr.cc uses same pattern with ~600 KB sdram_bss
// — PROGBITS in .o, but linker .sdram_bss is NOLOAD so no flash cost.)
constexpr int W = 320;
constexpr int H = 240;
constexpr int N = W * H;

static uint8_t s_gray[N] __attribute__((section(".sdram_bss")));
static uint8_t s_bin[N]  __attribute__((section(".sdram_bss")));
static uint8_t s_edge[N] __attribute__((section(".sdram_bss")));

}  // anon namespace

// Externally-visible sink (no static, no anon ns) to defeat -O3
// constant folding of the memory-scan kernel.
volatile uint32_t s_aruco_bench_sink __attribute__((section(".sdram_bss")));

namespace {

// Generate a synthetic test pattern: light bg (200), dark square in
// center (50), with a small bit pattern inside (mimics ArUco).  Plus
// gaussian-ish noise so adaptive threshold actually has to think.
__attribute__((section(".sdram_text"), noinline))
static void init_pattern(void) {
    // Background gradient + light noise.
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            // Gradient 180..220 across image + LFSR-ish noise.
            uint32_t lfsr = (uint32_t)(y * W + x) * 2654435761u;
            uint8_t noise = (lfsr >> 16) & 0x1F;        // ±16 amplitude
            int v = 180 + (x * 40) / W + (int)noise - 8;
            if (v < 0) v = 0;
            if (v > 255) v = 255;
            s_gray[y * W + x] = (uint8_t)v;
        }
    }
    // Dark square: 80×80 centered.
    int cx = W / 2, cy = H / 2;
    for (int y = cy - 40; y < cy + 40; ++y) {
        for (int x = cx - 40; x < cx + 40; ++x) {
            // 4×4 bit pattern inside the square (mimics ArUco 4×4_50).
            int bx = (x - (cx - 40)) / 20;
            int by = (y - (cy - 40)) / 20;
            // Arbitrary bit pattern: id 0 from 4×4_50 dict (just for
            // visual; not bit-perfect).
            uint8_t bits[4] = {0b1001, 0b0110, 0b1100, 0b0011};
            bool dark = (bits[by] >> (3 - bx)) & 1;
            s_gray[y * W + x] = dark ? 30 : 220;
        }
    }
}

// DWT cycle counter for Cortex-M7.  Inlined assembly read.
static inline uint32_t dwt_cyc(void) {
    return *((volatile uint32_t*)0xE0001004u);  // DWT->CYCCNT
}

// === Benchmark kernels ====================================

// Kernel 1: pure memory scan (touch every pixel, accumulate sum).
// Establishes the memory-bandwidth floor for any per-pixel operation.
// IMPORTANT: kernels run in SDRAM (`.sdram_text`) because m_text/ITCM
// is full.  This means measurements include SDRAM instruction-fetch
// latency, which is REALISTIC for any new ArUco code we'd add (it
// would also live in SDRAM unless we free ITCM).  Lower bound on real
// perf: same kernels in ITCM would be faster.
__attribute__((section(".sdram_text"), noinline))
static uint32_t k_scan(const uint8_t* src, int n) {
    uint32_t sum = 0;
    const uint8_t* p = src;
    const uint8_t* end = src + n;
    while (p < end) {
        sum += *p++;
    }
    return sum;
}

// Kernel 2: adaptive threshold via 7×7 mean window.
// Naive impl (no separable filter, no SIMD).  Output bin=255 if
// src - mean > offset, else 0.  This is the "expensive" stage in
// ArUco preprocessing.
__attribute__((section(".sdram_text"), noinline))
static void k_adaptive_threshold(const uint8_t* src, uint8_t* dst,
                                  int w, int h) {
    constexpr int K = 7;          // window size
    constexpr int H_HALF = K / 2;
    constexpr int OFFSET = 5;
    for (int y = H_HALF; y < h - H_HALF; ++y) {
        for (int x = H_HALF; x < w - H_HALF; ++x) {
            uint32_t sum = 0;
            for (int dy = -H_HALF; dy <= H_HALF; ++dy) {
                const uint8_t* row = src + (y + dy) * w + x - H_HALF;
                sum += row[0] + row[1] + row[2] + row[3] +
                       row[4] + row[5] + row[6];
            }
            uint8_t mean = (uint8_t)(sum / (K * K));
            int v = src[y * w + x];
            dst[y * w + x] = (v + OFFSET < mean) ? 255 : 0;
        }
    }
}

// Kernel 3: Sobel-like edge filter (3×3 |dx|+|dy|).
// Approximates the gradient computation that precedes contour finding.
__attribute__((section(".sdram_text"), noinline))
static void k_edge(const uint8_t* src, uint8_t* dst, int w, int h) {
    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const uint8_t* r0 = src + (y - 1) * w + x - 1;
            const uint8_t* r1 = src + y * w + x - 1;
            const uint8_t* r2 = src + (y + 1) * w + x - 1;
            int gx = (int)r0[2] - r0[0] + 2*((int)r1[2] - r1[0]) + r2[2] - r2[0];
            int gy = (int)r2[0] - r0[0] + 2*((int)r2[1] - r0[1]) + r2[2] - r0[2];
            int g = (gx < 0 ? -gx : gx) + (gy < 0 ? -gy : gy);
            dst[y * w + x] = (uint8_t)(g > 255 ? 255 : g);
        }
    }
}

}  // namespace


// Public entry point — called from app_main at boot.  Runs each
// kernel 3 times, reports min cycle count (warm cache).
// Result struct shared between C++ kernel runner and MP binding.
struct aruco_bench_result_t {
    uint32_t w, h;            // image size
    uint32_t scan_cyc;        // raw memory traversal cycles
    uint32_t thresh_cyc;      // adaptive threshold cycles
    uint32_t edge_cyc;        // sobel-like edge filter cycles
};

extern "C" __attribute__((section(".sdram_text"), noinline))
void aruco_bench_run(aruco_bench_result_t* out) {
    // Enable DWT cycle counter (idempotent).
    volatile uint32_t* DEMCR    = (volatile uint32_t*)0xE000EDFCu;
    volatile uint32_t* DWT_CTRL = (volatile uint32_t*)0xE0001000u;
    *DEMCR    |= (1u << 24);
    *DWT_CTRL |= 1u;

    init_pattern();

    uint32_t scan_min = ~0u, thresh_min = ~0u, edge_min = ~0u;
    // Externally-visible sink so compiler can't fold k_scan() away.
    extern volatile uint32_t s_aruco_bench_sink;

    for (int trial = 0; trial < 3; ++trial) {
        uint32_t t0 = dwt_cyc();
        s_aruco_bench_sink = k_scan(s_gray, N);
        uint32_t t1 = dwt_cyc();
        if (t1 - t0 < scan_min) scan_min = t1 - t0;

        t0 = dwt_cyc();
        k_adaptive_threshold(s_gray, s_bin, W, H);
        t1 = dwt_cyc();
        if (t1 - t0 < thresh_min) thresh_min = t1 - t0;

        t0 = dwt_cyc();
        k_edge(s_gray, s_edge, W, H);
        t1 = dwt_cyc();
        if (t1 - t0 < edge_min) edge_min = t1 - t0;
    }

    out->w = W; out->h = H;
    out->scan_cyc = scan_min;
    out->thresh_cyc = thresh_min;
    out->edge_cyc = edge_min;
}
