// flow_phase_corr.cc -- on-board phase-correlation optical flow.
//
// Replaces the SAD block-matching estimator (flow_task.cc:sad_match)
// with an FFT-based phase correlation, validated bit-perfect on Linux
// against replay_phase_corr.py over experiments/s083_flow_m7_validate/.
//
// Pipeline per frame:
//   1. crop center 64x60 of 80x60 gray, zero-pad to 64x64
//   2. apply Tukey (alpha=0.25) window, subtract per-frame mean
//   3. 2D FFT (row FFT then col FFT, both arm_cfft_f32 length 64)
//   4. cross-power spectrum: cross = curr . conj(prev) / |curr . conj(prev)|
//   5. 2D IFFT(cross) -> real correlation surface
//   6. peak search + Foroosh-Zerubia sub-pixel + confidence (peak/mean)
//
// Why phase correlation:
//   * fps-invariant: no per-frame motion magnitude tuning
//   * sub-pixel native via Foroosh formula -- no parabolic-fit fragility
//   * FFT is global -- robust to repetitive-pattern false-positives that
//     trip up local block-matching (SAD)
//
// Memory footprint (SDRAM, .sdram_bss):
//   - 4 KB Tukey window (64x64 floats, precomputed once)
//   - 32 KB cached prev FFT (interleaved complex)
//   - 32 KB curr FFT scratch
//   - 32 KB cross / ifft scratch
// = ~100 KB SDRAM, no heap.
//
// embeded.md compliance:
//   - bounded loops (no while-event-wait)
//   - explicit ownership (single-writer s_prev_fft from publisher only)
//   - no dynamic alloc
//   - failure semantics: invalid FFT plan -> return 0, log SERR

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cmath>

#include "examples/sentai_runtime/flow_shared.h"

extern "C" {
#include "arm_math.h"
#include "arm_const_structs.h"
}

namespace {

constexpr int N        = 64;          // 2D FFT size (must be power of 2)
constexpr int N2       = N * N;
constexpr int CROP_W   = 64;          // we use full 64 cols of 80 input
constexpr int CROP_H   = 60;          // bottom 4 rows zero-padded
constexpr int SR       = N / 2;       // search half-range = ±32
constexpr int CROP_OFF = (FLOW_GRAY_W - CROP_W) / 2;  // 8: skip 8 cols each side

// SDRAM-resident buffers.  All single-precision float (M7 FPU).
//
// arm_cfft_f32 layout: interleaved real,imag pairs.
// Buffer of N2 complex elements = 2 * N2 floats.
static float s_window[N2]                __attribute__((section(".sdram_bss")));
static float s_prev_fft[2 * N2]          __attribute__((section(".sdram_bss")));
static float s_curr_fft[2 * N2]          __attribute__((section(".sdram_bss")));
static float s_cross[2 * N2]             __attribute__((section(".sdram_bss")));

static int  s_have_prev = 0;             // 0 = first call, no prev FFT yet
static int  s_initialized = 0;
static const arm_cfft_instance_f32* s_cfft = nullptr;

// Crash-isolation breadcrumbs.  Persistent across CPU reset (SDRAM
// retains content unless full power cycle).  Inspect via JTAG/GDB:
//   (gdb) p s_bc.last_stage
//   (gdb) p/x s_bc.ring[s_bc.idx]
// to see what step was executing when the fault hit.
//
// Stage IDs (see code below):
//   0x10 = enter compute      0x11 = init_once done
//   0x20 = mean computed      0x21 = window+pack done
//   0x30 = fft2d fwd entry    0x31 = fft2d fwd exit
//   0x32 = first-call cache   0x33 = first-call return
//   0x40 = cross-power loop   0x41 = cross-power done
//   0x50 = fft2d inv entry    0x51 = fft2d inv exit
//   0x60 = peak search done   0x61 = subpix done
//   0x70 = output written     0x71 = prev cache done
//   0xF0 = NaN/Inf detected   0xF1 = bad input ptr
//
// Ring 16 entries; last_stage is the latest write.  Each entry:
//   {stage, frame_seq, dwt_now() at write, value (stage-specific)}
struct Breadcrumb {
    uint32_t stage;
    uint32_t frame_seq;
    uint32_t dwt;
    uint32_t value;
};
struct BreadcrumbRing {
    uint32_t magic;          // 0xFCBC1234 once initialized
    uint32_t idx;            // next write slot
    uint32_t last_stage;     // duplicated for fast inspection
    uint32_t fault_count;    // bumps on detected NaN/bad-input
    Breadcrumb ring[16];
};
static volatile BreadcrumbRing s_bc __attribute__((section(".sdram_phase_corr_bc")));

static volatile uint32_t s_call_seq = 0;

static inline uint32_t dwt_cyc(void) {
    extern volatile uint32_t* const _bc_dwt_addr;
    return *((volatile uint32_t*)0xE0001004u);  // DWT->CYCCNT
}

static inline void bc_log(uint32_t stage, uint32_t value) {
    if (s_bc.magic != 0xFCBC1234u) {
        // First touch: zero the ring.
        for (int i = 0; i < 16; ++i) {
            s_bc.ring[i].stage = 0;
            s_bc.ring[i].frame_seq = 0;
            s_bc.ring[i].dwt = 0;
            s_bc.ring[i].value = 0;
        }
        s_bc.idx = 0;
        s_bc.fault_count = 0;
        s_bc.magic = 0xFCBC1234u;
    }
    uint32_t i = s_bc.idx & 15u;
    s_bc.ring[i].stage = stage;
    s_bc.ring[i].frame_seq = s_call_seq;
    s_bc.ring[i].dwt = dwt_cyc();
    s_bc.ring[i].value = value;
    s_bc.idx = (s_bc.idx + 1u) & 0xFFFFu;
    s_bc.last_stage = stage;
}

// Build Tukey window of length n with taper alpha (fraction tapered on
// each side).  alpha=0.25 -> central 75% flat at 1.0, outer 12.5%
// cosine ramp.  Same shape as numpy replay.
static void make_tukey_1d(float* w, int n, float alpha) {
    int edge = (int)(alpha * (n - 1) / 2);
    if (edge < 1) edge = 1;
    for (int i = 0; i < n; ++i) {
        if (i < edge) {
            float t = (float)i / (float)edge - 1.0f;
            w[i] = 0.5f * (1.0f + cosf((float)M_PI * t));
        } else if (i >= n - edge) {
            int j = n - 1 - i;
            float t = (float)j / (float)edge - 1.0f;
            w[i] = 0.5f * (1.0f + cosf((float)M_PI * t));
        } else {
            w[i] = 1.0f;
        }
    }
}

static void init_once(void) {
    if (s_initialized) return;
    // Build 2-D separable Tukey by outer product of 1-D windows.
    float wy[N], wx[N];
    make_tukey_1d(wy, N, 0.25f);
    make_tukey_1d(wx, N, 0.25f);
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            s_window[y * N + x] = wy[y] * wx[x];
        }
    }
    s_cfft = &arm_cfft_sR_f32_len64;   // pre-built 64-point CFFT instance
    s_initialized = 1;
}

// 2D FFT in place: arm_cfft_f32 each row, transpose, arm_cfft_f32 each
// row again (which acts as columns of the original), transpose back.
// `data` is N x N interleaved complex: [r00,i00,r01,i01,...,r0(N-1),i0(N-1),
//                                       r10,i10,...]
//
// `inverse_flag`: 0 = forward FFT, 1 = inverse FFT (CMSIS convention).
// `do_bit_reverse` always 1 here (we want natural-order output).
//
// Bounded: 2*N row FFTs + 1 transpose, all O(N log N).  Total ~512 µs
// at 800 MHz for N=64 (CMSIS arm_cfft_f32 ~4000 cycles per row FFT).
static float s_transpose_scratch[2 * N2]
    __attribute__((section(".sdram_bss")));

static void fft2d(float* data, uint8_t inverse) {
    // Row FFTs.
    for (int y = 0; y < N; ++y) {
        arm_cfft_f32(s_cfft, data + y * 2 * N, inverse, 1);
    }
    // Transpose into scratch.
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            s_transpose_scratch[(x * N + y) * 2 + 0] = data[(y * N + x) * 2 + 0];
            s_transpose_scratch[(x * N + y) * 2 + 1] = data[(y * N + x) * 2 + 1];
        }
    }
    // FFT the transposed rows (= original columns).
    for (int y = 0; y < N; ++y) {
        arm_cfft_f32(s_cfft, s_transpose_scratch + y * 2 * N, inverse, 1);
    }
    // Transpose back into data.
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            data[(y * N + x) * 2 + 0] = s_transpose_scratch[(x * N + y) * 2 + 0];
            data[(y * N + x) * 2 + 1] = s_transpose_scratch[(x * N + y) * 2 + 1];
        }
    }
}

// Foroosh-Zerubia sub-pixel formula for phase-correlation peak.
// Returns offset in milli-grid-pixels in [-500, +500].
static int foroosh_q1000(float a, float b, float c) {
    if (b <= 0.0f) return 0;
    float d;
    if (c >= a) {
        float denom = b + c;
        if (denom <= 0.0f) return 0;
        d = c / denom;
    } else {
        float denom = a + b;
        if (denom <= 0.0f) return 0;
        d = -a / denom;
    }
    if (d >  0.5f) d =  0.5f;
    if (d < -0.5f) d = -0.5f;
    return (int)(d * 1000.0f);
}

}  // namespace

// Public entry: input is 80x60 gray (curr frame); writes
// (dx_q1000, dy_q1000, conf) to outputs.  Confidence is a peak/mean
// ratio (typical clean shift: 30-200; below kConfFloor we report 0).
//
// Returns 0 on first call (no prev cached yet) -- caller treats as
// "no motion".  Subsequent calls return real motion.
//
// dx,dy are in milli-grid-pixels (1000 = 1 grid-px = 8 raw-px after
// PXP downscale), same convention as SAD path -- so the rest of the
// flow_task pipeline (deadband, conf-floor, publish) works unchanged.
extern "C" void sentai_flow_phase_corr_compute(const uint8_t* gray80x60,
                                                int* dx_q1000_out,
                                                int* dy_q1000_out,
                                                uint8_t* conf_out) {
    s_call_seq++;
    bc_log(0x10, (uint32_t)gray80x60);
    init_once();
    bc_log(0x11, (uint32_t)s_cfft);

    if (!gray80x60 || !dx_q1000_out || !dy_q1000_out || !conf_out) {
        bc_log(0xF1, 0);
        s_bc.fault_count++;
        return;
    }

    *dx_q1000_out = 0;
    *dy_q1000_out = 0;
    *conf_out     = 0;

    // Step 1: crop 64x60 from 80x60 (drop 8 cols each side), zero-pad
    // bottom rows, mean-subtract, apply Tukey, store as complex (imag=0).
    //
    // Per-frame mean removal kills DC drift from AEC; window kills
    // wrap-around bleed; both required for a clean phase-corr peak.
    float mean = 0.0f;
    int active_pixels = 0;
    for (int y = 0; y < CROP_H; ++y) {
        const uint8_t* src = gray80x60 + y * FLOW_GRAY_W + CROP_OFF;
        for (int x = 0; x < CROP_W; ++x) {
            mean += (float)src[x];
            active_pixels++;
        }
    }
    mean /= (float)active_pixels;
    bc_log(0x20, (uint32_t)mean);

    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            float v = 0.0f;
            if (y < CROP_H && x < CROP_W) {
                const uint8_t s = gray80x60[y * FLOW_GRAY_W + (CROP_OFF + x)];
                v = ((float)s - mean) * s_window[y * N + x];
            }
            // else zero (padding)
            s_curr_fft[(y * N + x) * 2 + 0] = v;
            s_curr_fft[(y * N + x) * 2 + 1] = 0.0f;
        }
    }
    bc_log(0x21, 0);

    // Step 2: forward 2D FFT.
    bc_log(0x30, 0);
    fft2d(s_curr_fft, /*inverse=*/0);
    bc_log(0x31, 0);

    if (!s_have_prev) {
        bc_log(0x32, 0);
        memcpy(s_prev_fft, s_curr_fft, sizeof(s_prev_fft));
        s_have_prev = 1;
        bc_log(0x33, 0);
        return;
    }

    // Step 3: cross-power spectrum, normalized.
    // R = curr . conj(prev) / |curr . conj(prev)|
    bc_log(0x40, 0);
    uint32_t bad_count = 0;
    for (int i = 0; i < N2; ++i) {
        float cr = s_curr_fft[i * 2 + 0];
        float ci = s_curr_fft[i * 2 + 1];
        float pr = s_prev_fft[i * 2 + 0];
        float pi = s_prev_fft[i * 2 + 1];
        // curr * conj(prev) = (cr + i ci) * (pr - i pi)
        //                   = (cr*pr + ci*pi) + i (ci*pr - cr*pi)
        float xr = cr * pr + ci * pi;
        float xi = ci * pr - cr * pi;
        // Bounded NaN/Inf guard: if any operand is non-finite,
        // substitute zero (degraded) and bump fault counter.
        if (!(xr == xr) || !(xi == xi)) {
            xr = 0.0f; xi = 0.0f;
            bad_count++;
        }
        float mag = sqrtf(xr * xr + xi * xi) + 1e-10f;
        float nr = xr / mag;
        float ni = xi / mag;
        if (!(nr == nr) || !(ni == ni)) {
            nr = 0.0f; ni = 0.0f;
            bad_count++;
        }
        s_cross[i * 2 + 0] = nr;
        s_cross[i * 2 + 1] = ni;
    }
    bc_log(0x41, bad_count);
    if (bad_count > 0) {
        s_bc.fault_count++;
        bc_log(0xF0, bad_count);
        // Degraded mode: skip IFFT, return zeros (caller treats as
        // "no motion this frame").  Cache curr FFT so next frame can
        // try again.  Bounded recovery, no infinite retry.
        memcpy(s_prev_fft, s_curr_fft, sizeof(s_prev_fft));
        return;
    }

    // Step 4: inverse 2D FFT -> real correlation surface.
    bc_log(0x50, 0);
    fft2d(s_cross, /*inverse=*/1);
    bc_log(0x51, 0);

    // Find peak in fftshifted layout (origin at center).  Equivalently,
    // search the unshifted [0..N-1]^2 array and unwrap shifts > N/2 to
    // negative.  We do the latter (saves a copy).
    float peak_val = -1e30f;
    int peak_y = 0, peak_x = 0;
    float sum_abs = 0.0f;
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            float v = s_cross[(y * N + x) * 2 + 0];   // real part only
            float av = v < 0 ? -v : v;
            sum_abs += av;
            if (v > peak_val) {
                peak_val = v;
                peak_y = y;
                peak_x = x;
            }
        }
    }
    float mean_abs = sum_abs / (float)N2 + 1e-10f;
    float peak_ratio = peak_val / mean_abs;  // 5-200 typical

    // Unwrap to signed shift.
    int dy_int = (peak_y > N / 2) ? (peak_y - N) : peak_y;
    int dx_int = (peak_x > N / 2) ? (peak_x - N) : peak_x;

    // Sub-pixel via Foroosh.  Need correlation values at peak ± 1
    // (with wrap-around).
    auto val_at = [&](int yy, int xx) -> float {
        // wrap to [0,N)
        if (yy < 0) yy += N;
        if (yy >= N) yy -= N;
        if (xx < 0) xx += N;
        if (xx >= N) xx -= N;
        return s_cross[(yy * N + xx) * 2 + 0];
    };
    float a_x = val_at(peak_y, peak_x - 1);
    float b_x = peak_val;
    float c_x = val_at(peak_y, peak_x + 1);
    float a_y = val_at(peak_y - 1, peak_x);
    float c_y = val_at(peak_y + 1, peak_x);

    int delta_x = foroosh_q1000(a_x, b_x, c_x);
    int delta_y = foroosh_q1000(a_y, b_x, c_y);

    // Map FFT bins (1 bin = 1 cropped pixel = 1 grid-px after the
    // 8x PXP downscale) directly to grid-px units.  N=64 grid-px max
    // shift ±32 grid-px = ±256 raw-px; no wrap-around concerns within
    // the operational motion range.
    int dx_q = dx_int * 1000 + delta_x;
    int dy_q = dy_int * 1000 + delta_y;

    bc_log(0x60, ((uint32_t)(peak_y) << 16) | (uint32_t)peak_x);
    bc_log(0x61, ((uint32_t)(uint16_t)delta_y << 16) | (uint32_t)(uint16_t)delta_x);

    *dx_q1000_out = dx_q;
    *dy_q1000_out = dy_q;
    // Map peak ratio to 0..255 confidence proxy.  Empirical: <5 =
    // garbage, 5-20 = weak, >50 = clean shift.  Saturating linear map.
    int c = (int)(peak_ratio * 5.0f);
    if (c < 0) c = 0;
    if (c > 255) c = 255;
    *conf_out = (uint8_t)c;
    bc_log(0x70, ((uint32_t)(uint16_t)dy_q << 16) | (uint32_t)(uint16_t)dx_q);

    // Step 5: cache curr FFT as prev for next frame.
    memcpy(s_prev_fft, s_curr_fft, sizeof(s_prev_fft));
    bc_log(0x71, 0);
}

// Reset cached state.  Call at flow.start() to drop stale prev FFT.
extern "C" void sentai_flow_phase_corr_reset(void) {
    s_have_prev = 0;
}

// Dump persistent breadcrumb ring to printf().  Call once at boot
// (after USB CDC + DEBUG_PRINT init) so a WDOG reset between
// crash-and-this-call leaves the ring intact in SDRAM (.sdram_bss
// is NOT zeroed by C runtime since we mark the section so).  Reads
// the magic word; if absent, prints "no prior fault" instead.
extern "C" void sentai_flow_phase_corr_dump_bc(void) {
    if (s_bc.magic != 0xFCBC1234u) {
        printf("[flowpc] bc: no prior breadcrumbs (magic=%08lx)\r\n",
               (unsigned long)s_bc.magic);
        return;
    }
    printf("[flowpc] bc: last_stage=0x%02lx fault_count=%lu idx=%lu\r\n",
           (unsigned long)s_bc.last_stage,
           (unsigned long)s_bc.fault_count,
           (unsigned long)s_bc.idx);
    // Print ring oldest-first.  Start at idx (oldest), end at idx-1.
    uint32_t start = s_bc.idx & 15u;
    for (int k = 0; k < 16; ++k) {
        uint32_t i = (start + (uint32_t)k) & 15u;
        printf("  bc[%2lu] stage=0x%02lx seq=%-4lu dwt=%010lu val=%08lx\r\n",
               (unsigned long)i,
               (unsigned long)s_bc.ring[i].stage,
               (unsigned long)s_bc.ring[i].frame_seq,
               (unsigned long)s_bc.ring[i].dwt,
               (unsigned long)s_bc.ring[i].value);
    }
    // Clear magic so next boot won't double-print unless a new run
    // populates it again.
    s_bc.magic = 0u;
}
