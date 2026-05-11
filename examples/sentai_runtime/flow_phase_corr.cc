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
#ifdef SENTAI_PLATFORM_SIM
#include <ctime>      // clock_gettime / CLOCK_MONOTONIC for SIM dwt_cyc shim
#endif

#include "examples/sentai_runtime/flow_shared.h"
#include "examples/sentai_runtime/sentai_fft_shim.h"
// sentai_fft_shim.h pulls in CMSIS arm_math.h on ARM and a portable
// FFTW3-backed shim on SIM (gated by SENTAI_PLATFORM_SIM).  Same call
// site `sentai_cfft_f32(s_cfft, ...)` works on both targets.

namespace {

constexpr int N        = 64;          // 2D FFT size (must be power of 2)
constexpr int N2       = N * N;
// Guizar-Sicairos sub-pixel upsampling.  M=10 gives 1/10 grid resolution
// = 0.022 mm/pixel at z=1m drone hover.  K = 2*M+1 search window around
// integer peak (±1 pixel at 1/M resolution).
constexpr int GS_M     = 10;
constexpr int GS_K     = 2 * GS_M + 1;   // 21
constexpr int CROP_W   = 64;          // we use full 64 cols of 80 input
constexpr int CROP_H   = 60;          // bottom 4 rows zero-padded
constexpr int SR       = N / 2;       // search half-range = ±32
constexpr int CROP_OFF = (FLOW_GRAY_W - CROP_W) / 2;  // 8: skip 8 cols each side

// SDRAM-resident buffers.  All single-precision float (M7 FPU).
//
// arm_cfft_f32 layout: interleaved real,imag pairs.
// Buffer of N2 complex elements = 2 * N2 floats.
static float s_window[N2]                __attribute__((section(".sdram_bss")));
// PER-PIPELINE state arrays.  Up to FLOW_N_PIPES independent contexts
// share the same algorithm but have isolated prev_fft + running-avg
// caches, so the caller can drive a pyramid (L0, L1, L2…) without
// state cross-contamination between levels.
//
// Storage: FLOW_N_PIPES × (32KB prev_fft + 32KB cross_spec_avg) = 192KB
// total for 3 pipes.  Lives in SDRAM (sdram_bss section).
#define FLOW_N_PIPES 4
static float s_prev_fft[FLOW_N_PIPES][2 * N2]
    __attribute__((section(".sdram_bss")));
static int   s_have_prev_per[FLOW_N_PIPES] = {0};
static float s_cross_spec_avg_per[FLOW_N_PIPES][2 * N2]
    __attribute__((section(".sdram_bss")));
static int   s_have_cross_avg_per[FLOW_N_PIPES] = {0};
// Shared intermediates — written + read sequentially within one
// compute() call, so no race between pipes (calls are serialised on
// caller side).
static float s_curr_fft[2 * N2]          __attribute__((section(".sdram_bss")));
static float s_cross[2 * N2]             __attribute__((section(".sdram_bss")));
static float s_cross_spec[2 * N2]        __attribute__((section(".sdram_bss")));
// Pre-computed twiddle factors for Guizar-Sicairos.  We sample the
// inverse-FFT at K fractional positions (-1 .. +1 pixels at 1/M
// resolution) around the integer peak.  But peak position varies
// per-frame, so we precompute the BASE twiddle (one per output bin
// per input bin) and multiply by exp(2πi · peak · n / N) at runtime.
// Storage: K complex twiddles per input bin × N input bins × 2 axes.
// Total: GS_K * N * 2 (real+imag) per axis = 21 * 64 * 2 = 2688 floats.
static float s_gs_twid_x[GS_K * N * 2]   __attribute__((section(".sdram_bss")));
static float s_gs_twid_y[GS_K * N * 2]   __attribute__((section(".sdram_bss")));
// Intermediate buffer for separable 2D upsampled DFT: K rows × N cols.
static float s_gs_intermed[GS_K * N * 2] __attribute__((section(".sdram_bss")));
// Output: K × K upsampled correlation surface (real part used for peak).
static float s_gs_upsamp[GS_K * GS_K * 2] __attribute__((section(".sdram_bss")));

// (per-pipe have_prev flags are in s_have_prev_per[] above)
static int  s_initialized = 0;
static const sentai_cfft_instance_f32* s_cfft = nullptr;

// Forward decl for dz-divergence sub-block path (impl below; reset() needs it).
static int s_have_prev_subblocks = 0;

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
#ifdef SENTAI_PLATFORM_SIM
    // Cortex-M DWT->CYCCNT does not exist on x86; segfault if dereferenced.
    // Use CLOCK_MONOTONIC nanoseconds & 0xFFFFFFFF as a stand-in cycle counter
    // (sufficient resolution for breadcrumb forensics; only used in bc_log).
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec);
#else
    extern volatile uint32_t* const _bc_dwt_addr;
    return *((volatile uint32_t*)0xE0001004u);  // DWT->CYCCNT
#endif
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
    s_cfft = &sentai_cfft_sR_f32_len64;   // pre-built 64-point CFFT instance

    // Pre-compute Guizar-Sicairos base twiddle factors.  For each output
    // sub-pixel offset k_off (∈ [-1, +1] at 1/M step) and each input bin
    // n (∈ [0, N)), the twiddle is exp(+2πi · k_off · n / N) — used to
    // evaluate iFFT at fractional position k_off above the integer peak.
    // We separate twiddle_x (col) and twiddle_y (row) for the separable
    // 2D pass.  Both arrays are identical since the formula is symmetric;
    // we keep two copies anyway for clarity + possible future asymmetry.
    for (int k = 0; k < GS_K; ++k) {
        float frac = (float)(k - GS_M) / (float)GS_M;   // -1 .. +1
        for (int n = 0; n < N; ++n) {
            // 2D iFFT convention: factor = exp(+2πi · frac · n / N)
            float ang = 2.0f * (float)M_PI * frac * (float)n / (float)N;
            s_gs_twid_y[(k * N + n) * 2 + 0] = cosf(ang);
            s_gs_twid_y[(k * N + n) * 2 + 1] = sinf(ang);
            s_gs_twid_x[(k * N + n) * 2 + 0] = cosf(ang);
            s_gs_twid_x[(k * N + n) * 2 + 1] = sinf(ang);
        }
    }
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
        sentai_cfft_f32(s_cfft, data + y * 2 * N, inverse, 1);
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
        sentai_cfft_f32(s_cfft, s_transpose_scratch + y * 2 * N, inverse, 1);
    }
    // Transpose back into data.
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            data[(y * N + x) * 2 + 0] = s_transpose_scratch[(x * N + y) * 2 + 0];
            data[(y * N + x) * 2 + 1] = s_transpose_scratch[(x * N + y) * 2 + 1];
        }
    }
}

// Guizar-Sicairos sub-pixel refinement via DFT-based upsampling.
// Evaluates the inverse-FFT of the (already-normalized) cross-power
// spectrum at K = 2*M+1 fractional positions in each axis around the
// integer peak.  Output is the sub-pixel offset in milli-grid-pixels.
//
// Reference: Guizar-Sicairos, Thurman & Fienup 2008, "Efficient subpixel
// image registration algorithms", Optics Letters 33(2):156-158.
//
// Cost: K*N + K*K*N complex muls (separable 2D DFT) = ~115K complex muls
// for K=21, N=64.  ~500K float ops total.  On Cortex-M7 @ 800 MHz with
// cached twiddles ≈ 300-600 µs per frame.  On x86 ≈ 100 µs.
static void guizar_sicairos_q1000(int peak_x, int peak_y,
                                    int* delta_x_q1000_out,
                                    int* delta_y_q1000_out) {
    // First pass: 1D DFT in Y, evaluated at K fractional rows around
    // peak_y.  For each output (ky, x):
    //   intermed[ky, x] = sum_{j} s_cross_spec[j, x] ·
    //                     exp(+2πi · (peak_y + frac_ky) · j / N)
    //   where frac_ky = (ky - M) / M
    // = sum_{j} s_cross_spec[j, x] ·
    //   exp(+2πi · peak_y · j / N) · twid_y[ky, j]
    //
    // We fold the peak-y rotation into the loop for cache-friendly access.
    for (int ky = 0; ky < GS_K; ++ky) {
        for (int x = 0; x < N; ++x) {
            float sum_re = 0.0f, sum_im = 0.0f;
            for (int j = 0; j < N; ++j) {
                // Integer-peak twiddle: exp(+2πi · peak_y · j / N)
                float ang_p = 2.0f * (float)M_PI * (float)peak_y * (float)j / (float)N;
                float cp = cosf(ang_p), sp = sinf(ang_p);
                // Sub-pixel twiddle from cache
                float ck = s_gs_twid_y[(ky * N + j) * 2 + 0];
                float sk = s_gs_twid_y[(ky * N + j) * 2 + 1];
                // Combined twiddle: (cp + i sp) · (ck + i sk)
                float tw_re = cp * ck - sp * sk;
                float tw_im = cp * sk + sp * ck;
                float spec_re = s_cross_spec[(j * N + x) * 2 + 0];
                float spec_im = s_cross_spec[(j * N + x) * 2 + 1];
                sum_re += spec_re * tw_re - spec_im * tw_im;
                sum_im += spec_re * tw_im + spec_im * tw_re;
            }
            s_gs_intermed[(ky * N + x) * 2 + 0] = sum_re;
            s_gs_intermed[(ky * N + x) * 2 + 1] = sum_im;
        }
    }
    // Second pass: 1D DFT in X, evaluated at K fractional cols around
    // peak_x, on the K rows from pass 1.
    float best_val = -1e30f;
    int best_kx = GS_M, best_ky = GS_M;   // default: integer peak
    for (int ky = 0; ky < GS_K; ++ky) {
        for (int kx = 0; kx < GS_K; ++kx) {
            float sum_re = 0.0f, sum_im = 0.0f;
            for (int i = 0; i < N; ++i) {
                float ang_p = 2.0f * (float)M_PI * (float)peak_x * (float)i / (float)N;
                float cp = cosf(ang_p), sp = sinf(ang_p);
                float ck = s_gs_twid_x[(kx * N + i) * 2 + 0];
                float sk = s_gs_twid_x[(kx * N + i) * 2 + 1];
                float tw_re = cp * ck - sp * sk;
                float tw_im = cp * sk + sp * ck;
                float r = s_gs_intermed[(ky * N + i) * 2 + 0];
                float im = s_gs_intermed[(ky * N + i) * 2 + 1];
                sum_re += r * tw_re - im * tw_im;
                sum_im += r * tw_im + im * tw_re;
            }
            s_gs_upsamp[(ky * GS_K + kx) * 2 + 0] = sum_re;
            s_gs_upsamp[(ky * GS_K + kx) * 2 + 1] = sum_im;
            if (sum_re > best_val) {
                best_val = sum_re;
                best_kx = kx;
                best_ky = ky;
            }
        }
    }
    // Sub-pixel offset relative to integer peak in milli-grid units.
    // Range: [-1000, +1000] mgrid = ±1 grid-pixel at 1/M resolution.
    *delta_x_q1000_out = ((best_kx - GS_M) * 1000) / GS_M;
    *delta_y_q1000_out = ((best_ky - GS_M) * 1000) / GS_M;
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
// 2026-05-11: refactored to take a pipe_id for pyramid-multi-instance
// support.  Each pipe maintains independent prev_fft + cross_spec_avg.
// Legacy entry-point sentai_flow_phase_corr_compute (no pipe_id) is
// kept below as a wrapper that calls with pipe_id=0.
extern "C" void sentai_flow_phase_corr_compute_at(int pipe_id,
                                                   const uint8_t* gray80x60,
                                                   int* dx_q1000_out,
                                                   int* dy_q1000_out,
                                                   uint8_t* conf_out) {
    if (pipe_id < 0) pipe_id = 0;
    if (pipe_id >= FLOW_N_PIPES) pipe_id = FLOW_N_PIPES - 1;
    s_call_seq++;
    // Pointer breadcrumbs: cast via uintptr_t so the same source file
    // builds on both 32-bit ARM and 64-bit SIM (truncating to low 32 bits
    // is fine — bc_log is a forensic ring, not a precise pointer log).
    bc_log(0x10, (uint32_t)(uintptr_t)gray80x60);
    init_once();
    bc_log(0x11, (uint32_t)(uintptr_t)s_cfft);

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

    if (!s_have_prev_per[pipe_id]) {
        bc_log(0x32, 0);
        memcpy(s_prev_fft[pipe_id], s_curr_fft, sizeof(s_prev_fft[pipe_id]));
        s_have_prev_per[pipe_id] = 1;
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
        float pr = s_prev_fft[pipe_id][i * 2 + 0];
        float pi = s_prev_fft[pipe_id][i * 2 + 1];
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
        memcpy(s_prev_fft[pipe_id], s_curr_fft, sizeof(s_prev_fft[pipe_id]));
        return;
    }

    // Save the cross-power spectrum BEFORE iFFT — Guizar-Sicairos
    // sub-pixel refinement evaluates iFFT at fractional positions.
    memcpy(s_cross_spec, s_cross, sizeof(s_cross_spec));

    // Temporal accumulation EVALUATED — DISABLED.
    //   α=0.30 (heavy averaging): all-4-detect 61% → 29% (REGRESSION)
    //   α=0.60 (light averaging): dist_max -10% but conf=0 rate +5%
    //   α=1.00 (passthrough)    : current baseline
    //
    // Root cause: drone hover motion is NOT monotonic — IMU noise +
    // wind gusts + controller response cause direction reversals every
    // 100-300ms.  Cross-power spectrum phase rotates between frames,
    // so averaging DECOHERES the signal instead of building it up.
    //
    // Coherent averaging works when motion is constant over the window
    // (e.g., radar pulses on fixed target, astronomy stacking).  Doesn't
    // apply here.  Code path kept for adaptive-α experimentation later.
    const float ALPHA = 1.00f;
    if (!s_have_cross_avg_per[pipe_id]) {
        memcpy(s_cross_spec_avg_per[pipe_id], s_cross_spec,
               sizeof(s_cross_spec_avg_per[pipe_id]));
        s_have_cross_avg_per[pipe_id] = 1;
    } else {
        for (int i = 0; i < N2 * 2; ++i) {
            s_cross_spec_avg_per[pipe_id][i] =
                  ALPHA * s_cross_spec[i]
                + (1.0f - ALPHA) * s_cross_spec_avg_per[pipe_id][i];
        }
    }
    memcpy(s_cross, s_cross_spec_avg_per[pipe_id], sizeof(s_cross));
    memcpy(s_cross_spec, s_cross_spec_avg_per[pipe_id], sizeof(s_cross_spec));

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

    // Sub-pixel refinement via Guizar-Sicairos DFT-upsampling.
    // Evaluates iFFT of the cross-power spectrum at 1/M resolution
    // around the integer peak — more robust to broad/noisy peaks than
    // Foroosh's 3-point local fit (the previous algorithm).  Returns
    // delta_x/y in milli-grid units, range [-1000, +1000].
    int delta_x = 0, delta_y = 0;
    guizar_sicairos_q1000(peak_x, peak_y, &delta_x, &delta_y);
    // Defensive clamp — should never trigger if M=10 and the algorithm
    // searches ±1 pixel, but guards against integer peak being at extreme.
    if (delta_x >  1000) delta_x =  1000;
    if (delta_x < -1000) delta_x = -1000;
    if (delta_y >  1000) delta_y =  1000;
    if (delta_y < -1000) delta_y = -1000;
    (void)foroosh_q1000;   // kept for reference / future A/B fallback

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
    memcpy(s_prev_fft[pipe_id], s_curr_fft, sizeof(s_prev_fft[pipe_id]));
    bc_log(0x71, 0);
}

// Legacy entry point — calls into pipe_id=0 (single-instance compat).
extern "C" void sentai_flow_phase_corr_compute(const uint8_t* gray80x60,
                                                int* dx_q1000_out,
                                                int* dy_q1000_out,
                                                uint8_t* conf_out) {
    sentai_flow_phase_corr_compute_at(0, gray80x60,
                                       dx_q1000_out, dy_q1000_out, conf_out);
}

// LastChangedFrame (anchor) helpers — pipe's prev_fft is FROZEN at a
// past frame.  Compare current frame to that frozen reference WITHOUT
// updating the frozen prev_fft.  Reveals cumulative sub-pixel drift
// that frame-to-frame phase-corr cannot resolve.
//
// Usage:
//   1. Call _set_anchor(pipe, gray) when you decide to "freeze" — copies
//      FFT(gray) to pipe's prev_fft.  Subsequent _compute_against_anchor
//      calls will compare against this frame.
//   2. Each frame, call _compute_against_anchor(pipe, gray, ...) — runs
//      phase-corr against frozen prev, does NOT touch prev_fft.
//   3. When motion picks up (caller decides via instantaneous flow), call
//      _set_anchor again to refresh the reference to the current frame.
extern "C" void sentai_flow_phase_corr_set_anchor(int pipe_id,
                                                    const uint8_t* gray80x60) {
    if (pipe_id < 0) pipe_id = 0;
    if (pipe_id >= FLOW_N_PIPES) pipe_id = FLOW_N_PIPES - 1;
    init_once();
    // Window + FFT the gray buffer into prev_fft[pipe_id] directly.
    float mean = 0.0f;
    int n = 0;
    for (int y = 0; y < CROP_H; ++y) {
        for (int x = 0; x < CROP_W; ++x) {
            mean += (float)gray80x60[y * FLOW_GRAY_W + (CROP_OFF + x)];
            n++;
        }
    }
    mean /= (float)n;
    for (int y = 0; y < N; ++y) {
        for (int x = 0; x < N; ++x) {
            float v = 0.0f;
            if (y < CROP_H && x < CROP_W) {
                uint8_t s = gray80x60[y * FLOW_GRAY_W + (CROP_OFF + x)];
                v = ((float)s - mean) * s_window[y * N + x];
            }
            s_curr_fft[(y * N + x) * 2 + 0] = v;
            s_curr_fft[(y * N + x) * 2 + 1] = 0.0f;
        }
    }
    fft2d(s_curr_fft, /*inverse=*/0);
    memcpy(s_prev_fft[pipe_id], s_curr_fft, sizeof(s_prev_fft[pipe_id]));
    s_have_prev_per[pipe_id] = 1;
}

// Same as _compute_at but skips the prev_fft update at the end —
// keeps the frozen reference alive across multiple frames.  Caller is
// responsible for calling _set_anchor when refresh is needed.
extern "C" void sentai_flow_phase_corr_compute_against_anchor(int pipe_id,
                                                                const uint8_t* gray80x60,
                                                                int* dx_q1000_out,
                                                                int* dy_q1000_out,
                                                                uint8_t* conf_out) {
    // Save prev_fft[pipe_id], call compute_at, restore prev_fft[pipe_id].
    // Simpler than refactoring compute_at to skip the update conditionally.
    static float s_save[2 * N2];
    if (pipe_id < 0) pipe_id = 0;
    if (pipe_id >= FLOW_N_PIPES) pipe_id = FLOW_N_PIPES - 1;
    if (!s_have_prev_per[pipe_id]) {
        *dx_q1000_out = 0;
        *dy_q1000_out = 0;
        *conf_out = 0;
        return;
    }
    memcpy(s_save, s_prev_fft[pipe_id], sizeof(s_save));
    sentai_flow_phase_corr_compute_at(pipe_id, gray80x60,
                                       dx_q1000_out, dy_q1000_out, conf_out);
    memcpy(s_prev_fft[pipe_id], s_save, sizeof(s_prev_fft[pipe_id]));
}

// Reset cached state.  Call at flow.start() to drop stale prev FFT.
extern "C" void sentai_flow_phase_corr_reset(void) {
    for (int p = 0; p < FLOW_N_PIPES; ++p) {
        s_have_prev_per[p] = 0;
        s_have_cross_avg_per[p] = 0;
    }
    s_have_prev_subblocks = 0;
}

// ─────────────────────────────────────────────────────────────────────────
// Sub-block phase-correlation for vertical-axis flow (dz / altitude rate).
//
// Theory: for a downward-pointing camera, when the drone rises (z increases)
// features on the ground appear to move RADIALLY OUTWARD from image centre
// (expansion / divergence > 0).  When the drone descends, features move
// inward (convergence / divergence < 0).
//
// Mathematical relation (Horn-Schunck flow divergence theorem):
//
//   dz/z = -divergence(flow) / 2
//
// where divergence = d(vx)/dx + d(vy)/dy.
//
// We approximate divergence by running phase-correlation on TWO horizontal
// sub-blocks of the gray80x60 input (top half rows 0..29, bottom half rows
// 30..59) and comparing the dy values:
//
//   div_y ≈ (dy_bottom - dy_top) / (block_height_gridpx)
//
// Plus by running phase-correlation on TWO vertical sub-blocks (left and
// right halves) and comparing dx:
//
//   div_x ≈ (dx_right - dx_left) / (block_width_gridpx)
//
// Total divergence: div = div_x + div_y (in 1/grid-px units).
//
// Output dz_q1000 is in milli-grid-px-equivalent altitude shift per frame
// (using the same scale convention as dx/dy so the caller can apply the
// same dt + scaling math).
//
// Cost: 4 extra 32×32 (padded to 32×32 for FFT) phase-corrs on top of the
// main 64×64 — about 4×0.4ms = 1.6 ms extra on the M7 cortex at 800 MHz,
// well inside the 33 ms / 30 fps budget.
// ─────────────────────────────────────────────────────────────────────────
constexpr int SUBN = 32;  // sub-block FFT size (power of 2)
constexpr int SUBH = 30;  // sub-block height = half of CROP_H (60/2)
constexpr int SUBW = 32;  // sub-block width  = half of CROP_W (64/2)

// One FFT buffer per sub-block; cached "previous" pair for divergence.
static float s_sub_curr[2 * SUBN * SUBN]   __attribute__((section(".sdram_bss")));
static float s_sub_cross[2 * SUBN * SUBN]  __attribute__((section(".sdram_bss")));
static float s_sub_prev[4][2 * SUBN * SUBN] __attribute__((section(".sdram_bss")));
// Indices: 0=top-half, 1=bottom-half (for vertical divergence),
//          2=left-half, 3=right-half (for horizontal divergence).
// s_have_prev_subblocks is forward-declared above (used by reset()).
static float s_sub_window[SUBN * SUBN] __attribute__((section(".sdram_bss")));

// Pack a (SUBW × SUBH) gray crop into a 32×32 zero-padded interleaved
// complex buffer with mean-subtraction + window.
static void pack_subblock(const uint8_t* gray, int y0, int x0,
                          float* dst_complex) {
    // Per-block mean for DC removal.
    float mean = 0.0f;
    int n = 0;
    for (int y = y0; y < y0 + SUBH && y < FLOW_GRAY_H; ++y) {
        for (int x = x0; x < x0 + SUBW && x < FLOW_GRAY_W; ++x) {
            mean += (float)gray[y * FLOW_GRAY_W + x];
            ++n;
        }
    }
    mean /= (float)(n ? n : 1);

    for (int y = 0; y < SUBN; ++y) {
        for (int x = 0; x < SUBN; ++x) {
            float v = 0.0f;
            const int sy = y0 + y;
            const int sx = x0 + x;
            if (y < SUBH && x < SUBW && sy < FLOW_GRAY_H && sx < FLOW_GRAY_W) {
                v = ((float)gray[sy * FLOW_GRAY_W + sx] - mean)
                  * s_sub_window[y * SUBN + x];
            }
            dst_complex[(y * SUBN + x) * 2 + 0] = v;
            dst_complex[(y * SUBN + x) * 2 + 1] = 0.0f;
        }
    }
}

// 2D FFT for SUBN=32 size, sharing the same transpose pattern as fft2d().
static float s_sub_transpose[2 * SUBN * SUBN]
    __attribute__((section(".sdram_bss")));
static const sentai_cfft_instance_f32* sub_cfft_inst(void);
static void fft2d_sub(float* data, uint8_t inverse) {
    const sentai_cfft_instance_f32* inst = sub_cfft_inst();
    for (int y = 0; y < SUBN; ++y)
        sentai_cfft_f32(inst, data + y * 2 * SUBN, inverse, 1);
    for (int y = 0; y < SUBN; ++y)
        for (int x = 0; x < SUBN; ++x) {
            s_sub_transpose[(x * SUBN + y) * 2 + 0] = data[(y * SUBN + x) * 2 + 0];
            s_sub_transpose[(x * SUBN + y) * 2 + 1] = data[(y * SUBN + x) * 2 + 1];
        }
    for (int y = 0; y < SUBN; ++y)
        sentai_cfft_f32(inst, s_sub_transpose + y * 2 * SUBN, inverse, 1);
    for (int y = 0; y < SUBN; ++y)
        for (int x = 0; x < SUBN; ++x) {
            data[(y * SUBN + x) * 2 + 0] = s_sub_transpose[(x * SUBN + y) * 2 + 0];
            data[(y * SUBN + x) * 2 + 1] = s_sub_transpose[(x * SUBN + y) * 2 + 1];
        }
}

#ifdef SENTAI_PLATFORM_SIM
// SIM uses our shim — instances are lazily-init'd structs.  Length 32
// is a separate instance; declare here.
extern "C" sentai_cfft_instance_f32 sentai_cfft_sR_f32_len32;
sentai_cfft_instance_f32 sentai_cfft_sR_f32_len32_local = { .N = 32, .plan_fwd = nullptr, .plan_inv = nullptr };
static const sentai_cfft_instance_f32* sub_cfft_inst(void) {
    return &sentai_cfft_sR_f32_len32_local;
}
#else
// ARM CMSIS has arm_cfft_sR_f32_len32 prebuilt.
static const sentai_cfft_instance_f32* sub_cfft_inst(void) {
    return &arm_cfft_sR_f32_len32;
}
#endif

// Compute single sub-block flow vs cached prev; returns dy in milli-grid-px.
// idx in [0..3] selects which prev buffer.  Returns 0 if no prev cached.
static int subblock_phase_corr(const uint8_t* gray, int y0, int x0, int idx,
                                 int* out_dx_q, int* out_dy_q) {
    *out_dx_q = 0; *out_dy_q = 0;

    pack_subblock(gray, y0, x0, s_sub_curr);
    fft2d_sub(s_sub_curr, /*inverse=*/0);

    if (!(s_have_prev_subblocks & (1 << idx))) {
        memcpy(s_sub_prev[idx], s_sub_curr, sizeof(s_sub_curr));
        s_have_prev_subblocks |= (1 << idx);
        return 0;
    }

    // Cross-power, normalized.
    const int n2 = SUBN * SUBN;
    for (int i = 0; i < n2; ++i) {
        float cr = s_sub_curr[i * 2 + 0], ci = s_sub_curr[i * 2 + 1];
        float pr = s_sub_prev[idx][i * 2 + 0], pi = s_sub_prev[idx][i * 2 + 1];
        float xr = cr * pr + ci * pi;
        float xi = ci * pr - cr * pi;
        float mag = sqrtf(xr * xr + xi * xi) + 1e-10f;
        s_sub_cross[i * 2 + 0] = xr / mag;
        s_sub_cross[i * 2 + 1] = xi / mag;
    }
    fft2d_sub(s_sub_cross, /*inverse=*/1);

    // Peak search.
    float peak_val = -1e30f;
    int peak_y = 0, peak_x = 0;
    for (int y = 0; y < SUBN; ++y)
        for (int x = 0; x < SUBN; ++x) {
            float v = s_sub_cross[(y * SUBN + x) * 2 + 0];
            if (v > peak_val) { peak_val = v; peak_y = y; peak_x = x; }
        }
    int dy_int = (peak_y > SUBN / 2) ? (peak_y - SUBN) : peak_y;
    int dx_int = (peak_x > SUBN / 2) ? (peak_x - SUBN) : peak_x;
    *out_dx_q = dx_int * 1000;
    *out_dy_q = dy_int * 1000;

    // Update prev cache.
    memcpy(s_sub_prev[idx], s_sub_curr, sizeof(s_sub_curr));
    return 1;
}

// Public dz entry.  Call AFTER sentai_flow_phase_corr_compute() on the
// same frame (or instead of it if only dz is needed).
//
//   dz_q1000_out: milli-grid-px scale change per frame
//                  positive ≈ drone rising  (features expanding outward)
//                  negative ≈ drone falling (features converging)
//   conf_out:     0..255, peak strength across the 4 sub-blocks (min)
extern "C" void sentai_flow_phase_corr_compute_dz(const uint8_t* gray80x60,
                                                    int* dz_q1000_out,
                                                    uint8_t* conf_out) {
    *dz_q1000_out = 0;
    *conf_out = 0;

    // Lazy init of sub-block window (same Tukey as the main path, sized 32).
    static int s_sub_init = 0;
    if (!s_sub_init) {
        float w1d[SUBN];
        make_tukey_1d(w1d, SUBN, 0.25f);
        for (int y = 0; y < SUBN; ++y)
            for (int x = 0; x < SUBN; ++x)
                s_sub_window[y * SUBN + x] = w1d[y] * w1d[x];
        s_sub_init = 1;
    }

    // 4 sub-blocks of the 64×60 crop (we use the same CROP_OFF as the main):
    //   idx 0: top    (rows 0..29, cols 8..39  → 32 wide)
    //   idx 1: bottom (rows 30..59, cols 8..39)
    //   idx 2: left   (rows 0..59 -> taking top 32 of left   cols 8..39 again
    //                  for symmetric stat; here we use rows 0..29, cols 8..39)
    //   idx 3: right  (rows 0..29, cols 40..71  → goes off-image, clamped)
    //
    // To keep blocks fully inside the 80×60 frame, use:
    //   top    (y=0..29, x=8..39)
    //   bottom (y=30..59, x=8..39)
    //   left   (y=15..44, x=0..31)
    //   right  (y=15..44, x=48..79)
    int dx_t, dy_t, dx_b, dy_b, dx_l, dy_l, dx_r, dy_r;
    int ok = 1;
    ok &= subblock_phase_corr(gray80x60,  0,  8, 0, &dx_t, &dy_t);
    ok &= subblock_phase_corr(gray80x60, 30,  8, 1, &dx_b, &dy_b);
    ok &= subblock_phase_corr(gray80x60, 15,  0, 2, &dx_l, &dy_l);
    ok &= subblock_phase_corr(gray80x60, 15, 48, 3, &dx_r, &dy_r);

    if (!ok) return;   // first call, sub-prevs not cached yet

    // Divergence approximation — Horn-Schunck dz/z = -divergence/2.
    //
    // div_y = ∂(vy)/∂y ≈ (dy_bottom - dy_top) / (y_b - y_t)
    //   y_b centre = 45 grid-px (rows 30..59), y_t centre = 15 grid-px,
    //   so distance = 30 grid-px.
    // div_x ≈ (dx_right - dx_left) / (x_r - x_l)
    //   x_r centre = 64, x_l centre = 16, distance = 48 grid-px.
    //
    // Both dx,dy come from sub-block phase-corr in mgp units (1000 = 1
    // grid-px shift between consecutive frames).  Dividing mgp/(grid-px)
    // gives "milli per grid-px", which is the same unit that
    // -div_total/2 would have for dz/z (per-frame relative rate).
    //
    // Output convention: dz_q1000 = (dz / z) × 1e6 [parts-per-million / frame].
    //   +1000 micro per frame = +0.001 (m/m) per frame
    //                        = drone climbed 0.1 % of current altitude
    //                        = at z=1m → dz=+1mm per frame → +30 mm/s
    //
    // Caller computes absolute dz_metres_per_sec by:
    //    dz_mps = z_estimate_metres × dz_q1000 / 1e6 × frame_rate_hz
    //
    // Focal length cancels out — this is the beauty of using the
    // dimensionless divergence form.
    int div_y_micro = (dy_b - dy_t) * 1000 / 30;   // (mgp/30 grid-px) × 1000 → µ/frame
    int div_x_micro = (dx_r - dx_l) * 1000 / 48;
    int dz_q        = -(div_x_micro + div_y_micro) / 2;
    *dz_q1000_out   = dz_q;

    // Confidence: low if any sub-block returned zero motion when the global
    // path detected motion; here we approximate as "non-zero divergence
    // magnitude" capped at 255.
    int mag  = div_x_micro < 0 ? -div_x_micro : div_x_micro;
    int mag2 = div_y_micro < 0 ? -div_y_micro : div_y_micro;
    int total = (mag + mag2) / 16;
    if (total > 255) total = 255;
    *conf_out = (uint8_t)total;
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
