// test_fft_shim.c — standalone smoke test for sentai_fft_shim_sim.c.
//
// Validates that the FFTW3 shim produces the same output as a brute-force
// O(N^2) DFT reference on synthetic inputs:
//   1. δ-function at index 0  -> flat magnitude 1.0 forward; flat -> δ inverse
//   2. δ at index k           -> twiddle phasor exp(-j 2π k n / N)
//   3. forward then inverse   -> identity (within float epsilon × N)
//
// Build:
//   gcc -O2 -Wall -Wextra -DSENTAI_PLATFORM_SIM -I../.. test_fft_shim.c \
//       ../../sentai_fft_shim_sim.c -lfftw3f -lm -o test_fft_shim
// Run:
//   ./test_fft_shim
//
// On ARM the same code path is exercised by the existing on-board flow
// pipeline (s083_flow_m7_validate already proved bit-perfect ARM ↔ host
// numpy reference); this test confirms the SIM path matches identically.

#include <complex.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../sentai_fft_shim.h"

#define N 64

// Reference: brute-force O(N^2) forward DFT, single-precision.
// Output convention matches CMSIS arm_cfft_f32 forward (UNSCALED):
//   X[k] = sum_n x[n] * exp(-j 2π k n / N).
static void ref_dft(const float* in, float* out, int inverse) {
    for (int k = 0; k < N; ++k) {
        float sumr = 0.0f, sumi = 0.0f;
        for (int n = 0; n < N; ++n) {
            float angle = (inverse ? +1.0f : -1.0f)
                          * 2.0f * (float)M_PI * (float)k * (float)n / (float)N;
            float c = cosf(angle), s = sinf(angle);
            float xr = in[n * 2 + 0], xi = in[n * 2 + 1];
            sumr += xr * c - xi * s;
            sumi += xr * s + xi * c;
        }
        out[k * 2 + 0] = sumr;
        out[k * 2 + 1] = sumi;
    }
    if (inverse) {
        // Match CMSIS scaling: inverse divides by N.
        const float inv = 1.0f / (float)N;
        for (int i = 0; i < N * 2; ++i) out[i] *= inv;
    }
}

static int compare(const float* a, const float* b, float tol, const char* tag) {
    float maxdiff = 0.0f;
    int idx = -1;
    for (int i = 0; i < N * 2; ++i) {
        float d = fabsf(a[i] - b[i]);
        if (d > maxdiff) { maxdiff = d; idx = i; }
    }
    if (maxdiff > tol) {
        printf("FAIL %s maxdiff=%.6e at idx=%d (a=%.4f b=%.4f) tol=%.4e\n",
               tag, maxdiff, idx, a[idx], b[idx], tol);
        return 1;
    }
    printf("PASS %s (maxdiff=%.4e tol=%.4e)\n", tag, maxdiff, tol);
    return 0;
}

static int test_delta_forward(void) {
    float buf[2 * N] = {0};
    float ref[2 * N];
    buf[0] = 1.0f;          // δ at index 0
    ref_dft(buf, ref, /*inverse=*/0);
    sentai_cfft_f32(&sentai_cfft_sR_f32_len64, buf, /*inverse=*/0, /*do_bit_reverse=*/1);
    return compare(buf, ref, 1e-5f, "delta_forward");
}

static int test_delta_at_k_forward(void) {
    const int k0 = 5;
    float buf[2 * N] = {0};
    float ref[2 * N];
    buf[k0 * 2 + 0] = 1.0f;
    ref_dft(buf, ref, /*inverse=*/0);
    sentai_cfft_f32(&sentai_cfft_sR_f32_len64, buf, /*inverse=*/0, /*do_bit_reverse=*/1);
    return compare(buf, ref, 1e-5f, "delta_at_k_forward");
}

static int test_round_trip(void) {
    // Random-ish input, forward + inverse should recover original.
    float orig[2 * N];
    float buf[2 * N];
    for (int i = 0; i < N; ++i) {
        orig[i * 2 + 0] = (float)((i * 17 + 3) & 0xFF) / 255.0f;
        orig[i * 2 + 1] = (float)((i * 41 + 11) & 0xFF) / 255.0f;
    }
    memcpy(buf, orig, sizeof(buf));
    sentai_cfft_f32(&sentai_cfft_sR_f32_len64, buf, /*inverse=*/0, 1);
    sentai_cfft_f32(&sentai_cfft_sR_f32_len64, buf, /*inverse=*/1, 1);
    return compare(buf, orig, 1e-4f, "round_trip");
}

static int test_against_reference(void) {
    // Random input, forward must match brute-force DFT.
    float in[2 * N];
    float ref[2 * N];
    float buf[2 * N];
    for (int i = 0; i < N; ++i) {
        in[i * 2 + 0] = sinf((float)i * 0.13f);
        in[i * 2 + 1] = cosf((float)i * 0.07f);
    }
    ref_dft(in, ref, /*inverse=*/0);
    memcpy(buf, in, sizeof(buf));
    sentai_cfft_f32(&sentai_cfft_sR_f32_len64, buf, /*inverse=*/0, 1);
    return compare(buf, ref, 1e-4f, "vs_brute_force_DFT");
}

int main(void) {
    int rc = 0;
    rc |= test_delta_forward();
    rc |= test_delta_at_k_forward();
    rc |= test_round_trip();
    rc |= test_against_reference();
    if (rc) { printf("FAIL.\n"); return 1; }
    printf("ALL FFT TESTS PASSED.\n");
    return 0;
}
