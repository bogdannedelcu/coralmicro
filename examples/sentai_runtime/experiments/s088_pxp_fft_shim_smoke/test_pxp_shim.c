// test_pxp_shim.c — standalone smoke test for sentai_pxp_shim_sim.c.
//
// Build:
//   gcc -O2 -Wall -Wextra -I../.. test_pxp_shim.c \
//       ../../sentai_pxp_shim_sim.c -o test_pxp_shim
// Run:
//   ./test_pxp_shim          (expect: ALL TESTS PASSED)
//
// Validates:
//   1. 8x integer downscale (640x480 -> 80x60) — exact area-average path
//   2. uniform grey input -> uniform grey output (no DC drift)
//   3. solid red XRGB -> solid red RGB (channel order preserved)
//   4. byte-order: src [B,G,R,X] -> dst [R,G,B]
//   5. timing on x86: prints elapsed ms for 100 frames

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "../../sentai_pxp_shim.h"

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static int test_uniform_grey(void) {
    const int sw = 640, sh = 480, dw = 80, dh = 60;
    uint8_t* src = malloc((size_t)sw * sh * 4);
    uint8_t* dst = malloc((size_t)dw * dh * 3);
    // XRGB8888 in memory is [B, G, R, X].
    for (int i = 0; i < sw * sh; ++i) {
        src[i * 4 + 0] = 0x80;  // B
        src[i * 4 + 1] = 0x80;  // G
        src[i * 4 + 2] = 0x80;  // R
        src[i * 4 + 3] = 0xFF;  // X
    }
    int rc = sentai_pxp_scale(src, sw, sh, dst, dw, dh);
    if (rc != 0) { printf("FAIL uniform_grey rc=%d\n", rc); return 1; }
    for (int i = 0; i < dw * dh; ++i) {
        if (dst[i * 3 + 0] != 0x80 || dst[i * 3 + 1] != 0x80 || dst[i * 3 + 2] != 0x80) {
            printf("FAIL uniform_grey px[%d]=(%u,%u,%u)\n",
                   i, dst[i*3+0], dst[i*3+1], dst[i*3+2]);
            free(src); free(dst); return 1;
        }
    }
    free(src); free(dst);
    printf("PASS uniform_grey\n");
    return 0;
}

static int test_solid_red(void) {
    const int sw = 640, sh = 480, dw = 80, dh = 60;
    uint8_t* src = malloc((size_t)sw * sh * 4);
    uint8_t* dst = malloc((size_t)dw * dh * 3);
    for (int i = 0; i < sw * sh; ++i) {
        src[i * 4 + 0] = 0x00;  // B
        src[i * 4 + 1] = 0x00;  // G
        src[i * 4 + 2] = 0xFF;  // R
        src[i * 4 + 3] = 0xFF;  // X
    }
    int rc = sentai_pxp_scale(src, sw, sh, dst, dw, dh);
    if (rc != 0) { printf("FAIL solid_red rc=%d\n", rc); return 1; }
    // RGB888P output: byte order [R, G, B].
    for (int i = 0; i < dw * dh; ++i) {
        if (dst[i * 3 + 0] != 0xFF || dst[i * 3 + 1] != 0x00 || dst[i * 3 + 2] != 0x00) {
            printf("FAIL solid_red px[%d]=(%u,%u,%u)\n",
                   i, dst[i*3+0], dst[i*3+1], dst[i*3+2]);
            free(src); free(dst); return 1;
        }
    }
    free(src); free(dst);
    printf("PASS solid_red (R-channel preserved, byte order [R,G,B])\n");
    return 0;
}

// Vertical gradient: row y has B=y%256.  After 8x downscale the dst row Y
// covers src rows 8Y..8Y+7, so dst B = avg(8Y..8Y+7) = 8Y+3.5 -> 8Y+4 with
// round-to-nearest.
static int test_vertical_gradient(void) {
    const int sw = 640, sh = 480, dw = 80, dh = 60;
    uint8_t* src = malloc((size_t)sw * sh * 4);
    uint8_t* dst = malloc((size_t)dw * dh * 3);
    for (int y = 0; y < sh; ++y) {
        for (int x = 0; x < sw; ++x) {
            uint8_t v = (uint8_t)(y & 0xFF);
            src[(y*sw + x)*4 + 0] = v;       // B
            src[(y*sw + x)*4 + 1] = 0;
            src[(y*sw + x)*4 + 2] = 0;
            src[(y*sw + x)*4 + 3] = 0xFF;
        }
    }
    int rc = sentai_pxp_scale(src, sw, sh, dst, dw, dh);
    if (rc != 0) { printf("FAIL gradient rc=%d\n", rc); return 1; }
    for (int Y = 0; Y < dh; ++Y) {
        // y0..y1 = 8Y..8Y+8 in source rows; src[y] B = y & 0xFF (wraps).
        int sum = 0;
        for (int y = 8 * Y; y < 8 * Y + 8; ++y) sum += (y & 0xFF);
        int expected = (sum + 4) / 8;  // round-to-nearest
        for (int X = 0; X < dw; ++X) {
            int got = dst[(Y * dw + X) * 3 + 2];  // B is at byte idx 2 in [R,G,B]
            if (abs(got - expected) > 1) {
                printf("FAIL gradient Y=%d X=%d got_B=%d expect=%d\n",
                       Y, X, got, expected);
                free(src); free(dst); return 1;
            }
        }
    }
    free(src); free(dst);
    printf("PASS vertical_gradient (area-average semantic correct)\n");
    return 0;
}

static void bench(int frames) {
    const int sw = 640, sh = 480, dw = 80, dh = 60;
    uint8_t* src = malloc((size_t)sw * sh * 4);
    uint8_t* dst = malloc((size_t)dw * dh * 3);
    // pseudo-random-ish content
    for (int i = 0; i < sw * sh * 4; ++i) src[i] = (uint8_t)(i * 31u + 7u);

    double t0 = now_ms();
    for (int n = 0; n < frames; ++n) {
        sentai_pxp_scale(src, sw, sh, dst, dw, dh);
    }
    double t1 = now_ms();
    double per = (t1 - t0) / frames;
    printf("BENCH 640x480 -> 80x60 area-avg: %.3f ms/frame (%d frames in %.1f ms)\n",
           per, frames, t1 - t0);
    free(src); free(dst);
}

int main(void) {
    int rc = 0;
    rc |= test_uniform_grey();
    rc |= test_solid_red();
    rc |= test_vertical_gradient();
    if (rc) { printf("FAIL.\n"); return 1; }
    bench(100);
    printf("ALL TESTS PASSED.\n");
    return 0;
}
