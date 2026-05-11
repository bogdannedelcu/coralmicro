// sentai_fft_shim_sim.c — FFTW3-backed CMSIS arm_cfft_f32 shim for SIM.
//
// Built only when SENTAI_PLATFORM_SIM is defined.  Links against fftw3f
// (single-precision FFTW), already a Debian/Ubuntu apt package.
//
// Why FFTW3 over a hand-rolled radix-2: realtime requirement.  We compute
// 128 row-FFTs per frame at 30 fps = 3840 length-64 transforms/second.
// FFTW3's FFTW_ESTIMATE plans are within a few % of FFTW_MEASURE for
// such small sizes and need no tuning step at boot.
//
// Threading: FFTW3 is NOT thread-safe at plan creation (fftwf_plan_*).
// flow_phase_corr.cc is single-threaded (one task), and the lazy plan
// build runs once on the first call.  If we ever multi-thread, wrap
// ensure_plans() in a mutex.

#include "sentai_fft_shim.h"

#include <fftw3.h>
#include <stdlib.h>

sentai_cfft_instance_f32 sentai_cfft_sR_f32_len64 = {
    .N        = 64,
    .plan_fwd = NULL,
    .plan_inv = NULL,
};

// Length-32 instance — used by the dz divergence sub-block path in
// flow_phase_corr.cc (32×32 sub-blocks).  Same lazy-init pattern.
sentai_cfft_instance_f32 sentai_cfft_sR_f32_len32 = {
    .N        = 32,
    .plan_fwd = NULL,
    .plan_inv = NULL,
};

static void ensure_plans(sentai_cfft_instance_f32* inst) {
    if (inst->plan_fwd) return;

    // Allocate a 16-byte aligned scratch buffer for plan creation.  Both
    // plans share the same in-place layout.  FFTW_UNALIGNED tells the
    // planner to emit code that works on any alignment — required because
    // the caller's data buffers (static arrays in flow_phase_corr.cc) are
    // not guaranteed 16-byte aligned.
    fftwf_complex* tmp = fftwf_alloc_complex((size_t)inst->N);
    if (!tmp) return;  // best-effort: leave plans NULL, sentai_cfft_f32 will no-op

    inst->plan_fwd = fftwf_plan_dft_1d(inst->N, tmp, tmp,
                                       FFTW_FORWARD,
                                       FFTW_ESTIMATE | FFTW_UNALIGNED);
    inst->plan_inv = fftwf_plan_dft_1d(inst->N, tmp, tmp,
                                       FFTW_BACKWARD,
                                       FFTW_ESTIMATE | FFTW_UNALIGNED);
    fftwf_free(tmp);
}

void sentai_cfft_f32(const sentai_cfft_instance_f32* inst_const,
                      float* data,
                      uint8_t inverse,
                      uint8_t do_bit_reverse) {
    (void)do_bit_reverse;  // FFTW always produces natural-order output

    sentai_cfft_instance_f32* inst = (sentai_cfft_instance_f32*)inst_const;
    if (!inst || !data) return;

    ensure_plans(inst);
    if (!inst->plan_fwd || !inst->plan_inv) return;

    fftwf_plan plan = inverse ? (fftwf_plan)inst->plan_inv
                              : (fftwf_plan)inst->plan_fwd;

    // Execute in-place on the caller's buffer.  FFTW3 "guru" execute_dft
    // accepts in/out pointers different from the planning buffer as long
    // as the plan was made with FFTW_UNALIGNED (or both buffers share the
    // planner's alignment).  Cast is safe: fftwf_complex is layout-
    // compatible with float[2], same as CMSIS arm_cfft_f32.
    fftwf_execute_dft(plan, (fftwf_complex*)data, (fftwf_complex*)data);

    if (inverse) {
        // CMSIS arm_cfft_f32 inverse scales by 1/N; FFTW3 does not.
        const int N = inst->N;
        const float scale = 1.0f / (float)N;
        for (int i = 0; i < N * 2; ++i) {
            data[i] *= scale;
        }
    }
}
