// sentai_fft_shim.h — single-precision complex FFT entry point shared by
// flow_phase_corr.cc on both ARM and SIM.
//
// ARM target: maps directly onto CMSIS-DSP's arm_cfft_f32 + the prebuilt
//             length-64 instance arm_cfft_sR_f32_len64.  Zero overhead;
//             the macros expand to the original symbols.
//
// SIM target: maps onto a thin FFTW3-backed wrapper (sentai_fft_shim_sim.c).
//             Allocates plans lazily on first use.
//
// API contract (matches CMSIS-DSP arm_cfft_f32 exactly):
//   - data is interleaved real/imag float pairs, length 2*N (N power of two).
//   - inverse=0 => forward DFT, UNSCALED.
//   - inverse=1 => inverse DFT, SCALED by 1/N (CMSIS convention).
//                  FFTW3 IDFT is unscaled by default — the SIM wrapper
//                  applies the 1/N divide so callers see identical output
//                  on both platforms.
//   - do_bit_reverse must be 1 (natural-order output); the SIM wrapper
//     ignores it because FFTW3 always produces natural order.

#ifndef SENTAI_FFT_SHIM_H_
#define SENTAI_FFT_SHIM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef SENTAI_PLATFORM_SIM

// Host side.  Opaque instance carries the FFT length plus FFTW plans.
typedef struct {
    int   N;
    void* plan_fwd;   // fftwf_plan, lazily allocated
    void* plan_inv;
} sentai_cfft_instance_f32;

// Pre-built length-64 instance — analogue of arm_cfft_sR_f32_len64.
extern sentai_cfft_instance_f32 sentai_cfft_sR_f32_len64;

void sentai_cfft_f32(const sentai_cfft_instance_f32* inst,
                      float* data,
                      uint8_t inverse,
                      uint8_t do_bit_reverse);

#else  // ARM build — defer to CMSIS-DSP.

#include "arm_math.h"
#include "arm_const_structs.h"

typedef arm_cfft_instance_f32 sentai_cfft_instance_f32;
#define sentai_cfft_sR_f32_len64 arm_cfft_sR_f32_len64
#define sentai_cfft_f32(inst, data, inv, br) \
    arm_cfft_f32((inst), (data), (inv), (br))

#endif  // SENTAI_PLATFORM_SIM

#ifdef __cplusplus
}
#endif

#endif  // SENTAI_FFT_SHIM_H_
