# s088 — SIM PXP + FFT shim smoke tests

Goal: prove that the SIM-side replacements for the RT1176 PXP DMA and the
CMSIS-DSP `arm_cfft_f32` produce numerically equivalent output to the
hardware paths, so `flow_phase_corr.cc` can be linked into `sentai_sim`
unchanged.

## Prerequisites

```
sudo apt install -y libfftw3-dev
```

## Run

```
cd examples/sentai_runtime/experiments/s088_pxp_fft_shim_smoke
make run
```

## Pass criteria

Both binaries finish with `ALL ... TESTS PASSED.` and no `FAIL` lines.

## What each test covers

### `test_pxp_shim`

| Subtest             | Property checked                              |
|---------------------|-----------------------------------------------|
| `uniform_grey`      | DC preserved, no rounding bias                |
| `solid_red`         | Channel ordering (XRGB src → RGB888P dst)     |
| `vertical_gradient` | Area-average semantic vs analytic mean        |
| `BENCH`             | Per-frame timing (must be << 33 ms @ 30 fps)  |

### `test_fft_shim`

| Subtest                   | Property checked                                  |
|---------------------------|---------------------------------------------------|
| `delta_forward`           | δ → flat magnitude (forward FFT correctness)      |
| `delta_at_k_forward`      | δ at index k → twiddle phasor                     |
| `round_trip`              | IFFT(FFT(x)) == x with CMSIS scaling convention   |
| `vs_brute_force_DFT`      | Bit-near match vs O(N²) reference DFT             |

## Why this matters for Phase 4

`flow_phase_corr.cc` runs the same FFT-based phase-correlation algorithm on
ARM and SIM.  The SIM build replaces:

- **PXP DMA** → `sentai_pxp_shim_sim.c` (pure-C area-average resize)
- **CMSIS `arm_cfft_f32`** → `sentai_fft_shim_sim.c` (FFTW3 wrapper)

If these tests pass, any divergence between firmware (board) and SIM flow
output will be due to upstream input differences (Gazebo image vs OV5640
capture), not numerical drift in the inner kernels.
