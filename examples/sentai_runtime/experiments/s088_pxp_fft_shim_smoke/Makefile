# Smoke tests for the SIM-side PXP and FFT shims.
#
# Both tests are standalone — they pull in just the shim C file and exercise
# its public API.  Pass criteria: each test prints PASS for every subtest
# and finishes with "ALL TESTS PASSED."
#
# Prerequisite: libfftw3-dev installed (sudo apt install libfftw3-dev).

CFLAGS  := -O2 -Wall -Wextra -DSENTAI_PLATFORM_SIM -I../..
LDFLAGS_FFT := -lfftw3f -lm
LDFLAGS_PXP := -lm

all: test_pxp_shim test_fft_shim

test_pxp_shim: test_pxp_shim.c ../../sentai_pxp_shim_sim.c ../../sentai_pxp_shim.h
	$(CC) $(CFLAGS) $< ../../sentai_pxp_shim_sim.c -o $@ $(LDFLAGS_PXP)

test_fft_shim: test_fft_shim.c ../../sentai_fft_shim_sim.c ../../sentai_fft_shim.h
	$(CC) $(CFLAGS) $< ../../sentai_fft_shim_sim.c -o $@ $(LDFLAGS_FFT)

run: all
	./test_pxp_shim
	./test_fft_shim

clean:
	rm -f test_pxp_shim test_fft_shim

.PHONY: all run clean
