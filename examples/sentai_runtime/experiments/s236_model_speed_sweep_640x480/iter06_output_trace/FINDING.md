# s236 iter06 - c2f_thick root cause (TPU trace, build 1548 single-EP + trace binding)

Added `sentai.tpu.trace([on])` binding (libs/tpu/edgetpu_executable.cc setter +
bindings/modsentai_tpu.c) to print every DMA hint + bulk-IN chunk per invoke.

## Definitive root cause: multi-pass parameter-caching executable; 2nd-pass output never produced

c2f_thick loads as a PARAMETER-CACHING executable (`parameter_caching_exe=present`,
the only model in the sweep that does, because it is the largest/widest). Its
executable runs in TWO compute passes with an output read BETWEEN them. Trace of
one invoke (camera tensor input):

```
N v1=0 ...            instruction bitstream 0
P bytes=782336        parameters (cached on-chip)
N v1=0 v2=261984      more instructions (pass 0)
I ... bytes=460800    input image part 1
I ... bytes=466560    input image part 2
O StatefulPartitionedCall:0 bytes=38400   bulkin D 36864 + D 1536 = 38400  OK
N v1=1 v2=17984       <-- SECOND instruction bitstream (compute pass 1)
O StatefulPartitionedCall:1 bytes=9600    bulkin S 9600 -> T (TIMEOUT)     FAIL
E:0B63:0  GetOutputs failed
```

- Pass 0 output (38400 B) reads back fine.
- The 2nd instruction bitstream (`N v1=1`, 17984 B) uploads fine (bulk-OUT OK).
- The 2nd-pass output (9600 B) NEVER arrives. With `urb_timeout_ms=3000` the
  invoke waits the full 3.15 s and still fails -> the TPU does not produce the
  pass-1 output at all. NOT a timeout-tuning issue.

## Why only c2f_thick

It is the widest arch (1.25 MB peak activation, 866 KiB cached params) -> the
EdgeTPU compiler splits its compute into a multi-pass parameter-caching
executable. The other 7 models are single-pass (one set of output reads, no
intermediate instruction bitstream) so they never exercise this path.

## Conclusion

The board's single-EP libedgetpu port does not correctly drive the SECOND
compute pass of a multi-pass / parameter-caching executable: after reading
pass-0 outputs and sending the pass-1 instruction bitstream, the TPU never
produces pass-1 outputs. The same model runs on the Coral USB stick (full
libedgetpu, USB3, multi-EP). This is a board firmware / libedgetpu-port
limitation in multi-pass execution, NOT: on-chip memory (chip is identical to
the stick), NOT transport size, NOT a timeout, NOT a rejected op (all ops map).

Fix options: (a) deep-fix the multi-pass execution in the single-EP port
(likely a missing inter-pass event/doorbell handshake) - substantial; (b)
recompile c2f_thick so the compiler emits a single-pass (non-parameter-caching)
executable, if achievable; (c) accept and use one of the 7 single-pass champions.
