# s234 iter3 - source audit for contested TPU input timing

**Date**: 2026-06-24

## Question

Reviewer 5 notes that a 512x512x3 uint8 tensor is 786432 bytes, so a full
USB 2.0 high-speed transfer cannot complete in the reported `input_ms`
values around 4.4-4.9 ms. This audit checks whether the source code explains
the discrepancy, especially whether repeated identical tensors are cached.

## Findings

### 1. The model input is not smaller

The re-run in `iter2_fs_restore_and_run/run_t_timing.log` reports:

- `Input: uint8[1,512,512,3] (786432 bytes)`
- `bytes/invoke: input=811008B ins=371664B output=10752B`

So the reviewer concern cannot be resolved by claiming a smaller tensor.

### 2. There is no input-tensor cache in the visible source path

`libs/tpu/edgetpu_executable.cc` handles
`Description_BASE_ADDRESS_INPUT_ACTIVATION` by calling:

```c++
tpu_driver.SendInputs(
    input_tensor->data.uint8 + dma_hint->offset_in_bytes(),
    dma_hint->size_in_bytes())
```

Then it increments:

```c++
g_sentai_tpu_cyc_input += (tpu_cyc() - t0);
g_sentai_tpu_n_input   += 1;
g_sentai_tpu_by_input  += dma_hint->size_in_bytes();
```

The descriptor cache in the same file applies only to parameter and
instruction hints. A cache hit skips `BASE_ADDRESS_PARAMETER` and
`InstructionHint` paths, not `BASE_ADDRESS_INPUT_ACTIVATION`.

### 3. The known caches do not explain `input_ms`

There are two relevant cache-like mechanisms:

- `EdgeTpuManager` parameter caching: skips/reduces model parameter upload
  for a matching token. This explains tiny `params_ms`, not `input_ms`.
- `g_sentai_tpu_desc_cache_enabled`: optional descriptor/instruction cache.
  It is default OFF, and the experiment notes require it OFF for yolo_1.
  Even when enabled, it skips parameters/instructions, not input activations.

### 4. `SendInputs` still submits bytes to USB

`libs/tpu/edgetpu_driver.cc` increments:

```c++
g_sentai_tpu_send_inputs_calls++;
g_sentai_tpu_send_inputs_bytes += length;
```

Then it routes through one of:

- staged path, if `g_sentai_tpu_zero_copy_input == 0`
- pipelined async input path, if `g_sentai_tpu_async_input_enabled != 0`
- normal `SendData(DescriptorTag::kInputActivations, data, length)`

The stable experiment config says:

- `diag.tpu_zero_copy = 1`
- `diag.tpu_async_input = 0`
- `diag.tpu_desc_cache = 0`
- `diag.tpu_chunk_size = 36864`

So the expected path is normal zero-copy `SendData`, split into 36 KB chunks.

### 5. The EHCI callback is intended to mean transfer completion

`USB_HostSend` in the NXP host stack cleans D-cache and submits the transfer
to the controller. The EdgeTPU wrapper waits on a semaphore that is given by
the USB callback.

The EHCI host stack calls the callback when the qTD tail has IOC set and
`ACTIVE=0`, then computes `transferSofar`. This is not merely a submit-time
callback in the source.

### 6. The measured value is still physically inconsistent as full bus time

For the re-run:

- bytes: `811008`
- measured input stage avg: `4.924 ms`
- implied rate: about `164.7 MB/s`, or `1317.6 Mbps`

This is about `2.75x` the USB 2.0 high-speed raw signaling rate of
`480 Mbps`, before protocol overhead. Therefore `input_ms` must not be
published as "full USB bus wall-clock transfer time" unless we add a lower
level validation that proves the measurement semantics.

## Current Interpretation

The source audit rules out a simple "same tensor is cached" explanation in
the visible SentAI EdgeTPU path. The best current interpretation is:

`input_ms` is a reproduced board-side DWT measurement around the SentAI/NXP
driver `SendInputs` path, but its semantics are not yet proven to be physical
USB bus occupancy. It may be affected by firmware/build differences,
measurement placement, EHCI stack behavior, stale semaphores/callbacks, or
another low-level timing artifact.

For the paper, the safe wording is to call these values "driver-stage DWT
measurements" rather than "USB 2.0 transfer time" until verified.

## Next Check

Instrument a new firmware build with per-URB timestamps:

- before `USB_HostSend`
- after `USB_HostSend` returns
- inside the EdgeTPU wrapper callback
- inside EHCI callback processing, with `transferLength` and `transferSofar`

Then compare the per-URB sum with an external USB analyzer or with a strict
USB 2.0 lower-bound sanity check. The test should also print current
`sentai.diag.tpu_call_stats()`, `sentai.diag.async_stats()`,
`sentai.diag.tpu_chunk_size()`, `sentai.diag.tpu_zero_copy()`,
`sentai.diag.tpu_async_input()`, and `sentai.diag.tpu_desc_cache()`.
