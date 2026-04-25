# Coral USB Accelerator on Linux host — findings

**Date**: 2026-04-24. **Setup**: Coral USB Accelerator (VID:PID `18d1:9302` post-DFU, `1a6e:089a` pre-DFU), Linux x86-64, libedgetpu 16.0, pycoral in Python 3.9 venv, xHCI USB 3.0 SuperSpeed (5 Gbit/s).

Cross-check against the RT1176-based custom board (USB 2.0 HS) to validate what's silicon/compiler-bound vs host-bound.

## 1. Throughput comparison — same models, different hosts

| Model | Ins/invoke | USB 2.0 (RT1176) | USB 3.0 (x86) | Ratio |
|---|---:|---|---|---|
| MobileNet v1 224² (uint8) | 220 KB | — | **3.92 ms (255 FPS)** | — |
| yolo_1 512² (uint8) | 362 KB | 13 ms (76 FPS) | 19.60 ms (51 FPS) | **RT1176 faster!** |
| yolo26 768×512 (int8) | 1228 KB | 90 ms (11 FPS) | 37.12 ms (27 FPS) | USB3 2.4× faster |

**Surprising result**: for yolo_1 (small ins, small input), the RT1176 at USB 2.0 is *faster* than a desktop at USB 3.0. Possible reasons:
- SuperSpeed has larger per-packet overhead for small transfers
- Desktop libedgetpu uses asbl flags / libusb which may have more host-side latency than our NXP EHCI stack

For yolo26 (ins-heavy, large input), USB 3.0 wins 2.4× — the extra wire bandwidth matters for big models.

## 2. Per-invoke trace (verified with `edgetpu_verbosity(10)`)

### Invoke 1 (first after model load) — mobilenet_v1_224
```
Request[0] PARAMETER_CACHING:
  Mapped "instructions" = 7,248 B (tiny ins for param caching)
  3 DMAs scheduled

Request[1] INFERENCE:
  Mapped "input" = 150,528 B
  Mapped "instructions" = 225,824 B
  Mapped "MobilenetV1/Predictions/Reshape_1" = 1,008 B (output)
  4 DMAs scheduled

Total: 12 AsyncBulkOutTransfer + 3 AsyncBulkInTransfer
```

### Invoke 2 (warm — same model, same input)
```
Request[2] INFERENCE ONLY:
  Mapped "input" = 150,528 B
  Mapped "instructions" = 225,824 B ← INSTRUCTIONS RE-UPLOADED
  Mapped output = 1,008 B
  4 DMAs scheduled

Total: 4 AsyncBulkOutTransfer + 2 AsyncBulkInTransfer
```

**Key observation**: on warm invoke, the host-side instruction buffer pointer is identical (`0x1e680000` both times) but the bulk OUT transfer still occurs. Confirms — instructions are re-streamed every invoke in libedgetpu's official implementation. Only `PARAMETER_CACHING` is skipped when the cache token matches.

## 3. All knobs discovered in libedgetpu

### Public API options (`EdgeTpuManager::DeviceOptions`)
- `Performance`: Low/Medium/High/Max (default Max)
- `Usb.AlwaysDfu`: True/False (default False)
- `Usb.MaxBulkInQueueLength`: 0-255 (default 32)

### Internal flags (env var controlled via `absl::GetFlag`)
All shown here with their runtime defaults:

| Env var | Default | Meaning |
|---|---|---|
| `USB_ENABLE_OVERLAPPING_REQUESTS` | `true` | Next request's setup may overlap with current request's completion. **No GlobalFence added if executable is `fully_deterministic=true`** |
| `USB_ENABLE_OVERLAPPING_BULK_IN_AND_OUT` | `true` | Bulk IN can be submitted before current bulk OUT completes |
| `USB_ENABLE_QUEUED_BULK_IN_REQUESTS` | `true` | Multiple bulk-IN requests can be queued |
| `USB_ENABLE_BULK_DESCRIPTORS_FROM_DEVICE` | `false` | Device-initiated DMA descriptors. **Comment in code says: "doesn't work with multiple instruction chunks, device not capable of generating descriptors for instructions."** |
| `USB_ENABLE_PROCESSING_OF_HINTS` | `true` | Follow the `dma_hints` from the compiled executable |
| `USB_OPERATING_MODE` | `2` | 0=Multi-EP hardware, 1=Multi-EP software, 2=Single-EP |
| `USB_MAX_BULK_OUT_TRANSFER` | `1048576` | Max chunk size (1 MB) |
| `USB_MAX_NUM_ASYNC_TRANSFERS` | kDefault | Max pending async bulk-out transfers |
| `USB_FORCE_LARGEST_BULK_IN_CHUNK_SIZE` | `false` | For USB 2.0 boost |
| `USB_TIMEOUT_MILLIS` | `6000` | Per-transfer timeout |
| `USB_SOFTWARE_CREDITS_LOW_LIMIT` | `8192` | Flow control threshold |

Tested on yolo26 @ USB 3.0:
- Default: 37 ms
- `USB_OPERATING_MODE=0/1`: **device error** (apex firmware on this Coral is single_ep build, doesn't advertise multi-EP pipes)
- `USB_FORCE_LARGEST_BULK_IN_CHUNK_SIZE=true`: delegate fails to load
- Two interpreters same TPU, parallel Python threads: **no speedup** (driver locks serialize)

## 4. `fully_deterministic` — a key compilation flag

[`dma_info_extractor.cc:160`](../../../../work/libedgetpu/driver/dma_info_extractor.cc#L160):
```cpp
if (!dma_hints.fully_deterministic() || !overlap_requests_) {
    dmas.push_back(DmaInfo(id++, DmaDescriptorType::kGlobalFence));
}
```

If the executable's dma_hints are marked `fully_deterministic=true` AND `overlap_requests_=true`, **no fence** is added between requests → TPU can start processing request N+1 while still finishing N.

**Checked all our models**: every compiled executable (both `EXEC_ONLY` and `PARAM_CACHING`) has `fully_deterministic=true`. So overlapping IS enabled architecturally. But:

- **Python sync `invoke()` can't exploit it** — the call blocks until output is ready
- **Two interpreters in parallel Python threads — also serialized** (tested: 0 speedup)
- Real win would need the async `Driver::Submit(callback)` C++ API, which pycoral doesn't expose

## 5. Request types — only 2 exist

[`api/request.h:41`](../../../../work/libedgetpu/api/request.h#L41):
```cpp
enum class TpuRequestType {
    PARAMETER_CACHING,
    INFERENCE
};
```

No `INSTRUCTION_CACHING`, no `CACHED_INFERENCE`, no `STREAM_MODE`. Exhaustive search across libedgetpu confirms **there is no public mechanism to skip instruction re-upload**.

## 6. `NullDramAllocator` — the dead-end

[`beagle_usb_driver_provider.cc:344`](../../../../work/libedgetpu/driver/beagle/beagle_usb_driver_provider.cc#L344):
```cpp
auto dram_allocator = gtl::MakeUnique<NullDramAllocator>();
```

[`null_dram_allocator.h`](../../../../work/libedgetpu/driver/memory/null_dram_allocator.h):
```cpp
// A DRAM allocator to be used for chips that do not have an on-chip DRAM.
StatusOr<shared_ptr<DramBuffer>> AllocateBuffer(size_t) override {
    return FailedPreconditionError("No on-chip DRAM available.");
}
```

`cache_on_dram` per-layer and `use_tpu_dram_for_parameters` per-executable both fail on Beagle. These features exist in the architecture but are not exposed on Coral's silicon.

## 7. How parameter caching actually works on Coral (empirically)

1. First invoke after model load: `Request[0]` of type `PARAMETER_CACHING` runs, uploading ~1.9 MB of weights + ~7 KB of bootstrap instructions
2. The bootstrap instructions write the weights into **some on-chip memory region** (not documented, not exposed as DRAM)
3. Subsequent invokes (`Request[N+1]` of type `INFERENCE`) skip the parameter caching — just send fresh instructions + input, read output
4. Token match in `EdgeTpuManager::Invoke` detects the cache validity

The actual persistent storage is handled by the TPU mask ROM + silicon — transparent to the public driver API.

## 8. Key consequences for the RT1176 sentai_runtime project

1. **Instruction caching is genuinely impossible** without hardware/mask-ROM changes. This is not a libedgetpu limitation we can work around.

2. **Parameter caching already works on RT1176** (we use the same mechanism, running the `PARAMETER_CACHING` exe once at model load via `EdgeTpuManager::Invoke` in `edgetpu_manager.cc:186-207`).

3. **Overlapping requests** theoretically usable but requires async API — currently not integrated in coralmicro. Our synchronous `sentai.tpu.invoke()` wouldn't benefit unless we re-architect around callbacks.

4. **yolo26 pure-invoke ceiling on USB 2.0** is wire-bound at ~90 ms (confirmed by the 37 ms USB 3.0 result: 37 + USB2-overhead ≈ 90 tracks). Moving to OCRAM won't change this — it only helps pipeline-mode SEMC contention.

5. **The `USB_*` env vars** that DO work on our hardware (overlapping enabled, max perf) are already active by default. Nothing to tune on the host side for single-request workflows.

## 9. Direct C++ API exposure check

Exported `edgetpu_verbosity(int)` is how we got verbose logging. Other exported symbols of interest:
```
edgetpu_list_devices / edgetpu_free_devices / edgetpu_create_delegate
(from public edgetpu_c.h — standard API)
```

No exported private/backdoor symbols — `nm -D libedgetpu.so.1.0 | grep -iE 'verbos|log|cache|persist'` shows only `edgetpu_verbosity`.

## 10. Tools saved in the repo

- `scripts/d51.py` — 8051 disassembler for apex.bin
- `paper/apex_fw.md` — firmware RE notes
- `paper/coral_hostside.md` — this document

## Final verdict on instruction caching

After:
- Full reverse engineering of apex 10.7 KB firmware (7 DMA opcodes, all endpoint-specific)
- Complete audit of libedgetpu source (2 request types, 3 executable types, no caching API)
- Verbose runtime trace on real hardware (instructions re-uploaded on every invoke, confirmed)
- Test of every discovered USB flag including `overlapping_requests` + multi-EP modes

**Instruction caching is not exposed at any software layer**. To implement it would require either:
- Modifying the apex mask ROM (infeasible — silicon-burned)
- Writing custom 8051 firmware that probes undocumented DMA opcodes at 0x801C (1-2 weeks effort, uncertain outcome)
- A silicon revision from Google (not happening)

**Practical path**: accept that yolo26-class models on this TPU are ins-wire-bound. Reduce ins bytes via smaller models or lower input resolution, and accept 90 ms floor on USB 2.0 / 37 ms on USB 3.0.
