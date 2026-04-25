# Cale 1 — OCRAM ring buffer for TPU USB transfers

**Status**: **PURE variant SHIPPED 2026-04-24 build #868+** (post earlier failed flat-prestage).  Untested on hardware at time of writing.

**Goal**: free ~700 KB OCRAM while keeping pipeline-mode SEMC-contention fix from V22 (the `.tpu_input` move).  Also unlock yolo26-class models (1.2 MB ins) benefiting from OCRAM routing.

---

## Why a ring buffer is the right shape

1. `BulkOutTransfer` already chunks every USB transfer at 36 KB (`g_sentai_tpu_chunk_size`), for **all** tags: ins, input, params.  Individual SendX calls don't need a full-size OCRAM buffer — they need 36 KB slots.
2. USB EHCI reads from OCRAM (AXBS crossbar) vs SDRAM (SEMC) → routing ANY bulk-OUT through OCRAM avoids SEMC contention with CSI camera DMA.
3. A ring buffer works for **any source size** — yolo_1 (371 KB ins), yolo26 (1228 KB ins), VGA camera input (786 KB), SXGA (1152 KB), future models.  No per-model fit calculation.
4. 2 slots × 36 KB = 72 KB — vs current `.tpu_input[786 KB]`, net OCRAM saving ≈ 700 KB.

## Failed attempt 2026-04-24 — flat pre-stage (what NOT to repeat)

- Linker: added `.tpu_ins_cache[600 KB]` in OCRAM, moved `.tpu_input[786 KB]` to SDRAM (only way to fit 1386 KB into 1016 KB OCRAM).
- At `tpu.load()`: memcpy all `instruction_bitstreams[]` into the OCRAM cache.  `SendInstructions` sends from OCRAM pointer instead of flatbuffer SDRAM pointer.

What happened:
- Pure invoke yolo_1: 13 ms → 12.7 ms (~1 ms gain, marginal).
- **Pipeline yolo_1: catastrophic regression 42 → 0 FPS (100% `-2` fail)**.
  V22's `.tpu_input` in OCRAM is load-bearing: PXP writes processed input directly to OCRAM, avoiding SDRAM contention during invoke.  Moving it undid V22.
- Sub-scale (128 KB ins_cache coexisting with .tpu_input) fits neither yolo_1 (371 KB) nor yolo26 (1228 KB) → useless for our workloads.
- Side bug: `ClearPackageCache()` call caused use-after-free (interpreter's `node->user_data` points to freed packages).
- All changes `git stash drop`-ed.

**Lesson**: cannot add a second OCRAM-resident large buffer — OCRAM is saturated.  Must DISPLACE `.tpu_input` with something universal.

---

## The actual plan

### Memory layout

```
BEFORE (V22 stable):                AFTER (Cale 1 ring):
OCRAM:                              OCRAM:
  .tpu_input  786 KB   ─┐             .tpu_ring   72 KB   (2×36 KB slots)
  .usb_host     13 KB   │             .usb_host   13 KB
  .a71ch / .curl / .mbedtls / .wiced  .a71ch / .curl / .mbedtls / .wiced
  (~200 KB others)      │             (~200 KB others)
                        │             ~700 KB FREE ← reclaimed
  TOTAL: ~1000 KB used  ┘             TOTAL: ~300 KB used
```

### Per-BulkOutTransfer flow

```
BulkOutTransfer(endpoint, src_ptr, total_size):
    # src_ptr may be SDRAM (flatbuffer) or any location.
    for offset in range(0, total_size, 36_KB):
        slot = ring.next_producer_slot()            # wait if both full
        eDMA_ch31_async(src=src_ptr+offset, dst=slot, size=36_KB)
        wait dma_done_sema                           # IRQ-fed
        USB_HostEdgeTpuBulkOutSendAsync(endpoint, slot, 36_KB, iocb=...)
        # next iteration: eDMA fires for slot[k+1] while USB sends slot[k]
    wait final_usb_iocb
```

Producer (eDMA) and consumer (USB) run concurrently, 2-slot ping-pong.

### Coordination

- **eDMA ch31** already in use (`sentai_dma_memcpy` in `detection_task.cc`, ~300 MB/s, 32-byte AXI bursts).  Distinct AXBS master ID from CSI+USB+PXP (per NXP AN12437: eDMA=001b, others share 011b).
- **PXP identity copy** alternative — set scaler 1:1, CSC identity.  Useful if eDMA ch31 is busy or for parallel transfers.
- **Interrupt-based**: eDMA completion IRQ → `xSemaphoreGiveFromISR(producer_done)`; USB IOC → `xSemaphoreGiveFromISR(consumer_done)`.  Classic producer-consumer.
- **Cache maintenance**: OCRAM slot regions are MPU Region 6 WB-cacheable.  Need `DCACHE_CleanInvalidateByRange(slot, 36_KB)` before USB reads the slot (USB_HostSend does this automatically via `DCACHE_CleanByRange`).

### Trade-off vs V22

- V22 (current): PXP writes directly to OCRAM `.tpu_input`.  Zero SDRAM touch for input during invoke.
- Cale 1: PXP writes to SDRAM.  eDMA does SDRAM→OCRAM 22× 36 KB copies for input (~2.6 ms extra per invoke).  SEMC traffic during eDMA reads exists but is spread across small bursts interleaved with USB send.

For pipeline yolo_1, this MIGHT regress 42 FPS → 35-38 FPS due to the extra memcpy.  Mitigation: keep `.tpu_input` OCRAM for **input only**, use ring buffer for **ins only** (since ins for yolo26 is the real goal).  Hybrid:

```
OCRAM:
  .tpu_input  786 KB   ← kept for input (V22 preserved for pipeline stability)
  .tpu_ring   72 KB    ← NEW, used only for ins (SendInstructions path)
  ~150 KB free after other sections
```

Hybrid total: 786 + 72 + ~200 = ~1058 KB → just over 1016 KB!  Must move something else (e.g., `.curl` 208 KB to SDRAM).

---

## Implementation checklist

1. **Linker** (`MIMXRT1176xxxxx_cm7_ram_mp.ld`):
   - Move `.curl` to m_sdram (~208 KB freed in OCRAM).
   - Add `.tpu_ring (NOLOAD)` 72 KB in m_ocram.
   - Keep `.tpu_input` 786 KB in m_ocram (V22 preserved).
   - ASSERT total fits.

2. **Ring buffer infrastructure** (`libs/tpu/edgetpu_driver.cc` — new section):
   ```c
   // Ring slots
   __attribute__((section(".tpu_ring"))) __attribute__((aligned(32)))
   static uint8_t s_ring_slot[2][36 * 1024];
   static SemaphoreHandle_t s_ring_producer_ready[2];  // DMA done
   static SemaphoreHandle_t s_ring_consumer_done[2];   // USB IOC

   // eDMA descriptor + IRQ handler register
   static void ring_init_edma(void);
   static void ring_edma_irq_handler(void);  // feeds producer_ready sema
   static ssize_t ring_sdram_to_ocram_async(void* src, uint8_t* dst_slot, size_t n);
   ```

3. **New transfer function** replacing existing paths:
   ```c
   bool BulkOutTransferRingStaged(uint8_t ep, const uint8_t* src, uint32_t total) {
       int slot = 0;
       uint32_t off = 0;
       while (off < total) {
           uint32_t n = MIN(36*1024, total - off);
           xSemaphoreTake(s_ring_consumer_done[slot], portMAX_DELAY);  // slot free
           // producer: async eDMA
           ring_sdram_to_ocram_async(src + off, s_ring_slot[slot], n);
           xSemaphoreTake(s_ring_producer_ready[slot], portMAX_DELAY);
           // consumer: async USB
           USB_HostEdgeTpuBulkOutSendAsync(usb, ep, s_ring_slot[slot], n,
               ring_usb_iocb, &s_ring_consumer_done[slot]);
           off += n;
           slot ^= 1;
       }
       // drain last slot
       xSemaphoreTake(s_ring_consumer_done[slot^1], portMAX_DELAY);
       return true;
   }
   ```

4. **Wire ring path into `SendInstructions`** (keep zero-copy for SendInputs and SendParameters since input uses `.tpu_input` OCRAM already):
   ```c
   bool TpuDriver::SendInstructions(const uint8_t* data, uint32_t length) const {
       if (g_sentai_tpu_ring_enabled) {
           uint8_t header[8];
           PrepareHeaderInto(DescriptorTag::kInstructions, length, header);
           if (!BulkOutTransfer(kSingleBulkOutEndpoint, header, 8)) return false;
           return BulkOutTransferRingStaged(kSingleBulkOutEndpoint, data, length);
       }
       return SendData(DescriptorTag::kInstructions, data, length);
   }
   ```

5. **MicroPython diag**: `sentai.diag.tpu_ring(bool)` toggle.

## Regression tests required

1. **Pure invoke yolo_1** — baseline 13 ms.  Hybrid should stay ≤15 ms.
2. **Pipeline yolo_1** — baseline 42 FPS, 0 fails.  Must match.
3. **Pipeline yolo26 NEW test** — baseline ??? (never ran pipeline yolo26 because wedges).  Target: stable, maybe 15-20 FPS.
4. **Pure invoke yolo26** — baseline 90 ms.  Hybrid MIGHT reach 80-85 ms if SEMC contention from ins-read was adding measurable time even in pure-invoke (probably not; SEMC contention is pipeline-only issue).

## Files that had today's failed changes (dropped stash)

Reference for files that need touching in ring-buffer redo:
- `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`
- `libs/tpu/edgetpu_driver.cc` / `.h`
- `libs/tpu/edgetpu_executable.cc` / `.h`
- `libs/tpu/edgetpu_manager.cc` / `.h`  — (DO NOT re-add `ClearPackageCache()` — use-after-free)
- `examples/sentai_runtime/sentai_slow_bridge.cc`
- `examples/sentai_runtime/modsentai_diag.c`

The useful diagnostic bindings from the dropped attempt that are worth re-adding:
- `sentai.diag.tpu_csr_read(addr)` / `tpu_csr_write(addr, val)` — raw 64-bit CSR access
- `sentai.diag.tpu_skip_ins(mask)` — hint-position bitmask skip (kept its diagnostic value)
- `TpuDriver::CsrRead64()` / `CsrWrite64()` public wrappers

---

## As-built (PURE variant shipped 2026-04-24, build #868+)

### Memory map (objdump-verified)

```
OCRAM  0x20240000..0x2033E000 (1016 KB)
  .a71ch     / .mbedtls / .wiced / .usb_host           ~200 KB
  .tpu_ring  0x20243400  73728 B   (2x36 KB slots)      73 KB
  ~740 KB free  (was 180 KB before Cale 1)

SDRAM 0x8XXXXXXX
  .tpu_input 0x817EDEC0  786432 B  (was OCRAM in V22)
  .curl (moved from OCRAM; 208 KB freed)
  .lwip, .micropython, .sentai_slow, .libjpeg, .libm, .tensorflow, etc
```

### Code changes

- `MIMXRT1176xxxxx_cm7_ram_mp.ld`: `.curl` OCRAM→SDRAM; `.tpu_input` OCRAM→SDRAM; new `.tpu_ring (NOLOAD)` in OCRAM with ASSERT.
- `libs/tpu/edgetpu_driver.cc`: ring slot storage (`s_ring_slot[2][36 KB]` in `.tpu_ring`), per-slot `StaticSemaphore_t`, eDMA ch30 handle, `InitRing()` one-shot, `RingDmaCopyToSlot()` with DCACHE_CleanByRange on SRC, `RingUsbCallback()` (task-context give), `BulkOutTransferRingStaged()` bounded-loop + drain.  `SendParameters` / `SendInstructions` / `SendInputs` check `g_sentai_tpu_ring_enabled` and route through ring with an 8-byte header pre-fire via the legacy direct path.
- `libs/tpu/edgetpu_driver.cc` defaults `g_sentai_tpu_ring_enabled = 1` — MANDATORY with PURE layout because zero-copy USB reads of SDRAM `.tpu_input` would now contend on SEMC with CSI (V13-style failure).  Operators can set 0 via `sentai.diag.tpu_ring(0)` for A/B but pipeline will regress — that is intentional.
- `examples/sentai_runtime/modsentai_diag.c`: `sentai.diag.tpu_ring([bool])` toggle + `sentai.diag.tpu_ring_stats([reset])` dict with keys `xfers, bytes, dma_fail, usb_fail, slot_to, drain_to`.
- `examples/sentai_runtime/error_codes.csv` + `sentai_error.h`: five new codes 0x0B50..0x0B54 (RING_LOOP_BOUND, RING_SLOT_TIMEOUT, RING_DMA_FAIL, RING_USB_SUBMIT, RING_DRAIN_TO).

### Safety / NASA-JPL compliance (see embeded.md)

- Loop bounded: `max_iters = (total/36KB)+2` explicit cap + counter-emit on breach.
- All waits timeout-bounded via `g_sentai_tpu_urb_timeout_ms` (default 200 ms, ~10x nominal).
- Zero heap in hot path (StaticSemaphore_t + static eDMA handle + sectioned slot storage).
- Failure paths release any held sema before return (no deadlock on caller retry).
- Drain emits per-slot timeout and does not re-give timed-out slots (avoids stranded permits — IOC eventually gives).
- ISR discipline: USB IOC fires in USB host task context (not ISR); eDMA is polled-sync (no IRQ to handle).
- Self-healing unaffected: USB CDC comes up in main_freertos BEFORE any ring code; a wedged ring call returns false and InferTask/REPL callers see a single-frame skip.
- Counters (`sentai.diag.tpu_ring_stats()`) + SERR_LOG codes give full post-mortem visibility.

### Expected behaviour (untested)

- **Pure TPU invoke yolo_1 512x512**: baseline 13 ms.  Expected +0.1..0.5 ms eDMA overhead ~= 13-14 ms.
- **Pipeline yolo_1 VGA**: baseline V22 42 FPS.  PURE adds SDRAM memcpy pressure for input (+2.6 ms) + possible PXP-vs-eDMA SEMC contention.  Plan estimate: 35-38 FPS.  If measured regression is larger, next step is fine-grained coord (sem given after SendInputs, not before invoke).
- **Pipeline yolo26 NEW**: first time this model can run pipeline at all.  Target: stable, 15-25 FPS.

### Test recipe (next session, board-side)

```python
import sentai
sentai.verbose(1)

# 1. Baseline pure TPU with ring on (default).
sentai.tpu.load('/models/yolo_1.tflite')
for i in range(10): sentai.tpu.invoke()
print(sentai.diag.tpu_ring_stats())  # xfers>0, dma_fail=0, usb_fail=0

# 2. A/B: ring off vs on (pure TPU).
sentai.diag.tpu_ring(0)  # regress to legacy zero-copy from SDRAM
# ... measure
sentai.diag.tpu_ring(1)

# 3. Pipeline regression check.
sentai.camera.init(1, 640, 480, 45)
sentai.pipeline.start()
# ... measure infer_stats, diag.tpu_ring_stats
sentai.pipeline.stop()

# 4. yolo26 NEW pipeline test (never ran before).
sentai.tpu.load('/models/yolo26_768x512.tflite')
sentai.pipeline.start()
```

### Next-step candidates (if PURE baseline regresses >10%)

1. **Fine-grained coord**: have SendInputs signal `g_sentai_tpu_input_done_sema` (InferTask's `s_sem_bufs_free`) after its final drain instead of InferTask giving the sema before `sentai_tpu_invoke_with_input()`.  Eliminates PXP-vs-eDMA SEMC overlap during SendInputs, at the cost of slightly reduced prep/invoke overlap.
2. **PXP target back to OCRAM** (revert half of PURE): keep `.tpu_input` in OCRAM for pipeline stability but keep the ring for yolo26-class ins (where the flatbuffer source is SDRAM anyway).  That's the HYBRID variant described earlier in this doc.
3. **eDMA IRQ-driven producer** (async instead of polled).  Lets M7 yield during eDMA fills, gives PrepTask CPU back quicker.
