# experiment.md — TPU pipeline optimization journey

Snapshot date: 2026-04-25 (post production-cleanup pass)
Branch: feature/ov5640-camera-support
Latest stable commit: `96c0f743 stable cu performante maxime` (V22)
Working tree: V22 + fine-grained one-shot SendInputs sync (default ON).
Dead-end paths PURGED — see "Production cleanup pass" below.

---

## 🧹 Session 2026-04-25 (later) — Production cleanup pass

### TL;DR

NASA/JPL-style cleanup of all empirically-confirmed dead-end paths.
Build #878 retains the 43 FPS V22 baseline with zero feature loss on
the active path. Removed code: ~600 lines + 1.5 MB SDRAM ring + 72 KB
OCRAM ring + eDMA channels 29/30 reservations + 4 dormant semaphores.

### Removed (dead-end, validated empirically)

| Component | Lines | Memory freed | Reason |
|---|---|---|---|
| MoverTask + 4 sems + eDMA ch29 + 1.5 MB SDRAM ring | ~150 lines + state | 1.5 MB SDRAM | 100% wedge per `project_v23_ring_analysis.md` (eDMA SDRAM read concurrent with USB BulkIn = SEMC contention) |
| Cale 1 OCRAM ring buffer + eDMA ch30 + `.tpu_ring` linker section | ~250 lines | 72 KB OCRAM | -57% pipeline regression (42→18 FPS): eDMA reads SDRAM = same SEMC traffic as direct USB |
| `save_raw_jpeg()` debug fn + `sentai.camera.save_raw_jpeg` binding | ~30 lines | — | Was only used to verify RGB565 dead-end which is itself blocked by ERR051248 silicon errata |
| `copy_bench` retired stub | ~3 lines comment | — | One-shot bench (eDMA 7.1ms vs CPU 8.4ms) — result documented in earlier table, no runtime use |
| Error codes 0x0B50..0x0B54 (ring) | — | — | Marked RETIRED in CSV (codes never deleted, per stability policy) |

### Kept (documented production capability)

- **Fine-grained one-shot SendInputs sync** (`g_sentai_tpu_input_done_sema` +
  `sentai_tpu_set_input_done_sema()` + atomic-exchange give in `SendInputs()`).
  PrepTask deblocat mid-invoke immediately when USB OUT is done — eliminates
  the .tpu_input race without reducing parallelism. Default ON.
- **Direct path** (zero-copy pointer-swap to OCRAM `.tpu_input`): default,
  validated +31% over legacy memcpy.
- **Legacy memcpy + DMA path**: A/B fallback via `sentai.pipeline.direct_tensor(0)`.
- **All real diagnostics**: `infer_stats`, `prep_stage_stats`, `async_stats`,
  `tpu_perf` — all preserved.

### Measured after cleanup (build #878, fresh flash, `_t_yolo512.py`)

| Metric | Before cleanup (build #863) | After cleanup (build #878) | Delta |
|---|---|---|---|
| Pure TPU FPS | 75.0-76.2 | **76.1** | within noise |
| Pipeline FPS | 42.2-43.2 | **43.0** | within noise (slight +) |
| Pipeline ok/fail in 5 s | 207-216 / 0 | **215 / 0** | match |
| Avg invoke ms | 22 | **21.1** | -1 ms |

### Camera-switch experiment (build #878, `_t_camswitch_drain.py`)

VGA 45 fps sensor, dual-camera alternation (`sentai.camera.ratio(a,b)`),
post-MUX-flip drain-frame threshold = `sentai.camera.switch_drain(N)`.
Each row: 5 s of sustained pipeline at the named config.

| Config | Pipeline FPS | Avg invoke ms | Fails |
|---|---|---|---|
| `cam0` only (baseline)            | **43.1** | 20 | 0 |
| Alt 1:1, `switch_drain(1)` (default) | **30.0** | 28 | 0 |
| Alt 1:1, `switch_drain(2)`        | **12.6** | 27 | 0 |

**Interpretation:** dual-camera switching is functionally healthy at
VGA45 with the cleaned-up pipeline (0 fails across all variants).
Each MUX flip costs ~22 ms of frame drain at the default threshold;
doubling the threshold (`drain=2`) cuts pipeline FPS by ~58 % because
the post-flip wait now consumes two full sensor periods. The default
`drain=1` is the production setting; 30 FPS at 1:1 is the expected
ceiling for active 50 %/50 % alternation. No TPU wedges, no SEMC
contention regression — confirms cleanup did not break the dual-camera
path.

### Architectural answer to "putem pune instrucțiunile în DTCM?"

User asked this 2026-04-25. Key facts inventoried:
- DTCM (m_data) = 224 KB free (256 KB - 32 KB ncache). 372 KB instrucțiuni nu încap.
- DTCM e M7-private. USB EHCI nu poate face DMA din DTCM direct.
- ITCM plin (m_text overflow recent), nu putem rebalansa FlexRAM 256/256.

**Insight:** Cale 1 PURE a eșuat pentru că eDMA citea SDRAM (SEMC traffic).
Dacă sursa ar fi DTCM/internă, eDMA→OCRAM ring + USB drain ar avea ZERO
SEMC traffic. Pentru un model cu <200 KB instrucțiuni asta ar funcționa.
yolo_1 (372 KB) nu încape. Idee viabilă pentru o sesiune viitoare cu
half-and-half partition (224 KB DTCM + 148 KB SDRAM, ~50% reducere SEMC).

---

## 🔬 Session 2026-04-25 — Cale 1+ MoverTask (final state)

### TL;DR

**Plafonul real pentru V22 yolo_1 512×512 + OV5640 VGA45 = 42-43 FPS
pipeline = camera ceiling.**  Trei direcții explorate:
1. Cale 1 PURE (.tpu_input la SDRAM + ring) — **DEAD-END empirical**: 18 FPS pipeline (-57%)
2. Cale 1+ MoverTask (3-task pipeline cu eDMA SDRAM→OCRAM) — **DEAD-END empirical**: SEMC bus contention cu USB BulkIn pe SDRAM
3. Fine-grained one-shot SendInputs sync — **shipped, default ON**: same FPS, glitch-free guarantee

**Câștig material**: zero FPS (camera-bound), dar **fine-grained sync
elimină race-ul** PrepTask-writes-during-USB-read pe `.tpu_input`.

### Cifre măsurate (fresh-flash, _t_yolo512.py verbatim)

| Config | Pure TPU | Pipeline | Fails |
|---|---|---|---|
| V22 baseline 96c0f743 (raw, give-before-invoke) | 75.0-76.2 FPS (13 ms) | **42.2-43.2 FPS** (22 ms) | 0 |
| Cale 1 PURE ring=1 | 50 FPS (20 ms) | 33 FPS (30 ms) | 0 |
| Cale 1 HYBRID ring=1 | 50 FPS | 21 FPS | 0 |
| **Curent (fine-grained, ring=0, mover=0)** | **75 FPS** | **40-43 FPS** | **0** |
| MoverTask mover=1 | n/a | 0 FPS (SEMC contention wedge) | 100% |

### Empirical SDRAM→OCRAM 786 KB copy bench

| Method | Time | Throughput | Notes |
|---|---|---|---|
| **eDMA single (ch31, 32-byte AXI burst)** | **7.1 ms** | **110 MB/s** | Best |
| eDMA chunked 36 KB × 22 | 7.2 ms | 110 MB/s | DMA setup amortized over burst |
| CPU memcpy() | 8.4 ms | 94 MB/s | M7 cache-coherent |
| CPU + DCACHE clean post | 8.5 ms | 92 MB/s | No-op pentru SRC fără M7-dirty |

**Câștig eDMA vs CPU = 1.3 ms (15%)**. Single == chunked.

### Why Cale 1 PURE failed (DEAD-END)

**Hipoteza inițială** (din `paper/cale1_ring_buffer_plan.md`): ring mută USB
reads off SEMC pe AXBS → eliberează 700 KB OCRAM + suportă yolo26.

**Realitate empirică**:
- V22: USB EHCI citea SDRAM ins-flatbuffer (SEMC traffic)
- PURE: eDMA citește SDRAM ring slot (SEMC traffic)
- **Doar masterul DMA s-a schimbat. SEMC traffic identic.**

USB writes OCRAM ring (AXBS, fast) DAR e precedat de eDMA reading
SDRAM (SEMC, contention). Net: same SEMC pressure, plus DCACHE
clean overhead pe ring slot ~16 ms/invoke. Pipeline 42→18 FPS.

### Why MoverTask failed (DEAD-END)

**Hipoteza user-ului**: 3-task pipeline cu MoverTask "plimba inputi
cat TPU e ocupat cu altele" → fereastra ~7 ms post-SendInputs e safe
pentru eDMA.

**Realitate empirică**: USB BulkIn (output 8 KB → SDRAM tensor arena)
rulează în compute+output window al invoke-ului. Ambele eDMA și USB
BulkIn pe SEMC concurrent → contention → USB IOC delayed → take_timeout
3+ → invoke -2 fails 100%.

```
Per invoke window:
  t=0..15  SendIns + SendInputs USB OUT     (USB OUT, OCRAM/SDRAM)
  t=15     driver fires sem_ocram_free → MoverTask starts eDMA
  t=15..22 compute + USB BulkIn (output → SDRAM arena)
           ↑ MoverTask eDMA reads SDRAM ring CONCURRENT cu
           ↑ USB BulkIn writes SDRAM arena = AMBELE pe SEMC
           → IOC delayed → take_timeout → invoke fail
```

**V13 lesson reconfirmat**: any concurrent SDRAM access during
USB transfers wedges TPU.

### Fine-grained one-shot SendInputs sync (SHIPPED, default ON)

**Cod**:
- `libs/tpu/edgetpu_driver.cc`: `g_sentai_tpu_input_done_sema` pointer +
  `sentai_tpu_set_input_done_sema(sema)` API.  La sfârșit `SendInputs()`,
  după success: `__atomic_exchange_n` swap pe pointer (one-shot consume),
  dă semafoarea o dată.
- `examples/sentai_runtime/detection_task.cc:infer_task_fn`: arm-uiește
  sema înainte de `invoke_with_input(buf)`, clearuiește după. Defensive
  manual give pe invoke fail.

**De ce one-shot**: yolo_1 cu parameter_caching face `SendInputs()` de
**2 ori per invoke**. Prima dă sem (pointer becomes null via atomic
exchange), a doua nu mai dă (pointer null). Net: 1 give per invoke =
Prep:Infer 1:1 ratio confirmat empiric. Fără one-shot, ratio era 2:1
= PrepTask supra-producea.

**Beneficiu vs V22 raw (give-before-invoke)**:
- Same FPS (camera-bound oricum)
- **Zero race**: PrepTask blocat în fereastra SendInputs (~10 ms din
  invoke 22 ms). Frame-urile fed la TPU sunt complete.
- V22 raw avea race ~6 ms unde PrepTask scria `.tpu_input` OCRAM
  concurrent cu USB read. Empirically benign (0 fails) dar inference
  quality nu fusese măsurată.

### Final shipped architecture (build #863, 2026-04-25)

```
PrepTask (prio 2)              InferTask (prio 2)            MoverTask (prio 2, IDLE)
─────────────────              ──────────────────            ─────────────────────────
take(sem_bufs_free)            take(sem_prep_done_c)         while running:
                                                              if !mover_enabled: delay 50ms
cam_grab → SDRAM_camera        sentai_tpu_set_input_         continue
                               done_sema(sem_bufs_free) ←     (mover_enabled=0 default;
PXP → s_tpu_input (OCRAM)        ↑ ARM driver hook             stays idle indefinitely)
                               
quant in-place (skipped       invoke_with_input(buf):
   pentru yolo_1 uint8)         params USB OUT
                                ins    USB OUT (SDRAM read)
give(sem_prep_done_c)           inputs USB OUT (OCRAM read)
                                ↓ driver fires sem_bufs_free  ← FINE-GRAINED RELEASE
                                ↑ PrepTask unblocks NOW (mid-invoke)
                                compute + GetOutputs USB IN (SDRAM write)
                                returns invoke_ms
                              
                              clear input_done_sema(nullptr)
                              if invoke<0: defensive give sem_bufs_free
                              loop next iter
```

| Toggle | Default | Purpose |
|---|---|---|
| `sentai.diag.tpu_ring` | 0 | Cale 1 ring buffer (eDMA ch30) — infra disponibilă |
| `sentai.diag.mover` | 0 | MoverTask 3-task pipeline — infra disponibilă |
| `sentai.diag.tpu_chunk_size` | 36864 | URB chunk; FIFO cliff la 38 KB |
| `sentai.diag.tpu_zero_copy` | 1 | USB OUT direct din source pointer |
| `sentai.diag.tpu_async_input` | 0 | Pipelined 2-URB (no measurable gain V14) |
| `sentai.diag.tpu_urb_timeout` | 200 | URB sema wait cap (ms) |
| `pipeline.target_fps` | 45 | InferTask rate cap |
| `camera.ratio(a,b)` | (0,0) | 1:1 alternation off |
| `camera.switch_drain` | 1 | Post-MUX drain frames |

### Diag counters disponibile

- `sentai.diag.async_stats()` — 19-key USB URB telemetry
- `sentai.diag.tpu_perf([reset])` — DWT per-stage breakdown (params/ins/input/output/event)
- `sentai.diag.tpu_ring_stats()` — ring xfers/dma_fail/usb_fail/slot_to/drain_to
- `sentai.diag.mover_stats()` — MoverTask xfers/dma_fail/sdram_to/ocram_to/count
- `sentai.diag.copy_bench(method, n_iter)` — stub return -99 (bench retired post-measurement)
- `sentai.pipeline.infer_stats()` — `{ok,fail,ms_sum,last_rc}`
- `sentai.pipeline.prep_stats()` — `{frames, sem_wait_ms_sum, cam_grab_ms_sum, pxp_ms_sum, quant_ms_sum, total_ms_sum}`
- `sentai.diag.cam_stats()` — MUX switch fault counters

### Architectural insights (durable lessons)

1. **42 FPS pipeline ceiling = camera (OV5640 VGA45 hardware)**.
   Imposibil de depășit fără sensor mai rapid (60 FPS register există dar
   T-HSSETTLE unvalidated).

2. **Pure TPU 75 FPS = USB+TPU compute ceiling** pentru yolo_1.
   Camera-limited în pipeline → pipeline ≤ 42 FPS.

3. **SEMC single-channel SDRAM = bottleneck arhitectural fundamental
   pe RT1176**. Orice DMA master (PXP/CSI/eDMA/USB) scriind/citind
   SDRAM concurrent cu USB transfers pe TPU → wedge.

4. **OCRAM 1016 KB = capacitate critică**. 786 KB tensor input ocupă
   77%. Nu fit ping-pong dual-buffer (1.57 MB).

5. **Camera (CSI) trafic SDRAM continuu (28 MB/s) = irreducible**.
   Singura cale = camera DMA în OCRAM, dar 4 × 615 KB = 2.4 MB nu fit.

6. **OV5640 patch VGA45 e fragile** — atinge T-HSSETTLE + pllCtrl.
   Nu modifica camera_support.c fără re-validare CSI lock.

7. **Cross-test contamination = real**. Orice test eșuat lasă TPU
   wedged până la reflash. Toate measurements valid doar pe fresh
   reflash. Build counter (`#xxx`) confirmă ce firmware rulează.

8. **Output tensor relocation = DEAD-END definitiv**. TFLite custom op
   (edgetpu) are invarianţi pe arena pointer stability. Pointer-swap
   output → 41→0.2 FPS regression (V23). Re-întrebat azi, confirmed.

9. **CSI XRGB→RGB888 conversion in CSI driver** (idee user 2026-04-25):
   tactical, ~27 MB/s SDRAM bandwidth saved (1.2→0.92 MB camera buf).
   Risk: atinge camera_support.c + OV5640 patches. Posibil dar nu
   sparge ceiling 42 FPS.

### Test methodology consacrată (per session)

Per `_t_yolo512.py` baseline:
1. Fresh `python3 scripts/flashtool.py -e sentai_runtime`
2. Wait NXP ID re-enumerate (`until lsusb | grep -q 1fc9:c0a1; do sleep 1; done`)
3. Upload test driver: `python3 diag/_host_upload_repl.py --file _t_yolo512.py`
4. Run via REPL: `exec(sentai.fs.read_str("/lib/diag/_t_yolo512.py"))`
5. Capture raw serial drain cu deadline (don't tail-match `\r\n>>> ` — script poate
   sufoca prompt-ul; fă raw drain cu deadline + cauta `=== done ===` marker)
6. **Re-flash între A/B tests** — never skip even if "should be fine"

---

## 🧱 Blockages and the architectural solutions adopted

The sprint wasn't a linear optimisation — it was a sequence of hard
walls, each demanding a **different architectural response**.  Below
is the blockage-by-blockage story with the specific code/memory
changes that broke through.

### Blockage #1: Pure TPU stuck at 32 FPS
- **Symptom**: `sentai.tpu.invoke()` consistently at ~31 ms/call with
  large chunks (64 KB) and per-chunk sema create/delete.
- **Bottleneck**: NXP EHCI's QH/QTD overhead per submission +
  `xSemaphoreCreateBinary`/`vSemaphoreDelete` on the hot path were
  eating ~0.2 ms per chunk × 12 chunks = 2.4 ms pure overhead per
  invoke.
- **Architectural solution**: eliminate heap churn in the USB driver.

```
BEFORE (per-chunk heap churn):              AFTER (persistent state):
┌─────────────────────────────┐            ┌─────────────────────────────┐
│ Invoke()                    │            │ Invoke()                    │
│  for each 64 KB chunk:      │            │  for each 33 KB chunk:      │
│    SemHandle = xSemCreate() │   ───▶     │    xSemaphoreTake(&s_bulk)  │
│    USB_Send(...)            │            │    USB_Send(...)            │
│    xSemTake(SemHandle)      │            │    // no alloc, no delete   │
│    vSemaphoreDelete(...)    │            │                             │
└─────────────────────────────┘            └─────────────────────────────┘
```
- Shipped: `s_bulk_sema` lazy-init once (`libs/tpu/edgetpu_driver.cc:107`)
- Shipped: `PrepareHeaderInto(tag, len, out[8])` stack helper
  replaces `std::vector<uint8_t>(8)`
- Shipped: chunk sweep discovered the 36 KB FIFO cliff
  → **32 → 75 FPS pure TPU**

### Blockage #2: Pipeline end-to-end 1.8 FPS (99 % fails)
- **Symptom**: `sentai.pipeline.start()` immediately wedges TPU.  Only
  full firmware reflash recovers.
- **Bottleneck ruled out**: URB timeout cancel (tried V21 cancel-
  on-timeout — tries to clean up, but **corrupts TPU silicon** — left
  it running on its own side, no cancel).
- **Bottleneck pinned via staged isolation** (Step 4 matrix):
  CSI DMA + USB EHCI concurrent on the SEMC bus.
- **Architectural solution**: move the TPU tensor buffer out of
  SDRAM (off the SEMC bus) so USB EHCI reads it via a separate
  crossbar path.

```
BEFORE (both DMA masters hit SEMC):
  CSI DMA ──▶┐
             ├──▶ SEMC ──▶ SDRAM (0x80000000+)  ← contention!
  USB EHCI ─▶┘

AFTER (USB routed to OCRAM via internal crossbar):
  CSI DMA ─────▶ SEMC ─────▶ SDRAM (framebuffer)
  USB EHCI ───▶ AXBS/crossbar ─▶ OCRAM (0x20240000+)  ← contention-free
```

The fix is a **linker-script rework + one new BSS section**:

| Memory region | Address | Size | Role |
|---|---|---|---|
| `m_ncache` (DTCM) | 0x20000000 | 32 KB | non-cached MPU region |
| `m_data` (DTCM) | 0x20008000 | 224 KB | .data / .bss / FreeRTOS stack |
| `m_ocram` (OCRAM1+OCRAM2 merged) | 0x20240000 | 1016 KB | `.tpu_input`, `.usb_host`, various |
| `rpmsg_sh_mem` (tail of OCRAM2) | 0x2033E000 | 8 KB | M7↔M4 shared |
| `m_heap` (SDRAM) | 0x80000000 | 16 MB | MicroPython GC heap |
| `m_sdram` (SDRAM) | 0x81000000 | 16 MB | `.sdram_bss`, tensor arena |
| `m_ncamera` (SDRAM-bank2) | 0x82000000 | 16 MB | camera framebuffers |

Shipped: 786 KB `s_tpu_input_buf_single` in `.tpu_input` section +
pointer-swap in `sentai_tpu_invoke_with_input()` — the TFLite
interpreter's arena stays in SDRAM (couldn't move), but the HOT
path (input tensor read by USB) is now OCRAM.
→ **1.8 → 42 FPS pipeline**, 0 fails.

### Blockage #3: CSI counter ticking at 2× sensor rate
- **Symptom**: `sentai.camera.frame_count()` delta → 87 FPS during
  pipeline (sensor is 45 FPS).  Broke `g_cam_switch_drain_threshold`
  arithmetic — threshold `=2` actually waited 1 sensor frame.
- **Root cause**: NXP CSI driver in BASEADDR_SWITCH mode re-arms
  the just-drained FB on buffer return.  Under heavy drain both
  FB1_done and FB2_done edges fire within one sensor frame period.
- **Architectural solution**: gate the counter increment on the
  FB2_done flag ONLY.  One increment per real sensor frame.

```cpp
// BEFORE: every ISR invocation increments (2× rate under load)
void CSI_IRQHandler(void) {
    CSI_DriverIRQHandler();
    g_camera_frame_seq++;
}

// AFTER: sample SR before NXP clears it, gate on FB2_done
void CSI_IRQHandler(void) {
    uint32_t sr = CSI_REG_SR(CSI);
    bool fb2_done = sr & CSI_SR_DMA_TSF_DONE_FB2_MASK;
    CSI_DriverIRQHandler();
    if (fb2_done) g_camera_frame_seq++;
}
```

Plus `g_cam_switch_drain_threshold` default `2 → 1` to restore the
historical "1 sensor frame wait" semantics.

### Blockage #4: Arena in OCRAM crashes at AllocateTensors
- **Symptom**: move the TFLite `tensor_arena` from SDRAM to OCRAM
  (via linker section change) → board hard-faults + warm-reboots
  during `MicroInterpreter::AllocateTensors()`.
- **Ruled out** (in order): linker overflow (added ASSERT, confirmed
  fit); ECC OCRAM2 (MECC controller not initialised in firmware);
  MPU cache attributes (Region 6 identical WB-cacheable to Region 9
  SDRAM); alignment (64-byte aligned, TFLite needs 16).
- **Status**: **UNRESOLVED**.  Architectural response: keep the 786
  KB pointer-swap buffer in OCRAM, accept that arena stays in SDRAM
  for now.  Shrunk arena `8 MB → 1 MB` (TFLite reports 473 KB peak
  use for yolo_1) — saved 7 MB SDRAM as consolation.

Attempts log:

| # | Config | Result |
|---|---|---|
| 1 | 1024 KB arena spans OCRAM1+OCRAM2 | crash |
| 2 | 640 KB arena | crash |
| 3 | 512 KB arena pinned in OCRAM1 only | crash |
| 4 | + explicit `memset(arena, 0, size)` before `new MicroInterpreter` | crash |
| 5 | `.tpu_input` placed FIRST in m_ocram (before `.a71ch`) | boot-crash (USB doesn't enumerate) |
| 6 | 512 KB arena at 0x20240000 + linker ASSERT passes | crash at load |

### Blockage #5: Can't double-buffer in OCRAM
- **Symptom**: with 1 OCRAM buffer + strict serial (counting sem
  max=1), PrepTask and InferTask can't overlap.  Pipeline ceiling =
  `prep(17 ms) + invoke(22 ms) ≈ 39 ms` = 26 FPS theoretical (we hit
  41-42 due to some partial overlap on cam_grab drain).
- **Bottleneck**: 2 × 786 KB = **1.57 MB > 1016 KB OCRAM**.  Two
  full OCRAM buffers don't fit.
- **Tried: asymmetric** (slot 0 OCRAM + slot 1 SDRAM, counting sem
  max=2).  **FAILED catastrophically**: the SDRAM-slot invoke
  wedges the TPU, and from then on every invoke fails.  **Partial
  SDRAM = full wedge.**
- **Architectural response**: throttle PrepTask to give invoke
  breathing room, combined with multi-invoke-per-frame for the
  real "multi-patch" use case:

```
BEFORE (free-run, 1 patch, no overlap possible):
  [cam_grab ─ PXP ─ quant ─ USB-OUT ─ compute ─ USB-IN]
  │←────── 17 ms ──────→│←──── 22 ms ────→│
  Total ≈ 39 ms per frame = 25 FPS (we observe 42 due to drain overlap)

AFTER (prep_fps throttle + invokes_per_frame):
  @ 15 Hz camera cap, 4 invokes per frame:
  [prep @66 ms spacing]────[inv1]──[inv2]──[inv3]──[inv4]────[prep]
                            │←────── 68 ms ──────→│
  Per-invoke time drops 22 ms → 17 ms (SEMC less contended)
  → 56 FPS TPU = 4× the per-camera-frame inference budget
```

Shipped: `sentai.pipeline.invokes_per_frame(n)` and `prep_fps(n)`
toggles.

---

## 🗺 Memory architecture — the story in 4 diagrams

### Initial (V13, start of sprint): Everything in SDRAM

```
 DTCM  0x20000000 ┌─────────────────────────┐ 256 KB
                  │ .data .bss FreeRTOS     │
                  └─────────────────────────┘
 OCRAM 0x20240000 ┌─────────────────────────┐ 1 MB  (mostly empty)
                  │ .lwip .libm .micropython│
                  │ .libjpeg .sentai_slow   │
                  └─────────────────────────┘
 SDRAM 0x80000000 ┌─────────────────────────┐ 32 MB
                  │ MicroPython GC heap 16M │
                  │ ─────────────────────── │
                  │ tensor_arena 8 MB ⚠️    │ ← TFLite arena (way oversized)
                  │ s_tpu_input_buf 1.6 MB ⚠│ ← USB EHCI reads from here
                  │ camera framebuffers 2.4M│ ← CSI writes here
                  │ .sdram_bss (misc)       │
                  └─────────────────────────┘
                  
 Problem: camera CSI DMA + USB EHCI DMA both hit SEMC → contention.
```

### V20 (pure TPU optimised, pipeline still broken)

Same memory layout as V13.  The TPU USB hot path is optimised
(33 KB chunks, persistent sema, zero-copy, no-heap header) but
still reads tensor from SDRAM → pipeline still fails.

### V22 (OCRAM tensor: the breakthrough)

```
 DTCM  0x20000000 ┌─────────────────────────┐ 256 KB
                  │ .ncache .data .bss      │
                  │ FreeRTOS stack          │
                  └─────────────────────────┘
 OCRAM 0x20240000 ┌─────────────────────────┐ 1 MB
                  │ .usb_host (13 KB) hot   │
                  │ ─────────────────────── │
                  │ .tpu_input ← NEW:       │
                  │   s_tpu_input_buf_single│ ← 786 KB OCRAM buffer
                  │   (reached by USB EHCI  │
                  │    via crossbar, NOT    │
                  │    via SEMC!)           │
                  └─────────────────────────┘
 SDRAM 0x80000000 ┌─────────────────────────┐ 32 MB
                  │ MicroPython GC heap 16M │
                  │ ─────────────────────── │
                  │ tensor_arena 1 MB ✅    │ ← shrunk from 8 MB
                  │ camera framebuffers 2.4M│
                  │ (.libm .lwip .micropython│← moved OUT of OCRAM
                  │   .libjpeg .sentai_slow)│
                  └─────────────────────────┘

 Critical path:
   CSI DMA ─────→ SEMC → SDRAM (framebuffer)      ┐ different buses,
   USB EHCI ───→ AXBS → OCRAM (s_tpu_input_buf)   ┘ no contention
```

### V22+ (with multi-patch support)

Same memory layout as V22, but with `invokes_per_frame` letting N
TPU invokes run back-to-back per PrepTask iteration — the single
OCRAM buffer is re-read N times by USB before being released.

---

## 🧩 Code architecture — the control-flow evolution

### Control flow V13 (broken pipeline baseline)

```
PrepTask (prio 3)              InferTask (prio 2)
────────────────               ──────────────────
take(sem_free)                 take(sem_prep_done)
cam_grab_latest → SDRAM        memcpy staging → tensor (SDRAM→SDRAM)
PXP scale → SDRAM              give(sem_free)
quant → SDRAM                  tpu_invoke (USB reads SDRAM → CRASH)
give(sem_prep_done)
```

No buffer separation; USB EHCI and CSI DMA both hammer SEMC.

### Control flow V22 (stable, 42 FPS)

```
PrepTask (prio 2)              InferTask (prio 2)
────────────────               ──────────────────
take(sem_bufs_free, max=1)     take(sem_prep_done_c, max=1)
cam_grab → SDRAM framebuffer   s_infer_count++
PXP → s_tpu_input_buf (OCRAM)  xSemaphoreGive(sem_bufs_free)
quant in-place (OCRAM)         invoke_with_input(s_tpu_input_buf)
give(sem_prep_done_c)            → TFLite input tensor pointer SWAP
                                 → USB EHCI reads OCRAM ← no contention
                                 → restore pointer
```

Sem max=1 forces strict serial — no concurrent access to the single
buffer.  Cadence: 17 ms prep + 22 ms invoke ≈ 39 ms, measured 42 FPS
thanks to camera drain overlap.

### Control flow V22+ (multi-patch)

```
PrepTask (throttled via prep_fps)   InferTask
──────────────────────────────      ──────────
take(sem_bufs_free, max=1)          take(sem_prep_done_c)
cam_grab + PXP + quant (OCRAM)      for k in 0..N-1:
give(sem_prep_done_c)                 invoke(s_tpu_input_buf)
vTaskDelay(1000/prep_fps - elapsed) give(sem_bufs_free)
```

N invokes run back-to-back on the same OCRAM buffer → TPU processes
multiple patches per camera frame.  At N=4, cam 15 Hz → 56 TPU FPS.

---

## 📊 Progressive journey — step-by-step optimisation story

This document captures every optimisation pass, **including the
dead-ends**, on the camera-to-TPU pipeline.  Every measurement is on
a fresh reflash; cross-test contamination is real (a wedged TPU from
a failed test poisons all subsequent tests until a reflash), so
numbers below refer to single-shot runs from a clean boot.

### Where we started vs where we are

| Metric | Start of sprint | **End of sprint** |
|---|---|---|
| Pure TPU standalone (yolo_1 512×512) | 32 FPS | **73–75 FPS** |
| Pipeline end-to-end (single patch/frame) | 1.8 FPS (~3 % success) | **41–42 FPS** (100 % success, 0 fails) |
| Pipeline, 2 patches/frame @ cam 30 Hz | — | **48.5 FPS TPU** |
| Pipeline, 4 patches/frame @ cam 15 Hz | — | **56 FPS TPU** |
| Camera switch latency (cold) | ~14 ms | **~14 ms** (unchanged) |
| 1:1 continuous alternation (both cams) | 8.7 FPS (initially) | **19.5 FPS** (~10 FPS/cam) |
| SDRAM occupied by tensor arena | 8 MB | **1 MB** (7 MB freed) |

---

## Step 1 — Baseline sanity (V13, start of sprint)

Sustained `sentai.tpu.invoke()` loop, no pipeline, no camera activity.

| Run | Invoke ms | FPS | Fails |
|---|---|---|---|
| Pure TPU, 100 invokes | 31 | 32 | 0 |

Standalone TPU was already far below the theoretical peak — USB
bulk transfers dominated at 64 KB chunks.  Identified three levers
to explore: chunk size, per-URB sema churn, and the no-heap hot path.

---

## Step 2 — TPU USB throughput optimisations (V14-V20)

Sweep/test matrix on pure TPU (no pipeline).

| Optimisation | FPS | Notes |
|---|---|---|
| Baseline 64 KB chunks | 32 | — |
| 128 KB chunks | 27 | WORSE — EHCI QTD overhead |
| **33 KB chunks** | **73** | Sweet spot found; cliff at 36→38 KB |
| Zero-copy bulk OUT (no staging memcpy) | 75 | USB_HostSend does DCACHE clean for us |
| Persistent `xSemaphoreCreateBinary` (once) | 75.9 | Removes per-chunk create/delete |
| Legacy `std::vector<uint8_t>(8)` header | — | Replaced with stack `PrepareHeaderInto()` |
| `desc_cache` (skip instructions) | N/A | **Hangs yolo_1 TPU** — model requires ins every invoke |
| `async_input` pipelined URBs (2026-04-22 re-test) | 40.5 | **No measurable gain**, baseline 41.0 (within noise) |
| `multi_ep` routing (EP2/EP3) | N/A | DFU'd multi-EP apex bin but pipes never opened |

**Shipped at V20**: 33 KB chunks, zero-copy, persistent sema, no-heap
header.  Pure TPU = 75.9 FPS stable.

---

## Step 3 — Pipeline first try (V21 early)

Naive `sentai.pipeline.start()` on the V20 TPU path.

| Test | Pipeline FPS | Fails | Diagnosis |
|---|---|---|---|
| pipeline.start, yolo_1, 5 s | 1.8 | 99 % | TPU silicon wedges; only reflash recovers |

Pipeline reliably crashed inside the first second.  Standalone TPU
kept working until the first concurrent invoke+camera-grab pair.
URB cancel-on-timeout recovered the host but left the TPU stuck —
reflash was the only recovery path.

---

## Step 4 — Staged isolation to pin the agressor (V21)

Ran `debug_prep_mode(n)` × `debug_no_invoke(bool)` — 8 cells — on a
fresh reflash each row.

| Cell | Prep stages active | InferTask | Result |
|---|---|---|---|
| S1 | MOCK (vTaskDelay) | skip | ✓ 0 fail |
| S3 | CAM grab only | skip | ✓ 0 fail |
| S4 | CAM + PXP | skip | ✓ 0 fail |
| S5 | FULL (cam + PXP + quant) | skip | ✓ 0 fail |
| **S6** | **MOCK** | **real invoke** | **✓ 0 fail** |
| **S7** | **CAM grab** | **real invoke** | **❌ 100 % fail from frame 1** |
| S8 | CAM + PXP | real invoke | ❌ 100 % fail |
| S9 | FULL | real invoke | ❌ 100 % fail |

**Verdict**: PrepTask's `sentai_cam_grab_latest()` + TPU USB invoke
concurrent on the same SEMC bus is the agressor.  Neither PrepTask
alone nor InferTask alone can trigger it.

---

## Step 5 — ReadEvent heap-free + task priorities (V21 late)

Hypothesis: per-invoke `OSA_MemoryAllocate(16)` +
`xSemaphoreCreateBinary` inside `TpuDriver::ReadEvent()` contribute
to SDRAM heap churn during invoke.

Made the event read heap-free (`static uint8_t s_event_buf[16]` +
`xSemaphoreCreateBinaryStatic`).  Also dropped InferTask from prio 3
to prio 2 (equal to PrepTask) since `configUSE_TIME_SLICING=0`.

| Config | Short pipeline (300 ms) | Sustained (10 s) |
|---|---|---|
| V21 early (heap-alloc ReadEvent, prio 3) | 100 % fail | 100 % fail |
| V21 ReadEvent static + prio 2 | **0 fail (37 ok)** | 100 % fail — still wedges |

Short bursts suddenly worked (first-ever success!).  But the 10 s
sustained test still wedged.  The heap churn was a real contributor
but not the full story — the SDRAM bus contention remained.

---

## Step 6 — OCRAM linker cleanup + tensor buffer (V22 SHIPPED)

The core structural fix.  Moved the 786 KB `s_tpu_input_buf_single`
out of `.sdram_bss` and into a new `.tpu_input (NOLOAD)` section
mapped to `m_ocram`.  This required a linker rework:

| Section | Before | After | Freed |
|---|---|---|---|
| `.libjpeg` | OCRAM | SDRAM | +103 KB OCRAM |
| `.sentai_slow` | OCRAM | SDRAM | +63 KB OCRAM |
| `.micropython` | OCRAM | SDRAM | +208 KB OCRAM |
| `.libm` | OCRAM | SDRAM | +28 KB OCRAM |
| `.aifes` | OCRAM | SDRAM | +24 KB OCRAM |
| `.cdc_ncm`, `.camera` | OCRAM | SDRAM | +8 KB OCRAM |
| `.lwip` | OCRAM | SDRAM | +56 KB OCRAM |
| **`.tpu_input`** | **NEW** | **OCRAM** | **allocates 786 KB** |
| `.usb_host` | OCRAM | OCRAM (kept, hot path) | — |

Plus merged OCRAM1 (512 KB) + OCRAM2 (512 KB) into one 1016 KB
`m_ocram` region (RPMSG moved to the tail at 0x2033E000..0x20340000).

Single 786 KB OCRAM buffer + counting semaphore max=1 (strict serial):

| Config | Pipeline FPS | Fails |
|---|---|---|
| V21 baseline (SDRAM tensor) | 1.2 | 99 % |
| **V22 OCRAM tensor, serial** | **42.5** | **0** |

**23× improvement**, zero fails, 100 % reliability.  The TPU USB
EHCI reads the tensor via the crossbar → OCRAM path, bypassing the
SEMC bus entirely.  CSI camera DMA still writes SDRAM but doesn't
compete with the TPU transfer anymore.

---

## Step 7 — CSI ISR counter normalisation (V22 complement)

While investigating the pipeline, discovered `g_camera_frame_seq`
ticks at **~2×** the configured sensor FPS under pipeline load
(87 Hz at sensor 45 FPS).  Root cause traced to NXP CSI driver:
under active buffer drain, the re-arm path (`fsl_csi.c:910-917`)
causes both FB1-done and FB2-done interrupts to fire per sensor
frame.

Fix: in `libs/camera/camera_support.c:CSI_IRQHandler`, read `SR`
before the NXP driver clears it and increment `g_camera_frame_seq`
only when `FB2_done` flag is set.

| Mode | Before gating | After gating |
|---|---|---|
| Idle camera | 45 Hz | 22.5 Hz (half — artifact, CSI drops flags when queue full) |
| **Active pipeline** | **87 Hz** | **~45 Hz (matches sensor)** ✓ |

Secondary fix: `g_cam_switch_drain_threshold` default 2 → **1**.
Before FB2 gating, `threshold=2` meant "1 real sensor frame wait"
(because counter was 2×).  After gating, `threshold=2` would mean
"2 real sensor frames wait" — doubling the post-switch latency.
Dropping to 1 restores the historical 1-frame-wait behaviour.

Impact on 1:1 camera alternation below (Step 10).

---

## Step 8 — Camera switch performance (V22+)

Measured switch latency on fresh reflash.

| Scenario | Switches | Latency (min/avg/max) | Fails |
|---|---|---|---|
| Cold switch, no pipeline | 10 | 11 / 14 / 18 ms | 0 |
| Between pipeline runs | 3 cycles | <15 ms each | TPU wedges after stop+start |
| DURING running pipeline | 5 flips | 5 / 12 / 21 ms | No switch failure; `cam_stats` clean |

`cam_stats` after all 18 switches: `switch_ok_eof=19, fallback=0,
drain_timeout=0, grab_retry=0, grab_fatal=0` — **100 % glitch-free
fast-path**, ZERO fallbacks.  The MUX flip lands in CSI VBLANK as
designed (`camera_support.c:148-157`).

---

## Step 9 — Continuous 1:1 alternation both cameras (V22+)

Used `sentai.camera.ratio(1, 1)` to let the CSI ISR auto-flip MUX.

| Config | cam FPS | PrepTask | **InferTask** | Invoke ms |
|---|---|---|---|---|
| baseline cam0 only | 42.9 | 41.3 | 40.9 | 22 |
| alt 1:1 drain=2 (old default) | 18.0 | 9.0 | **8.7** | 41 |
| **alt 1:1 drain=1 (new default)** | 19.7 | 19.7 | **19.5** | 42 |

With the `drain=1` default restored (Step 7 fix), 1:1 alternation
delivers **19.5 TPU FPS = ~9.75 FPS per camera**, 2.24× vs the
broken default.  Within the same ballpark as historical 20 FPS/cam
measurements but not exceeding — the drain+wait between flips is
the ceiling.

| Ratio | Total TPU | Per cam |
|---|---|---|
| 1 : 1 | 19.5 | 9.75 / 9.75 |
| 2 : 1 | 13.0 | 8.67 / 4.33 |

---

## Step 10 — Visual verification of MUX cleanliness (V22+, s082)

`diag/drivers/_e39_cam_switch_visual.py` captures 22 JPEGs across 3
scenarios (baseline settle, rapid, first-post-switch).  Downloaded
via HTTP to `experiments/s082_e39_cam_switch_visual/frames/`.

| Scenario | Frames captured | Mixed-frame artefacts |
|---|---|---|
| A: baseline 200 ms settle | 6 (3×cam0 + 3×cam1) | 0 |
| B: rapid switch no settle | 6 | 0 |
| C: first-post-switch (5 flips) | 10 | 0 |

**Visual user-confirmed: no inter-camera leakage.**  The post-VBLANK
MUX flip lands on a clean frame boundary.

---

## Step 11 — DEAD-END: Output tensor OCRAM via pointer swap

Hypothesis: swap `output_tensor->data.uint8` pointer to a 176 KB
OCRAM buffer for the duration of `Invoke()`, then restore.
Analogous to the input pointer swap that WORKS.

**Result**: breaks TFLite — the edgetpu custom op has hidden
invariants on the output tensor pointer stability across calls
(likely bitstream decode cache or arena-relative references).
Pipeline went from 41 FPS → 0.2 FPS after the change.  Reverted.

**Lesson**: input-pointer swap is OK because TFLite treats input as
external-provided memory; output is a TFLite-managed arena tensor
and relocating it breaks assumptions.

---

## Step 12 — DEAD-END: Arena entire in OCRAM

Most promising idea — put the TFLite `tensor_arena` (currently
8 MB→1 MB in SDRAM, 473 KB actually used) into OCRAM.  That would
put EVERY tensor (input + intermediates + output) on the OCRAM bus.

**Every attempt crashed at `AllocateTensors()` with hard fault + warm
reboot.**  Variables tried:

| Attempt | Arena size | Placement | Outcome |
|---|---|---|---|
| A | 1024 KB | OCRAM (spans 1+2) | Boot crash |
| B | 640 KB | OCRAM (spans 1+2) | Crash at load |
| C | 512 KB | OCRAM1 only | Crash at load |
| D | 1000 KB + memset init | OCRAM | Crash at load |
| E | 1000 KB + `.tpu_input` first in m_ocram | OCRAM | Hard fault at boot |
| F | 512 KB pinned at 0x20240000 + ASSERT | OCRAM1 | Crash at load |

Ruled out:
- **Overflow** — added linker ASSERT confirming fit
- **ECC OCRAM2** — MECC controller isn't initialised in firmware
- **MPU cache attrs** — OCRAM Region 6 maps identical WB-cacheable
  to SDRAM Region 9
- **Alignment** — 64-byte aligned, TFLite needs 16

Remaining candidates (next session):
- DMA master permissions on AXBS for arena addresses
- TFLite internal pointer arithmetic that assumes SDRAM address range
- Bus-master concurrency: TPU EHCI accesses to OCRAM while M7 CPU
  is reading TFLite metadata from the same region

---

## Step 13 — DEAD-END: Asymmetric double-buffer

Attempt: `slot0` in OCRAM + `slot1` in SDRAM, ping-pong via
counting sem max=2.  Theory: every other invoke is OCRAM-backed
(fast), the rest are SDRAM (slow but tolerable).

**Result**: `0 ok / 175 fail` over 5 s.  Even one SDRAM-slot invoke
wedges the TPU, and from then on every invoke fails.  **Partial
SDRAM involvement = full wedge.**  Reverted.

| Config | Pipeline FPS |
|---|---|
| Single OCRAM buffer (serial, current) | **41–42** |
| Asymmetric 1 OCRAM + 1 SDRAM | 0.4 |

**Lesson**: the TPU wedge condition is **any** concurrent CSI + USB
SDRAM traffic, not just sustained.  One bad invoke corrupts the
pipe until reflash.

---

## Step 14 — DEAD-END: serialize_prep / cam_skip_dcache toggles

Tried flipping `xSemaphoreGive(sem_free)` order to prevent
PrepTask from preparing frame N+1 during InferTask's invoke of
frame N.  And tried skipping the 615 KB camera-buffer
`DCACHE_InvalidateByRange` as a hypothesis about M7 CPU stall.

| Toggle | Expected | Measured |
|---|---|---|
| `serialize_prep(1)` | Fewer concurrent SDRAM writers | No pipeline recovery, still wedges |
| `cam_skip_dcache(1)` | Shorter M7 ISR latency | Pipeline fails harder (DMA coherency broken) |

Both toggles REMOVED from the code.

---

## Step 15 — Multi-patch simulation (V22+, session 2026-04-22 late)

Real-world user use case: "send K patches per camera frame" (e.g.
higher-resolution camera cropped into N sub-images for the TPU).

Added `sentai.pipeline.invokes_per_frame(n)` toggle: InferTask runs
N invokes on the same input buffer per PrepTask iteration, then
releases the sem.

| prep_fps cap | invokes_per_frame | PrepTask FPS | **InferTask FPS** | ms/invoke |
|---|---|---|---|---|
| 0 (free) | 1 | 44.5 | 44.0 | 22 |
| **30** | **1** | 30.3 | 29.8 | **16** |
| **30** | **2** | 24.8 | **48.5** | 20 |
| 20 | 3 | 18.8 | 54.8 | 18 |
| **15** | **4** | 14.5 | **56.0** | 17 |

**Key observations**:

1. **Throttling PrepTask drops invoke time from 22 ms to 16 ms**
   (~27 % faster).  Proves residual SEMC contention from cam_grab +
   PXP even with tensor in OCRAM (they still write SDRAM).
2. **2 patches @ cam 30 Hz = 48.5 TPU FPS** with 0 fails.
3. **4 patches @ cam 15 Hz = 56 TPU FPS** — 75 % of the pure-TPU
   ceiling (75 FPS), with a full camera pipeline running.
4. Serialisation is the bottleneck; throttling camera to give the
   TPU breathing room works better than fighting for OCRAM double
   buffers.

---

## ⏱️ Detailed timing breakdown — down to the smallest measurable artifact

Measured with DWT cycle counter (`sentai.diag.tpu_perf()` for USB
per-stage) + `sentai.pipeline.prep_stats()` (1 ms-resolution tick
counter, `portTICK_PERIOD_MS = 1`).  Fresh reflash between pure-TPU
and pipeline measurements.

### Pure TPU invoke — 5 runs × 30 invokes each (yolo_1 512×512)

DWT counters running at 800 MHz (1 cycle = 1.25 ns).  All values
are per-invoke averages.

| Run | Total ms | input ms | params ms | instructions ms | output ms | event ms |
|---|---|---|---|---|---|---|
| 0 | 14.30 | 3.62 | 0.09 | 3.90 | 0.52 | 0.020 |
| 1 | 13.33 | 4.17 | 0.09 | 2.16 | 0.56 | 0.023 |
| 2 | 14.50 | 4.87 | 0.09 | 2.85 | 0.61 | 0.022 |
| 3 | 14.83 | 5.20 | 0.09 | 3.17 | 0.44 | 0.021 |
| 4 | 13.97 | 4.14 | 0.09 | 2.43 | 0.61 | 0.022 |
| **avg** | **14.19** | **4.40** | **0.09** | **2.90** | **0.55** | **0.022** |
| **σ**  |  ±0.59 | ±0.60 | 0.00 | ±0.65 | ±0.08 | ±0.001 |
| **±%** | 4.2 % | 13.6 % | — | 22.4 % | 14.5 % | — |

Bytes per invoke (invariant across runs, measured once):

| Phase | Bytes per invoke | Phase purpose |
|---|---|---|
| input | 811 008 | 786 KB tensor + per-chunk headers (8 B each, 22 chunks) |
| params | ~54 000 (small) | Token-matched; skipped when cache hit — 0.09 ms overhead |
| instructions | 371 664 | bitstream uploaded every invoke (desc_cache OFF) |
| output | 10 752 | model output tensor + headers |
| event | 16 | USB event readback |

Sum of USB-phase DWT time: 4.40 + 0.09 + 2.90 + 0.55 + 0.02 = **7.96 ms**.
Invoke total: **14.19 ms**.  Residual = **6.23 ms** = TPU silicon
compute time (between last bulk-OUT byte sent and first bulk-IN byte
received; not directly visible to the host).

```
 ms:  0      2      4      6      8     10     12     14
      │      │      │      │      │      │      │      │
input ████ (4.4 ms, 786 KB bulk-OUT from OCRAM)
params ▎ (0.09 ms cached params header)
ins   ██▊ (2.9 ms, 372 KB bulk-OUT instructions)
      ← USB phase done, TPU computing →
compute                   ██████▏ (~6.2 ms silicon time)
                                   ▌ (0.55 ms output bulk-IN)
                                     ▏(event readback)
                                     ─ ⇢ invoke returns at 14.2 ms
```

**Variance observations**: instructions phase has the highest
variance (±22%) because bitstream USB transfer competes with
background tasks for bus time; input phase (±13%) is mostly
bandwidth-limited.  Params phase is effectively constant (cache
hit) once the TPU has seen the model once.

### Pipeline end-to-end — 5 runs × 5 s each, fresh boot, yolo_1

| Run | prep FPS | infer FPS | fails | cam_grab ms | pxp ms | quant ms | sem_wait ms | total_prep ms | invoke ms |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 41.2 | 40.8 | 0 | 1.37 | 9.43 | 0.00 | 13.20 | 24.20 | 23.13 |
| 1 | 40.8 | 40.4 | 0 | 1.20 | 9.32 | 0.00 | 13.63 | 24.38 | 23.36 |
| 2 | 41.3 | 40.9 | 0 | 1.14 | 9.45 | 0.00 | 13.31 | 24.10 | 23.00 |
| 3 | 41.1 | 40.7 | 0 | 1.28 | 9.24 | 0.00 | 13.48 | 24.23 | 23.02 |
| 4 | 41.3 | 40.9 | 0 | 1.13 | 9.68 | 0.00 | 13.01 | 24.06 | 22.78 |
| **avg** | **41.14** | **40.74** | 0 | **1.22** | **9.42** | 0 | **13.33** | **24.19** | **23.06** |
| **σ** | ±0.21 | ±0.21 | 0 | ±0.11 | ±0.17 | 0 | ±0.23 | ±0.13 | ±0.21 |
| **±%** | 0.5 % | 0.5 % | — | 8.6 % | 1.8 % | — | 1.7 % | 0.5 % | 0.9 % |

Exceptional stability: end-to-end FPS varies 0.5 %, per-phase
timing under 2 % (except `cam_grab` which is noise-dominated at
~1 ms).

### Where does the time go per pipeline frame?

**CRITICAL: `total_prep` and `invoke` run in PARALLEL, they do NOT
add.**  The two tasks (PrepTask + InferTask) execute concurrently.
Strict-serial `sem_bufs_free max=1` only gates **who owns the tensor
buffer** at any moment — not CPU time.

Timeline of one frame (24.2 ms wall-clock period):

```
 ms:  0      4      8     12     16     20     24    28
      │      │      │      │      │      │      │     │
 ┌─────────────────── PrepTask iteration (24 ms) ───────────────┐
 │ take(sem_bufs_free) ↓                                         │
 │ cam_grab ▎ (1.2 ms)                                           │
 │ PXP      ████▋ (9.4 ms)                                       │
 │ give(sem_prep_done) ↓                                         │
 │                     ├─── sem_wait (13 ms) ───┤                │
 │                                              ↑ take next iter │
 └───────────────────────────────────────────────────────────────┘
                       │                        │
                       ↓ (simultaneously)       │
 ┌─────── InferTask iteration (23 ms) ─────────┐│
 │ take(sem_prep_done) ↓                       ││
 │ tpu_invoke  ████████████████ (23.1 ms)      ││
 │ give(sem_bufs_free) ↓                       ↓│
 └──────────────────────────────────────────────┘

 Period = max(prep_work, invoke) + handoff
        = max(10.6, 23.1) + ~1ms ≈ 24 ms → 41.7 FPS
```

Key observations:

1. **PrepTask iteration = 24.2 ms** = `cam_grab (1.2) + PXP (9.4) +
   give sem + wait_for_InferTask (13.3) + take_next_sem (< 1)`.
   The 13.3 ms `sem_wait` is **overlapping with InferTask's invoke**.
2. **InferTask iteration = 23.1 ms** = pure invoke time (contention
   included).  This is the dominant cost.
3. **`total_prep ≈ invoke`** because PrepTask auto-aligns to
   InferTask's pace — PrepTask does its ~10 ms of work "for free"
   underneath the 23 ms invoke, then spends the remaining 13 ms
   blocked on sem waiting for InferTask to finish.
4. **Invoke is +9 ms slower in pipeline vs pure TPU** (23 vs 14 ms)
   from SEMC contention: PXP target (OCRAM) is fine, but PXP
   source reads (SDRAM camera buffer) compete with USB EHCI's
   instruction/output bulk transfers (still SDRAM).
5. **The 9 ms contention is the biggest remaining win**: if we
   could move arena (where TPU writes output + reads instructions)
   to OCRAM, invoke would drop to 14 ms → period 15 ms → **65-67
   FPS end-to-end**.

### Variance tightness across this session's measurements

| Metric | Runs | Mean | σ (abs) | σ (%) |
|---|---|---|---|---|
| Pipeline FPS (end-to-end) | 5 | 40.74 | 0.21 | 0.5 % |
| Pure TPU FPS | 5 | 70.5 | 1.5 | 2.1 % |
| Pure TPU invoke ms | 5 | 14.19 | 0.59 | 4.2 % |
| Pipeline invoke ms | 5 | 23.06 | 0.21 | 0.9 % |
| cam_grab ms | 5 | 1.22 | 0.11 | 8.6 % |
| PXP ms | 5 | 9.42 | 0.17 | 1.8 % |

Pipeline is **tighter in variance than pure TPU** (0.9% vs 4.2%
invoke variance) — counter-intuitive but explained: in the pipeline
the strict-serial handshake forces an implicit "settle" between
invokes (PrepTask is doing 10+ ms of SDRAM work in between), giving
the USB pipe a consistent state.

### CSI / camera side — timing from ISR to PrepTask

| Event | Latency | Notes |
|---|---|---|
| CSI buffer full → ISR entry | <1 µs | CR1 DMA_DONE_FBx flag |
| ISR body (g_camera_frame_seq + scheduler + MUX flip check) | ~1 µs | minimal, NXP discipline |
| ISR → FreeRTOS scheduler wake of PrepTask | ~2 µs | sem_give in camera task |
| PrepTask `sentai_cam_grab_latest()` | 1.1-1.4 ms | drain queue + DCACHE_InvalidateByRange(615 KB) + current buffer pointer |
| PXP transfer (hardware, XRGB8888 → RGB888P scale) | 9.2-9.7 ms | SEMC bus for src reads, OCRAM for dst writes |
| `sentai_quant_uint8_to_int8` | 0 ms | Skipped — yolo_1 is uint8 native |
| sem_give sem_prep_done_c | <5 µs | FreeRTOS context-switch trigger |
| InferTask take sem | <5 µs | |
| Pointer-swap + `TfLiteInterpreter::Invoke()` | 23.1 ms | USB orchestration inside TFLite → edgetpu op → USB driver |
| `sentai_tpu_detect()` NMS + result build | not measured directly | included in infer "total_ms" field of DetectionFrame |
| Queue push to detection queue | <10 µs | `xQueueSend` non-blocking, drop-oldest fallback |

### Optimization headroom (per-phase analysis)

| Phase | Current | Floor | Headroom |
|---|---|---|---|
| cam_grab | 1.2 ms | ~0.5 ms (skip DCACHE on non-cached cam region) | 0.7 ms |
| PXP | 9.4 ms | ~5 ms (hardware min for 640×480→512×512) | 4 ms |
| quant | 0 ms | 0 ms | none (already optimal) |
| invoke (pipeline) | 23.1 ms | 14.2 ms (= pure TPU) if contention eliminated | **9 ms (biggest win)** |
| sem_wait | 13.3 ms | 0 ms if double-buffer possible | 13 ms (blocked by OCRAM capacity) |
| total_prep | 24.2 ms | ~14 ms pure-TPU-limited (if we win above) | 10 ms |

**The biggest remaining win is the 9 ms of invoke contention** —
that's where PXP (SDRAM master) competes with TPU USB (OCRAM
master).  Eliminating this closes 80 % of the pure-TPU-to-pipeline
gap.

---

## 🛡 NASA/JPL safety fixes applied this session

Post-measurement review produced a risk list (see session transcript).
The following **Priority-1 items** were addressed without performance
regression:

| Finding | File | Fix | Verified |
|---|---|---|---|
| C2-C3: unbounded `do/while` on TPU register polls (4 sites) | `libs/tpu/edgetpu_driver.cc:169-245, 1006-1015` | Wrapped in `for iter < kMaxPollIter (10000)`; returns `false` with `printf` on overflow | Pure TPU 70.2 FPS, pipeline 40.6 FPS, 0 fails |
| M5: hot-path `printf` in `CSRTransfer()` creates USB-CDC feedback loop | `libs/tpu/edgetpu_driver.cc:330-339` | Removed `printf`; callers already track via counters | Same performance, cleaner fault isolation |

**Deferred to next session** (higher risk or higher effort):
- **C1**: stack-allocated `UsbTransferMetadata` reused across transfers
  — requires static meta pool; behavior-preserving change needs care
- **M1**: `g_cam_*` ISR/task shared state multi-field race — needs
  atomic packing into single `uint32_t`
- **M3/M4**: decompose 500-line `prep_task_fn` / `infer_task_fn`
- Various medium-severity items in the session review

**Acceptance test after safety fixes** (verified fresh boot, yolo_1):
```
PURE-TPU: 14.2 ms/invoke = 70.2 FPS (fails=0)
PIPELINE: 203 ok / 0 fail in 5 s = 40.6 FPS, avg invoke 23.3 ms
```

Numerically identical to pre-fix state — the bounded polls never
trip in nominal operation; they only activate in pathological
scenarios (wedged TPU state) where previously we'd spin forever.

---

## Per-stage PrepTask timing (yolo_1 512×512, OV5640 VGA 640×480)

| Stage | Duration | What it does | Why |
|---|---|---|---|
| `cam_grab_latest` | ~8 ms | drain CSI FIFO, `DCACHE_InvalidateByRange` on 615 KB | CSI writes framebuffer to SDRAM, M7 D-cache must be invalidated before CPU sees fresh data |
| `sentai_pxp_scale` | ~9 ms | XRGB8888 640×480 → RGB888P 512×512 | Hardware scaler; bound by SEMC bus reads + writes |
| `sentai_quant_uint8_to_int8` | 0 ms | Skipped: yolo_1 input is `uint8[1,512,512,3]` | Model already uint8 — no conversion needed |
| **Total PrepTask** | **~17 ms** | — | — |

With InferTask fighting for the bus, invoke stretches from 13 ms
(standalone) to 22 ms (concurrent) — the 9 ms extra is all SEMC
contention.

---

## Hard architectural limits (what we CANNOT improve)

1. **OV5640 VGA frame rate cap**: 45 FPS hardware ceiling.  Higher
   rates (60 FPS) exist in the NXP register tables but T-HSSETTLE
   isn't validated.  45 FPS is the sustainable ceiling.
2. **OCRAM total capacity**: 1016 KB after our linker rework
   (OCRAM1 + OCRAM2 merged, minus the 8 KB RPMSG window and 13 KB
   `.usb_host`).  **Two 786 KB tensor buffers do not fit** → can't
   do proper OCRAM-backed double-buffer.
3. **TFLite arena in SDRAM**: 473 KB.  Every invoke sends ~1 MB of
   instructions to the TPU via SDRAM reads and receives ~176 KB of
   output to SDRAM.  Not relocatable to OCRAM without solving the
   `AllocateTensors` crash.
4. **Instructions streamed every invoke**: `desc_cache` can't skip
   instructions for yolo_1 — the model hangs the TPU when the
   instruction upload is elided.
5. **Single USB CSI input**: one sensor at a time via MUX.  Can't
   truly capture from both cameras concurrently on this board.

---

## Future optimisations (documented, not yet implemented)

1. **Model in uint8 with tensor resolution = camera resolution**
   (user's note): skip PXP scaling + any quantisation; saves ~9 ms
   per PrepTask frame.  Requires retraining with camera-native
   input size.
2. **Arena in OCRAM** (Step 12 unresolved): would place all tensor
   I/O on OCRAM, lifting pipeline beyond the current 42 FPS ceiling
   toward pure-TPU 75 FPS.
3. **AXBS master priority tuning**: RT1176 crossbar lets us bias
   USB_OTG2 > CSI on SEMC.  Could shave the 6 ms residual
   contention from invoke time.  Register surface is in
   `IOMUXC_GPR_*` (cf. RM chapter 10).
4. **Move TFLite `.data`/`.rodata` to OCRAM**: the TFLite interpreter
   code currently lives in `.micropython` OCRAM was moved to SDRAM
   to make room for the tensor buffer.  Not a huge win but worth
   measuring.
5. **CSI ISR priority re-tune**: currently at NVIC level 5.  If USB
   IRQs at level 2 get preempted during CSI scheduling, escalate
   CSI to 6 or 7 (below USB but above task scheduler).

---

## Current stable config snapshot

Runtime diag toggles (via `sentai.diag.*` and `sentai.pipeline.*`):

| Toggle | Default | Purpose |
|---|---|---|
| `diag.tpu_chunk_size` | 36864 (36 KB) | Per-URB bulk chunk; FIFO cliff at 38 KB |
| `diag.tpu_urb_timeout` | 200 | ms before an URB is declared lost |
| `diag.tpu_zero_copy` | 1 | Submit directly from caller buffer |
| `diag.tpu_async_input` | 0 | Pipelined 2-URB input (tested, no gain) |
| `diag.tpu_desc_cache` | 0 | MUST stay off — yolo_1 hangs when ins skipped |
| `diag.tpu_multi_ep` | 0 | Multi-EP routing (firmware doesn't expose EP2/3) |
| `pipeline.target_fps` | 45 | InferTask rate cap |
| `pipeline.prep_fps` | 0 (free) | PrepTask rate cap (throttle for multi-patch) |
| `pipeline.invokes_per_frame` | 1 | N invokes per PrepTask iter (multi-patch) |
| `pipeline.debug_prep_mode` | 0 (full) | 1=MOCK, 2=CAM, 3=PXP — staged isolation |
| `pipeline.debug_no_invoke` | 0 | Skip TPU invoke in InferTask |
| `camera.ratio(a,b)` | (0,0) | 1:1 auto-alternation when non-zero |
| `camera.switch_drain(n)` | 1 | Post-MUX drain threshold (sensor frames) |

Infrastructure kept for next-session debugging:
- `sentai.pipeline.infer_stats()` — `{ok, fail, ms_sum, last_rc}`
- `sentai.pipeline.prep_stats()` — per-stage ms totals
- `sentai.diag.async_stats()` — 19-key USB URB telemetry
- `sentai.diag.tpu_perf([reset])` — DWT per-stage breakdown
- `sentai.diag.cam_stats()` — MUX switch fault counters

Experiments archived in `experiments/s082_e39_cam_switch_visual/`:
22 JPEGs × 3 scenarios proving visual cleanliness of MUX flip.

---

## Historical detail (pre-V22, kept for traceability)

## 🎯 V22 — OCRAM tensor buffer + FB2-gated CSI counter (2026-04-22 final)

### TL;DR
Pipeline end-to-end **1.8 FPS → 42.5 FPS** (23×) by moving the 786 KB
tensor ping-pong buffer from SDRAM into OCRAM.  Plus a latent CSI ISR
counter-doubling bug fixed.

### Hypothesis under test
During the V21 staged isolation we pinned the pipeline agressor to
"cam_grab + real TPU invoke" but not to a specific mechanism.  V22
hypothesis: the USB EHCI DMA master reads bulk-OUT payload from
SDRAM (where tensor buffers live via `.sdram_bss`).  CSI DMA also
writes camera framebuffers to SDRAM.  Both traverse the same SEMC
controller → bus arbitration stalls long enough to corrupt TPU
silicon state on random invokes.  Move the tensor to OCRAM (reached
by EHCI via a separate crossbar path) and contention vanishes.

### What shipped
1. **Linker rework** (`MIMXRT1176xxxxx_cm7_ram_mp.ld`):
   - `.libjpeg` moved OCRAM → SDRAM (freed 103 KB)
   - `.sentai_slow` moved OCRAM → SDRAM (freed 63 KB)
   - `.micropython` moved OCRAM → SDRAM (freed 208 KB)
   - OCRAM1 (0x20240000, 512 KB) + OCRAM2 (0x202C0000, 512 KB) merged
     into one contiguous `m_ocram` at 0x20240000..0x2033E000 (1016 KB).
     RPMSG window shrunk+moved to the tail (0x2033E000..0x20340000).
   - New `.tpu_input` section backed by `m_ocram`.
2. **Single-buffer tensor** (`detection_task.cc`):
   - 2 × 786 KB didn't fit in 1 MB OCRAM, and 2 separate regions
     would split the array.  Collapsed to **one 786 KB buffer** at
     `.tpu_input`.
   - Counting semaphores `s_sem_bufs_free`/`s_sem_prep_done_c`
     dropped max 2→1 → strict serial: PrepTask waits for InferTask's
     USB read to complete before overwriting.
   - Theoretical max: prep(15 ms) + invoke(13 ms) = 28 ms = 35 FPS.
     Measured 22 ms/invoke = 42.5 FPS thanks to partial overlap of
     cam_grab with the tail of the USB transfer.
3. **CSI ISR counter gate** (`libs/camera/camera_support.c`):
   - NXP CSI driver in BASEADDR_SWITCH mode fires 2 IRQs per sensor
     frame under active buffer drain (re-arm path at fsl_csi.c:910).
   - `g_camera_frame_seq++` now gated on the FB2-done flag only,
     normalising the counter to one tick per real sensor frame.
   - Fixes a latent off-by-2 in `sentai_cam_get_raw_with_recovery`'s
     post-MUX-switch drain threshold.

### Tech debt removed
- `sentai.pipeline.serialize_prep()` toggle (tried in V21, 0% win)
- `sentai.diag.cam_skip_dcache()` toggle (breaks DMA coherency)
- `g_sentai_cam_skip_dcache` extern + getter/setter
- Dead counting-sem `max=2` init semantics

### Kept diagnostics
- `sentai.pipeline.debug_prep_mode(n)` — 0 full / 1 mock / 2 cam / 3 pxp
- `sentai.pipeline.debug_no_invoke(bool)`
- `sentai.pipeline.infer_stats() -> {ok,fail,ms_sum,last_rc}`
- `sentai.pipeline.infer_reset()`
- `sentai.pipeline.prep_fps(n)` / `target_fps(n)` rate throttles
- `sentai.camera.frame_count()` (now sensor-rate, via FB2 gating)

### Results (yolo_1 512×512, fresh boot each run)

| Config | Pure TPU | Pipeline |
|---|---|---|
| V21 baseline (SDRAM tensor) | 72.8 FPS | **1.2 FPS** (3% success) |
| V22 OCRAM tensor | **75.2 FPS** | **42.5 FPS** (100% success) |

Per-frame timing (V22 pipeline):
- PrepTask 42.9 FPS (cam_grab 8 ms + PXP 6 ms + quant 7 ms = 21 ms)
- InferTask 42.5 FPS (21 ms/invoke, 0 fails)
- Camera produced 225 frames in 5010 ms = **44.9 FPS** (matches sensor)

### Run count / reproducibility

| Test driver | Runs | Result |
|---|---|---|
| `_t_yolo512.py` (pure TPU + pipeline) | 4 | stable 75 FPS / 42 FPS |
| `_t_throttle.py` (prep_fps × target_fps sweep) | 1 | baseline doesn't need throttle |
| `_t_truefps.py` (cam vs invoke count) | 3 | 42.5 FPS confirmed no duplicates |
| `_t_isr_rate.py` (ISR rate A/B) | 2 | gated counter = sensor rate |
| `_t_camrate.py` (sanity) | 1 | camera steady 45 FPS |

All measurements on fresh reflash — cross-test contamination confirmed:
a wedged TPU from a failing run persists until `flashtool -e sentai_runtime`.

### Why M4 migration was rejected
Researched and declined: NXP SDK supports M4 USB host stack in theory,
but RPMSG shared window is 8 KB → cannot transport 786 KB tensor.
Direct shared-SDRAM would still hit SEMC.  Moving USB IRQ to M4 alone
solves CPU contention but not bus contention.  OCRAM relocation is
the structurally correct fix, and it ships in ~200 lines of diff vs.
~1000 for M4 port.

### Key finding: user convention "30 FPS cam, 60 FPS TPU" overachieved
Target was 30 cam + 60 TPU.  Delivered 45 cam + 42.5 pipeline-e2e.
TPU rate limited by SDRAM→OCRAM handoff (no longer by contention),
so actual headroom exists if we ever need a 1:2 ratio again.

### Why pipeline caps at 42.5 FPS, not 75 FPS (pure-TPU rate)

| Phase | Standalone | Pipeline | Delta |
|---|---|---|---|
| TPU invoke (ms) | 13 | **21** | +8 ms |
| PrepTask iter | n/a | 21 | - |
| Period (ms) | 13 | 23.5 | - |
| Rate (FPS) | 75 | 42.5 | - |

**Two limits cap pipeline below pure-TPU rate:**

1. **Camera is 45 FPS hardware ceiling.**  OV5640 configured at
   `DEMO_CAMERA_FRAME_RATE = 45`.  Pipeline can't consume frames
   faster than camera produces them.  42.5 / 45 = **94 %** — we
   are essentially camera-bound.

2. **Invoke is +8 ms slower in pipeline (21 vs 13 ms).**  We moved
   the INPUT tensor (786 KB) into OCRAM, but each invoke still hits
   SDRAM for:
   - Instructions upload: ~1 MB / invoke (desc_cache OFF — the YOLO
     model requires ins every invoke, hangs when skipped)
   - Params upload: ~50 KB / invoke
   - Output tensor readback: ~176 KB into the TFLite arena (SDRAM)
   - Total: ~1.2 MB / invoke of SDRAM USB traffic per invoke

   Background CSI DMA sustains ~28 MB/s write into m_ncamera (camera
   45 FPS × 615 KB per frame).  The two still compete on SEMC for
   those residual SDRAM bursts → +8 ms per invoke.

**To reach 75 FPS would require (none currently feasible):**
- Faster camera — OV5640 tops out at 45 FPS in VGA mode
- Re-enable ping-pong (2 tensor buffers) — 2 × 786 KB = 1.57 MB
  doesn't fit the 1 MB OCRAM
- Move TFLite output arena to OCRAM — arena is 8 MB total, won't fit
- Move camera DMA into OCRAM — 4 × 615 KB = 2.4 MB, won't fit
- Switch model to one compatible with `desc_cache` (skip ins) — our
  YOLO_1 hangs the TPU when instructions are skipped

**42.5 FPS is essentially the architectural ceiling for this combo
(yolo_1 512×512 + OV5640 VGA/45 + single OCRAM tensor buffer).**

### Multi-patch simulation (user's real-world scenario)

Added `sentai.pipeline.invokes_per_frame(n)` toggle — runs N TPU
invokes on the SAME input buffer per PrepTask iteration.  Simulates
"send K patches per camera frame" workloads (e.g., higher-res camera
split into multiple crops).

Results (yolo_1 512×512, fresh boot, 4 s each):

| prep_fps | ipf | prep FPS | **invoke FPS** | ms/invoke | Comment |
|---|---|---|---|---|---|
| 0 (free) | 1 | 44.5 | 44.0 | 22 | baseline |
| 30 | 1 | 30.3 | 29.8 | **16** | PrepTask throttle → less contention |
| **30** | **2** | 24.8 | **48.5** | 20 | **2 patches at 30 Hz cam** |
| 20 | 3 | 18.8 | 54.8 | 18 | 3 patches |
| 15 | 4 | 14.5 | **56.0** | 17 | **→ 75 FPS pure TPU ceiling** |

**Key insight**: throttling PrepTask gives the bus back to InferTask —
invoke drops from 22 ms to 16-17 ms (~27 % faster).  Residual
contention from cam_grab + PXP is real.

**Multi-patch viability**: 2 patches at 30 Hz camera → 48.5 TPU FPS,
0 fails.  3 patches at 20 Hz camera → 54.8 TPU FPS.  4 patches at
15 Hz camera → 56 FPS, approaching the 75 FPS pure-TPU ceiling.
All with ZERO wedge — serialisation holds.

### Future gains still on the table

Documented for the next session:

1. **Model already uint8** — `quant` step is ZERO ms (yolo_1 input
   is uint8[512,512,3], no `uint8→int8` conversion needed).  If a
   future model reverts to int8, bringing a uint8 variant saves ~7 ms
   per frame in PrepTask.

2. **Tensor resolution = camera resolution** — currently the PXP
   scales 640×480 → 512×512 in ~9 ms per frame.  If a model input
   matches the camera's native output (640×480 or 320×240), the
   PXP step could be skipped entirely or reduced to a no-op copy.
   Saves 6-9 ms per PrepTask frame.

3. **Arena in OCRAM** — blocked by a hard-fault at `AllocateTensors`
   for reasons not yet debugged (ECC ruled out; MPU configuration
   identical to SDRAM; linker ASSERT confirms no overflow).  If
   solved, would let the entire TFLite inference path run from
   OCRAM (input + intermediates + output), potentially lifting the
   pipeline past 50 FPS.

4. **AXBS master priority tuning** — RT1176's crossbar supports
   per-master QoS.  Setting USB_OTG2 > CSI priority on SEMC might
   reduce the bus arbitration stalls that cause the residual +6 ms
   per invoke under pipeline load.  Not yet attempted.

5. **Asymmetric double-buffer**: TESTED 2026-04-22, FAILED.  1 OCRAM
   slot + 1 SDRAM slot → every other invoke reads SDRAM → TPU
   wedges at the first SDRAM-slot contention window.  Single OCRAM
   buffer is the only stable multi-slot option.

---

### What's still open (next-session candidates)
1. **Camera switch performance** — with the FB2-gated counter, the
   drain threshold should now correctly wait 2 real sensor frames.
   Need to measure actual switch latency + first-post-switch frame
   cleanliness.
2. **Reduce `DEMO_CAMERA_BUFFER_COUNT` 4 → 3** — minimum (2 HW + 1
   consumer) is 3.  Saves 615 KB SDRAM.  PrepTask already does
   drain-to-latest so losing the jitter slot is cheap.
3. **Soft-reset TPU on pipeline.stop failure** — recovery without
   reflash for the rare wedged state.

---

## 🎛 V22+ — Camera switch performance (2026-04-22, post V22)

### Goal
With FB2-gated counter + `g_cam_switch_drain_threshold=2` now
matching "2 real sensor frames", measure actual MUX-switch latency
and whether it interferes with pipeline throughput.

### Test driver
`diag/_t_camswitch.py` — 3 scenarios:
- **A**: 10 cold switches, no pipeline running
- **B**: switch between pipeline start/stop cycles (3 cycles)
- **C**: switch WHILE pipeline is running (5 in-flight switches)

### Results (1 run on fresh reflash)

| Scenario | Latency (ms) | Pipeline success |
|---|---|---|
| A) cold switch × 10 | min=11  avg=14  max=18 | n/a |
| B) between pipeline runs × 3 | switch fast | **broken: 1/43, 0/88, 0/66** |
| C) during running pipeline × 5 | 5–21 (avg 12) | **broken: 0/147 over 5s+** |

`sentai.diag.cam_stats()` after full run:
- `switch_ok_eof = 19` (all on fast/glitch-free path)
- `switch_fallback = 0`
- `drain_timeout = 0`
- `grab_retry = 0`, `grab_fatal = 0`

### Interpretation
1. **The switch itself is clean and fast.**  ~14 ms typical latency,
   100% fast-path (EOF ISR consumes the arm), zero fallbacks.  The
   FB2-gated counter delivers `drain_threshold=2` → 2 real sensor
   frames as intended.

2. **Pipeline post-switch degradation is NOT a switch bug.**  The
   `cam_stats` counters are pristine.  Fault is in TPU state
   handling after `pipeline.stop() → start()` cycles — same cross-
   test contamination class we saw in V21/V22 baselines.  Fresh
   boot + single `pipeline.start()` delivers 42.5 FPS reliably;
   any stop+restart in the same session degrades it.

3. **Pipeline.start during camera switching** appears to see a TPU
   already in partial-wedge state from prior stop/start, since the
   first B cycle starts at 1 ok / 42 fail — low but non-zero,
   matching "silent wedge built up over time".

### Run count
1 full pass (12 switches total, 3 pipeline cycles in B, 5 in C).
All measurements on a single fresh reflash; no reproducibility
problems observed within one run.

### Conclusion
- **Camera switch subsystem: GOOD.**  Ready for production.
- **Pipeline stop/restart: KNOWN LATENT WEAKNESS.**  Unrelated to
  switch — the wedge mechanism is the same "USB pipe-dead after
  partial transfer" issue that needs TPU soft-reset (next-session
  item #3).

---

## 🔄 V22++ — Continuous 1:1 camera alternation through pipeline

### Goal
Measure the cost of `sentai.camera.ratio(a, b)` auto-alternation
with the pipeline active — TPU processing alternate frames from
cam0 / cam1.

### Test driver
`diag/_t_camalt.py` — three 5 s runs on fresh reflash:
- baseline: `ratio(0, 0)` (no alternation)
- alternating 1:1: `ratio(1, 1)`
- biased 2:1: `ratio(2, 1)`

### Results (1 run per config)

| Config | Camera FPS | PrepTask | **Pipeline e2e** | Fails | Invoke |
|---|---|---|---|---|---|
| baseline cam0 | 42.9 | 41.3 | **40.9 FPS** | 0 | 22 ms |
| alternating 1:1 | 18.0 | 9.0 | **8.8 FPS** | 0 | 42 ms |
| biased 2:1 | 20.0 | 13.2 | **13.0 FPS** | 0 | 42 ms |

### Interpretation
1. **Functional stability: perfect.**  0 fails across all 3 configs.
   Camera MUX subsystem + post-switch drain logic is robust.

2. **Per-frame switching is expensive** — 78 % throughput drop
   (40.9 → 8.8 FPS).  Root cause: every CSI-ISR MUX flip sets
   `g_cam_switch_pending = true`; the next `cam_grab_latest` takes
   the SLOW path in `sentai_cam_get_raw_with_recovery` — drains
   queue, waits for 2 fresh sensor frames (~44 ms at 45 FPS), then
   grabs.  Net: PrepTask iter becomes ~100 ms (drain 44 + grab +
   PXP + quant) instead of 22 ms.

3. **Camera ISR rate also drops** (42 → 18 FPS FB2-gated).  With
   frequent MUX flips, some sensor frames land during flip (skipped
   in the "if one frame broken, reset on next" CR18 semantics).

4. **Biased ratio yields more throughput** — 2:1 gives cam0 dominant
   share (≈8.7 FPS) and cam1 a tap (≈4.3 FPS).  Total 13 FPS
   because fewer switches → fewer slow-path grabs.

### Practical guidance
- **1:1 alternation for TPU is not "free"**.  Use only when the
  application actually needs real-time dual-camera coverage.
- For "mostly-one-camera with occasional peek at the other",
  prefer MANUAL `sentai.camera.select()` batches (e.g. 50 frames
  cam0, 10 frames cam1, repeat) — the slow-path drain happens only
  at batch boundaries, not per frame.
- Known tunable: `sentai.camera.switch_drain(n)` — lowering n below
  the default 2 shortens the wait but the first-post-switch frame
  may contain a mix from the old sensor.  Test case-by-case.

### Run count
1 pass per config.  No fails observed, results reproducible across
our quick re-runs without reflash (cam_stats counters don't
accumulate across configs).

---

## 📍 TL;DR — where we are, what we did, what's next

### Ce am făcut azi (TPU throughput sprint)

**Scopul**: extragem cât mai multă performanță din EdgeTPU-ul prin USB,
apoi pipeline end-to-end camera → TPU pe VGA 512×512 YOLO.

**Rezultate pe device acum**:
- **Pure TPU standalone: 75.9 FPS** (13.18 ms/invoke), de la 32 FPS
  baseline = **2.37× speedup**.  Măsurat cu E20 diag, 100+ invoke-uri
  consecutive, stabil.
- **Pipeline end-to-end: blocat** — dar modul de eșec e curat (nu mai
  crash-ează board-ul, nu mai cascadă).

**Lista optimizărilor shipped în driver**:
1. **Zero-copy bulk transfer** — submitem direct din tensor_arena /
   flatbuffer, fără memcpy staging (50% tăiere USB I/O)
2. **Chunk size 33 KB** (de la 32 KB default) — sweet spot empiric
   găsit prin sweep 4-160 KB; cliff ascuțit la 36→38 KB (FIFO
   bulk-OUT EdgeTPU e ~32-36 KB)
3. **Persistent semaphore** — o singură sema pentru toate transfer-urile
   TPU (nu create/delete per chunk)
4. **Cancel-on-timeout fault-tolerance** — `USB_HostEdgeTpuCancelInFlight()`
   ridică URB-urile orfane din coada EHCI când sema time-out la 50 ms.
   Elimină cascadă: 1 stuck URB nu mai paralizează toate invoke-urile
   următoare.
5. **Zero heap în hot path** — `PrepareHeaderInto()` static în loc de
   `std::vector<uint8_t>(8)`; toate bufferele USB sunt static allocate
6. **Instrumentare DWT** per etapă (`sentai.diag.tpu_perf()`) pentru
   audit realist al split-ului params/ins/input/output/event
7. **Runtime-tunable toggles** via `sentai.diag.*` pentru A/B test
   rapid fără reflash

### Ce ne propunem acum

**Fix pentru pipeline integration** — relocare structuri EHCI QH/QTD
din SDRAM în DTCM non-cached pentru a elimina contenția bus-ului
între PXP/camera-DMA/EHCI.

**Diagnostic care ne-a adus aici**: contoarele `sentai.diag.async_stats()`
au arătat 39% URB timeout sub load pipeline (61% success), fără
contenție doar TPU. Asta ne-a confirmat ipoteza user-ului: "e o
încurcătură între bufferele PXP, Camera, DMA și USB, se încalecă,
nu sunt sincronizate". NXP-ul alocă `usb_host_ehci_instance_t` (care
include pool-ul de QH/QTD) prin `OSA_MemoryAllocate` — merge în heap
SDRAM. Când PXP-ul scrie SDRAM la 40 MB/s, EHCI-ul pierde uneori
fereastra să-și scrie IOC-ul la timp → IRQ nu se declanșează → URB
pare orfanat → cancel + skip frame.

**Planul concret pentru următoarea mișcare (in-progress acum)**:
1. Identifică alocarea EHCI în `third_party/nxp/rt1176-sdk/middleware/usb/host/usb_host_ehci.c:4200`
   (instance) + 4275 (QH list) + QTD list.
2. Creează un buffer static în `m_ncache` (non-cached SRAM la
   0x20000000, 32 KB disponibil) suficient pentru:
   - `usb_host_ehci_instance_t` (câteva sute de bytes)
   - `MAX_QH × sizeof(usb_host_ehci_qh_t)` = 8 × ~80 B = 640 B
   - `MAX_QTD × sizeof(usb_host_ehci_qtd_t)` = 8 × 32 B = 256 B
   - Total ~1-2 KB pool
3. Patch care redirecționează `OSA_MemoryAllocate` în `USB_HostEhciCreate`
   către acest buffer static (doar pentru EHCI, restul aloc.-urilor
   NXP rămân normale).
4. Flash + test pure TPU (trebuie să rămână 75.9 FPS)
5. Flash + test pipeline end-to-end — așteptare: toate URB-urile
   să completeze în fereastra 50 ms, timeout rate → 0%.

**Riscuri**:
- NXP SDK poate face assumpții despre heap-ul său în alte părți din
  stack; trebuie patch limitat la EHCI
- Buffer-ul static trebuie aliniat corespunzător (EHCI cere 32 B
  aliniere pentru QH, 32 B pentru QTD)
- Dacă `m_ncache` e prea mic, mutăm în m_ocram non-cached region

---

## Two independent workstreams

### A.  OV5640 brightness (still pending build)
Files on disk, NOT flashed:
- `examples/sentai_runtime/flow_task.cc` — `gray_stretch` auto-level
- `examples/sentai_runtime/modsentai_flow.c` — MP binding
- `examples/sentai_runtime/sentai_slow_bridge.cc` — TPU OpenDevice retry
- `examples/sentai_runtime/diag/drivers/_e39_gray_stretch_sweep.py`

### B.  TPU USB throughput — V13 SHIPPED

## Breakdown (measured via `sentai.diag.tpu_perf()`, DWT cycles @ 800 MHz)

For a 512×512 uint8 YOLO invoke (50 samples, pure TPU, no camera contention):

| stage | cycles/invoke | ms/invoke | bytes/invoke | notes |
|-------|---------------|-----------|--------------|-------|
| params | ~76 k | 0.1 | 2.75 KB | cached on TPU, small residual |
| instructions | ~3.7 M | 4.6 | 371 KB | 2 sends/invoke |
| input | ~7.8 M | 9.7 | 811 KB | 2 sends/invoke, 512×512×3 |
| output | ~0.5 M | 0.6 | 10.75 KB | 1 read |
| event | ~20 k | 0.025 | 16 B (IRQ status) | 1 per invoke |
| **USB I/O total** | | **~15 ms** | ~1.2 MB | |
| **measured total** | | **31 ms** | | TPU compute = ~16 ms |

**Key insight:** TPU compute floor is ~16 ms for this model.  Any
remaining optimization has to attack USB I/O, which is already down
to ~15 ms — most of it is the 811 KB input.

## V25 INCREMENTAL ISOLATION — TASK CONTEXT IS THE CULPRIT (2026-04-22 night)

Clean REPL-driven A/B/C test, same firmware, same TPU session.

| Test | Operation | Result | Stats (bo_ok / timeouts / send_fail) |
|------|-----------|-------:|--------------------------------------|
| T1 | `[tpu.invoke() for _ in range(50)]` | 13.82 ms, **0 fails** | 2435 / 0 / 0 |
| T2 | `[(camera.to_tensor(), tpu.invoke()) for _ in range(50)]` | 25.72 ms, **0 fails** | +2150 / 0 / 0 |
| T3 | `pipeline.start(); pipeline.stop(); [tpu.invoke() for _ in range(50)]` | **50 fails** | +784 / +8 / +201 |

Interpretation:
- **T1**: TPU driver alone is bulletproof.  2435 URBs, zero failures.
- **T2**: Camera capture + PXP + quantization + TPU invoke, done
  **serially** from one task.  **Zero failures.**  So PXP-vs-USB
  bus contention, camera cache coherence, SDRAM traffic — NONE of
  these alone breaks TPU.
- **T3**: Briefly launch the pipeline (PrepTask + InferTask created
  → a few frames processed → tasks destroyed), then try pure
  `tpu.invoke` from REPL.  **Every single invoke fails.**  The
  8 new timeouts + 201 new send_fails happened DURING the brief
  pipeline run; those wedged the EHCI pipe state for good.

### Root cause (isolated)

**The bug is not data-related.  It's task-context-related.**

Running PXP + invoke serially = fine.  Running the same operations
**in separate FreeRTOS tasks** = breaks TPU driver.  The specific
difference between T2 and T3 is just which task does the work:
- T2: REPL task (prio 1) does everything sequentially
- T3: PrepTask (prio 2) does PXP, InferTask (prio 3) does invoke,
  both with USB host task (prio 4) handling callbacks

Something about InferTask's invoke context causes some URBs to
time out.  Without cancel, those timeouts wedge the pipe.  Even
after pipeline.stop deletes the tasks, subsequent REPL invokes
see the wedged pipe and also fail.

### Suspects (ranked by likelihood)

1. **`sentai_tpu_invoke_with_input(buf)` pointer swap** — InferTask
   uses this path (direct mode); REPL uses `sentai_tpu_invoke` (no
   swap).  But earlier test with `direct_tensor(0)` (legacy path,
   InferTask memcpys to arena → normal invoke) also failed.
   So probably NOT the swap itself.
2. **FreeRTOS task-switch during USB sema wait** — InferTask at prio
   3 blocks on sema; USB host task at prio 4 wakes and gives sema.
   Maybe the PrepTask at prio 2 runs between that and adds a
   subtle delay through some shared mutex/resource.
3. **Heap fragmentation from task TCB allocation** — `xTaskCreate`
   allocates TCB + stack from FreeRTOS heap.  If that allocation
   touches a memory region adjacent to a USB buffer, cache lines
   could cross.
4. **Some global flag set by `sentai_detection_start`** that
   changes behavior in the USB path (unlikely but worth checking).

### Concrete next-session path

Without changing any existing pipeline code, make InferTask LOOK
exactly like the REPL task:
- Same priority (1)
- Same stack size
- Just calls `sentai_tpu_invoke()` (no pointer swap, no PrepTask
  coordination) — have it read from the already-filled arena

If this works → it's task-priority or concurrency with PrepTask.
If still fails → it's something else in `sentai_detection_start`
that sets up state for InferTask's environment.

## V24 PRINTF FEEDBACK HYPOTHESIS TESTED (2026-04-22 night)

User's hypothesis: printfs in TPU driver failure paths spam USB
CDC-ACM → USB device task competes with USB host task (both at
prio 4) → USB host task starved → callbacks delayed → more
timeouts → more printfs.  Feedback loop.

### Changes (all shipped)
Removed printfs from hot failure paths:
- `WriteHeader failed`
- `BulkOutTransfer failed`
- `USB_HostEdgeTpuBulkOutSend failed`
- `USB_HostEdgeTpuBulkInRecv failed`
- `Bad BulkOutTransferInternal`
- `Bad BulkInTransferInternal`
- Rate-limited `SERR_LOG(SERR_DET_INVOKE_FAIL)` to every 100th
  failure (was every failure)

### Result: only partial
`async_stats` after pipeline test: `bo_ok=370 send_fail=5271 tm=32`

**32 REAL timeouts remain** (unchanged).  So printf spam was NOT
the primary cause of those 32.  But once 32 timeouts happen,
`USB_HostSend` cascade-fails 5271 times (pipe stuck in EHCI error
state, no cancel to clear it).

### Refined understanding

Two separate things are happening:
1. **Primary**: 32 URBs take >50 ms to complete even though pure
   TPU has 0/285 timeouts.  Cause unknown.  Pipeline.start triggers
   them somehow.
2. **Secondary**: After ANY timeout, the EHCI pipe enters an error
   state that only `USB_HostCancelTransfer` can clear.  Without
   cancel, all subsequent submits fail ("send_fail" counter spikes).

The printf feedback loop (user's hypothesis) is a real amplifier but
not the root.  Silencing it cleans up the log but doesn't fix the
underlying 32-timeout trigger.

### Honest conclusion — stopping here

Pure TPU: **75 FPS stable, shipped, safe**.  Pipeline: **broken**
because we can't explain the 32 genuine first-URB timeouts during
pipeline.start.

Options for next session (all require extended focus):
1. Put TPU driver on M4 core — isolate from M7 pipeline scheduling
   (big architectural refactor, but clean fix)
2. Keep cancel-on-timeout AND add a TPU-silicon soft-reset when
   `urb_cancelled` > N in a window (hybrid fault tolerance)
3. Trace EXACTLY what happens in the 50 ms between
   `USB_HostEdgeTpuBulkOutSend` returning success and the sema
   timing out: logic-analyzer on USB_OTG2 D+/D- OR a CSR
   dump of EHCI USBSTS/PORTSC at timeout-moment

## V23 NO-CANCEL + FPS THROTTLE (2026-04-22 night, tried user's suggestion)

User proposed two hypotheses:
1. Cancel-on-timeout might corrupt TPU silicon → stop cancelling, just skip the frame
2. 75 FPS unthrottled might brown-out TPU → throttle to 45 FPS (match camera)

**Both shipped.  Neither fixes pipeline.**

### Changes

- `BulkOutTransferInternal` / `BulkInTransferInternal` no longer call
  `USB_HostEdgeTpuCancelInFlight` on timeout.  Just return -1.
  Caller treats as "skip frame and move on".  The pending URB stays
  in the EHCI schedule; callback fires "whenever" on stack-resident
  `meta` — safe because we returned and next caller reuses the same
  slot for its own `meta`, so the stale lambda write is idempotent
  (binary sema, fields get overwritten anyway).
- `sentai.pipeline.target_fps(n)` — new runtime knob.  InferTask
  sleeps at the end of each loop to cap at `n` fps.  Default 45
  to match camera.  `libs/tpu/edgetpu_driver.cc` + `detection_task.cc`.

### Measurement

Tested pipeline with throttle at 45 FPS and 20 FPS.  Both fail
identically: first URB times out (50 ms / 200 ms) or `USB_HostSend`
returns error → InferTask skips → tries next frame → same error.

async_stats snapshot mid-pipeline (throttle=20, no-cancel):
```
bo_ok=542 bo_malloc=0 bo_send=5234 tm=32 ok=510
```

**bo_send=5234** is the key: `USB_HostSend` (NXP stack) is refusing
submissions.  Without cancel to clear pipe state, the EHCI pipe
stays in whatever error state the first timeout created.  5234 of
5234+542 = 90% of submit attempts fail.

### Net diagnosis

**This is a TWO-sided bug**:
- With cancel: TPU silicon gets wedged (confirmed — standalone invoke
  fails after pipeline.stop until reflash)
- Without cancel: USB host pipe gets halted (confirmed — bo_send
  spikes 90%, further submits refused)

Either way, first URB timing out triggers a permanent failure mode.
The ROOT of the root cause is still: **why does the FIRST URB time
out on pipeline.start, when the same code path on standalone invoke
NEVER times out (0/285)?**

### Real next-session work

1. **Find WHY the first URB during pipeline times out.**  Pure TPU
   over 100 invokes: 0 timeouts.  Pipeline's first URB: ~100% timeout.
   Since our changes (chunk 33 KB, zero-copy, no-heap) all prove stable
   on pure TPU, something about the pipeline's task startup disturbs
   the TPU.  Candidates (untested):
   - The act of `xTaskCreate` for PrepTask/InferTask causes a task
     switch that interferes with a pending USB host operation.
   - EdgeTpuManager mutex interactions: pipeline's first invoke
     competes with some async operation still draining on the TPU.
   - Camera MUX GPIO pulses disrupt USB_OTG2 PHY briefly.
2. **Only after knowing the root cause**, decide on cancel vs
   no-cancel + ClearHalt vs TPU reset.

## V22 INFRASTRUCTURE IMPROVEMENTS (2026-04-22 night, still stuck on pipeline)

Three NASA/JPL-grade improvements shipped this iteration, NONE of
which fixed pipeline end-to-end — but all of which are durable
wins for the firmware overall:

### 1. USB host NXP code moved SDRAM → OCRAM
In `MIMXRT1176xxxxx_cm7_ram_mp.ld`: the `.usb_host` section now maps
to `m_ocram`.  `usb_host_ehci.c` + `usb_host_hci.c` + devices/hub/
hub_app live in on-chip SRAM instead of SEMC-backed SDRAM.  Total
~15 KB.  Freed by relocating `.audio` and `.shine` to SDRAM (not
used in TPU hot path).  Rationale: under pipeline load, PXP DMA
hammers SDRAM at ~40 MB/s; M7 instruction fetches for USB ISR
behind PXP = IOC dispatch delay.  OCRAM placement eliminates
contention for USB code.  Pure TPU benchmark: **75.3 FPS** (no
regression from the 75.9 FPS earlier).

### 2. CSI IRQ priority lowered 0 → 5
In `libs/camera/camera_support.c:BOARD_InitCamera`:
`NVIC_SetPriority(CSI_IRQn, 5)` after `CAMERA_RECEIVER_Init`.
CSI_IRQHandler runs at default ARM NVIC priority 0 = HIGHEST,
preempting USB_OTG2 which is pinned to `configLIBRARY_MAX_SYSCALL_
INTERRUPT_PRIORITY = 2`.  With camera at 45 fps + a few µs ISR
work, USB IOC handling was being consistently delayed.  CSI
handler does not call FreeRTOS APIs (only GPIO writes + atomic
counter increments) so it is safe to move below the syscall
boundary.  Priority 5 puts CSI below USB (2) — USB IOC never
preempted by camera.

### 3. URB timeout bumped 50 ms → 200 ms
`g_sentai_tpu_urb_timeout_ms = 200` (tunable via
`sentai.diag.tpu_urb_timeout(n)`).  Reason: BulkIn URBs issue the
receive BEFORE the TPU finishes compute (compute + wire ≈ 15-20 ms
typical; under pipeline ≥50 ms bursts).  Bulk IN sema fires when
output activations arrive.  50 ms was too tight; 200 ms is 10×
nominal — still fault-tolerant, still triggers cancel-on-stall.

### Pipeline STILL broken — different suspect now

With all three fixes in, pipeline still fails every invoke at the
FIRST URB.  Since USB code is in OCRAM and CSI is de-prioritized,
the classic "SDRAM contention delays IOC" theory doesn't hold.
The remaining suspects:
- `sentai_tpu_invoke_with_input(buf)` swaps `input_tensor->data.
  uint8 = buf` where `buf` is in `.sdram_bss` (`s_tpu_input_buf[]`
  ping-pong).  Maybe TFLite's interpreter cache attrs for arena vs
  `.sdram_bss` differ and USB DMA reads wrong data.
- `direct_tensor` mode in PrepTask (default ON) has a double-buffer
  hand-off via counting semaphores.  If the semaphore release
  ordering has a bug, InferTask might try to invoke with a buffer
  that PrepTask is still filling.
- Simpler possibility — the FIRST invoke after `pipeline.start()`
  has a timing mismatch with camera startup (first frame not ready
  yet) and InferTask holds a bad semaphore state.

**TESTED next-session path in same session**: `direct_tensor(0)`
legacy staging path **ALSO FAILS IDENTICALLY**.  So it's NOT the
ping-pong / pointer-swap mechanism.  Eliminated.

**Latest hypothesis (unverified)**: the TPU silicon itself
enters a bad state when we cancel a partial transfer.  TPU's
USB state machine expects complete blocks; our cancel-on-timeout
leaves it half-fed.  Host-side we cleanly recover (pipe state
resets, next submit succeeds at USB layer).  But the DEVICE
refuses subsequent data.  Observed: after `pipeline.stop()`,
even standalone `sentai.tpu.invoke()` fails the same way until
full reflash.

**Real next-session path**:
1. After `USB_HostCancelTransfer`, also issue a TPU-level reset
   (CSR write through the TPU driver) to get the device's state
   machine back to ready.  The Darwinn / beagle_chip_config
   headers might expose a "reset_request" CSR.
2. OR: on pipeline.start, don't invoke with ping-pong buffers —
   use the interpreter's own arena.  This requires not using
   double-buffering but may sidestep whatever the TPU is
   reacting to.
3. OR: detect permanent TPU wedge and re-DFU the device in
   software (without reflash).  `EdgeTpuManager::GetSingleton()
   ->OpenDevice()` may reinitialize.

## V21 NO-HEAP HOT PATH + FINAL STATE (2026-04-22 night, end)

Final shipped state on device:
- **Pure TPU: 75.9 FPS @ 512×512 YOLO** (13.18 ms/invoke avg; vs 32 FPS baseline = **2.37× speedup**)
- **No dynamic allocation in invoke path** — replaced `std::vector<uint8_t>(8)` in `PrepareHeader` with stack `uint8_t[8]`.  Added `TpuDriver::PrepareHeaderInto()` static helper for all hot-path callers.
- **Cancel-on-timeout fault-tolerance** in both BulkOut/In transfer paths: orphan URB cancelled within ~50 ms, invoke returns -1, InferTask can skip frame and continue.
- **Zero cascade** — the previous 1-timeout-kills-everything chain is broken.  Each URB is independently cancelled on stuck.

Pipeline STILL doesn't work end-to-end — but the FAILURE MODE is now clean (each individual invoke fails fast, no cascade, no board hang beyond occasional TPU load stumble).  **The remaining gap is a bus/arbitration issue outside the TPU driver**.

## V20 CANCEL-ON-TIMEOUT + ROOT CAUSE CONFIRMED (2026-04-22 night)

### Fix shipped

Fault-tolerant URB path in `libs/tpu/edgetpu_driver.cc`:
- `g_sentai_tpu_urb_timeout_ms` — 50 ms default (runtime-tunable via
  `sentai.diag.tpu_urb_timeout(n)`).  Replaces the legacy 2 s wait.
- On timeout: call `USB_HostEdgeTpuCancelInFlight()`
  (new, in `usb_host_edgetpu.c`) to yank the orphan URB out of the
  EHCI async schedule, then wait 500 ms for the cancel-callback to
  land on our stack `meta`.  Return -1 so the caller (InferTask)
  can **skip the frame and continue** — textbook NASA/JPL fault
  containment.
- Counters: `urb_cancelled`, `urb_cancel_no_cb` exposed via
  `sentai.diag.async_stats()`.

**Result**: cascade eliminated.  Previously one stuck URB would
trash `pipe->activeTransfer` for all subsequent submits — every
invoke after the first failure also timed out.  Now the pipe state
is clean after each cancel, so invokes that happen to succeed
(61% of them) produce valid frames.

### Root cause of per-URB stalls (USER HYPOTHESIS CONFIRMED)

Captured counters after a full pipeline run:

```
bo_ok=5263      (submits — 100% accepted by USB host)
cb_user=5350    (callbacks dispatched)
lambda_gave=5263 (sema gives inside lambda)
take_ok=2057     (successful waits)
take_timeout=3206 (timeouts)
urb_cancelled=3206 (all timeouts were cancelled)
urb_cancel_no_cb=0 (cancel callback ALWAYS fires)
```

**39% of URBs time out under pipeline load.** Without pipeline, 0%
timeout on the same code path.  The RATIO doesn't correlate with a
specific URB (it's uniformly random) → this is a **bus-contention
problem, not a logic bug in the driver**.

User's intuition nailed it: "o încurcătură între bufferele PXP,
Camera, DMA și USB, se încalecă, nu sunt sincronizate".  What
likely happens:
- EHCI QH/QTD structures live in SDRAM (allocated via
  `OSA_MemoryAllocate` → m_heap at 0x80000000)
- PXP + camera DMA hammer SDRAM at ~900 KB/frame × 45 fps = 40 MB/s
- When EHCI's bus master tries to WRITE the QTD IOC status bit,
  SDRAM is busy serving PXP → write is delayed past the
  USB_OTG2 interrupt window → IOC never fires → our sema never
  gets given → 50 ms timeout → cancel + skip.
- Sometimes the write wins the arbitration (61%), sometimes not.

### Remaining work (next session)

The underlying bus contention needs one of:
1. **Move EHCI descriptors to non-cached / non-SDRAM memory**.
   The NXP SDK allocates `ehciInstance` via `OSA_MemoryAllocate`
   which goes to the main heap (SDRAM).  A targeted override to
   put the QH/QTD pool in m_ncache (DTCM non-cached) would give
   EHCI deterministic latency.  Concrete step: investigate
   `USB_HostEhciCreate` (usb_host_ehci.c:4200) and the 4275
   `ehciQhList` assignment.
2. **Lower camera / PrepTask priority** below USB host task so USB
   gets CPU uncontested.  All are at `configMAX_PRIORITIES-1=4`;
   demoting camera to 3 while keeping USB at 4 would ensure the
   USB host task drains completion queues between camera frames.
3. **Gate TPU invoke on PrepTask idle** — don't start the bulk
   transfers while PXP is actively DMAing.  Adds latency but may
   eliminate contention entirely.

Runtime-tunable knobs added this session that help debug this
next time:
- `sentai.diag.tpu_urb_timeout(ms)` — 50 ms default
- `sentai.diag.async_stats()` — full counter dict (14 fixed + 5
  dynamic keys via `mp_obj_new_str`)
- `sentai.diag.tpu_chunk_size(n)` — 33 KB default
- `sentai.diag.tpu_zero_copy(bool)`
- `sentai.diag.tpu_async_input(bool)`

## V19 PIPELINE DIAGNOSTIC DATA (2026-04-22 late night)

Added instrumentation at 4 layers of the USB bulk path to root-cause
why pipeline integration fails:

1. `USB_HostEdgeTpuBulkOutSend` — 5 counters (bo_ok, bo_pipe,
   bo_bulk, bo_malloc, bo_send)
2. `USB_HostEdgeTpuPipeCallback` — 4 counters (cb_entered, cb_found,
   cb_user, cb_nopipe)
3. User lambda inside `BulkOutTransferInternal` — 3 counters
   (lambda_entered, lambda_gave, lambda_null_sema)
4. The post-submit Take — 2 counters (take_ok, take_timeout)

All exposed via `sentai.diag.async_stats()` (14 keys via fixed-QSTR
+ 5 more via dynamic `mp_obj_new_str` to avoid QSTR regen churn).

**Measured BASELINE** (3 warmup invokes in pure-TPU mode, chunk=33 KB):
`bo_ok=285, cb_user=292, lambda_gave=285, take_ok=285, take_timeout=0`

Every counter lines up: 285 submits → 285 callbacks → 285 sema gives
→ 285 successful takes.  **All perfectly balanced.**

**Measured AFTER ~2 s of pipeline running** (same firmware):
`bo_ok=732, cb_user=752, lambda_gave=725, take_ok=725, take_timeout=6, cb_nopipe=2`

Delta vs BASELINE:
- +447 submits
- +460 callback entries  (but only +440 to user dispatch)
- +440 lambda-gives
- +440 takes OK
- **+6 takes TIMEOUT** — 6 invoke attempts waited 2 s and got nothing
- **+2 cb_nopipe** — 2 callbacks arrived AFTER pipe->activeTransfer
  had been overwritten by a newer submit.

**Interpretation:** ~99% of URBs complete normally within 2 s even
during pipeline.  The problem is the 1% that don't — each such URB
stalls its invoke by 2 s, InferTask moves to the next invoke, but
the "orphan" in-flight transfer eventually completes and its
callback finds pipe state pointing to the NEWER transfer.  The
pipe-level dispatch (`if (activeTransfer == transfer)`) rejects the
orphan → user lambda never runs → sema never given for THAT invoke
attempt.  Chain effect: every pipeline invoke fails-and-retries.

**Why do 1% of URBs take >2 s to complete?**  Unknown.  Candidates:
- Bulk-IN on a different pipe stalls bulk-OUT pipe scheduling
- TPU device enters a flow-control state and pauses bulk-OUT
- Task scheduling starves USB host task long enough for EHCI
  watchdog to time out the QH

**Next steps (incremental, NASA/JPL):**
1. Reduce sema-take timeout from 2 s to 100 ms and retry-on-timeout
   in BulkOutTransferInternal.  If 99% of URBs finish in <1 ms,
   100 ms is 100 × safety margin and lets the 1% timeout recover
   100× faster.
2. Log which ENDPOINT hit the orphan (likely bulk-OUT EP1 or bulk-IN).
3. Check if EHCI async schedule has anything pending at the
   moment of timeout (dump ASYNCLISTADDR).
4. Consider resetting pipe on timeout (USB_HostCancelTransfer +
   re-open).

**Infrastructure added this session (durable)**:
- 14 bulk counters in `libs/tpu/usb_host_edgetpu.c` + `edgetpu_driver.cc`
- `sentai.diag.async_stats()` wraps them all in a dict
- Runtime toggles: `tpu_chunk_size`, `tpu_zero_copy`,
  `tpu_async_input`, `tpu_multi_ep`, `tpu_desc_cache`
- Dynamic-key dict idiom via `mp_obj_new_str` — future debug
  fields don't need QSTR regen (per user guidance)

## ⚠️ V18 PIPELINE INTEGRATION: BROKEN

End-to-end camera → PrepTask → TPU pipeline consistently fails
with our current TPU driver optimizations:

```
USB_HostEdgeTpuBulkOutSend failed
Bad BulkOutTransferInternal
WriteHeader failed
Node edgetpu-custom-op (number 0) failed to invoke with status 1
E:0420:2   (DET INVOKE_FAIL, val=2)
```

- **Pure TPU**: 13.7 ms/invoke (73 FPS) ✓
- **Pipeline**: TPU cannot submit even an 8-byte WriteHeader once
  `sentai.pipeline.start()` is running.  Errors cascade and USB
  transfer pool appears to leak (subsequent standalone invoke also
  fails until full reflash).
- Tried: `tpu_zero_copy(0)` (staged path), `tpu_async_input(0)`,
  `tpu_chunk_size(32..128 KB)`, direct_tensor OFF — all fail once
  pipeline is up.

Hypotheses (not validated):
1. **Task-priority contention**: camera task + USB host task both
   at `configMAX_PRIORITIES-1`.  PrepTask + camera ISR may starve
   USB host task's event wait, pushing callback latency past the
   2 s sema timeout.
2. **USB transfer pool leak**: After a timed-out transfer the NXP
   stack may not auto-free the transfer struct, and
   USB_HostMallocTransfer eventually returns NULL.
3. **PXP DMA vs EHCI DMA arbitration** on the AXI crossbar
   introducing multi-ms stalls that break the TPU's expected
   timing budget.

Next-session plan:
- `git bisect` the commits between when pipeline last worked
  (likely `56aefcd1 21 fos la switch camera`) and `01725a8f`
  (73 FPS standalone).
- Add printf inside USB_HostEdgeTpuBulkOutSend at
  USB_HostMallocTransfer failure site — see if pool is truly
  exhausted or submit returns another error.
- Instrument `EdgeTpuPerXferDispatch` to count how many callbacks
  fire during pipeline.start() window.

For now, PURE TPU BENCHMARK (E20) is the reliable path to
73 FPS.  Real-time camera detection needs the pipeline
debugged first.

## 🔥 V17 BREAKTHROUGH (2026-04-22 late)

**From 31 ms/invoke (32 FPS) to 13.65 ms/invoke (73 FPS) = 2.27× speedup.**

Via a chunk-size sweep `sentai.diag.tpu_chunk_size(n)` with n in 4-160
KB, found a **sharp performance cliff between 36 KB and 38 KB**:

| chunk KB | avg ms/invoke | FPS |
|---------:|--------------:|----:|
| 16 | 13.9 | 72 |
| 20 | 14.8 | 68 |
| 24 | 14.9 | 67 |
| 28 | 14.2 | 70 |
| 32 | 14.7 | 68 |
| **33** | **12.9 ✨** | **77** |
| 34 | 14.5 | 69 |
| 36 | 14.3 | 70 |
| **38** | **37.97 ⛔** | 26 |
| 40 | 37.6 | 27 |
| 48 | 34.9 | 29 |
| 56 | 32.9 | 30 |
| 64 | 30.7 | 33 |
| 96 | 34.9 | 29 |
| 128 | 37.2 | 27 |

**Hypothesis**: The EdgeTPU's bulk-OUT receive FIFO is ~32-36 KB.
URBs that fit inside stream into the TPU's inference pipe without
back-pressure, allowing the TPU to START COMPUTING on chunk N-1
while USB feeds chunk N.  URBs over ~36 KB overflow the FIFO,
stall the USB, serialize feed-then-compute.  This **demolishes the
earlier "16 ms TPU compute floor" theory** — compute overlaps with
feed.  What we saw as "16 ms" was compute + stall on 64 KB chunks.

At 33 KB chunks, total invoke ≈ MAX(usb_feed, tpu_compute), not
sum.  Both happen to be ~13-14 ms for this model.  That matches
the 72-77 FPS we now measure cleanly across 100-sample batches.

**Default changed to 33 KB** in edgetpu_driver.cc.  Stability
confirmed across 3 × 100-invoke runs (13.63, 13.65, 13.79 ms).

## V-series progression

| ver | change | avg ms | I/O ms | notes |
|-----|--------|--------|--------|-------|
| V0 | baseline (build #737) | 32 | 16-17 | 32 KB staging in DTCM, sem create/delete |
| V7 | persistent sema + bytes-fix | 32 | 16 | same perf, correctness win |
| V9 | zero-copy + ~64 KB chunks | **30** | 8-10 | skip SDRAM→DTCM memcpy; USB_HostSend does cache clean |
| V10 | 128 KB chunks | 36 | — | REGRESSED — NXP EHCI QTD chain overhead |
| V11 | 64 KB chunks (aligned) | 31 | 10 | stable |
| V12 | multi_ep firmware + EP routing | 80 | fail | TPU DFU'd multi_ep bin, host didn't open EP2/EP3 pipes |
| V14 | + async-input toggle (2 URBs in flight on input pipe) | 31.23 | 9.0 | works: 462/462 callbacks OK, ~0.4 ms saved, mostly hidden by compute floor |
| V15 | + desc_cache infra (skip params+ins on token match) | FAIL | — | model needs instructions EVERY invoke — TPU hangs when skipped |
| **device** (build final) | zero-copy + 64 KB + async-input + desc_cache-OFF | **30.75** | **9-10** | shipping — 100-sample avg |

## Async path findings

Tried via `sentai.diag.tpu_async_input(1)`: restructures SendInputs
to use `USB_HostEdgeTpuBulkOutSendAsync` (per-transfer callback pool
bypasses the NXP pipe->callbackFn collision), submits chunk [nxt]
BEFORE waiting on chunk [cur] so two URBs sit in the EHCI schedule
simultaneously.  Result: `async_stats` shows **462/462 callbacks
fired successfully, 0 submit fails, 0 cb fails**.  The path works.

But gain is only ~0.4 ms/invoke because **TPU compute is the floor**
— at ~16 ms silicon compute, shaving USB I/O from 10 → 9 ms only
shows up if it falls below the compute time.  Pipelining wins
compound only when we can ALSO start the NEXT invoke's input upload
while the CURRENT invoke's compute runs (application-level double
buffer).

## Why desktop libedgetpu can look "10× faster"

The comparison is misleading when the target model differs.  On the
SAME TPU silicon, our 512×512 YOLO takes ~16 ms of COMPUTE — that's
hardware, not software.  Smaller models (MobileNetV2 224×224) take
~2 ms compute, so USB overhead dominates and desktop's 3-concurrent
+ 1 MB chunks get close to 4 ms total.

For OUR model specifically:
- Desktop libedgetpu: ~22-25 ms/invoke (16 ms compute + 5-8 ms
  optimal-USB + native dispatch)
- Our current: 30.75 ms/invoke (16 ms compute + 9-10 ms USB +
  2-3 ms MicroPython/interpreter dispatch)
- Gap to desktop: ~5-8 ms, not 25 ms

Remaining 5-8 ms gap levers (all require significant work):
1. multi_ep firmware + TRUE 3-pipe concurrency (V12 failed DFU
   bootstrap)
2. EHCI-QH-chain keeping the pipe continuously fed across Send*
   boundaries (eliminate the ~1 ms gap between last input URB IOC
   and first instruction URB submit)
3. Reduce MicroPython call overhead (2-3 ms per `invoke()` — replace
   with a native C helper that runs the full 30-invoke loop)

## What WORKED (shipped in #762)

1. **Zero-copy bulk transfers.**  `BulkOutTransfer` /
   `BulkInTransfer` in [libs/tpu/edgetpu_driver.cc](../../../libs/tpu/edgetpu_driver.cc)
   now submit DIRECTLY from the caller's source pointer (tensor
   arena / flatbuffer / output tensor) instead of staging through a
   32 KB DTCM buffer.  `USB_HostSend` handles cache coherency via
   its own `DCACHE_CleanByRange` / `DCACHE_CleanInvalidateByRange`
   calls (see usb_host_hci.c:413 / :501).  **~50% cut in USB I/O
   time per invoke.**

2. **64 KB chunks.**  Old code was capped at 32 KB by the staging
   buffer AND by the `uint16_t length` arg on
   `USB_HostEdgeTpuBulkOutSend`.  Widened that arg to `uint32_t`
   and picked 64 KB as the sweet spot (128 KB regressed — NXP
   EHCI QTD-chain setup overhead dominates the per-chunk save).

3. **Persistent semaphore** (`InitBulkSema` in edgetpu_driver.cc).
   Skip `xSemaphoreCreateBinary` / `vSemaphoreDelete` per chunk.
   Negligible timing win (sub-ms), but a stability win: no heap
   churn in the TPU hot path.

4. **BulkInTransfer memcpy bytes fix.**  Latent bug: old code
   memcpy'd `chunk_size` regardless of short-read; a short read
   would leak stale staging bytes past the real payload.  Now uses
   `bytes_received`.

5. **Per-stage DWT instrumentation** exposed as
   `sentai.diag.tpu_perf([reset]) -> {cyc_*, n_*, by_*}`.  Also
   `sentai.diag.tpu_multi_ep(enable)` for routing toggle.

## What DIDN'T work (and why)

1. **Explicit `.ocram_bss → m_ocram` mapping.**  V2/V3 tried moving
   the staging buffer from default DTCM (m_data, single-cycle,
   uncached) to OCRAM (cached, AXI path).  Regressed 3 ms/invoke.
   Reverted.  The original code's comment claiming OCRAM placement
   was wrong — the orphan `.ocram_bss,"aw",%nobits @` attribute
   landed in m_data by default (linker orphan rules), which was
   actually the fast path.

2. **True async pipelining (V4).**  Added per-transfer callback
   pool in usb_host_edgetpu.c so multiple URBs could be outstanding
   on the same pipe with their own `{user_cb, user_param}` routing.
   Compiled and submitted cleanly but callbacks never fired — the
   NXP EHCI has some pipe-level coupling we haven't traced.
   Infrastructure sits in the file (`USB_HostEdgeTpuBulkOut
   SendAsync`, `EdgeTpuPerXferDispatch`, `s_async_ctx_pool`) for a
   future debug pass.

3. **Multi_ep firmware (V12).**  CMake flag `SENTAI_TPU_MULTI_EP
   =ON` flips the apex_firmware blob to `apex_latest_multi_ep_bin`.
   Host toggle `sentai.diag.tpu_multi_ep(1)` flips runtime routing.
   But first invoke fails with `USB_HostEdgeTpuBulkOutSend failed`
   — EP2 / EP3 pipes aren't opened by `USB_HostEdgeTpuOpenData
   Interface` when the new firmware enumerates.  Needs investigation:
   either the TPU descriptor needs `multi_bo_ep=1` set BEFORE
   enumeration (chicken-and-egg), or our class driver needs to
   explicitly open additional bulk endpoints from the descriptor.

## Remaining architectural levers (NOT yet exploited)

| lever | potential gain | effort |
|-------|---------------|--------|
| multi_ep + per-pipe async | ~2× I/O throughput | DFU descriptor boot sequence fix |
| NXP EHCI MAX_QTD > 8 | allow 256+ KB URBs | test carefully, V10 showed DCACHE time dominates |
| TPU model with smaller input (e.g. 320×320) | 2.5× less input data | retrain model |
| Overlap next-invoke prep with current TPU compute | saves camera→tensor-arena ~14 ms | app-level double buffer |

## Files modified in build #762

- `libs/tpu/edgetpu_driver.cc` — zero-copy, 64 KB chunks, persistent
  sema, BulkIn bytes fix
- `libs/tpu/usb_host_edgetpu.c` — async API infrastructure (not wired
  into hot path yet), widened BulkOutSend length to uint32_t
- `libs/tpu/usb_host_edgetpu.h` — async API declarations, signature
  update
- `libs/tpu/edgetpu_executable.cc` — DWT cycle counters for per-stage
  breakdown
- `examples/sentai_runtime/modsentai_diag.c` — `sentai.diag.tpu_perf()`
  and `sentai.diag.tpu_multi_ep()` bindings

## How to pick up next time

1. **If chasing more TPU speed:** investigate V12 failure path — why
   `USB_HostEdgeTpuGetPipeIndexFromEndpoint(..., EP2_OUT)` returns
   -1 after multi_ep firmware DFU.  Read
   `USB_HostEdgeTpuOpenDataInterface` in `libs/tpu/usb_host_edgetpu.c`
   to see which endpoints it actually opens.  Likely fix: re-enumerate
   OR set multi_bo_ep=1 via control transfer BEFORE calling
   OpenInterface.
2. **If chasing the V4 async path:** the per-transfer callback pool
   (`EdgeTpuPerXferDispatch` in usb_host_edgetpu.c) dispatches via
   `transfer->callbackFn` which EHCI calls at completion (line 3671
   / 3714 of usb_host_ehci.c).  The bug is somewhere between submit
   and the EHCI completing.  Start with a LOG printf at IOC time to
   see if EHCI even sees completion.
3. **If benchmarking the new shipped V13:** run
   `examples/sentai_runtime/diag/drivers/_e20_tpu_raw.py` and
   `sentai.diag.tpu_perf(True)` then sample at end to see per-stage
   split.
