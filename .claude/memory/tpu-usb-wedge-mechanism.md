---
name: tpu-usb-wedge-mechanism
description: "Why sustained EdgeTPU load wedges the TPU + drops the CDC-ACM REPL, and the ranked fix candidates"
metadata: 
  node_type: memory
  type: project
  originSessionId: 218d6445-6c73-4c0f-bb51-4d13943aaeb1
---

**CORRECTION (2026-07-02, build 1576):** on a HEALTHY (physically power-cycled)
board with clean FileX and NO M4 booted, c2f_thick detection runs STABLE at full
rate — 50 s / ~1000 frames / fail=0 / 0 resets. The "wedge after seconds" seen
earlier in the s240 session was a BOARD-STATE artifact, NOT a code bug in the
detection/invoke path: most likely the M4 left spinning on OCRAM from the M4-flow-
offload tests (contending with the TPU input DMA), compounded by degraded state
from dozens of resets + [[reflash-corrupts-filex-littlefs-mismatch]] reformats.
A trace/tpu.trace(1) run is stable too — because the per-invoke printf throttles
the rate (masking any contention). Also confirmed via tpu.trace: **c2f_thick uses
NO host scratch** (zero [scratch]/OUTFEED lines) and its output DMA never overruns
(`dma == alloc`, 38400 + 9600 B) — so the scratch/output-overflow hypothesis does
NOT apply to c2f_thick. The invoke path is heap-clean (no per-frame allocation).
The text below describes the ORIGINAL sustained-load contention theory + fixes,
which remain valid as robustness hardening but are not the cause of this session's
instability.

Under sustained EdgeTPU inference (c2f_thick @ ~27 fps) the board wedges after
seconds: `edgetpu-custom-op failed to invoke with status 1` (TfLite kTfLiteError=1;
SERR 0x0B60-0x0B65 per invoke phase) AND the CDC-ACM REPL drops (host I/O error,
board seems to vanish), sometimes needing a physical unplug/replug.

**Architecture:** TPU is on USB **OTG2 (host)**, CDC-ACM/NCM on USB **OTG1
(device)** — separate controllers, but they share the M7 scheduler, NVIC, and SEMC
SDRAM bus.

**Cascade (root → symptom):**
1. **SEMC contention** (partially mitigated): PrepTask PXP writes to SDRAM +
   TPU USB bulk reads compete for SEMC. At high input bandwidth the SEMC arbiter
   delays the USB bulk-IN → the per-URB **200 ms** timeout
   (`g_sentai_tpu_urb_timeout_ms`, edgetpu_driver.cc:1379) fires.
2. **No URB recovery (deliberate):** on timeout the code does NOT cancel the URB —
   cancelling a partial bulk-OUT corrupts the TPU silicon (confirmed: needs reflash;
   USB_HostEdgeTpuCancelInFlight exists but is never called). It leaves the URB
   "orphaned" in the EHCI schedule. The rationalization at edgetpu_driver.cc:1383-
   1402 (stale give is "idempotent") is subtly flawed under load: the next caller's
   xSemaphoreTake returns EARLY on a stale give and reads the wrong meta.status →
   timeout-on-timeout cascade.
3. **No cross-controller isolation:** when the USB-host task stalls on the timeout,
   it starves the M7 scheduler → CDC-ACM (OTG1) can't be serviced → REPL drops even
   though it's a different USB controller.

Known-hard: some models (e.g. iarna p2p4_5ep) wedge the TPU PERMANENTLY on first
invoke; cross-test contamination within one boot; only reflash recovers (see
agent/experiment.md:587-596, 4920-4924). The [[reflash-corrupts-filex-littlefs-
mismatch]] bug means each recovery reflash also wipes FileX. Repeated reflash/
reformat + thermal degrade the board → the wedge comes faster each cycle (observed
near-instant wedge late in a long session); a physical power-cycle clears it.

**Ranked fixes (see s240 m4_offload_review.md / this investigation):**
- #4 per-frame invoke DEADLINE (~500 ms, drop frame) — low risk, stops the cascade
  from blocking the whole pipeline.
- #2 stage the bulk-IN output into OCRAM then memcpy to arena — low risk, removes
  ~half the SEMC contention that triggers the URB timeout.
- #1 endpoint reset (USB_HostClearEndpointStall) on timeout — medium risk.
- #3 full USB device reset after N failures — high risk (can permanently kill the
  TPU) / high reward (recover without reflash).
Recommended first: #4 + #2 (both low risk, attack cause + prevent cascade). Needs a
HEALTHY (power-cycled) board to test.

Key files: libs/tpu/edgetpu_driver.cc (BulkOut/InTransferInternal, URB timeout
1379/1576), edgetpu_executable.cc:252-483 (Invoke error path), libs/usb/
usb_host_task.cc (OTG2), usb_device_task.cc (OTG1), patches/coralmicro-rt1176-sdk/
0002-ehci-queue-depth-16.patch (QH/QTD 8→16, enables throughput, no stall recovery).
