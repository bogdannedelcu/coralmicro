---
name: project_tpu_urb_stack_metadata_crash_w24_t6
description: "OP-S10-W24-T6 (2026-10-31) — TPU URB timeout → late completion wrote a dead stack frame → HardFault/REPL silence; fixed with static URB slots + sequence tokens (bulk, event, CSR), per-model package unregister; fault-injection recipe; tpu.recover() needed after orphans"
metadata:
  node_type: memory
  type: project
  originSessionId: 0213fcbf-a574-4225-a0ea-7da5f47ce784
  modified: 2026-10-31T15:38:32.632Z
---

# TPU URB-timeout crash class (W24-T6), builds 1695–1702, 2026-10-31

**Symptom family:** after a TPU bulk URB timeout (SERR 0x0B60–0x0B63, frame
dropped by design, URB deliberately NOT cancelled because cancelling corrupts
the TPU) the board later HardFaulted (PRECISERR, PC in unrelated code such as
`caam_in_job_ring_add`, garbage r0) or the REPL went silent, or (fault
injection) a `configASSERT` in queue.c fired.  Seen in s257 when a 2.4 MB NCM
model upload preceded the sweep in the same boot.

**Root causes:** (1) `UsbTransferMetadata` lived on the caller's stack and its
address was the EHCI callback param; the orphaned URB completed after the
caller returned and wrote into a dead stack frame.  (2) `CSRTransfer` created a
heap semaphore per call and deleted it on timeout while the control URB stayed
in flight; the late completion gave the freed handle (reused by a queue →
`!(pvItemToQueue==NULL && uxItemSize!=0)` assert).

**Fix (libs/tpu/edgetpu_driver.cc):** static `UrbSlot` per pipe family
(`s_bulk_slot`, `s_event_slot`, `s_ctrl_slot` + static mutex for CSR) and an
opaque sequence token as the callback param; mismatching completions are
counted (`g_sentai_tpu_stale_completions`) and ignored.  Order at every site:
arm (seq++) → drain binary sema → submit.  Observability:
`sentai.tpu.urb_sync_stats()`.  Also: `EdgeTpuManager::UnregisterPackagesInRange`
on model free (package map keyed by buffer address leaked + stale-address
hazard; per-range because slots 1..2 hold raw package pointers).  Assert/CHECK
breadcrumbs now carry expr+file string addresses (`gdb x/s`), `CHECK` is one
out-of-line call (`sentai_check_fail`).

**Fault-injection recipe (reusable):** `sentai.tpu.chunk_size(160*1024)`,
`sentai.sys.poke32(<&g_sentai_tpu_urb_timeout_ms>, 1)` (the setter clamps to
5 ms), run the pipeline 6–8 s, restore 200 ms / 36 KB.  Expected post-fix:
board alive, `take_failed`/`csr_timeouts` > 0, every invoke 0B62 until
`sentai.tpu.recover()` ("invokes restored without reflash"), then clean.

**Why:** this is the real mechanism behind the long-standing "TPU load → REPL
drops / needs physical POR" lore in [[tpu-usb-wedge-mechanism]]; the orphan URB
was corrupting memory, not just the pipe.
**How to apply:** never hand a stack address to a USB completion callback;
never delete a semaphore a callback may still give.  After any URB timeout the
TPU needs `tpu.recover()`; auto-recovery in the pipeline is W24-T7.  J-Link SWD
attach does not connect on rev-1.1 (W24-T8) — rely on `/log/crash_prev.txt`,
`/log/crash_NNN.log` breadcrumbs and `boot_old.log` "Reset reason".
