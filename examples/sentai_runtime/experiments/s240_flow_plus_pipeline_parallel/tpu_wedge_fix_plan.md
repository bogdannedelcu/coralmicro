# TPU-USB wedge fix — PLAN (PMIC power-cycle recovery)

Anchor: TD-S10-A10 · follows tpu-usb-wedge-mechanism (memory) + this experiment's
m4_offload_review.md. Operator insight (2026-07-01): **the TPU can be power-cycled
via the PMIC/GPIO rail** — so instead of the risky USB-level reset (#3, which can
corrupt the TPU silicon), do a clean HARDWARE power-cycle + re-enumerate + reload.

## Root cause recap
Sustained TPU load saturates the SEMC/AHB bus (PrepTask PXP writes + TPU USB bulk
reads) → the 200 ms per-URB timeout fires → the orphaned-URB stale-give cascade
wedges the TPU → the stalled USB-host task + bus saturation stall the CDC-ACM DMA →
REPL drops, board "vanishes". Cancelling the URB corrupts the TPU silicon (needs
reflash), so the ONLY safe recovery is a full device reset — and a PMIC power-cycle
is the cleanest form of that.

## The recovery primitives already exist
- **Power rail**: `EdgeTpuTask::SetPower(bool)` toggles GPIO `kEdgeTpuPmic`
  (power-enable) + `kEdgeTpuReset`, waits `kEdgeTpuPgood` (edgetpu_task.cc:132,
  HandleSetPowerRequest). Refcounted via `enabled_count_`.
- **Auto re-enumeration**: when the TPU powers on, the USB host fires
  `kUSB_HostEventAttach` → kAttached → enumerate → kConnected
  (edgetpu_task.cc:50-121). No manual USB work needed.
- **Re-open + reload**: `sentai_load_model(path)` (sentai_slow_bridge.cc:64) already
  RE-RUNS `OpenDevice()` when `g_tpu_context` is null (lines 79-97), rebuilds the
  resolver (AddCustom edgetpu op) and the `g_interpreter`, re-uploading the model
  executable to the freshly-powered TPU.
- Context/interpreter globals: `g_tpu_context` (shared_ptr<EdgeTpuContext>),
  `g_interpreter` (MicroInterpreter*), `g_tpu_ready` (sentai_runtime.cc:907-910).
  Releasing `g_tpu_context` drops its power ref (dtor → SetPower(false)).

## Implementation

### Step 1 — remember the loaded model path (for reload)
In `sentai_load_model()`, store the last-loaded path in a global
`g_current_model_path[]`. Recovery reloads from it.

### Step 2 — `sentai_tpu_recover()` (core recovery)
New C function (sentai_slow_bridge.cc):
1. `g_tpu_ready = false` (block invokes during recovery).
2. Tear down: `delete g_interpreter; g_interpreter = nullptr;` free model vector;
   `g_tpu_context.reset()` (drops the power ref → dtor cuts power if last holder).
3. **Power-cycle**: `EdgeTpuTask::SetPower(false); vTaskDelay(50);
   SetPower(true); vTaskDelay(250);` — force the rail off then on; PGOOD wait is
   inside SetPower; the 250 ms lets USB re-enumerate to kConnected.
4. `return sentai_load_model(g_current_model_path);` — re-OpenDevice + rebuild
   interpreter + re-upload model. On success `g_tpu_ready = true`.

### Step 3 — MicroPython binding `sentai.tpu.recover()` (manual trigger, for test)
Lets us validate the recovery on the bench: wedge the TPU, call
`sentai.tpu.recover()`, confirm invokes work again WITHOUT reflash.

### Step 4 — auto-detect + auto-recover (after Step 1-3 proven)
In the InferTask invoke path (detection_task.cc), count CONSECUTIVE
`kTfLiteError` invokes; reset the counter on success. When it reaches a threshold
(e.g. 5), request recovery (set a flag; a management context runs
`sentai_tpu_recover` off the InferTask so we don't power-cycle mid-invoke). Log a
SERR. Bounded retries (e.g. give up after 3 recovery attempts → mark TPU dead).

### Step 5 — (orthogonal, later) reduce the SEMC contention that TRIGGERS the wedge
The clean root-cause fix (#2: bulk-IN output staging in OCRAM) is blocked by
OCRAM-full (1.5 KB free). Options for a future WP: free OCRAM (the 900 KB c2f_thick
input dominates — a smaller-input model would free a lot), or reduce PrepTask
PXP ops/frame. Recovery (Steps 1-4) makes the wedge SURVIVABLE; Step 5 makes it
RARE. Ship recovery first.

## Test plan
1. Build both cores; flash (triggers the FileX-reflash bug → reformat + re-upload
   model via MSC, see [[reflash-corrupts-filex-littlefs-mismatch]]).
2. Load model, run detection until it wedges (invoke fails / CDC still alive if
   Step 4 keeps the InferTask from starving it).
3. Call `sentai.tpu.recover()` → confirm `g_tpu_ready` back, invokes succeed,
   frame_count advances again — all WITHOUT reflashing.
4. With Step 4: confirm auto-recovery fires after N failures and detection resumes.

## Risk
Power-cycling the TPU mid-pipeline is much safer than a USB cancel (no silicon
corruption). Main risks: (a) refcount underflow if other holders exist — sentai has
a single TPU user (the pipeline), so `g_tpu_context.reset()` drops to 0; (b) USB
re-enumeration timing — the 250 ms delay + `sentai_load_model`'s 15× OpenDevice
retry loop cover it; (c) doing recovery from the wrong task context — run it from a
REPL/management context, never from inside a USB-host callback.
