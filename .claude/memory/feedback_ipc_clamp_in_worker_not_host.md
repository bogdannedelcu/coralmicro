---
name: ipc-clamp-in-worker-not-host
description: "HARD RULE — when dispatching a sentinel-routed message over IPC (M7↔M4 or any host↔worker channel), input-range clamp logic MUST live in the worker AFTER it has had a chance to recognize sentinel values, NOT in the host before send.  Operator-debugged 2026-05-20 (OP-S10-W18 session) — clamp [3, 511] in sentai_m4_bench_run silently destroyed all sentinel routing (0xCAFE, 0xC100..C108, 0xF10F, 0xF1D7, 0xF1DD), causing every M4 sentinel call to fall into the default ArUco-threshold branch at block=511 and silently report ~25-29 ms for every workload.  Invalidated all W17 M4 ablation numbers."
metadata:
  node_type: memory
  type: feedback
  originSessionId: ec63bebe-b035-4bfc-b74b-fd7091d4aac5
---

## The rule

**If your IPC handler dispatches on a sentinel range that overlaps
the input value space, do the input-range clamp/validation INSIDE
the worker — after the dispatch — never in the host before send.**

If you clamp in the host, sentinel values outside the "normal" input
range get smashed to the boundary and silently fall through to the
default path on the worker side.  The worker thinks it's processing
a normal request; you think you're invoking a special path.  All
results look "normal" — same code path on every call — and you have
no obvious symptom.

## Concrete bug that motivated this rule

`examples/sentai_runtime/m4_bench_host.cc:84-85` (pre-fix):

```c
extern "C" int sentai_m4_bench_run(int block, uint32_t* out_cyc, ...) {
    if (!s_m4_started || !out_cyc) return 0;
    if (block < 3) block = 3;
    if (block > 511) block = 511;   /* ← clamp BEFORE IPC send */
    ...
    app->block = (uint16_t)block;
    coralmicro::IpcM7::GetSingleton()->SendMessage(msg);
}
```

The legitimate ArUco-threshold range is `block ∈ [3, 511]` (Bradley
adaptive threshold block size).  But the worker also dispatches on
much-larger sentinel values to route to different workloads:

| Sentinel value | Intended workload |
|---|---|
| 0xCAFE  (51966) | IPC smoke test |
| 0xC101..C108    | WhyCon Phase A with N disks (N = block & 0xF) |
| 0xF10F  (61711) | Flow SAD exhaustive on shared OCRAM |
| 0xF1D7  (61911) | Flow SAD with OCRAM→DTCM pre-copy |
| 0xF1DD  (61917) | Flow diamond search |

All sentinels > 511.  Host clamp → block=511 over the wire → worker's
sentinel checks all fail → falls into the default ArUco-threshold
branch at block=511.  Every "special" workload was secretly running
the same code path.

## How the bug stayed hidden

The default ArUco-threshold path at block=511 on M4 takes ~25-29 ms.
That number was plausible across multiple workpackages and matched
order-of-magnitude estimates for "M4 is slow on bulk pixel pipelines":

- W17-T2  "M4 WhyCon Phase A 25.86 ms"
- W17-T4  "M4 Flow SAD exhaustive 26.51 ms"
- W17-T4  "M4 Flow SAD DTCM 26.09 ms" (the "DTCM = no benefit" finding)
- W18-T1  "M4 Flow diamond 25.93 ms" (the "diamond no help on M4" finding)

All four were the same measurement of `aruco_adaptive_threshold_m4(511)`,
not the intended workload.  The wrongness wasn't obvious because:

1. The numbers were in a plausible range (25-29 ms).
2. They all came from the same code path, so they were self-consistent
   across the chain of ablations.
3. The "M4 is uniformly slow on RT1170" verdict from W16 made all
   four results plausible-looking.

## How to catch it

1. **Print the dispatch flag from the worker** in the response (e.g.,
   `app->n_dets = 2` for diamond, `= 1` for DTCM, `= 0` for the
   default).  Operator instructed adding this immediately after the
   bug surfaced — and it would have caught the bug earlier had it
   been wired through to the MicroPython dict from the start.
2. **Sentinel values should be assertable on the worker** — e.g.
   if the worker only recognizes a finite set, an unknown value
   should produce an error response rather than a silent fall-through
   into a "looks normal" default path.
3. **For ablation matrices**, always include a sanity check: a
   workload-disabled cycle count (the "synth-only no-compute" path)
   to ground-truth the timing infrastructure.

## Related

- W16 ArUco threshold M4 bench used block ∈ {3..511}, NO sentinel —
  those numbers WERE valid (no clamp damage).  The DCE-artefact
  problem from [[op-s10-w16-ablation-findings]] is separate; both
  bugs coexisted in the W16/W17 chain.
- For thesis methodology: "always print the dispatch path from the
  worker side" is a discipline rule that catches both this bug AND
  the DCE-artefact (post-fix nm-check) simultaneously.

## Fix

Remove the clamp from the host.  Worker already has its own clamp
for the legitimate range (`m4_aruco_bench.cc` dispatch defaults to
`if (b < 3) b = 3; if (b > 511) b = 511;` inside the ArUco-threshold
branch).  Sentinel ranges bypass that clamp by hitting their
specific `else if (block == 0xF1DDu)` branches first.

Committed at 5f34a8dc.
