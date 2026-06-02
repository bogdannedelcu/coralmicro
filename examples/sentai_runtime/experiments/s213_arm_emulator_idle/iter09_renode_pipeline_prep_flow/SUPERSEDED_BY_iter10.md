# Superseded by iter10

iter09 used the names `PrepTask` and `FlowTask` for the two emu stages.
Operator note 2026-06-02: those names belong to existing production code
(`examples/sentai_runtime/detection_task.cc`,
`examples/sentai_runtime/flow_task.cc`), which does real PXP / quant /
USADA8 / phase-correlation work that the emu spike does not implement.

iter10 renames both stages to `Stage1Task` / `Stage2Task` and updates the
UART marker prefix to `STAGE2` (was `FLOW`).  The verdict topology is
unchanged; both runs PASS with `irq_count == stage1_processed ==
stage2_consumed == 5` and `last_sum == 320`.  No firmware regression.

This SUPERSEDED note + the iter08 SUPERSEDED note together cover the full
naming-history for B8.6:

```text
iter08  PrepTask + InferTask  → operator: TPU implication, drop Infer
iter09  PrepTask + FlowTask   → operator: production already owns names
iter10  Stage1Task + Stage2Task   ← canonical B8.6 PASS
```
