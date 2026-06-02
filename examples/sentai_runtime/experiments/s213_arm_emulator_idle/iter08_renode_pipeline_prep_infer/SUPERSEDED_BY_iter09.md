# Superseded by iter09

iter08 ran the same B8.6 firmware but with the second-stage task named
`InferTask` and UART markers labelled `INFER`.  Operator note 2026-06-02:
"Infer foloseste TPU si e mai complicat" — i.e. naming the stage InferTask
implies the EdgeTPU dependency, which is on the explicit defer list for
the emulator.

The B8.6 verdict logic is unchanged: PrepTask produces a scalar slot, a
downstream task consumes it, and both stages must move on every IRQ.
iter09 renames the second stage to `FlowTask` and the marker prefix to
`FLOW` to match what the emulator can actually validate without
modelling USB Coral EdgeTPU.

Both runs reached PASS, so iter08 is preserved as the naming-only
predecessor — no firmware regression between the two.
