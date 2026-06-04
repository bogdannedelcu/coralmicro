# S214 ARM Emulator FlowTask

Goal: run the production `examples/sentai_runtime/flow_task.cc` in the ARM
emulator and measure the FlowTask-only path before reintroducing virtual
camera, PrepTask, InferTask, or TPU.

The benchmark feeds `SENTAI_PREP_SLOT_FLOW_GRAY_80x60` directly from a guest
FreeRTOS task.  This keeps the boundary identical to the runtime prep fan-out
slot, while isolating FlowTask scheduling and the ARM phase-correlation
implementation.

Run:

```sh
python3 examples/sentai_runtime/experiments/s214_arm_emulator_flow_task/run_s214.py
```

Each run creates `iterNN_renode_flow_task_runtime/` with:

- `uart.log`: guest UART output
- `renode.log`: Renode monitor output
- `verdict_s214.json`: parsed pass/fail, detected offset flag, and FPS
- `assets/`: exact 80x60 input used by the guest
- `renode/`: script snapshot used for the run
