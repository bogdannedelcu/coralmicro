# S215 ARM Emulator Flow + Physical TPU Parallel

Goal: run production `FlowTask` and the guest-owned
`EdgeTpuManager -> TpuDriver::Send* -> host POSIX/libusb -> physical USB Coral`
path at the same time in Renode.

This experiment deliberately avoids the POSIX SIM runtime.  The guest binary is
an ARM-emulator firmware target that links SentAI runtime FlowTask code and the
Coral Micro TPU manager/executable path.  The host side only bridges the
`TpuDriver::SendParameters`, `SendInputs`, `SendInstructions`, `GetOutputs`, and
`ReadEvent` boundary to the physical Coral.

Run:

```bash
python3 examples/sentai_runtime/experiments/s215_arm_emulator_flow_tpu_parallel/run_s215.py
```

Expected pass criteria:

- FileX/NAND staging has model, cat image, and mission assets.
- FlowTask validates unequal X/Y offsets and measures throughput.
- TPU path performs repeated physical Coral invokes.
- Both tasks complete and set `parallel_done=1`.
