# TD-S10-B8 - Research And Spike ARM Emulator Runtime

## Status As Of 2026-06-04

| Gate | Topology / proof                                                 | Verdict iter / experiment                        | Commit     |
| ---- | ---------------------------------------------------------------- | ------------------------------------------------ | ---------- |
| B8.1 | CM7 startup -> ARM FreeRTOS scheduler -> heartbeat task          | iter01_renode_idle_heartbeat                     | b314edb2   |
| B8.2 | LPUART6 polled TX file backend                                   | iter03_renode_uart_heartbeat                     | b314edb2   |
| B8.3 | MicroPython embed REPL on LPUART6, `1+1 -> 2`                    | iter05_renode_repl_oneplusone                    | b314edb2   |
| B8.4 | In-firmware VFS + `import mission; mission.run()`                | iter06_renode_mission_import                     | 2400995f   |
| B8.5 | VCam Renode peripheral + NVIC IRQ 94 + frame-bytes consumer      | iter07_renode_camera_5frames                     | 5dd31da4   |
| B8.6 | Stage1Task -> Stage2Task chain behind VCam IRQ                   | iter10_renode_pipeline_stage1_stage2             | 6e8be311 + eb1420bd |
| B8.7 | Stage1Task -> {Stage2A, Stage2B} fan-out with seqlock            | iter11_renode_fanout_stage1_2a_2b                | 64575739   |
| B8.7c| Brute-force SAD flow on a 1-px/frame cat pan (B7 regression)     | iter12_renode_flowest_cat_pan_1px                | abbc7202   |
| B8.7d| Brute-force SAD flow on varied 2D motion across both axes        | iter13_renode_flowest_cat_2d_varied              | 68f86d7a   |
| B8.8a| Production `UsbHostTask` constructor links + returns             | iter17_renode_usbhost_probe_ctor                 | _open_     |
| B8.8b| Production `UsbHostTask::Init()` + scheduler heartbeat           | iter16_renode_usbhost_task_scheduler             | _open_     |
| B8.8c| Production `EdgeTpuManager` singleton over USB host task         | iter21_renode_edgetpu_manager_probe              | _open_     |
| B8.8d| Production `EdgeTpuTask` QueueTask registers on USB host         | iter23_renode_edgetpu_task_probe                 | _open_     |
| B8.8e| `OpenDevice()` no-device/error path from FreeRTOS task           | iter25_renode_edgetpu_opendevice_probe           | _open_     |
| B8.8f| Synthetic Coral attach/enumeration through real callback chain   | iter26_renode_edgetpu_synth_enum_probe           | _open_     |
| B8.8g| Renode MMIO Coral descriptor block through real callback chain    | iter35_renode_edgetpu_mmio_enum_probe            | _open_     |
| B8.8h| `OpenDevice()` after Renode MMIO enum reaches driver init         | iter36_renode_edgetpu_mmio_opendevice_probe      | _open_     |
| B8.8i| `TpuDriver::Send*` MMIO bridge boundary smoke                    | iter37_renode_edgetpu_mmio_send_bridge_probe     | _open_     |
| B8.8j| Physical USB Coral via SIM POSIX/libusb + COCO postprocess       | `build-sim/sim/tpu_posix_invoke_smoke` PASS x3   | _open_     |
| B8.8 | Transparent USB Coral via real `edgetpu_manager` (phased)        | Send* host bridge passed; EHCI passthrough deferred | _open_  |
| B8.9 | Production FileX/LevelX over Renode raw-NAND bridge              | iter38_renode_fx_storage_filex_levelx_nand       | _open_     |
| B8.9b| MicroPython `sentai.fs` over FileX/LevelX raw-NAND bridge        | iter40_renode_fs_repl_filex_levelx_nand          | _open_     |
| B8.9c| Idempotent pre-boot asset staging into FileX/LevelX raw-NAND image| iter54_renode_fs_stage_assets_filex_levelx_nand  | _open_     |
| B8.9d| Post-boot `sentai.fs` readback of staged model/image/mission      | iter43_renode_fs_asset_check_filex_levelx_nand   | _open_     |
| B8.10| Guest FS -> `sentai.tpu` -> host bridge -> physical USB Coral     | iter55_renode_tpu_cat_repl_filex_physical_coral  | _open_     |
| B8.10b| Guest FS -> persistent host bridge -> physical USB Coral FPS      | iter87_renode_tpu_fps_mem_session_repl_filex_physical_coral | _open_ |
| B8.10c| Lower-level TPU transport optimization plan / A/B matrix         | documented after iter60                          | _open_     |
| B8.10d| Low-level timing at SendParameters/SendInputs boundary            | iter83_renode_tpu_timing_repl_filex_physical_coral | _open_   |
| B8.10e| Guest `EdgeTpuManager` -> `TpuDriver::Send*` -> physical USB Coral | iter91_renode_tpu_physical_send_smoke_filex_coral | _open_    |
| B8.10f| Host wall-clock FPS for guest `Send*` -> physical USB Coral       | iter107_renode_tpu_physical_send_fps_filex_coral | _open_    |
| B8.10g| POSIX/libusb bulk-IN outfeed optimization (`0x80` / 1024B)        | iter98_renode_tpu_physical_send_fps_filex_coral  | _open_    |
| S214 | Production `FlowTask` ARM phase-corr over prep slot              | s214 iter02, PASS, 47.61 FPS                     | _open_     |
| S215 | Production `FlowTask` + guest Send* -> physical USB Coral        | s215 iter01/iter02, PASS x2, TPU ~13.2 FPS       | _open_     |
| B8.12| Crazyflie CRTP via real UART + cf2-SITL TCP bridge (planned)     | not started                                      | _open_     |

Run any gate via:

```sh
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py \
    --target {idle,uart,repl,mission,camera,pipeline,fanout,flowest,usbhost_task,edgetpu_manager_probe,edgetpu_task_probe,edgetpu_opendevice_probe,edgetpu_synth_enum_probe,edgetpu_mmio_enum_probe,edgetpu_mmio_opendevice_probe,edgetpu_mmio_send_bridge_probe,fx_storage,fs_repl,fs_stage_assets,fs_asset_check,tpu_cat_repl,tpu_fps_repl,tpu_fps_mem_repl,tpu_fps_mem_invoke_repl,tpu_fps_mem_session_repl,tpu_timing_repl,tpu_physical_send_smoke,tpu_physical_send_fps}
```

The runner produces an `iterNN_*/verdict_s213.json` with per-gate
assertions (counter equalities, UART byte matches, per-frame
detection sequences).  `pass: true` requires every assertion to
hold; partial PASS is not accepted.

Run the newer standalone emulator experiments via:

```sh
python3 examples/sentai_runtime/experiments/s214_arm_emulator_flow_task/run_s214.py
python3 examples/sentai_runtime/experiments/s215_arm_emulator_flow_tpu_parallel/run_s215.py
```

Latest B8.8 USB/TPU emulator slice:

- `iter22_renode_edgetpu_task_probe` intentionally remains as failed
  build-history.  It exposed the endpoint-log hooks pulled in by
  `libs/tpu/usb_host_edgetpu.c`.
- `iter23_renode_edgetpu_task_probe` is the passing P3b verdict:
  production `UsbHostTask::Init()`, production `EdgeTpuManager` singleton,
  and production `EdgeTpuTask::Init()` all return before the ARM FreeRTOS
  scheduler starts; the lower-priority heartbeat continues running
  (`boot_state == 0x600`, `usbhost_step == 11`, `heartbeat == 297` over 3 s).
- `iter24_renode_edgetpu_opendevice_probe` intentionally remains as failed
  build-history.  It exposed that `OpenDevice()` pulls the `TpuDriver`
  initialize symbol even on a no-device probe.
- `iter25_renode_edgetpu_opendevice_probe` is the passing P4a verdict:
  `EdgeTpuManager::OpenDevice()` runs from a FreeRTOS task, sees a controlled
  USB error/no-device state, returns `nullptr`, and the scheduler heartbeat
  stays alive (`boot_state == 0x600`, `usbhost_step == 13`).
- `iter26_renode_edgetpu_synth_enum_probe` is the passing P4b verdict:
  a FreeRTOS task creates a synthetic `usb_host_device_instance_t` with Coral
  VID/PID `18d1:9302`, a vendor-class interface, and the minimum three
  endpoints (bulk IN, bulk OUT, interrupt IN), then injects attach and
  enumeration-done through the real `UsbHostTask::HostEvent()` ->
  `EdgeTpuTask::USBHostEvent()` callback chain.  Scheduler heartbeat remains
  alive (`boot_state == 0x600`, `usbhost_step == 16`).
- `iter27..34_renode_edgetpu_mmio_enum_probe` intentionally remain as failed
  bring-up history for P5a.  They exposed Renode MMIO alignment rules
  (`MappedMemory` base and size must be 0x400-aligned), a brittle
  PythonPeripheral/IRQ path for this Coral scaffold, and a descriptor packing
  bug (`ep_count` must live in bits 16..23 of the interface word).
- `iter35_renode_edgetpu_mmio_enum_probe` is the passing P5a verdict:
  Coral-like VID/PID, vendor-class interface, and endpoint descriptors are now
  stored in a Renode-side MMIO descriptor block at `0x40900400`.  Firmware reads
  that block, constructs NXP USB host descriptor structs, and injects
  attach/enumeration through the real `UsbHostTask::HostEvent()` ->
  `EdgeTpuTask::USBHostEvent()` callback chain.  Scheduler heartbeat remains
  alive (`boot_state == 0x600`, `usbhost_step == 19`).  This is still a
  descriptor-block proof, not an EHCI device model and not physical Coral
  passthrough.
- `iter36_renode_edgetpu_mmio_opendevice_probe` is the passing P5b verdict:
  after the Renode-side MMIO descriptor block is consumed and enumeration is
  routed through the real `UsbHostTask`/`EdgeTpuTask` callback path,
  `EdgeTpuManager::OpenDevice()` runs from a FreeRTOS task, waits for the
  connected class instance, calls the emu-scoped `TpuDriver::Initialize()`
  success stub, and returns a context (`usbhost_step == 22`).  This proves
  manager/task sequencing after an emulator-provided Coral descriptor, but does
  not model real TPU bulk transfers yet.
- `iter37_renode_edgetpu_mmio_send_bridge_probe` is the passing P6a verdict:
  after P5b `OpenDevice()` succeeds, firmware calls the real `TpuDriver`
  transfer boundary methods once each:
  `SendParameters`, `SendInputs`, `SendInstructions`, `GetOutputs`, and
  `ReadEvent`.  Renode's `coral_tpu_bridge` MMIO peripheral at `0x40900800`
  acks the mailbox, reads guest bytes for OUT phases, writes a deterministic
  output pattern for `GetOutputs`, and the ARM FreeRTOS heartbeat remains live
  (`boot_state == 0x600`, `usbhost_step == 30`, all five bridge call counters
  equal 1, output sum `2680`).  This proves the chosen injection point is
  exactly the `TpuDriver::Send*` boundary requested by the operator.  It is
  still not a physical Coral USB transfer and not a COCO-model invoke.
- `build-sim/sim/tpu_posix_invoke_smoke` is the passing physical-Coral smoke
  for P6b/P6c research.  It uses the shared ARM-like TPU path
  (`EdgeTpuManager` -> `EdgeTpuExecutable` -> `TpuDriver`) and injects the
  existing SIM POSIX/libusb transport at `USB_HostEdgeTpu*`; it does not use
  PyCoral.  The smoke parses the COCO EdgeTPU `.tflite`, sends the real
  `edgetpu-custom-op` package to the physical USB Coral, dequantizes the two
  builtin outputs, and runs the bundled TFLite Micro
  `TFLite_Detection_PostProcess` custom op.  On 2026-06-02 it passed three
  consecutive runs with deterministic output checksum `877b9c24` and top
  detection `class=16 score=0.828 box=[0.218 0.166 0.842 0.679]` on the
  COCO cat baseline.  This proves the physical USB Coral path and custom-op
  postprocess are usable without Python; it is not yet wired into the Renode
  MMIO guest/host bridge.
- The P3b stubs are emu-scoped only: cache maintenance, debug-console,
  endpoint-log recording, GPIO/PMIC, and the P4a no-device `TpuDriver` link
  stub are not real peripheral models.  P4b is a firmware-side synthetic
  attach/enumeration proof; P5a/P5b move descriptor data into Renode MMIO and
  reach `OpenDevice()`; P6a moves from driver-init to the `TpuDriver::Send*`
  transfer boundary.  None of these are a Renode EHCI device model or physical
  Coral passthrough yet.  The POSIX SIM libusb branch in `usb_host_edgetpu.h`
  remains distinct from the ARM-emulator NXP USB path via `SENTAI_ARM_EMU`.

Latest B8.9 filesystem/storage slice:

- `iter38_renode_fx_storage_filex_levelx_nand` is the passing storage verdict.
  The firmware target links the production `FxUser*` stack, production
  FileX/LevelX libraries, and production `fx_nand_driver` callbacks.  The only
  emulator-specific injection is below the NAND driver:
  `SENTAI_ARM_EMU_NAND_BRIDGE` redirects raw page read, raw page program, and
  block erase operations to a Renode MMIO peripheral at `0x40900C00`, backed by
  `emu/output/sentai_emu_nand.bin`.
- This intentionally rejects a FileX-in-RAM fixture.  It preserves the ARM
  layering:

  ```text
  sentai.fs / FxUser* -> FileX -> LevelX -> fx_nand_driver -> raw NAND bridge
  ```

  so later REPL uploads, USB MSC behavior, model loading, and image loading can
  use the same runtime APIs and storage semantics as the physical board.
- The smoke performs `FxUserInit(force_format=1)`, `FxUserMakeDirs("/models")`,
  `FxUserWriteFile("/models/smoke.txt")`, `FxUserSync()`, `FxUserSize()`, and
  `FxUserReadFile()`.  Verdict: `boot_state == 0x600`, `fs_size == 41`,
  `fs_read_ok == 1`, `fx_reads == 204`, `fx_writes == 150`,
  `fx_erases == 448`, `fx_errors == 0`, UART contains `FX_STORAGE PASS`.
- Next storage gates should build on this exact block-device layer:
  test the documented small-file REPL uploader (`sentai.fs.append`) and the
  large-file USB-MSC path (`sentai.usb.drive`) without replacing FileX/LevelX.
- `iter39_renode_fs_repl_filex_levelx_nand` remains as bring-up history: the
  MicroPython smoke reached `FS_REPL_DONE`, but the Renode runner was stopped
  manually while the script still had an overly long `RunFor` window.
- `iter40_renode_fs_repl_filex_levelx_nand` is the passing B8.9b verdict:
  the emulator REPL exposes `sentai.fs` over `FxUser*` and executes, inside the
  MicroPython VM:
  `format`, `mkdir`, `write`, `append`, `sync`, `size`, `read_str`, `exists`,
  and `ls`.  UART proves `FS_READ hello world` and
  `FS_LS [('a.txt', 1, 11)]`; Renode symbols report `boot_state == 0x500`,
  `fs_smoke_ok == 1`, `fs_smoke_size == 11`, and `pass: true`.
- `iter41_renode_fs_stage_assets_filex_levelx_nand` is the first passing
  B8.9c full-write verdict: a host-backed Renode asset bridge stages three
  files into the emulated production FileX/LevelX NAND image before the later
  runtime boot: the COCO EdgeTPU model, `cat_640x480.bmp`, and `/mission.py`.
  The firmware writer still uses `FxUser*` and the FileX/LevelX NAND stack;
  only the source bytes come from a Renode MMIO host bridge.  Verdict:
  `boot_state == 0x700`, `stage_files == 3`, `stage_bytes == 8000205`,
  `stage_errors == 0`, `fx_errors == 0`, and `pass: true`.
- `iter54_renode_fs_stage_assets_filex_levelx_nand` is the current B8.9c
  idempotent reboot verdict.  The staging firmware now mounts the existing
  FileX/LevelX NAND image with `FxUserInit(0)` and formats only if mount
  fails.  For each asset it compares the existing guest file size with the
  host asset size; if they match, it logs `FS_STAGE SKIP` and does not rewrite
  the file.  UART proves:

  ```text
  FS_STAGE SKIP index=0 size=7077792 path=/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite
  FS_STAGE SKIP index=1 size=921654 path=/images/cat_640x480.bmp
  FS_STAGE SKIP index=2 size=759 path=/mission.py
  ```

  Verdict: `stage_files == 3`, `stage_bytes == 0`, `fx_writes == 0`,
  `fx_erases == 0`, `fx_errors == 0`, and `pass: true`.  This is the desired
  reboot behavior: if the model and cat image already exist at the expected
  sizes, boot-time staging verifies them and skips the expensive rewrite.
- `iter43_renode_fs_asset_check_filex_levelx_nand` is the passing B8.9d
  verdict: a separate boot mounts the existing NAND image without formatting
  and verifies staged guest paths through `sentai.fs`: model size `7077792`,
  image size `921654`, mission size `759`, and mission head `import sentai`.
  Verdict: `boot_state == 0x500`, UART contains `FS_ASSET_CHECK_DONE`, and
  `pass: true`.
- Current B8 FS policy: it is acceptable, for now, to write/read host files
  into the emulated FileX/LevelX NAND image before boot.  We do not need to
  bring up USB MSC just to load model/image/mission assets.  This preserves the
  guest runtime contract (`sentai.fs` sees normal files) while keeping the
  first emulator path simpler than physical-board USB-drive copy.

Latest B8.10 physical Coral bridge slice:

- `iter52_renode_tpu_cat_repl_filex_physical_coral` is the first passing B8.10
  verdict.  The guest firmware boots the ARM emulator target, mounts the
  staged FileX/LevelX NAND image, imports `/mission.py`, and runs:

  ```python
  sentai.tpu.load("/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite")
  sentai.tpu.load_image("/images/cat_640x480.bmp")
  sentai.tpu.invoke()
  sentai.pipeline.detections(100)
  ```

- The guest `sentai.tpu` module streams model/image bytes from guest FS to a
  Renode host bridge at the emulator boundary.  The host bridge invokes the
  existing no-PyCoral C++/libusb physical-Coral smoke
  `build-sim/sim/tpu_posix_invoke_smoke`, parses the COCO postprocess
  detections, and returns them to the guest.  This is intentionally a
  guest/host bridge, not a fake `detect_cat()` shortcut and not a Renode EHCI
  USB-device model.
- UART proof from iter52:

  ```text
  MODEL_SIZE 7077792
  IMAGE_SIZE 921654
  TPU_LOAD 0
  TPU_READY True
  TPU_LOAD_IMAGE 0
  TPU_INVOKE 3171
  TPU_OUTPUTS 2
  DETECTIONS_COUNT 5
  DETECTIONS [(16, 0.828, 0.218, 0.166, 0.842, 0.679), ...]
  DETECTIONS_WRITTEN True
  MISSION_TPU_CAT_DONE
  ```

- Verdict: `returncodes.renode == 0`, `boot_state == 0x500`, `heartbeat == 1`,
  `repl_lines == 1`, UART contains all `TPU_*` and detection markers, and
  `pass: true`.  The target Renode script uses `RunFor "5.0s"` because a long
  `RunFor "60s"` window is unnecessary and slow once the synchronous host
  bridge has completed.
- `iter55_renode_tpu_cat_repl_filex_physical_coral` is the current passing
  B8.10 rerun after idempotent staging.  No asset rewrite was needed first;
  the model and image already present in guest FS were loaded by
  `sentai.tpu.load()` / `sentai.tpu.load_image()`, `sentai.tpu.invoke()`
  completed through the physical USB Coral bridge (`TPU_INVOKE 3442`), and the
  mission again returned 5 COCO detections with top `class=16 score=0.828`.
- `iter60_renode_tpu_fps_repl_filex_physical_coral` is the current passing
  B8.10b benchmark.  The FPS window starts after guest `sentai.tpu.load()`,
  `sentai.tpu.load_image()`, and one warmup invoke.  The measured path keeps
  one host C++/libusb smoke process open for the benchmark call and runs 5
  measured invokes against the physical USB Coral:

  ```text
  TPU_FPS_RESULT (5, 1, 8609, 58, 8597, 20)
  TPU_FPS_COMPLETED 5
  TPU_FPS_WARMUP 1
  TPU_FPS_MEASURED_MS 8609
  TPU_FPS_X100 58
  TPU_FPS 0.58
  TPU_FPS_INVOKE_MS_SUM 8597
  DETECTIONS_COUNT 20
  ```

  Bridge proof:

  ```text
  fps begin runs=5 warmup=1
  fps ok completed_runs=5 fps_x100=58 detection_count=20 parsed_detections=20
  ```

  Important interpretation: `TPU_FPS_X100 58` means **0.58 FPS**, not 58 FPS.
  `completed_runs=5` means five measured invoke calls; `detection_count=20`
  means the final invoke produced the model's 20 COCO detections.  Earlier logs
  that said `detections=5` were counting only the top-five `DET[...]` lines
  printed by the smoke, not the model's total detections.
- Performance interpretation after iter60:

  - Renode/emulator throughput is host-PC dependent in general.  However, this
    specific TPU FPS number is not primarily a Renode CPU-speed artifact: the
    host-only `build-sim/sim/tpu_posix_invoke_smoke --runs 5 --warmup 1 ...`
    run measured the same order of magnitude (`~0.56 FPS`) without Renode.
  - Hardware ARM target history is **10 FPS minimum**, so `0.58 FPS` is a clear
    sign that the current POSIX/libusb/smoke path is not equivalent to the
    optimized ARM hot path.
  - The current smoke opens the EdgeTPU in `PerformanceMode::kLow`, while the
    SIM POSIX backend uses `PerformanceMode::kHigh`.  This is the first easy
    A/B axis.
  - The ARM TPU path already contains performance toggles that the smoke
    benchmark does not yet expose:
    `g_sentai_tpu_desc_cache_enabled`,
    `g_sentai_tpu_multi_ep_routing`,
    `g_sentai_tpu_async_input_enabled`,
    `g_sentai_tpu_zero_copy_input`, and
    `g_sentai_tpu_chunk_size`.
  - There are also existing counters in `edgetpu_executable.cc` /
    `edgetpu_driver.cc` for parameter/instruction/input/output/event timing and
    bytes.  B8 should reuse those instead of guessing where time goes.
- Remaining B8.10 gaps: make the bridge less specific to the smoke binary,
  decide whether `sentai.tpu.load_image()` should stay in TPU namespace or move
  behind camera/prep buffers later, and decide how much of this host bridge is
  acceptable as a long-term emulator fixture versus a temporary physical-Coral
  proof.
- Next lower-level B8.10 step: keep the guest contract the same, but move the
  host bridge from "launch a complete smoke executable per guest command" to a
  persistent host-side EdgeTPU session with explicit commands for model/package
  load, input upload, invoke, and output/postprocess retrieval.  That is the
  right level below the current smoke bridge before attempting real Renode EHCI
  USB passthrough/device modeling.
- B8.10c concrete optimization plan:

  1. Extend `tpu_posix_invoke_smoke` with benchmark flags for performance mode
     (`low/high/max`), descriptor cache, chunk size, async input, zero-copy
     input, and multi-endpoint routing.
  2. Print per-benchmark counters: completed invokes, FPS, per-invoke
     params/instructions/input/output/event calls and bytes, plus skip counters
     for descriptor-cache hits.
  3. Run host-only A/B first; only rerun Renode after the host-only path moves
     materially toward the ARM baseline.
  4. If host-only remains far below 10 FPS, build a persistent host bridge
     process/session so model/package/device setup is not repeated across guest
     commands and so the bridge boundary is closer to
     `SendParameters`/`SendInputs`/`SendInstructions`/`GetOutputs`.
  5. Only after that consider a lower Renode USB/EHCI model; do not spend time
     modeling bus details while the standalone host transport is still slow.
- B8.10c progress on 2026-06-03:

  - `tpu_posix_invoke_smoke` now exposes the planned A/B knobs:
    `--perf low|medium|high|max`, `--desc-cache`, `--async-input`,
    `--multi-ep`, `--zero-copy-input`, `--chunk-size`, `--chunk-kb`,
    `--break-short-bulkin`, and `--fast-sync-wait`.
    It also prints `TPU_BENCH_CONFIG`, `TPU_STAGE_STATS`,
    `TPU_CALL_STATS`, and `TPU_DESC_CACHE_STATS`.
  - The Renode host bridge now timestamps model/image staging and physical
    Coral invoke/FPS calls, and copies the smoke benchmark summary lines into
    `/tmp/sentai_emu_tpu_bridge.log`.
  - Host-only baseline after instrumentation:

    ```text
    --runs 3 --warmup 1 --perf low
    FPS_BENCH completed=3 measured_ms=5434 fps_x100=55
    TPU_STAGE_STATS ins_calls=3 ins_bytes=762912 ins_ticks=174
                    input_calls=3 input_bytes=810000 input_ticks=185
                    output_calls=6 output_bytes=552120 output_ticks=5059
    ```

    `--perf high` before the low-level fix was essentially unchanged:
    `fps_x100=56`, with output still dominating (`output_ticks=4915`).
  - Descriptor-cache is **not safe yet** on this physical-Coral smoke path:
    `--perf high --desc-cache` failed with `completed=0`, `skip_ins=1`, and
    left the USB Coral in a bad state where subsequent `OpenDevice()` failed at
    `read omc0_00`.  Treat descriptor-cache as disabled until its state machine
    is fixed and covered by a replug-safe test.
  - Short Bulk-IN is **not equivalent** across ARM/NXP and POSIX/libusb.  ARM
    treats a short Bulk-IN as stream termination for padded outputs.  A POSIX
    experiment that enabled the same break by default failed the COCO smoke
    (`completed=0`, changed output checksum, no detections).  Therefore POSIX
    keeps the historical continue-by-default behavior, and the break is exposed
    only as diagnostic `--break-short-bulkin`.
  - Similarly, skipping the final 1 ms sync-transfer pacing delay after libusb
    completion caused a failed invoke on this path.  The old pacing remains the
    default; the faster behavior is diagnostic-only via `--fast-sync-wait`.
  - Validation is blocked until USB Coral is replugged/restarted again: after
    the failed aggressive Bulk-IN / fast-sync experiment, the device still
    enumerates as `18d1:9302`, but subsequent `OpenDevice()` fails at
    `read omc0_00`.  No Renode, smoke, or `usbreset` processes are left
    running.
  - After physical replug, the safe low-level USB smoke passed end-to-end:

    ```text
    ./build-sim/sim/tpu_posix_invoke_smoke \
      --runs 1 --warmup 0 --perf high \
      --no-desc-cache --no-break-short-bulkin --no-fast-sync-wait

    DFU loading EdgeTPU firmware len=10783
    OK tpu_posix_invoke invoke_ms=3144
    output_checksum=877b9c24
    DETECTIONS n=20
    DET[0] class=16 score=0.828 box=[0.218 0.166 0.842 0.679]
    TPU_STAGE_STATS params_calls=1 params_bytes=6703232 params_ticks=1410
                    ins_calls=2 ins_bytes=264752 ins_ticks=61
                    input_calls=1 input_bytes=270000 input_ticks=59
                    output_calls=2 output_bytes=184040 output_ticks=1612
    ```

    This proves the no-PyCoral low-level USB path can load firmware, initialize
    the physical Coral, send the COCO model package/parameters/instructions,
    send the BMP-derived input tensor, read TPU outputs, and run COCO detection
    postprocess.
  - Safe repeated low-level benchmark after warmup:

    ```text
    --runs 3 --warmup 1 --perf high --no-desc-cache \
      --no-break-short-bulkin --no-fast-sync-wait
    FPS_BENCH completed=3 measured_ms=5160 fps_x100=58
    TPU_STAGE_STATS ins_calls=3 ins_bytes=762912 ins_ticks=167
                    input_calls=3 input_bytes=810000 input_ticks=175
                    output_calls=6 output_bytes=552120 output_ticks=4802
    ```

    Chunk-size A/B with `--chunk-kb 36` stayed correct and improved only
    slightly:

    ```text
    FPS_BENCH completed=3 measured_ms=4999 fps_x100=60
    TPU_STAGE_STATS ins_ticks=77 input_ticks=87 output_ticks=4825
    ```

    Interpretation: larger chunks reduce instruction/input transfer overhead,
    but steady-state invoke time is still dominated by the output/compute read
    phase, not by parameter upload or Python/Renode overhead.
  - Host-only PyCoral control benchmark on the same physical USB Coral, same
    COCO EdgeTPU model, and same `cat_640x480.bmp` proves the silicon and
    host libedgetpu path are fast:

    ```text
    venv-coral/bin/python
    PYCORAL_LOAD_MS 2641.863
    PYCORAL_WARMUP_MS min=11.266 avg=14.873 max=27.968
    PYCORAL_FPS runs=50 measured_ms=580.388 fps=86.149
                invoke_ms_min=11.127 invoke_ms_avg=11.600 invoke_ms_max=12.260
    OBJECTS n=20
    OBJ[0] id=16 score=0.844 bbox=BBox(xmin=53, ymin=65, xmax=202, ymax=252)
    ```

    This resolves the ambiguity around the low-level benchmark: the Coral can
    process this model at far above the hardware target's 20 FPS claim when
    driven by libedgetpu/PyCoral.  The `~0.6 FPS` no-PyCoral smoke bottleneck
    is therefore in our low-level `TpuDriver` / POSIX libusb transaction model
    (descriptor replay, output/event sequencing, polling/pacing, or protocol
    mismatch), not in Renode and not in EdgeTPU compute capability.
- Production runtime note: the real ARM MicroPython task already auto-runs
  `/main.py` from the user FS with safe-mode and timeout guards.  The current
  B8 emu REPL target still uses a minimal embed autorun of `/mission.py` for
  deterministic experiments.  Aligning the emu with `/main.py` auto-run is a
  follow-up once the TPU bridge/FPS path is stable.

## Purpose

Execute the A8 research path: decide how SentAI should run closer to the ARM
firmware model on x86 host machines, without relying on the FreeRTOS POSIX port
as the primary task/ISR proof.

B8 is research plus a small spike.  It should not refactor production SentAI
runtime code until the emulator choice and first boot target are clear.

## Current Recommendation

Use **Renode-first**, with **QEMU as a smaller kernel/FreeRTOS smoke backup**.

Why:

- QEMU can emulate Arm M-profile CPU architecture and has FreeRTOS Cortex-M
  demos, so it is good for proving ARM FreeRTOS basics.
- QEMU does not currently give us an off-the-shelf MIMXRT1176 SentAI board.
  Modeling RT1176 peripherals in QEMU would be a substantial custom-machine
  project.
- Renode is designed for custom virtual platforms and custom peripherals.  It
  is a better fit for "run our ARM ELF and gradually add the peripherals that
  SentAI touches".
- Renode's host integration, deterministic execution, GDB support, and
  peripheral modeling are closer to what we need for camera frames, FS images,
  UART/REPL, and CI-style experiments.

## 2026-06-02 Research Update

B8 should not assume that "ARM emulator" automatically means "all real board
peripherals are available".  The critical split is:

```text
ARM task/ISR fidelity          -> emulator can plausibly help
RT1176 board/peripheral parity -> custom platform work
Coral USB parity              -> likely not first-slice emulator work
```

Current host state:

- `renode` is not installed in the local PATH, but source was cloned locally at
  `/home/bogdan/work/renode` (`ac18b5a`, branch `master`).
- `qemu-system-arm` is not installed in the local PATH.
- Zephyr source was cloned locally at `/home/bogdan/work/zephyrOS`
  (`0570f6d6b`, branch `main`) to mine RT1176 board/devicetree/platform data.
- no ready `sentai_runtime.elf` was found in the current build directories; the
  current tree has SIM builds and ARM build definitions, but the B8 spike must
  first produce or locate a CM7 ELF.

Current source-boundary observations:

- `examples/sentai_runtime` builds an ARM CM7 `sentai_runtime` executable with
  `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld`.
- The runtime pulls in real board services early: `BOARD_InitHardware`,
  FileX/FxUser storage, `CameraTask`, CSI/PXP camera support, USB/EdgeTPU
  manager code, MicroPython task, FR, and several SentAI runtime tasks.
- The real camera path uses the NXP CSI receiver queue plus ISR bookkeeping in
  `libs/camera/camera_support.c`, and consumers eventually call
  `CAMERA_RECEIVER_GetFullBuffer` through `CameraTask`.
- Therefore the first emulated camera provider should inject into the same
  "full buffer available + interrupt/queue event" boundary, not into MP and not
  into a separate POSIX-only buffer model.

Clarification: Zephyr **does** document an RT1176 board target:
`mimxrt1170_evk@A/mimxrt1176/cm7`.  The Zephyr board page identifies the SoC
as `mimxrt1176`, the MCU as `MIMXRT1176DVMAA`, and lists the Cortex-M7/M4,
SDRAM, flash, USB connectors, PXP, CSI/MIPI camera-related nodes, clocks, and
other peripherals.  This is useful evidence that the RT1176 platform is well
described in modern embedded tooling.

Local Zephyr files of immediate interest:

```text
/home/bogdan/work/zephyrOS/boards/nxp/mimxrt1170_evk/
  mimxrt1170_evk_mimxrt1176_cm7.dts
  mimxrt1170_evk.dtsi
  mimxrt1170_evk-pinctrl.dtsi
  xip/*flexspi_nor_config*

/home/bogdan/work/zephyrOS/dts/arm/nxp/imxrt/nxp_rt1170*.dtsi
```

The CM7 DTS confirms useful anchors for our emulator profile:

- `compatible = "nxp,mimxrt1176"`;
- `zephyr,sram = &sdram0`;
- `zephyr,dtcm = &dtcm`;
- `zephyr,itcm = &itcm`;
- `zephyr,console = &lpuart1`;
- `zephyr,flash-controller = &ext_flash_ctrl`;
- `zephyr,flash = &is25wp128`;
- `sdram0` at `0x80000000`, size `64M`;
- enabled `systick`, `gpt_hw_timer`, `wdog1`, `lpuart1`, `usdhc1`,
  `usb1`/`usbphy1`, `mipi_csi2rx`, and `csi` references.

The important caveat is that **Zephyr board support is not emulator support**.
It gives us:

- a validated RT1176 peripheral map and devicetree-style naming reference;
- build/clock/memory clues for an emulator profile;
- possible comparison points for what a minimal RT1176 CM7 bring-up needs.

It does not give us, by itself:

- a Renode RT1176 machine;
- a QEMU RT1176 machine;
- USB host passthrough for Coral;
- CSI/PXP/SEMC/FlexSPI device models.

So B8 should use Zephyr as a platform map and sanity reference, not as proof
that the SentAI ELF can boot in an emulator out of the box.

Local Renode findings:

```text
/home/bogdan/work/renode/platforms/boards/mimxrt1064_evk.repl
/home/bogdan/work/renode/platforms/cpus/imxrt1064.repl
/home/bogdan/work/renode/platforms/boards/mimxrt700_evk.repl
/home/bogdan/work/renode/scripts/single-node/mimxrt700_evk.resc
```

Renode does not currently show an RT1170/RT1176 board platform in the cloned
tree.  It does have useful NXP/i.MX RT material:

- `CPU.CortexM` with `cpuType: "cortex-m7"` in `imxrt1064.repl`;
- `IRQControllers.NVIC` with SysTick frequency and priority mask;
- `UART.NXP_LPUART` model for LPUART instances;
- `GPIOPort.IMXRT_GPIO`;
- `SPI.IMXRT_FlexSPI`;
- `Timers.IMX_GPTimer`;
- broad MMIO tags for SEMC, USB, CSI, PXP, USDHC, FlexSPI FIFOs, etc.

This makes RT1064 the best local Renode starting template, while Zephyr/SDK
RT1176 sources provide the address map and board-specific deltas.

## B8 Objective Shift

We are pausing the POSIX SIM path as the primary proof vehicle.  B8's objective
is now:

```text
Boot a SentAI/RT1176-like ARM firmware image in an emulator, using the ARM
FreeRTOS port and ARM exception/task model, then incrementally add enough
peripheral models to reach REPL, FS, camera-provider frames, Stage1Task, Flow,
and later Crazyflie transport.
```

The POSIX SIM remains a historical/auxiliary tool, not the direction of record
for B8.

## Direction Decision For B8

### Primary path: Renode virtual platform

Renode remains the best fit for a SentAI-specific emulator because we can build
the board boundary incrementally:

1. create a minimal RT1176-like platform with CM7, RAM regions, SysTick/NVIC,
   and UART;
2. load a reduced or current CM7 ELF;
3. add just enough MMIO stubs for boot code that touches clocks, GPIO, cache,
   SEMC/FlexSPI, and storage;
4. add a camera-frame peripheral/provider that writes guest RAM and raises the
   same interrupt/event path expected by the ARM camera code;
5. add filesystem as a deterministic image or a documented host bridge.

This is the path most likely to preserve the ARM FreeRTOS exception model while
still allowing custom peripherals.

### Backup path: QEMU Cortex-M smoke

QEMU is useful for an ARM FreeRTOS kernel smoke, especially with known Cortex-M
boards such as MPS2/AN385.  It is not currently the best way to model RT1176
camera/PXP/USB host because that would require a custom QEMU machine and custom
device models.

QEMU should be used to answer one narrow question:

```text
Can our ARM FreeRTOS/MicroPython task assumptions survive on an ARM emulator at
all, independent of RT1176 board peripherals?
```

### Not first slice: real Coral USB inside emulator

Full Coral USB in an ARM MCU emulator is too large for the first B8 milestone.
It requires all of:

- an emulated RT1176 USB host controller/PHY path;
- USB device attach/enumeration semantics;
- DFU/runtime transitions;
- EdgeTPU endpoint protocol, model upload, instruction/weight flow, and output
  response behavior;
- nonblocking integration with the guest RTOS.

The current B8 first milestone should keep `sentai.tpu` present but not use it
as the gate.  TPU returns as one of these later tracks:

1. **host mailbox peripheral**: guest Stage2Task writes tensors/commands to a
   modeled peripheral; host runs PyCoral/libedgetpu and writes outputs back.
   This tests SentAI scheduling and parsing, but it is not USB parity.
2. **hardware-in-loop Coral**: run the EdgeTPU path on the physical board or
   the existing host baseline while the emulator validates camera/Stage1Task/Flow.
3. **USB model/pass-through**: later research only, after basic ARM-emulated
   SentAI reaches REPL and camera/FS are stable.

## Peripheral Plan

| Subsystem | B8 first slice | Reason |
| --- | --- | --- |
| CM7/FreeRTOS | real ARM port in emulator | This is the core reason to pause B7. |
| UART/REPL | emulated UART to host console/PTY | Needed for `import mission; mission.run()`. |
| Filesystem | first: host bridge or RAM-backed image; later: flash/SD image | We need reproducible per-run mission + FR artifacts. |
| Camera | virtual provider -> guest frame buffers + ISR/event/queue boundary | Do not emulate MIPI-CSI first. Preserve ARM producer/consumer shape. |
| PXP | stub/bypass only if boot requires it; real prep code later | Avoid modeling PXP registers before boot/REPL. |
| Flow/Markers | real ARM tasks after Stage1Task output exists | Consumer fidelity matters after camera works. |
| Crazyflie UART/CRTP | emulated UART/socket bridge after REPL | Lets emulator drive a real/simulated radio bridge. |
| EdgeTPU USB | deferred | Too much protocol + controller fidelity for first slice. |

## 2026-06-02 SentAI Boot Source Review

The current ARM `sentai_runtime` artifact is a CM7 RAM image loaded from:

```text
build/examples/sentai_runtime/sentai_runtime.stripped
```

Important linker/runtime anchors:

```text
examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld

.interrupts        0x00000800
.ramfunc           0x00000c00
.text              0x00000e10
.data              0x20008000
.noinit_boot       0x2000ad28  size 0x14
.bss               0x2000ad40
.usb_host          0x20240000
.tpu_input         0x20243400
.ocram_bss         0x20324400
.heap              0x80000000  size 0x00e00000
.sdram_text/data   0x80e00000+
.ncamera           0x82000000  size 0x004b0000 in current ELF
```

The vector table is at `0x00000800`, the reset vector points to
`0x00000ecd`, and the initial MSP is `0x20040000`.

Boot flow, from source:

```text
startup_MIMXRT1176_cm7.S
  Reset_Handler
    -> SystemInit()
       -> SystemInitHook()
          -> MCMGR_EarlyInit()
    -> C/C++ runtime init
    -> real_main(...)

libs/base/main_freertos_m7.cc
  real_main()
    g_boot_persist.prev_progress = progress
    progress = 0
    BOARD_InitHardware(true)
    SEMA4_Init
    Timer/GPIO/IPC/Console init
    sentai_boot_progress_mark(0x01)
    storage-mode latch
    LfsInit/LfsUserInit
    USB device/host + EdgeTPU tasks
    LPI2C5/LPI2C6 setup
    CameraTask::Init(...)
    PmicTask::Init(...)
    xTaskCreate(app_main)
    sentai_boot_progress_mark(0x06)
    vTaskStartScheduler()

examples/sentai_runtime/sentai_runtime.cc
  app_main()
    sentai_boot_progress_mark(0x10)
    SentAI runtime / FR / MicroPython setup
    micropython_start_repl_task(...)
```

The debug console is **LPUART6**, not the Zephyr EVK default LPUART1:

```text
third_party/modified/nxp/rt1176-sdk/board.h
  BOARD_DEBUG_UART_INSTANCE_M7 = 6
  BOARD_DEBUG_UART_BAUDRATE_M7 = 115200

third_party/nxp/rt1176-sdk/devices/MIMXRT1176/MIMXRT1176_cm7.h
  LPUART6_BASE = 0x40090000
  LPUART6_IRQn = 25
```

`BOARD_InitHardware(true)` is too board-specific for a first emulator slice:

```text
libs/nxp/rt1176-sdk/board_hardware.c
  MCMGR_Init()
  BOARD_InitBootPins()
  BOARD_InitBootClocks()
  BOARD_ConfigMPU()
  BOARD_InitDebugConsole()
  BOARD_InitSEMC()
  memcpy/memset SDRAM sections
  BOARD_InitCAAM()
  BOARD_InitNAND()
```

Renode can map SDRAM directly, so B8 does not need to model SEMC timing before
it can run the firmware.  Similarly, NAND, CAAM, USB host, LPI2C, PMIC, and
physical CameraTask init should not be blockers for the first REPL milestone.

Current Renode spike result:

```text
emu/renode/sentai_rt1176.repl
emu/renode/sentai_rt1176.resc
```

The provisional platform now loads the current ELF, maps ITCM/DTCM/OCRAM/SDRAM,
uses a Cortex-M7 with 16 MPU regions, includes DWT, and exposes LPUART6.
Running for 1 emulated second no longer aborts on missing DWT/MPU support, but
the UART log is still empty.  Reading `.noinit_boot_persist` after the run
shows all zeros, so the current production image has not reached
`sentai_boot_progress_mark(0x01)`.  The next spike should not continue adding
random peripheral stubs; it should build an explicit emulator board profile.

## Emulator Board Profile Decision

B8 should introduce a deliberate `SENTAI_EMU_ARM`/`BOARD_EMU` style build
profile instead of trying to run the production board initialization unchanged.

First milestone rule: **boot to FreeRTOS idle before initializing any real
board peripheral**.  This is intentional.  The first proof should answer only:

```text
Can the current ARM startup, vector table, MPU/SysTick/NVIC shape, C/C++
runtime, FreeRTOS scheduler, and an idle/heartbeat task run under the emulator?
```

Until that is true, the emulator build should not initialize SEMC, NAND/FileX,
USB device, USB host, EdgeTPU, LPI2C, PMIC, physical CameraTask, CSI, MIPI, PXP,
or Crazyflie transport.  Each of those becomes a separate opt-in milestone once
the scheduler heartbeat is stable.

The profile should preserve:

- CM7 startup, vector table, ARM FreeRTOS port, SysTick/NVIC, MPU shape;
- `real_main`/`app_main` structure;
- MicroPython as a low-priority command layer;
- SentAI runtime modules that do not require physical peripherals;
- FlightRecorder logging and mission import semantics.

The profile should bypass or replace:

- SEMC hardware initialization, while keeping the linker SDRAM layout mapped by
  the emulator;
- NAND/FileX/LevelX physical driver, replaced initially by a RAM-backed or
  host-backed filesystem fixture;
- USB device/host and EdgeTPU task init for the first REPL milestone;
- LPI2C/PMIC physical setup;
- physical `CameraTask::Init()`/MIPI-CSI/OV5640 setup.

Recommended staged boot gates:

```text
B8.1  CM7 Reset_Handler -> main -> vTaskStartScheduler -> idle heartbeat
B8.2  add UART/debug-console output, still no storage/camera/USB
B8.3  add MicroPython REPL task over UART, no filesystem mission yet
B8.4  add emulator filesystem fixture and import mission; mission.run()
B8.5  add virtual camera provider -> common frame-ready boundary
B8.6  add Stage1Task consumers
B8.7  add Flow/markers consumers
B8.8  decide EdgeTPU path: host mailbox, HIL, or USB model
```

The first camera milestone in emulator is **not** MIPI-CSI emulation.  It is a
runtime virtual camera provider that writes frames into the same frame-ready
boundary consumed by Stage1Task/Flow/Stage2Task.  This can be implemented with a
provider interface:

```text
FrameProvider
  file-sequence provider      -> BMP/fixture frames
  memory-repeat provider      -> one loaded frame, repeated at configured FPS
  gazebo-provider bridge      -> host/simulator frames, same publication API
  future physical camera      -> ISR-backed CSI producer on real ARM
```

All providers publish through one camera-frame event/queue boundary.  Consumers
do not know whether a frame came from MIPI-CSI, a file sequence, or Gazebo.

This keeps B8 aligned with the original ARM model without requiring an
impractical MIPI-CSI peripheral model.

## B8.1 Minimal ARM FreeRTOS Spike

The first buildable emulator spike is intentionally smaller than production
`sentai_runtime`.  It proves:

```text
CM7 startup/vector table -> C/C++ runtime -> ARM FreeRTOS scheduler ->
static heartbeat task -> SysTick-backed tick progress
```

It does **not** initialize real board peripherals:

```text
no SEMC/NAND/FileX/USB/EdgeTPU/LPI2C/PMIC/CameraTask/CSI/MIPI/PXP/Crazyflie
```

Files added:

```text
emu/CMakeLists.txt
emu/sentai_emu_idle.cc
emu/sentai_emu_freertos_hooks.c
emu/renode/sentai_emu_idle.resc
```

Build:

```sh
cmake -S . -B build_emu -DSENTAI_ARM_EMU=ON -DSENTAI_SKIP_SDK_PATCHES=ON
cmake --build build_emu --target sentai_emu_idle -j$(nproc)
```

Run:

```sh
/home/bogdan/work/renode_portable/renode --plain --console --disable-xwt \
  emu/renode/sentai_emu_idle.resc
```

Validated result on 2026-06-02:

```text
sentai_emu_idle boot_state: 0x00000300
sentai_emu_idle heartbeat:  0x0000002F
sentai_emu_idle last_tick:  0x000001CC
```

Interpretation:

- startup reaches `main()`;
- `xTaskCreateStatic()` succeeds;
- `vTaskStartScheduler()` starts the first task;
- SysTick/FreeRTOS tick advances under Renode;
- the heartbeat task wakes repeatedly through `vTaskDelay()`.

Implementation notes:

- the target uses the existing RT1176 startup file and ARM FreeRTOS port;
- CMSIS Core include paths are explicit in the emu target;
- `vPortSetupTimerInterrupt()` is overridden only for the B8.1 emu smoke so
  Renode's Cortex-M SysTick frequency matches the SDK's default
  `SystemCoreClock`;
- `vPortSuppressTicksAndSleep()` is stubbed for this smoke because B8.1 is not
  testing low-power tickless idle.

Remaining B8.1 caveat:

- Renode still logs a benign priority-mask warning while FreeRTOS sets system
  handler priority.  The scheduler/tick proof works, but B8.2 should either
  confirm the priority-mask model is acceptable or adjust the `.repl` with a
  documented rationale.

## Console / REPL Transport Clarification

The current SentAI runtime has three related but distinct concepts:

```text
debug console UART  -> LPUART6, BOARD_InitDebugConsole(), DbgConsole_*
USB CDC ACM console -> ConsoleM7 CDC ACM endpoint, default REPL target
sentai.uart         -> raw UART serial bridge, available only when REPL is USB
```

Important code references:

```text
third_party/modified/nxp/rt1176-sdk/board.h
  BOARD_DEBUG_UART_INSTANCE_M7 = 6
  BOARD_DEBUG_UART_BAUDRATE_M7 = 115200

libs/base/console_m7.h
  enum class ReplTarget { kUsb, kUart };
  repl_target_ = ReplTarget::kUsb;

examples/sentai_runtime/bindings/modsentai_top.c
  sentai.console()        -> "usb" or "uart"
  sentai.console("usb")   -> sentai_console_set_target(0)
  sentai.console("uart")  -> sentai_console_set_target(1)

examples/sentai_runtime/bindings/modsentai_uart.c
  sentai.uart.open() requires REPL on USB

examples/sentai_runtime/bindings/modsentai_usb.c
  sentai.usb.open() requires REPL on UART
```

Therefore B8 should not treat `sentai.uart` as the REPL transport.  The correct
reading is:

- default REPL is USB CDC ACM on the real board;
- UART/LPUART6 is the debug console and can be selected as REPL target through
  `sentai.console("uart")`;
- the non-REPL side becomes available for raw Python serial I/O.

`examples/sentai_runtime/SENTAI_API.md` still contains an older statement that
the REPL is on UART and `/dev/ttyACM0` is not the REPL.  That conflicts with
the current code and `agent.md`, which says MicroPython REPL is over USB CDC
ACM by default.  B8 should follow the code and update the stale API note later.

For the emulator path:

- B8.2 validates LPUART6 output as a debug/console smoke;
- B8.3 may force REPL target to UART as an intermediate milestone because
  modeling USB CDC ACM is larger;
- a later milestone can model/bridge USB CDC ACM if we need exact real-board
  REPL transport parity.

## B8.2 Minimal UART Smoke

`sentai_emu_uart` extends the B8.1 heartbeat target with direct LPUART6 writes.
It is intentionally not full `ConsoleM7`; it only proves that the Renode
RT1176-like platform can capture the SentAI debug UART.

Files:

```text
emu/renode/sentai_emu_uart.resc
examples/sentai_runtime/experiments/s213_arm_emulator_idle/
```

Experiment run:

```sh
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target uart
```

Validated result:

```text
examples/sentai_runtime/experiments/s213_arm_emulator_idle/
  iter03_renode_uart_heartbeat/
    verdict_s213.json  -> pass: true
    uart.log           -> boot + task online + heartbeat messages
```

The UART log contains:

```text
SentAI EMU UART boot
SentAI EMU UART task online
SentAI EMU UART heartbeat
```

This confirms:

- Renode `UART.NXP_LPUART` at `0x40090000` works for LPUART6 TX;
- file-backend logging works for experiment artifacts;
- FreeRTOS task execution and UART output coexist.

Next gate: B8.3 should start a reduced MicroPython REPL task on UART, without
filesystem/main.py/crazy/USB/storage/camera/TPU.

## B8.3 Minimal MicroPython REPL On UART

`sentai_emu_repl` is the first emulator target that runs the MicroPython VM.
It boots through the same CM7 startup + ARM FreeRTOS path as B8.1/B8.2, then
hands control to a single REPL task on LPUART6.

Files:

```text
emu/sentai_emu_repl.cc            # main + heap + REPL loop
emu/sentai_emu_mphalport.c        # mp_hal_stdin_rx_chr / mp_hal_stdout_tx_*
emu/sentai_emu_stub_modules.c     # empty `sentai` module (linker satisfier)
emu/mp_inc/mpconfigport.h         # minimal MP config for emu_repl
emu/renode/sentai_emu_repl.resc
examples/sentai_runtime/experiments/s213_arm_emulator_idle/
```

Build:

```sh
cmake --build build_emu --target sentai_emu_repl -j$(nproc)
```

Run (experiment-driven):

```sh
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py \
    --target repl
```

Validated result (`iter05_renode_repl_oneplusone`):

```text
boot_state              = 0x00000500  (kBootReplBanner)
repl_lines              = 1
last_tick               > 0
uart_log raw bytes      = "\r\nSentAI EMU REPL B8.3\r\n"
                          "MicroPython embed ready\r\n"
                          ">>> 1+1\r\n"
                          "2\r\n"
                          ">>> "
pass                    = true
```

What this proves:

- the emulator can host the MicroPython embed port on a true ARM build (not
  POSIX), driven by the production CM7 startup and ARM FreeRTOS scheduler;
- LPUART6 RX delivers characters into `mp_hal_stdin_rx_chr` via Renode's
  `lpuart6 WriteChar` injection;
- `mp_embed_init` (GC heap, stack ctrl, QSTR pool, module table) completes;
- `mp_embed_exec_str` compiles and evaluates a top-level Python expression
  and the result reaches LPUART6 TX via `mp_hal_stdout_tx_strn_cooked`.

Notes and gotchas captured by B8.3:

- The shared `genhdr/moduledefs.h` references `mp_module_sentai` and the rest
  of the production module set unconditionally.  The emu mpconfigport must
  therefore enable the same module set; disabling `MICROPY_PY_SYS`,
  `MICROPY_PY_ARRAY`, `MICROPY_PY_MATH`, `MICROPY_PY_COLLECTIONS`, or
  `MICROPY_PY_STRUCT` would not save flash — it would only produce
  "undefined reference to mp_module_X" link errors.  An empty `mp_module_sentai`
  stub is supplied in `sentai_emu_stub_modules.c` so we do not pull modsentai.c
  or any binding for B8.3.
- The QSTR pool is shared with production.  `MICROPY_PY_BUILTINS_STR_OP_MODULO`
  must be enabled in the emu config so `hex()`/`oct()` use `%#x`/`%#o` instead
  of the `{:#x}` form, which is not in the baked QSTR table.
- `mp_embed_exec_str` parses with `MP_PARSE_FILE_INPUT`, yet the embed VM still
  prints the top-level expression result.  B8.3 confirms this empirically:
  `1+1\r` causes `2\r\n` to be written to the UART.  No `MP_PARSE_SINGLE_INPUT`
  variant is required for the smoke.
- `Renode read_text()` strips `\r` via newline translation; verdict checks
  must read the UART log in binary mode and assert against raw bytes, or the
  REPL answer assertion silently flips to false.  Fixed in `run_s213.py`.
- The B8.1 critique that the emu target should use `ARM_CM7/r0p1/port.c`
  instead of `ARM_CM4F/port.c` was wrong: `libs/FreeRTOS/CMakeLists.txt`
  shows production firmware itself compiles `ARM_CM4F/port.c` for the M7.
  The `ARM_CM7/r0p1` directory is only added as an include path, not as a
  source.  Emulator and production are aligned on this point.

Open caveats:

- Boot-time priority-mask warning from Renode persists (same as B8.1).  Now
  that the build emits proper FreeRTOS interrupt priorities, B8.4+ can either
  raise `cpu PRIGROUP` in the `.repl` or document the model gap.
- The REPL is single-line only: no history, no multi-line blocks, no Ctrl+C,
  no watchdog hook, no scheduler drain.  All deferred to a later milestone
  if the emulator path becomes a daily driver.
- `mp_embed_init` is fed a 32 KiB GC heap.  Enough for `1+1` and simple
  expressions; will need raising before importing a mission module.

Next gate: B8.4 should mount a RAM-backed or host-bridged filesystem fixture
so `import mission; mission.run()` works without USB/NAND/FileX.

## B8.4 In-Firmware VFS + import mission; mission.run()

`sentai_emu_mission` extends the B8.3 REPL target with a tiny in-firmware
virtual filesystem: `mission.py` is baked into `.rodata` as a C string array
and exposed to the MicroPython import machinery through a custom
`mp_import_stat` + `mp_lexer_new_from_file` pair.

Files added:

```text
emu/sentai_emu_fs.c               # in-firmware mission.py + table lookup
emu/mp_inc_mission/mpconfigport.h # B8.3 config + MICROPY_ENABLE_EXTERNAL_IMPORT
                                  # + MICROPY_PY_SYS_PATH + delegation
emu/renode/sentai_emu_mission.resc
```

The REPL source `emu/sentai_emu_repl.cc` is reused with two guarded
additions when `SENTAI_EMU_MISSION=1`:

- 96 KiB GC heap instead of 32 KiB (import allocates a parse tree + module
  dict; the smaller heap hits `MemoryError` partway through `import`);
- a one-shot `mp_embed_exec_str("import sys\nsys.path.append('')\n")` runs
  right after `mp_embed_init` so the import machinery has at least one
  prefix to combine with `mission.py` before calling `mp_import_stat`.

The mission fixture is intentionally non-trivial:

```python
def run():
    print('MISSION OK from B8.4', 2 + 3)
```

`mission.run()` prints the marker AND the arithmetic result.  The verdict
script asserts on `MISSION OK from B8.4 5` (not just `MISSION OK`) so that
a passing run actually proves the function body executed, rather than just
catching an import-time side effect.

Validated result (`iter06_renode_mission_import`):

```text
boot_state                          = 0x00000500
repl_lines                          = 2
heartbeat                           = 2
uart_log raw bytes                 ⊃ "\r\nSentAI EMU REPL B8.3\r\n"
                                     "MicroPython embed ready\r\n"
                                     ">>> import mission\r\n"
                                     ">>> mission.run()\r\n"
                                     "MISSION OK from B8.4 5\r\n"
                                     ">>> "
uart_log_contains_repl_banner       = true
uart_log_contains_mission_marker    = true
pass                                = true
```

What this proves:

- `mp_embed_exec_str` survives the larger `sys.path.append('')` trampoline;
- the embed import machinery routes `import mission` through our custom
  `mp_import_stat("mission.py")` and `mp_lexer_new_from_file(MP_QSTR_mission)`;
- the in-firmware lexer reads bytes out of `.rodata` (`mp_reader_new_mem`
  with `free_len = 0`) without touching FileX/LevelX/USB MSC;
- the compiled bytecode runs and the `print()` reaches LPUART6 TX.

Notes and gotchas captured by B8.4:

- The B8.3 build kept a stub `mp_module_sentai` to satisfy
  `genhdr/moduledefs.h`.  Once `MICROPY_ENABLE_EXTERNAL_IMPORT=1`, that stub
  is still required because the import machinery still pulls in the registered
  module table at link time.  Both stub and import-by-name coexist cleanly.
- `mp_reader_new_mem(reader, buf, size, free_len)` with `free_len = 0`
  prevents the GC from trying to `m_del` the `.rodata`-resident buffer when
  the lexer finishes.  Setting `free_len = size` (as the production
  LittleFS-backed reader does) would corrupt the heap.
- Path matching has to strip leading `/` and `.` so `mp_import_stat` accepts
  both `mission.py` (with `sys.path = [""]`) and the longer `./mission.py`
  /`/mission.py` forms the import code may construct internally.
- 32 KiB GC heap is enough for `1+1` but not for `import` — the parse tree
  and module dict push us over.  Bumped to 96 KiB for the mission target;
  later milestones with multi-module imports will need more.
- The mission marker assertion intentionally includes the `5` digit
  (= 2+3) so a regression that imports the module but fails to execute the
  function body cannot pass.  Pure `print("MISSION OK")` would not catch
  this class of failure.

### Operator caveat: REPL transport is UART, real board's REPL is USB CDC ACM

Captured 2026-06-02: production SentAI runtime exposes the MicroPython REPL
over **USB CDC ACM** (`/dev/ttyACM0`), not over LPUART6.  LPUART6 is the
debug console, and `sentai.console("uart")` is the operator-driven switch.
The B8.x emulator targets use LPUART6 for REPL because Renode's RT1176 model
has a working `UART.NXP_LPUART` but no USB device controller / CDC ACM
endpoint.

The follow-on constraint matters for B8.5+ and for the Crazyflie integration:

- once UART becomes the Crazyflie CRTP bridge transport (operator note
  2026-06-02), the same LPUART can no longer multiplex an interactive REPL.
- B8.4's interactive REPL injection (`lpuart6 WriteChar`) is therefore a
  development convenience, not the production REPL contract.  Treat it as
  test scaffolding.
- Two options open for later milestones, ordered by fidelity to the real
  board:
  1. model USB CDC ACM in Renode (likely a custom peripheral; the upstream
     Renode RT1064/700 platforms do not give us one out of the box);
  2. drop interactive REPL entirely once a CRTP UART is wired and run the
     mission in `import + run` form from a baked or host-staged file,
     using the UART only for CRTP frames.
- Until then, every emu milestone that needs Python input should drive it
  through `mission.py` (baked or staged) and not assume a long-lived
  interactive REPL.

Open caveats inherited from earlier gates:

- Renode `nvic` priority-mask warning persists.
- Single-line REPL only.
- Mission GC heap is 96 KiB; large enough for the smoke, not sized for
  importing a real flight mission.

Next gate: B8.5 should add the first virtual camera frame provider feeding
the same camera-frame event boundary used by ARM Stage1Task, still without
modeling MIPI-CSI or PXP register-level state.

## B8.5 VCam Renode Peripheral + Real NVIC IRQ Frame Boundary

`sentai_emu_camera` proves the full ARM camera ISR path under Renode, end to
end, without modeling MIPI-CSI/PXP/OV5640.  A new `vcam` Python peripheral
implements a "host-triggered DMA + IRQ" model that exercises:

```text
host  ──CONTROL.ARM=1──►  Renode vcam
                            │
                            ├─ DMA: WriteBytes(frame_seq, FRAME_PTR..+FRAME_LEN)
                            ├─ FRAME_SEQ++
                            ├─ STATUS.frame_ready = 1
                            └─ sysbus.WriteDoubleWord(NVIC_ISPR2, bit 30)
                                                │
                                                ▼
                                       Cortex-M IRQ entry
                                       (vector table @ 0x800 + (16+94)*4)
                                                │
                                                ▼
                                Reserved110_IRQHandler (strong override)
                                       │
                                       ├─ STATUS = 1 (W1C ack)
                                       └─ vTaskNotifyGiveFromISR
                                                │
                                                ▼
                                        Consumer FreeRTOS task
                                       (validate bytes, log marker, ack)
```

Files added:

```text
emu/sentai_emu_camera.cc      # consumer task + strong IRQ handler
emu/renode/sentai_emu_camera.resc
emu/renode/sentai_rt1176.repl # vcam peripheral block (mapped 0x40900000)
```

Critical design points:

- **VCam is at `0x40900000`, NOT the real RT1176 CSI base `0x40800000`.**
  Mapping it at the real CSI address would have invited "is this modeling
  CSI?" confusion and clashed with the existing CSI-region placeholder used
  by board init code.  Putting VCam at an unused MMIO region keeps it
  obviously emu-only test scaffolding, consistent with the SentAI air-gap
  rule.
- **IRQ 94 = `Reserved110_IRQn`** is unused in production firmware.  The
  startup ASM declares `Reserved110_IRQHandler` as a `def_irq_handler` weak
  alias to `DefaultISR`; the B8.5 build supplies a strong override and the
  linker resolves to it without changes to the vector table layout.
- **The peripheral itself raises the IRQ**, not the host script.  After
  filling the frame buffer, the inline Python in the `.repl` block does
  `sysbus.WriteDoubleWord(0xE000E208, 0x40000000)` (NVIC ISPR2, bit 30 =
  IRQ 94).  CONTROL.ARM and the NVIC pend are atomic from the guest's
  point of view, matching real peripheral semantics.
- **ISR uses `vTaskNotifyGiveFromISR` + `portYIELD_FROM_ISR`**.  The
  NVIC priority for IRQ 94 is set to 8 (numeric value 0x80 after the
  4-bit shift), well below `configMAX_SYSCALL_INTERRUPT_PRIORITY`
  (numeric 0x20), so FromISR APIs are safe.
- **Frame validation in the consumer** is `g_frame_buffer[0] == (seq & 0xFF)`
  AND `g_frame_buffer[kFrameBytes-1] == (seq & 0xFF)`.  Checking only the
  head would not catch a partial DMA; checking both head and tail proves
  the peripheral wrote the entire `FRAME_LEN` window.

Validated result (`iter07_renode_camera_5frames`):

```text
boot_state                            = 0x00000600  (kBootConsumerReady)
frames_consumed                       = 5
frames_valid                          = 5
irq_count                             = 5
last_tick                             > 0
uart_log raw bytes                   ⊃ "SentAI EMU CAMERA B8.5"
                                       "Consumer ready, awaiting frames"
                                       "FRAME 1 seq=1 byte=0x01 ok=1"
                                       "FRAME 2 seq=2 byte=0x02 ok=1"
                                       "FRAME 3 seq=3 byte=0x03 ok=1"
                                       "FRAME 4 seq=4 byte=0x04 ok=1"
                                       "FRAME 5 seq=5 byte=0x05 ok=1"
uart_log_contains_camera_first        = true
uart_log_contains_camera_last         = true
pass                                  = true
```

What this proves:

- Renode Python peripherals can pend NVIC IRQs by writing the ISPR register
  from inline scripts (verified: `frames_valid == irq_count == 5`).
- The Cortex-M exception entry path, vector lookup, register stacking,
  ISR execution, ACK, and exception return all run through the real ARM
  FreeRTOS port.  No part of this path is faked in C; only the trigger
  source is host-driven.
- The consumer task wakes from `ulTaskNotifyTake(portMAX_DELAY)` on
  every IRQ, validates the entire frame window, and re-arms the next
  cycle without missing or doubling frames.
- The boundary at which a real CameraTask would consume a CSI-DMA'd
  buffer is unchanged: a task blocked on a queue/notification, woken
  from ISR with the frame metadata available in MMIO.  Future B8.6+
  Stage1Task work can plug in at exactly this boundary.

Notes and gotchas captured by B8.5:

- Inline Python peripherals (`script: '''...'''`) do NOT bind `self.SystemBus`
  the way file-backed peripherals do.  The working pattern is to grab the
  bus at init via `emulationManager.Instance.CurrentEmulation.Machines[0].SystemBus`
  and call `sysbus.WriteBytes(arr, addr)` / `sysbus.WriteDoubleWord(addr, val)`.
  Trying `self.SystemBus...` from an inline script fails with
  "'PythonPeripheral' object has no attribute 'SystemBus'".  This caught us
  on the first run; check
  `platforms/cpus/mimxrt798s.repl` (rstctl1_i3c0) for the canonical
  inline-bus-access pattern.
- IronPython 2 needs `System.Array.CreateInstance(System.Byte, n)` to build
  the byte array passed to `WriteBytes`.  Plain `bytes([...])` does not
  marshal correctly to the .NET `System.Byte[]` signature.
- The B8.5 frame size is 64 bytes — enough to prove the DMA wrote the whole
  buffer (head and tail bytes are validated) without making the inline
  Python loop slow.  640x480 RGB24 frames (~900 KB) would be impractical
  in IronPython; for B8.6+ the bytes should either be pre-computed at init
  and `WriteBytes` called once, or the peripheral should be rewritten as
  a C# plugin.
- `Reserved110_IRQn` (= 94) was picked because it has a vector slot in the
  RT1176 CM7 startup file but no production code uses it.  Future emu
  milestones that touch real IRQs (e.g. real LPI2C/SDMA via Renode models)
  must verify the chosen IRQ does not collide.

Open caveats inherited from earlier gates:

- Renode `nvic` priority-mask warning persists.  Not affecting IRQ delivery
  (verified: irq_count == frames_consumed == 5).
- VCam is NOT MIPI-CSI: it does not test pixel format conversion, line/frame
  sync semantics, PXP-driven RGB→Y8, or backpressure under sustained
  high-rate frames.  All of those are explicitly deferred per the B8
  peripheral fidelity matrix.

Next gate: B8.6 should wire a Stage1Task consumer to the same VCam frame
boundary (perhaps as a second consumer task pulling from a shared queue
the ISR posts to) and validate that Stage1Task scalars/counters move on each
emulated frame, the same way they do on the real camera.

## B8.6 Stage1Task + Stage2Task Pipeline Behind VCam IRQ

`sentai_emu_pipeline` reuses the B8.5 VCam peripheral and the same IRQ 94
boundary, then inserts a real two-stage FreeRTOS pipeline on the firmware
side.  The contract mirrors the production W11 pipeline almost verbatim:

```text
Renode vcam ─CONTROL.ARM=1─► IRQ 94 ─FromISR notify─► Stage1Task
                                                       │
                                                       ├─ scan g_frame_buffer
                                                       ├─ sum, avg
                                                       ├─ publish g_scalar_slot
                                                       │  (fields → __DMB → valid=1)
                                                       └─ xTaskNotifyGive(stage2)
                                                                │
                                                                ▼
                                                          Stage2Task
                                                          ├─ read slot (DMB)
                                                          ├─ clear valid
                                                          └─ UART marker line
```

Files added:

```text
emu/sentai_emu_pipeline.cc      # Stage1Task + Stage2Task + shared slot
emu/renode/sentai_emu_pipeline.resc
```

Design points specific to B8.6:

- **Two real FreeRTOS tasks**, not one consumer wearing two hats.  Stage1Task
  runs at `tskIDLE_PRIORITY + 3` (higher than Stage2Task) so the wakeup
  ordering ISR → Stage1 → Stage2 matches the production W11 scheduling
  intent shape (Stage1 runs first, Stage2 runs after).  The emu does NOT
  reuse the production task names — production `PrepTask` and `FlowTask`
  do PXP / quant / USADA8 work that this spike deliberately does not
  attempt.  Stand-in stage names keep the emu visibly separate from the
  production symbols.
- **Shared slot publish/observe contract**.  Stage1Task writes the data
  fields, runs `__DMB()`, then sets `valid = 1`.  Stage2Task reads
  `valid`, runs `__DMB()`, then consumes the fields.  This is a
  single-writer / single-reader version of the production seqlock and
  is the minimum that survives ARM CM7 store reordering.  No seqlock
  counter is needed because the task notification provides the wake-up
  signal and serialises the two sides.
- **Per-stage counters in C globals** (`g_sentai_emu_irq_count`,
  `_stage1_processed`, `_stage2_consumed`, `_pipeline_errors`).  The
  verdict reads them post-run and asserts equality.  A partial pipeline
  failure (Stage1Task runs but Stage2Task is starved) would show as
  `stage1_processed > stage2_consumed`.
- **Arithmetic verdict on `last_sum`**.  The frame body is `frame_seq`
  bytes repeated 64 times, so the last reduction must equal `5 * 64 =
  320 = 0x140`.  The runner asserts on the exact value, not just
  monotonic progress.  A regression that drops half the bytes
  (e.g. a misaligned cache invalidate) would produce a different sum
  and fail the gate.
- **Per-frame UART marker** (`STAGE2 N frame_seq=N sum=S avg=A`).  Lines
  carry both the running stage counter (`N`) and the source frame
  sequence (`frame_seq`), so a duplicate-delivery bug would print the
  same `frame_seq` twice and fail the per-line assertion.

Validated result (`iter09_renode_pipeline_stage1_stage2`):

```text
boot_state                                = 0x00000900  (kBootBothReady)
irq_count                                 = 5
stage1_processed                            = 5
stage2_consumed                          = 5
pipeline_errors                           = 0
last_sum                                  = 320          (= 0x140)
uart_log raw bytes                       ⊃ "Stage1Task ready"
                                           "Stage2Task ready"
                                           "STAGE2 1 frame_seq=1 sum=64 avg=1"
                                           "STAGE2 2 frame_seq=2 sum=128 avg=2"
                                           "STAGE2 3 frame_seq=3 sum=192 avg=3"
                                           "STAGE2 4 frame_seq=4 sum=256 avg=4"
                                           "STAGE2 5 frame_seq=5 sum=320 avg=5"
uart_log_contains_pipeline_first          = true
uart_log_contains_pipeline_last           = true
pass                                      = true
```

What this proves:

- the camera-frame-ready boundary at IRQ 94 feeds a real downstream
  consumer, not just an ISR-side counter;
- ARM CM7 FreeRTOS schedules the two-stage notification chain
  ISR → Stage1Task → Stage2Task in the right order on every frame;
- shared-slot publish/observe through `__DMB()` survives across task
  context switches;
- the consumer reduction (sum of bytes) reproduces exactly under
  emulation, with no per-byte drift between Renode RAM writes and
  CPU reads.

Notes captured by B8.6:

- Production sentai_prep uses a seqlock + multiple readers.  B8.6
  uses a single-reader equivalent because there is only one consumer
  in this slice.  The full seqlock + multi-reader pattern is a B8.7+
  concern when adding a markers consumer pulling from the same slot
  in parallel.
- HARD RULE established 2026-06-02 (operator note): emu scaffolding
  MUST NOT name its tasks `PrepTask`, `InferTask`, `FlowTask`, or
  `CameraTask` — those names belong to production symbols in
  `examples/sentai_runtime/detection_task.cc`,
  `examples/sentai_runtime/flow_task.cc`, and
  `libs/camera/camera.cc` respectively.  Stand-in scaffolding uses
  neutral names (`Stage1Task` / `Stage2Task`) so a grep for the
  production names does not land in emu test code by mistake.
  Recorded in auto-memory as
  `feedback-emu-must-not-reuse-production-task-names`.
- `__DMB()` is sufficient on CM7 single-core; no `__DSB()` needed
  because we are not touching DMA-coherency boundaries (the slot
  lives in SDRAM but is purely CPU-written and CPU-read).
- Task creation order matters only weakly: both handles are set
  inside their tasks before the first `ulTaskNotifyTake`.  Spawning
  the higher-priority Stage1Task first happens to make Stage1 set its
  handle before Stage2 in practice, but the design tolerates either
  order because the 3s boot RunFor gives both tasks time to reach
  their first block before any host frame is triggered.
- The build re-uses VCam from `sentai_rt1176.repl` (no changes to the
  `.repl` were required for B8.6).  The same Renode peripheral can
  feed any number of consumer pipelines built on top.

Open caveats inherited from earlier gates:

- Renode `nvic` priority-mask warning persists.
- The reduction is `sum of bytes`, not real preprocessing (RGB→Y8,
  resize, normalise).  Real Stage1Task work belongs in a later gate
  once an actual frame format is locked in.
- Only one downstream consumer.  Multi-reader fan-out (Flow + ArUco
  + Stage2Task reading the same prep slot) is deferred.

Next gate: B8.7 should split the consumer side into Flow + markers
tasks reading the same prep slot concurrently, exercising the real
seqlock contract and validating that all consumers see consistent
data when Stage1Task is faster than they are.

## B8.7 Fan-out Stage1Task → {Stage2ATask, Stage2BTask} With Seqlock

`sentai_emu_fanout` adds a second parallel consumer to the B8.6
pipeline.  Stage1Task remains the sole writer of the scalar slot.
Both Stage2ATask and Stage2BTask read the slot independently through
a real seqlock retry loop:

```text
Renode vcam ─CONTROL.ARM=1─► IRQ 94 ─FromISR notify─► Stage1Task
                                                       │
                                                       ├─ scan + reduce
                                                       ├─ SeqlockPublish:
                                                       │    version++ (odd)
                                                       │    write fields
                                                       │    DMB
                                                       │    version++ (even)
                                                       ├─ notify Stage2A
                                                       └─ notify Stage2B
                                                              │      │
                                                              ▼      ▼
                                                         Stage2A   Stage2B
                                                         │           │
                                                         └─ SeqlockRead:
                                                              loop:
                                                                v1 = version
                                                                if v1 odd: retry
                                                                DMB
                                                                load fields
                                                                DMB
                                                                v2 = version
                                                                if v1 != v2: retry
```

Files added:

```text
emu/sentai_emu_fanout.cc
emu/renode/sentai_emu_fanout.resc
```

Naming discipline:

- The two parallel consumers are `Stage2ATask` and `Stage2BTask`, not
  `MarkersTask` / `FlowTask` / `InferTask`.  Same HARD RULE as B8.6:
  production sentai_runtime owns the algorithm-bearing names; emu
  scaffolding must not pretend to be them.

Design points:

- **Three real FreeRTOS tasks** — Stage1Task at `tskIDLE_PRIORITY + 3`,
  Stage2ATask and Stage2BTask both at `tskIDLE_PRIORITY + 2`.  Stage1
  always runs to completion before either reader because of the
  priority delta, so the seqlock retry loop is NOT actually contended
  in the B8.7 happy path.  This is intentional: the topology proof is
  the goal, and the seqlock primitive is exercised as `SeqlockPublish`
  + `SeqlockRead` even when the retry counter stays at zero.
- **Two notifications per IRQ**.  Stage1Task calls
  `xTaskNotifyGive(stage2a_handle)` then `xTaskNotifyGive(stage2b_handle)`.
  Both consumers wake on the same frame and each reads the slot once.
- **Per-reader counters** (`g_sentai_emu_stage2a_consumed`,
  `_stage2b_consumed`).  A starvation regression would show as
  `stage2a_consumed != stage2b_consumed`.
- **`seqlock_torn_reads`** captures retry attempts inside
  `SeqlockRead`.  Stays zero under B8.7's non-contended setup but the
  counter is wired up so a later stress mode (back-to-back IRQs, or
  Stage1 running at the same priority as the readers) can observe
  the retry path firing without changing the firmware.
- **Boot state ladder**.  Each task transitions
  `g_sentai_emu_boot_state` from its predecessor's "ready" state to
  the next, ending at `kBootAllReady = 0xA00` once all three tasks
  have stored their handles and registered for notifications.

Validated result (`iter11_renode_fanout_stage1_2a_2b`):

```text
boot_state              = 0x00000A00  (kBootAllReady)
irq_count               = 5
stage1_processed        = 5
stage2a_consumed        = 5
stage2b_consumed        = 5
seqlock_torn_reads      = 0
pipeline_errors         = 0
last_sum                = 320  (= 5 * 64)
uart_log raw bytes     ⊃ "Stage1Task ready"
                         "Stage2ATask ready"
                         "Stage2BTask ready"
                         "STAGE2A 1 frame_seq=1 sum=64 avg=1"
                         "STAGE2B 1 frame_seq=1 sum=64 avg=1"
                         ...
                         "STAGE2A 5 frame_seq=5 sum=320 avg=5"
                         "STAGE2B 5 frame_seq=5 sum=320 avg=5"
pass                    = true
```

What this proves over B8.6:

- the same camera-frame-ready boundary scales to multiple downstream
  consumers without one consumer starving the other;
- the seqlock writer / reader pair compiles and runs correctly on
  ARM CM7 FreeRTOS under Renode, with `__DMB()` barriers on both
  sides of every payload access;
- both readers see identical data per frame — the arithmetic
  (`sum=N*64`, `avg=N`) matches between STAGE2A and STAGE2B for
  every frame_seq, proving the seqlock delivers consistent reads
  even though contention was not exercised here.

Notes captured by B8.7:

- `seqlock_torn_reads == 0` is expected on the happy path because
  Stage1 has higher priority and runs to completion before either
  reader wakes.  To actually exercise the retry loop, a later gate
  should either lower Stage1's priority equal to the readers'
  (forces interleaving) or fire IRQs back-to-back inside a single
  reader window.  The counter is already wired so that variant only
  needs a `.resc` change, not a firmware change.
- `kSeqlockMaxRetries = 100` is the bail-out bound.  If reached the
  reader logs a `pipeline_errors` increment and skips that frame.
  Under B8.7 conditions this can never happen, but a stress mode
  needs to know when to give up rather than spin forever.
- `xTaskNotifyGive` is used twice per frame (once per consumer).  An
  event group would be a more natural primitive for "many readers
  wait on one event" — production may pick a different mechanism
  later; the choice here was kept identical to B8.6 to isolate the
  fan-out change.

Open caveats inherited from earlier gates:

- Renode `nvic` priority-mask warning persists.
- Reduction is `sum of bytes`, not real prep / flow / markers work.
- VCam is single-shot per host write; no continuous frame source yet.

Next gate: B8.8 should re-evaluate the TPU strategy (host mailbox
peripheral vs HIL vs USB pass-through) now that the rest of the
pipeline topology — REPL, FS, camera ISR, prep + fan-out consumers
— is proved end-to-end under the ARM emulator.

## B8.7c Flow-Offset Detection From Host-Fed Cat Scenes

`sentai_emu_flowest` revives the B7 cat-flow regression test on the
ARM emulator.  Operator note 2026-06-02 mentions B7 had an equivalent
test running on the POSIX SIM that returned wildly wrong results
(`dx=243` and `dx=-129` for a 2-px shift; recovered from
`s209_virtual_camera_tpu_e2e/iter78` and `/iter195` flow_results.txt).
The emulator gate here checks whether the closer-to-real-board
execution model produces the expected answer.

Test setup:

- One single cat photo (`s209/.../cat_640x480.bmp`) is read on host,
  center-cropped, downsampled to 32x32 grayscale.
- `emu/scripts/prepare_cat_scenes.py` generates six variants
  `scene_off_0.bin` .. `scene_off_5.bin`, each shifted by +1, +2, ..,
  +5 pixels in X relative to the BASE crop (vacated columns filled
  with the left-edge column so the apparent motion is a clean
  translation rather than a black-band injection that would dominate
  SAD).
- Renode `.resc` injects each scene into guest SDRAM at
  `g_frame_buffer` using `sysbus LoadBinary`, then writes
  `CONTROL.ARM=1` on the VCam peripheral to fire IRQ 94.
- `sentai_emu_flowest` Stage1Task runs a brute-force SAD block-match
  estimator over a search window `[-5, +5]` in dx and dy, on the
  inner 22x22 region of the 32x32 frame.  It publishes
  `(dx, dy, sad)` per frame into a shared slot; Stage2Task logs a
  `FLOWEST N frame_seq=N dx=D dy=D sad=S` marker to LPUART6.

VCam peripheral change:

- A new register `FILL_MODE` (`0x18`) gates whether VCam overwrites
  the frame buffer with its synthetic byte pattern on
  `CONTROL.ARM=1`.  Default `1` preserves the B8.5/6/7 behavior.
  B8.7c writes `0` at Stage1 init so the host-injected scene
  survives.  The fix was load-bearing: without it, VCam stomped on
  every cat scene immediately after `LoadBinary` and SAD compared
  uniform images, returning `dx=-5, dy=-5, sad=484` (= 22*22 *
  1-byte-diff between successive synthetic patterns).

Pixel-format caveat (operator note 2026-06-02, saved as memory
entry `feedback_emu_pixel_format_caveat.md`):

- Production camera path: OV5640/CSI delivers XRGB8888, PrepTask
  converts to Y8 and publishes a `FLOW_GRAY_80x60` slot, FlowTask
  reads Y8.
- B8.7c skips the camera/PrepTask format conversion stage: VCam
  DMAs Y8 bytes directly at 32x32.  This validates the downstream
  flow consumer's ability to detect offset in Y8 frames, **not**
  the full camera/PrepTask/FLOW format pipeline.  Modelling
  XRGB8888 + PrepTask + the 80x60 grid is a later gate.

Validated result (`iter12_renode_flowest_cat_pan_1px`):

```text
boot_state                            = 0x00000900  (kBootBothReady)
irq_count                             = 6
stage1_processed                      = 6
stage2_consumed                       = 6
pipeline_errors                       = 0
last_dx                               = 1
last_dy                               = 0
last_sad                              = 0
flowest_detected_dx_per_frame         = [0, 1, 1, 1, 1, 1]
flowest_expected_dx_per_frame         = [0, 1, 1, 1, 1, 1]
uart_log raw bytes                   ⊃ "FLOWEST 1 frame_seq=1 dx=0 dy=0 sad=0"
                                       "FLOWEST 2 frame_seq=2 dx=1 dy=0 sad=0"
                                       "FLOWEST 3 frame_seq=3 dx=1 dy=0 sad=0"
                                       "FLOWEST 4 frame_seq=4 dx=1 dy=0 sad=0"
                                       "FLOWEST 5 frame_seq=5 dx=1 dy=0 sad=0"
                                       "FLOWEST 6 frame_seq=6 dx=1 dy=0 sad=0"
pass                                  = true
```

What this proves:

- the brute-force SAD block matcher running under ARM CM7
  emulation, fed real cat-photo pixels through `sysbus LoadBinary`
  per frame, recovers the injected 1-pixel X translation exactly
  (`sad = 0` = perfect block match);
- the camera-frame-ready boundary (VCam IRQ → Stage1Task) carries
  arbitrary host-fed image bytes intact; the FILL_MODE gate cleanly
  separates "VCam owns the pattern" (B8.5/6/7) from "host owns the
  pattern" (B8.7c) without breaking earlier gates;
- the same emulator path that B7 reported failing on POSIX SIM
  produces the textbook-correct answer here.  This does not prove
  the production FlowTask phase-correlation algorithm is correct —
  B8.7c uses a different algorithm and a different frame format
  from production — but it does prove the emulator pipeline carries
  pixel data faithfully and a downstream consumer that does block
  matching gets the right answer.

Notes captured by B8.7c:

- The cat is small after 32x32 downsampling but has enough variance
  to drive SAD to a unique minimum at the correct offset (verified
  on host: SAD at `dx=+1` is 0, at `dx=0` is 6339, at `dx=-5,-5`
  is 26455).  Uniformly-coloured frames would give SAD ties and the
  block matcher would have multiple winners; the cat avoids that.
- `shift_x` repeats the edge column for vacated pixels.  A
  zero-fill would have introduced a black band at the left that
  would dominate SAD and pull the winner toward `dx=0` or smaller,
  masking the actual motion.
- Anonymous-namespace globals do NOT export ELF symbols that
  `sysbus GetSymbolAddress` can resolve.  `g_frame_buffer` /
  `g_prev_buffer` had to be moved to `extern "C"` linkage so the
  `.resc` script could write into them.  This is a general rule for
  any future emu test that wants Renode-side data injection.
- The verdict reads per-frame `dx` values from the UART log via
  regex rather than from peripheral counters, because counters only
  carry the LAST result.  This is the first emu gate where the
  per-frame sequence (not just final state) is part of the contract;
  later gates that test trajectories should follow the same pattern.

Open caveats inherited from earlier gates:

- Renode `nvic` priority-mask warning persists.
- 32x32 Y8 is NOT production format; see pixel-format caveat above.
- SAD brute-force is NOT production phase correlation / USADA8.
- Search range `[-5, +5]` covers only small motions; large jumps
  would clip and report the edge candidate.

Next gate: same as before — B8.8 TPU strategy.  B8.7c is an
extra validation of the camera-ISR-to-flow chain, not a TPU step.

## B8.8 Transparent USB Coral via Real `edgetpu_manager` (open, phased)

Operator decision 2026-06-02: the emulator TPU gate must run the
**real** `libs/tpu/edgetpu_manager.cc` + `libs/usb/usb_host_task.cc`
on the emulated CM7, with USB transfers bridged through to the
host's physical Google Coral USB Edge TPU.  A pycoral host-mailbox
bridge (early attempt) was reverted because it bypasses the entire
production driver stack and breaks the emulator's predictive value.
HARD RULE saved as `feedback_emu_tpu_must_run_real_edgetpu_manager`.

Architectural pattern (saved as
`project_emu_transport_bridge_pattern`):

```text
emulated CM7 firmware (real code)
  └── libs/tpu/edgetpu_manager.cc + libs/usb/usb_host_task.cc
       └── NXP RT1176 SDK usb_host_ehci.c
            └── MMIO writes at 0x4042C000 (USB_OTG2)
                 └── Renode peripheral (NXP EHCI model — TBD)
                      └── host bridge (libusb / USB/IP — TBD)
                           └── physical USB Coral plugged into host
```

The same pattern applies to B8.9 Crazyflie (UART/CRTP -> TCP socket
-> cf2-SITL in CrazySim/Gazebo).  B8.9 is structurally simpler
because Renode's `UART.NXP_LPUART` model already exists and TCP
bridging is supported out-of-the-box.

Production transport inventory (collected during planning):

| Subsystem        | Source                                            | MMIO base    | IRQ                       |
| ---------------- | ------------------------------------------------- | ------------ | ------------------------- |
| USB OTG1         | NXP SDK + libs/usb/usb_device_task.cc             | 0x40430000   | USB_OTG1_IRQn = 136       |
| USB OTG2 (Coral) | NXP SDK + libs/usb/usb_host_task.cc               | 0x4042C000   | USB_OTG2_IRQn = 135       |
| USBNC OTG2       | NXP non-core registers                            | 0x4042C200   | (shares OTG2 IRQ)         |
| USB PHY1         | NXP `usb_phy.c` low-power init                    | 0x40434000   | (none)                    |
| CCM (clocks)     | `clock_config.c` `CLOCK_EnableUsbhs1*`           | 0x40CC0000   | (none)                    |

Coral is wired to USB OTG2 on the SentAI board.  `kUSBControllerId
= kUSB_ControllerEhci1` in `usb_host_task.cc` (the SDK enums
EHCI0/EHCI1 are NXP-style 0-indexed so EHCI1 = OTG2 = controller
id 3 in the `USB_BASE_ADDRS` table).

### Renode capability gap (assessment 2026-06-02)

| Component                              | What Renode portable ships             |
| -------------------------------------- | -------------------------------------- |
| Generic EHCI register model            | `USBDeprecated.EHCIHostController`     |
| Generic USB hub child                  | `USBDeprecated.UsbHub`                 |
| NXP RT1176-specific EHCI               | **none** — would need a fork/plugin    |
| USB device passthrough to host (libusb/USB-IP) | **none** in portable plugins      |
| New `USB.*` namespace (changelog 1.15.3) | exists but no NXP RT1176 models       |

So delivering "transparent USB Coral" end-to-end requires at minimum:

1. A Renode platform model that responds correctly to the NXP-specific
   register set (capability + operational + USBPHY + USBNC) at the
   addresses listed above.
2. A USB device backend that forwards URBs to the host's libusb so
   the real Coral receives them.

Both are real engineering work; the most likely path is a Renode
fork that adds an NXP-RT1176-EHCI peripheral and a libusb-backed
USB device, kept under `patches/coralmicro-renode/`.

### Phased plan

| Phase  | Goal                                                                       | Output                                                       |
| ------ | -------------------------------------------------------------------------- | ------------------------------------------------------------ |
| **P1** | Link and run a minimal probe target (`sentai_emu_usbhost_probe`) that calls the production `UsbHostTask::UsbHostTask()` and nothing else from the USB host chain. | `iter14_renode_usbhost_probe_ctor`, PASS: constructor returned under Renode with current CCM/USBPHY/USB OTG2 stubs. |
| **P2** | Call the production `UsbHostTask::Init()`, start the ARM FreeRTOS scheduler, and verify a lower-priority heartbeat task still runs while no USB model/device is attached. | `iter16_renode_usbhost_task_scheduler`, PASS: `UsbHostTask::Init()` returned, scheduler heartbeat advanced. |
| **P3a** | Pull in the production `EdgeTpuManager` singleton one level above USB host without calling `OpenDevice()`. | `iter21_renode_edgetpu_manager_probe`, PASS: manager constructs, scheduler heartbeat remains healthy. |
| **P3b** | Pull in `EdgeTpuTask` / EdgeTPU USB class registration and identify the first missing link/MMIO/model gap.  Then decide route. Options: | Decision recorded in B8 doc + first-step commit.            |
|        | a. fork Renode, add `NXP.RT1176_EHCI` peripheral that matches the SDK driver expectations                                                                                                  |                                                              |
|        | b. fork Renode, add `USB.LibusbDevice` backend that forwards URBs to a host libusb device                                                                                                  |                                                              |
|        | c. write a Renode-native "fake Coral" USB device that responds to the Coral protocol so the SDK driver completes enumeration (no real silicon, but proves the entire driver path runs)     |                                                              |
|        | d. pause B8.8 if effort exceeds budget and ship Crazyflie B8.9 first (smaller engineering, immediate value)                                                                                  |                                                              |
| **P4** | Execute the chosen route to first concrete milestone (enumeration complete, or DFU descriptor exchange, etc.).                                                                              | First UART log line proving the production driver code completed something against an emulator-bridged Coral / fake-Coral.  Verdict iter under s213. |
| **P5** | If P3 picked path (b) or actual silicon, demonstrate one real Coral inference invoked from inside the emulator with the production code path.  Determinism check (same input -> same output bit-identical to host pycoral).  This is the user's "transparent" goal. | Verdict iter, B8.8 SHIPPED.                                  |
| **P6a** | Inject at the exact `TpuDriver::SendParameters` / `SendInputs` / `SendInstructions` / `GetOutputs` / `ReadEvent` boundary, with a Renode MMIO responder. | `iter37_renode_edgetpu_mmio_send_bridge_probe`, PASS: every transfer-boundary method is called once and Renode writes output bytes back into guest memory. |
| **P6b** | Attach the P6a mailbox to a host-side physical Coral service.  Fastest practical route: host PyCoral/libedgetpu performs the actual COCO invoke while the guest still exercises the `TpuDriver::Send*` boundary. | One COCO/cat invoke from emulated CM7 with physical Coral-backed outputs; explicitly marked "host libedgetpu compute", not raw USB protocol fidelity. |
| **P6c** | Replace P6b's high-level host helper with a raw USB/libedgetpu-low-level bridge so guest `SendParameters`/`SendInstructions`/`SendInputs` are transported as Coral protocol bytes to the physical USB device. | Raw bridge or clear blocker report against local libedgetpu/coralmicro sources. |

### Open caveats

- Estimation: P2 may take a few hours, P3 + P4 may take a few days
  of focused C# / Renode work, P5 depends entirely on which route P3
  picks.  If route (b) — libusb passthrough — is feasible, B8.8 is
  the most valuable B8 gate and worth the investment.  If only route
  (c) — fake Coral protocol — is realistic, B8.8 stays research-only
  and the operator decides whether to invest.
- Renode fork policy: if a fork happens, the patch/branch lives in a
  separate repo, NOT inside `third_party/`.  See
  `external-repo-patch-log` skill.
- No production firmware regression: every B8.8 build target is
  isolated under `emu/`, and the ARM `sentai_runtime` build must
  remain unaffected.

### Status as of this doc update

- HARD RULE saved.
- Pycoral mailbox attempt reverted (no artefacts left on disk).
- B8.8 P1 shipped as `iter14_renode_usbhost_probe_ctor` and revalidated after
  P2 stubs in `iter17_renode_usbhost_probe_ctor`:
  `boot_state = 0xCAFE`, `usbhost_step = 2`, UART contains
  `UsbHostTask constructor returned`.
- B8.8 P2 shipped as `iter16_renode_usbhost_task_scheduler`:
  `boot_state = 0x0600`, `usbhost_step = 5`, `heartbeat = 297`,
  `last_tick = 2960`, UART contains `UsbHostTask::Init returned` and
  `heartbeat task online`.
- `iter15_renode_usbhost_task_scheduler` is intentionally preserved as the
  failed-link history.  It exposed the missing emu-only stubs for `CHECK`
  pulling `ConsoleM7`, SDK cache maintenance, and SDK debug-console printf.
  The fix is scoped to `emu/` (`sentai_emu_usb_stubs.c` plus
  `SENTAI_PLATFORM_SIM` for the P2 target) and does not touch production
  firmware.
- B8.8 P3a shipped as `iter21_renode_edgetpu_manager_probe`:
  production `EdgeTpuManager` constructs above `UsbHostTask`, `usbhost_step
  = 8`, `heartbeat = 297`, `last_tick = 2960`.
- `iter18_renode_edgetpu_manager_probe` and
  `iter20_renode_edgetpu_manager_probe` are intentionally preserved as
  failed-build history.  They exposed two architecture/header gaps:
  `usb_host_edgetpu.h` treated every `SENTAI_PLATFORM_SIM` build as POSIX
  libusb, and the manager target needed production FlatBuffers/TFLite include
  paths.
- The `usb_host_edgetpu.h` fix is important: POSIX SIM still uses libusb, but
  ARM emulator builds now take the NXP USB-host header path even when a target
  uses `SENTAI_PLATFORM_SIM` narrowly to avoid pulling `ConsoleM7` through
  `CHECK`.
- Next concrete step: B8.8 P3b, bring in `EdgeTpuTask` / EdgeTPU USB class
  registration, then decide whether to invest in RT1176 EHCI + libusb
  passthrough, a fake-Coral USB model, or pause TPU and move to B8.9
  Crazyflie UART/CRTP.

## B8.7d Varied 2D Motion Across Both Axes

Operator follow-up 2026-06-02: the constant +1 X pan from B8.7c is
not enough — a passing constant test could come from a block matcher
that always reports `(+1, 0)` for any input, regardless of content.
B8.7d feeds a varied 2D motion sequence to rule that out and to
cover both axes and both signs.

Setup change:

- `emu/scripts/prepare_cat_scenes.py` gains `shift_y(...)` and
  `shift_xy(...)` helpers, plus a `motion_2d` table of six absolute
  offsets `[(0,0), (+2,0), (+2,+2), (-1,+1), (-1,-2), (+3,-2)]`
  written as `scene_2d_0.bin` .. `scene_2d_5.bin`.
- `emu/renode/sentai_emu_flowest.resc` now loads the `scene_2d_*.bin`
  series and lists each absolute offset and per-frame delta in
  comments so the test intent is obvious from the script alone.
- No firmware change required: Stage1Task already iterates over a
  dx,dy search window of `[-5, +5]` so all the deltas below fit.

Per-frame expected motion deltas (curr - prev):

| Frame | Abs (X, Y) | Delta (dx, dy) | What it covers                |
| ----- | ---------- | -------------- | ----------------------------- |
| 1     | (0, 0)     | (0, 0)         | prime, no prev                |
| 2     | (+2, 0)    | (+2, 0)        | pure +X                       |
| 3     | (+2, +2)   | (0, +2)        | pure +Y                       |
| 4     | (-1, +1)   | (-3, -1)       | both axes negative (diagonal) |
| 5     | (-1, -2)   | (0, -3)        | pure -Y                       |
| 6     | (+3, -2)   | (+4, 0)        | pure +X near search edge      |

Independent host-side SAD with the same convention used by the
firmware (inner 22x22, search `[-5, +5]`) confirms each expected
delta has `sad = 0`, i.e., perfect block match.  This pre-flight
sanity check lives in the prep script's docstring and runs as a
single Python one-liner.

Validated result (`iter13_renode_flowest_cat_2d_varied`):

```text
boot_state                            = 0x00000900
irq_count                             = 6
stage1_processed                      = 6
stage2_consumed                       = 6
pipeline_errors                       = 0
flowest_detected_dx_per_frame         = [0, +2,  0, -3,  0, +4]
flowest_detected_dy_per_frame         = [0,  0, +2, -1, -3,  0]
flowest_detected_sad_per_frame        = [0,  0,  0,  0,  0,  0]
flowest_expected_dx_per_frame         = [0, +2,  0, -3,  0, +4]
flowest_expected_dy_per_frame         = [0,  0, +2, -1, -3,  0]
pass                                  = true
```

What this proves over B8.7c v1:

- The block matcher is not stuck on any single axis or sign.  It
  recovers pure-X positive, pure-X positive-near-edge, pure-Y
  positive, pure-Y negative, and diagonal-negative motions in the
  same run.
- The verdict's per-frame contract (`flowest_detected_dx_per_frame
  == expected`, `flowest_detected_dy_per_frame == expected`) makes
  a generic "always reports the same thing" implementation
  impossible to slip through.
- The earlier `sad = 0` proof carries over: every frame is a clean
  translation of the same image, so the correct candidate gives an
  exact match.

The constant-+1 v1 test in iter12 is preserved on disk as the
predecessor; iter13 is the canonical B8.7c/d PASS.

Notes captured by B8.7d:

- Search window `[-5, +5]` has 1 pixel of headroom past the largest
  delta in the sequence (`+4`).  A larger jump like `+5` or `+6`
  would clip and report the edge candidate; future tests that want
  to exercise larger motions should widen the search range or
  shrink the inner region.
- Edge-pixel replication on both axes (top/bottom row for Y, left/
  right column for X) keeps the SAD residual at 0 even in vacated
  bands.  Without that, the vacated band's content would differ
  from the corresponding band in the previous frame and the SAD
  minimum would shift, producing a small but nonzero `sad` and
  potentially a 1-pixel error.
- The host-side SAD pre-flight check is load-bearing: if it
  disagrees with the firmware's answer, the bug is in the firmware
  (or the SAD convention is mismatched).  Both checks use the same
  convention (`curr[y, x] vs prev[y - dy, x - dx]`).

## Crazyflie / UART Strategy

The whole Crazyflie experiment needs at least two communication channels:

1. operator/mission REPL channel;
2. drone/radio channel, likely UART/CRTP/CPX-shaped from SentAI's perspective.

B8 should model these as distinct UARTs or one UART plus one explicit bridge.
The emulator side should expose bytes to host PTYs/sockets.  A host bridge can
then forward to:

- a real Crazyradio/Crazyflie stack;
- a simulator bridge;
- a deterministic test harness that records CRTP/MAVLink packets.

Do not merge the REPL transport and drone transport in the emulator; doing so
would hide the exact class of coupling we are trying to uncover.

## Filesystem Strategy

Chosen B8.9 direction: keep the production FileX/LevelX user partition in the
guest and inject only the raw NAND page/program/erase backend in Renode.  This
matches the ARM board's storage layering and avoids a FileX-in-RAM fixture.

Experiment artifact layout remains:

```text
iterNNN/
  fs_root/              # optional host-side staging directory
  nand.img              # emulator-visible raw NAND image, if copied for audit
  fr/                   # exported/decoded artifacts
  uart.log
  emulator.log
```

The mission rule stays unchanged:

```python
import mission
mission.run()
```

No MP-side `exec` and no large byte-array file loads in MP.

## Initial SentAI Boot Inventory

Local source review of the actual SentAI ARM boot path shows that the first
B8 emulator slice must follow the SentAI board configuration, not the generic
Zephyr EVK defaults:

- `third_party/modified/nxp/rt1176-sdk/board.h` sets the M7 debug UART to
  **LPUART6**, baud `115200`.
- `MIMXRT1176_cm7.h` maps `LPUART6_BASE` to `0x40090000` and `LPUART6_IRQn`
  to `25`.
- `examples/sentai_runtime/MIMXRT1176xxxxx_cm7_ram_mp.ld` places the vector
  table at `0x00000800`, code in ITCM, data in DTCM, USB/tensor buffers in
  OCRAM, and the heap/large runtime sections in SDRAM.
- The current stripped ARM artifact has MSP `0x20040000` and reset vector
  `0x00000ecd`, so Renode must set `VectorTableOffset` to `0x800`.
- `BOARD_InitHardware(true)` touches pin mux, clocks, MPU, debug console,
  SEMC/SDRAM, SDRAM section copy/clear, CAAM, FlexSPI NAND.
- `real_main()` then initializes SEMA4, timer/GPIO/IPC/ConsoleM7, LFS,
  USB device/host, EdgeTPU tasks, LPI2C5/6, camera task, PMIC task, and only
  then starts `app_main`.
- `app_main()` initializes SentAI runtime state and starts the MicroPython REPL
  task.

This means the first milestone is not "USB REPL" yet.  It is:

```text
Load ELF -> run Reset_Handler -> pass enough board init to see UART6 output
```

The second milestone can then decide whether to stub/replace NAND/LFS and USB
for emulator mode or model them more fully.

## Emulator Config Added

Initial Renode bring-up files were added under:

```text
emu/
  README.md
  renode/sentai_rt1176.repl
  renode/sentai_rt1176.resc
```

The `.repl` maps the SentAI linker memory layout:

- ITCM `0x00000000`, `512 KiB`;
- DTCM `0x20000000`, `512 KiB`;
- OCRAM `0x20200000`, `2 MiB`;
- SDRAM `0x80000000`, `64 MiB`;
- LPUART6 `0x40090000`, IRQ `25`.

It also includes host-side Renode stubs for early SDK MMIO regions such as
watchdogs, CCM/SRC/IOMUXC, SEMC, FlexSPI1, CAAM, SEMA4, LPI2C5/6, USB, CSI,
MIPI CSI2RX, PXP, and GPIOs.  These are provisional emulator models whose
purpose is to expose the first real missing behavior boundary.

First run result:

- Renode portable `v1.16.1.4546` was installed under
  `/home/bogdan/work/renode_portable`.
- The `.resc` loads the current stripped SentAI RAM ELF and sets initial
  `PC = 0x00000ecd`, `SP = 0x20040000`.
- The first hard loop was at MU-A: repeated reads from `0x40C48020`, reached
  from PC `0x10278`.  This matches early `MCMGR_EarlyInit()` /
  `MCMGR_Init()` behavior, so MU-A was added as a provisional stub at
  `0x40C48000`.
- DCDC/ANATOP coverage was also widened because boot read `0x40CAC960`.
- After adding MU-A, boot reached clock/pinmux setup and looped on
  `0x40C84560` after writes under `0x40C94000`.  These addresses map to the
  RT1176 ANADIG/OSC/PLL/PMU block and IOMUXC_SNVS, so provisional stubs were
  added for `0x40C84000`, `0x40C90000`, `0x40C94000`, and `0x40C98000`.
- After adding ANADIG/IOMUXC_SNVS, boot reached SDK delay code polling
  `DWT->CYCCNT` at `0xE0001004`, so Renode's built-in `Miscellaneous.DWT`
  model was added at `0xE0001000`.
- After adding DWT, boot reached `BOARD_ConfigMPU()` and Renode aborted because
  the default Cortex-M model exposed only 8 MPU regions.  The local SentAI/NXP
  board code configures regions 0 through 15, so the platform now sets
  `numberOfMPURegions: 16`.

Open decision:

- if Renode can expose a simple host-file or block-device model quickly, use it
  for the spike;
- otherwise build a small board-profile storage shim that presents an in-memory
  image to FxUser/FileX and exports it at shutdown.

## Step Plan

### Step 0 - Tooling And ELF

- install or locate Renode and QEMU;
- build `examples/sentai_runtime/sentai_runtime.elf` for CM7;
- record ELF entry point, vector table address, RAM/flash sections, and linker
  memory map;
- confirm whether the RAM linker profile can boot without boot ROM/FlexSPI.

### Step 1 - Boot Boundary Inventory

Create and maintain:

```text
todo/A8_arm_emulator_boot_boundary.md
```

It must list every board service touched before the REPL is usable and classify
each as:

```text
real required | stub allowed | defer | unknown
```

### Step 2 - Renode Minimal Platform

- start from a generic Cortex-M7/RT-like platform, or create a custom RT1176
  platform description;
- map ITCM/DTCM/OCRAM/SDRAM/FlexSPI-style regions enough for the linker;
- add UART console;
- load ELF and capture first fault/boot log.

Success is not "full app": success is a reproducible fault or first UART line.

### Step 3 - Reduced SentAI Profile If Needed

If the full runtime blocks on too many peripherals before scheduler start,
create a reduced ARM-emulator profile:

- FreeRTOS;
- MicroPython REPL;
- FR;
- FS fixture;
- no camera, no USB, no Wi-Fi/Bluetooth, no EdgeTPU.

This is still ARM FreeRTOS, not POSIX SIM.

### Step 4 - Camera Provider

- implement modeled camera frame delivery at the ARM camera receiver boundary;
- support one 640x480 BMP repeated from memory;
- then support a deterministic frame sequence;
- verify Stage1Task frame counters and FR artifacts.

### Step 5 - Drone Transport

- add a second UART/socket bridge for CRTP/MAVLink/Crazyflie commands;
- record all bytes as experiment artifacts;
- only then run an end-to-end Crazyflie-style mission in emulator.

### Step 6 - TPU Choice Gate

After REPL + FS + camera + Stage1Task are stable, decide between:

- mailbox EdgeTPU host bridge;
- hardware-in-loop TPU validation;
- real USB model/pass-through research.

## Source Notes Added For This Update

- Renode custom peripherals are explicitly supported by the platform model
  workflow: https://renode.readthedocs.io/en/latest/advanced/writing-peripherals.html
- Renode supports connecting UART analyzers and host I/O fixtures:
  https://renode.readthedocs.io/en/latest/basic/using-renode.html
- QEMU documents Arm M-profile CPU/system emulation, but board support is
  machine-specific: https://qemu-project.gitlab.io/qemu/system/arm/emulation.html
- QEMU USB host passthrough exists for machines with usable USB host support,
  but that does not provide an RT1176 EHCI/PHY/board model by itself:
  https://qemu-project.gitlab.io/qemu/system/devices/usb.html
- FreeRTOS has an official QEMU MPS2/AN385 Cortex-M demo, useful as a kernel
  smoke rather than board parity:
  https://www.freertos.org/Why-FreeRTOS/Quick-connect/qemu-mps2-an385-demo
- Zephyr documents the maintained `mimxrt1170_evk@A/mimxrt1176/cm7` target and
  its RT1176 hardware/peripheral inventory, but this is board/toolchain support,
  not emulator support:
  https://docs.zephyrproject.org/latest/boards/nxp/mimxrt1170_evk/doc/index.html

## B8 Research Tasks

### 1. Inventory The ARM Firmware Boundary

Map the minimum board/peripheral functions touched before REPL is usable:

- reset vector, boot ROM assumptions, vector table placement;
- clock init and DCD/FCB/FlexSPI startup expectations;
- SDRAM/SEMC initialization;
- UART console;
- FreeRTOS tick source and interrupt priorities;
- FileX/LittleFS or storage init path;
- camera init path, but only mark what can be deferred;
- USB/EdgeTPU init path, but mark as deferred unless it blocks boot.

Deliverable:

```text
todo/A8_arm_emulator_boot_boundary.md
```

### 2. QEMU Kernel Smoke

Build or reuse a minimal Cortex-M QEMU target to prove:

- ARM GCC build;
- ARM FreeRTOS port, not POSIX;
- SysTick/PendSV/SVC task switching;
- UART output;
- GDB attach;
- at least two tasks plus one interrupt-like wakeup.

This can use QEMU MPS2/AN385 and does not need SentAI yet.

Success:

```text
qemu-system-arm boots a tiny ARM FreeRTOS ELF and logs task switches.
```

### 3. Renode SentAI Boot Spike

Try to load the current SentAI ARM ELF or a reduced SentAI profile into Renode:

- define memory map for ITCM/DTCM/OCRAM/SDRAM/FlexSPI-like regions;
- map UART;
- model enough reset/clock/storage behavior to reach boot log;
- avoid camera/USB initially;
- capture UART/FR output as artifacts.

Success ladder:

1. CPU starts from vector table and reaches early boot log.
2. FreeRTOS scheduler starts.
3. MicroPython REPL starts.
4. `import mission; mission.run()` works from an emulated/staged FS.

### 4. Camera Provider Model

Once REPL boots, add the first camera provider model:

- load one 640x480 BMP from host or emulated FS;
- write XRGB8888 into the same guest frame-buffer contract used by ARM;
- trigger the same camera ISR/notification path;
- verify Stage1Task consumes frames without needing MP to schedule them.

Initial tests:

```text
camera only -> Stage1Task RGB/gray slot increments
camera repeated static frame -> Flow reports zero motion
camera shifted sequence -> Flow reports non-zero motion
```

Do not add TPU yet.

### 5. Filesystem Strategy

Chosen after B8.9: production FileX/LevelX remains inside the guest; the
emulator supplies a persistent raw-NAND backend below `fx_nand_driver`.

Do not use FileX-in-RAM for SentAI emulator validation.  It differs from the ARM
board in the exact place where model/image transfer semantics matter.

Artifacts must land beside the experiment run exactly as B7 wanted:

```text
iterXYZ/fs_root_or_image/
iterXYZ/fr/debug.log
iterXYZ/fr/events.csv
iterXYZ/fr/scalars.csv
```

### 6. TPU Strategy

Do not model full Coral USB in the first B8 slice.

Candidate later options:

1. `sentai.tpu` not-ready in emulator mode, used only for non-TPU task tests.
2. TFLite Micro CPU model smoke in ARM emulator for tensor-path tests.
3. Host mailbox peripheral: guest writes input tensor and command to a modeled
   peripheral; host side runs PyCoral/libedgetpu and writes output tensors back.
   This tests SentAI task scheduling and output parsing but is not USB parity.
4. USB/IP or modeled Coral device: future research only; likely expensive.

## Peripheral Fidelity Matrix

| Peripheral / subsystem | First B8 fidelity | Later fidelity |
| --- | --- | --- |
| Cortex-M7 CPU | real emulated ARM core | same |
| FreeRTOS scheduler | ARM port with SysTick/PendSV/SVC | same |
| NVIC interrupts | modeled enough for UART/camera/tick | expand as needed |
| UART/REPL | emulated UART or semihost console | radio/CRTP bridge later |
| SDRAM/OCRAM | mapped memory regions | MPU/cache details if needed |
| Filesystem | FS image or host bridge | production-like block device |
| Camera | frame-buffer producer + ISR trigger | CSI/PXP register model if useful |
| PXP | initially bypassed or coarse modeled effect | register-level model only if needed |
| Stage2Task | real ARM task consuming Stage1Task output | same |
| Markers | real ARM task consuming Stage1Task/camera output | same |
| USB host | disabled/stubbed | maybe USB/IP/model later |
| Coral EdgeTPU | disabled/stubbed | mailbox or USB model later |

## First Practical Milestone

The first B8 milestone should be:

```text
ARM-emulated SentAI boots, starts FreeRTOS + MP REPL,
loads a mission from staged FS, starts virtual/emulated camera frames,
Stage1Task consumes N frames, FR artifacts are exported.
```

This gives us the key thing B7 lacked: the ARM task/ISR model, without getting
blocked immediately by Coral USB.

## Decision Gate

After the first spike, decide:

- if Renode can reach REPL within a bounded effort, continue Renode;
- if Renode setup blocks on boot/peripheral details, keep QEMU for kernel-level
  ARM regression and return to ARM hardware for full pipeline validation;
- if neither path reaches REPL quickly, stop and document exactly which
  peripherals block boot.

## Open Questions

- Can the current MCUXpresso/Sentai ELF be loaded directly, or do we need a
  reduced "emulator board" linker/profile?
- How much boot ROM/FlexSPI/DCD behavior must be bypassed or modeled?
- Does FileX require a production-like block device, or can the emulator provide
  a clean fixture layer?
- Can the camera ISR path be triggered cleanly without modeling all CSI
  registers?
- How much of the M4/shared-memory path matters for current SentAI tests?
- Is the Coral EdgeTPU worth modeling at USB level, or should emulator TPU
  parity stop at tensor/output mailbox semantics?

## Verification To Avoid B7 Repeat

Every B8 run should record:

- exact emulator and version;
- exact ELF/profile;
- full UART log;
- FR logs/artifacts;
- task list or trace if available;
- pass/fail condition;
- which peripheral models were real, stubbed, or bypassed.

No "it probably ran" results.

## B8.11 Low-Level Coral USB Alignment

2026-06-03 update: keep the Coral path low-level USB.  PyCoral is allowed only
as a host-side control benchmark, not as a SentAI implementation backend.

Comparison target:

- Coral Micro path: `EdgeTpuManager -> EdgeTpuExecutable -> TpuDriver`.
- libedgetpu path: `UsbDriver -> UsbIoRequest -> UsbMlCommands`.

Important libedgetpu behavior to mirror:

- single-endpoint mode uses bulk-out EP1 for parameters, inputs, and
  instructions;
- output activations use bulk-in EP1, events use bulk-in EP2, interrupt uses
  interrupt-in EP3;
- USB2 bulk-in is intentionally handled as small chunks, commonly 256B;
- libedgetpu keeps queued bulk-in readers installed in the worker thread
  (`usb_enable_queued_bulk_in_requests`, default queue capacity 32);
- bulk-in and bulk-out may overlap, but single-endpoint mode prevents unsafe
  bulk-out after an incomplete bulk-in.

Measured before the bulk-in cleanup:

```text
one-shot high, no desc-cache:
invoke_ms=3290
output_ticks=1706
TPU_USB_STATS in_calls=722 in_req=10927120 in_done=184040 in_us=65701 in_short=720
```

Measured after aligning SIM bulk-in to 256B and removing the artificial
post-bulk-in 1ms sleep:

```text
one-shot high, no desc-cache:
invoke_ms=1611
output_ticks=33
TPU_USB_STATS in_calls=722 in_req=184616 in_done=184040 in_us=27870 in_short=3
```

This proves the old slowdown was mostly host-side transport policy, not TPU
compute.  The next low-level step is to add a small queued bulk-in path, shaped
after libedgetpu's worker model, so those ~722 reads do not serialize through a
sync submit/wait loop.

Implemented but pending physical validation after Coral replug:

```text
--bulkin-queue-depth N
```

The queued path is SIM-only and changes only the POSIX USB transport policy for
output activations; it keeps Coral Micro `EdgeTpuManager`/`TpuDriver` as the
call path.  Default remains `queue_depth=0`.

Physical validation after Coral reboot:

```text
sync bulk-IN, 16KB OUT chunk:
runs=20 warmup=1 completed=20 fps_x100=680
invoke_ms_min=142 invoke_ms_max=151
timeouts=0 failed=0

sync bulk-IN, 64KB OUT chunk:
runs=10 warmup=1 completed=10 fps_x100=1479
invoke_ms_min=64 invoke_ms_max=68
timeouts=0 failed=0

sync bulk-IN, 160KB OUT chunk:
runs=20 warmup=1 completed=20 fps_x100=1930
invoke_ms_min=48 invoke_ms_max=55
timeouts=0 failed=0

sync bulk-IN, 160KB OUT chunk, longer confirmation:
runs=50 warmup=1 completed=50 fps_x100=1965
invoke_ms_min=47 invoke_ms_max=51
timeouts=0 failed=0
```

Decision: keep the ARM bulk-OUT chunk default unchanged and set only the
SIM/POSIX default to 160KB.  This keeps the low-level USB path and reaches
roughly 19 FPS after weights are loaded, above the 10 FPS hardware floor.

Queued bulk-IN with `--bulkin-queue-depth 4` was attempted but hung in the POSIX
async/event-pump path and was killed manually.  The Coral remained usable
afterward.  Do not enable queued bulk-IN by default; the validated path is sync
bulk-IN at 256B chunks with no post-IN sleep.

Do not use the current SentAI descriptor-cache shortcut as a benchmark source of
truth.  It skipped `InstructionHint` after warmup, but that is not equivalent to
libedgetpu's request scheduler for this COCO executable:

```text
--desc-cache --warmup 1 failed on first measured invoke
TPU_USB_STATS out_calls=4 out_req=49160 out_done=32776 timeouts=1
```

After that failure the Coral app device stopped responding to CSR read until USB
reset/replug, so desc-cache remains diagnostic-only.  FPS after weights should
be measured with normal `EdgeTpuManager` parameter caching and the low-level USB
transport fixes, not with skipped inference instructions.

### 2026-06-03 - simple REPL mission over physical Coral

Validated the simple mission path in emulator target `tpu_cat_repl`:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_cat_repl
iter61_renode_tpu_cat_repl_filex_physical_coral
pass=true
```

This exercises the intended B8 chain:

```text
FileX guest FS -> MicroPython import mission -> sentai.tpu.load/load_image/invoke
-> emu host bridge -> low-level POSIX/libusb Coral path -> physical USB Coral
-> detections returned to MicroPython
```

Mission UART markers:

```text
MODEL_SIZE 7077792
IMAGE_SIZE 921654
TPU_LOAD 0
TPU_READY True
TPU_LOAD_IMAGE 0
TPU_INVOKE 259
TPU_OUTPUTS 2
DETECTIONS_COUNT 20
MISSION_TPU_CAT_DONE
```

Host bridge timing for this one-shot mission:

```text
model stream: bytes=7077792 chunks=216 total_ms=2013 guest_read_ms=11 host_write_ms=31
image stream: bytes=921654 chunks=29 total_ms=303 guest_read_ms=0 host_write_ms=6
invoke: host_ms=418 smoke_invoke_ms=259 detection_count=20 parsed_detections=20
```

Conclusion: the simple REPL mission is now end-to-end functional from emulated
guest FS/MP code to the physical USB Coral.  The slower wall time is dominated
by Renode/emulated-time execution and staging, not by the physical TPU invoke.

### 2026-06-03 - corrected REPL FPS after low-level USB optimization

The older `tpu_fps_repl` result (`iter60`, `0.58 FPS`) was stale relative to
the current low-level USB transport and also used a too-short Renode run window
for the current boot/autorun timing.  `sentai_emu_tpu_fps_repl.resc` now runs
for the same 5s emulate-time window as `tpu_cat_repl`.

Fresh run:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_fps_repl
iter63_renode_tpu_fps_repl_filex_physical_coral
pass=true
```

Measured after `sentai.tpu.load()` and `sentai.tpu.load_image()`:

```text
TPU_FPS_COMPLETED 5
TPU_FPS_WARMUP 1
TPU_FPS_MEASURED_MS 388
TPU_FPS_X100 1288
TPU_FPS 12.88
TPU_FPS_INVOKE_MS_SUM 374
DETECTIONS_COUNT 20
```

Bridge log:

```text
model stream: bytes=7077792 chunks=216 total_ms=2403
image stream: bytes=921654 chunks=29 total_ms=350
fps bench: host_ms=770 measured_ms=388 fps_x100=1288 invoke_ms_sum=374
```

Conclusion: REPL/MP is not the steady-state invoke bottleneck in this target.
The measured FPS command crosses from MP into the host bridge once, then the
host bridge runs the native low-level POSIX/libusb TPU benchmark.  The slow
parts outside the FPS window are Renode wall time and guest FS/MMIO asset
staging; steady-state invoke currently measures about 12.9 FPS on this run.

### 2026-06-03 - FPS with cat image preloaded in emulated guest memory

Added `sentai.tpu.load_image_mem(path)` for the emulator TPU host module.  It
loads the BMP from the guest FileX FS into runtime memory once, then the FPS
path streams that resident buffer to the host bridge and benchmarks repeated
physical Coral invokes.  This approximates the "camera already prepared the
input image in memory" case.

Fresh run:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target fs_stage_assets
iter66_renode_fs_stage_assets_filex_levelx_nand
pass=true

python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_fps_mem_repl
iter67_renode_tpu_fps_mem_repl_filex_physical_coral
pass=true
```

UART markers:

```text
TPU_LOAD 0
TPU_READY True
TPU_LOAD_IMAGE_MEM 0
IMAGE_MEM_SIZE 921654
TPU_FPS_MEM_COMPLETED 5
TPU_FPS_MEM_WARMUP 1
TPU_FPS_MEM_MEASURED_MS 369
TPU_FPS_MEM_X100 1355
TPU_FPS_MEM 13.55
TPU_FPS_MEM_INVOKE_MS_SUM 357
DETECTIONS_COUNT 20
MISSION_TPU_FPS_MEM_DONE
```

Bridge log:

```text
model stream: bytes=7077792 chunks=216 total_ms=1764 guest_read_ms=8 host_write_ms=17
resident image stream: bytes=921654 chunks=29 total_ms=8 guest_read_ms=0 host_write_ms=1
fps bench: completed=5 measured_ms=369 fps_x100=1355 invoke_ms_sum=357
stage stats: input_calls=5 input_bytes=1350000 output_calls=10 output_bytes=920200 output_ticks=260
```

Conclusion: preloading the image into guest memory works and gives the same
steady-state class as the earlier REPL FPS run, slightly higher on this run
(13.55 FPS vs 12.88 FPS).  Reading the cat BMP from guest FS is not the main
steady-state bottleneck once the image has been loaded; the repeated invoke
window is dominated by the low-level Coral transaction, especially output/read
time on this profile.

### 2026-06-03 - guest-side invoke loop with resident image

Added a second measurement path: `sentai.tpu.fps_invoke(runs)`.  This keeps the
BMP resident in emulated guest memory and then calls the normal synchronous
`Invoke` bridge command once per frame from C runtime code, rather than using
the host-side batch FPS command.

Fresh run:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_fps_mem_invoke_repl
iter70_renode_tpu_fps_mem_invoke_repl_filex_physical_coral
pass=true
```

UART markers:

```text
TPU_LOAD 0
TPU_READY True
TPU_LOAD_IMAGE_MEM 0
IMAGE_MEM_SIZE 921654
TPU_FPS_MEM_LOOP_COMPLETED 5
TPU_FPS_MEM_LOOP_WARMUP 1
TPU_FPS_MEM_LOOP_MEASURED_MS 1248
TPU_FPS_MEM_LOOP_X100 400
TPU_FPS_MEM_LOOP 4.0
TPU_FPS_MEM_LOOP_INVOKE_MS_SUM 1248
DETECTIONS_COUNT 20
MISSION_TPU_FPS_MEM_LOOP_DONE
```

Important limitation: this path currently uses the bridge's single-invoke
command, which launches the low-level host smoke for each frame.  The bridge
log shows every individual invoke resends parameters/instructions:

```text
params_calls=1 params_bytes=6703232 ins_calls=2 ins_bytes=264752 input_calls=1 input_bytes=270000
```

Conclusion: the `4.0 FPS` figure is a diagnostic for the current synchronous
single-invoke bridge shape, not the final target architecture.  The stronger
baseline for "model loaded once, frame already prepared" remains the resident
image + host batch path at `13.55 FPS`.  The next useful low-level step is a
persistent host-side Coral session/bridge command that loads the model once and
then accepts per-frame input/invoke/output requests from the emulated guest,
instead of spawning the smoke process and retransmitting params per frame.

### 2026-06-03 - persistent host-side Coral session from guest REPL

Implemented a persistent host bridge session so `sentai.tpu.start()` launches
one long-lived low-level Coral host process, `sentai.tpu.fps_invoke(runs)` sends
per-frame invoke commands into that existing process, and `sentai.tpu.stop()`
shuts it down.  This keeps the `EdgeTpuManager`, executable package, model
state, and input/output tensors alive across frames.

Fresh run:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_fps_mem_session_repl
iter74_renode_tpu_fps_mem_session_repl_filex_physical_coral
pass=true
```

UART markers:

```text
TPU_LOAD 0
TPU_READY True
TPU_LOAD_IMAGE_MEM 0
IMAGE_MEM_SIZE 921654
TPU_START 0
TPU_FPS_MEM_SESSION_COMPLETED 5
TPU_FPS_MEM_SESSION_WARMUP 1
TPU_FPS_MEM_SESSION_MEASURED_MS 349
TPU_FPS_MEM_SESSION_X100 1432
TPU_FPS_MEM_SESSION 14.32
TPU_FPS_MEM_SESSION_INVOKE_MS_SUM 349
DETECTIONS_COUNT 20
TPU_STOP 0
MISSION_TPU_FPS_MEM_SESSION_DONE
```

Bridge log confirms the important behavior.  The first session invoke uploads
the model parameters:

```text
params_calls=1 params_bytes=6703232 ins_calls=2 ins_bytes=264752
```

All measured invokes then reuse the loaded state and do not retransmit weights:

```text
params_calls=0 params_bytes=0 ins_calls=1 ins_bytes=254304 input_calls=1 input_bytes=270000
```

Conclusion: the per-invoke process restart / model reload bottleneck is fixed
for the emulated REPL path.  With the image already resident in guest memory,
the full REPL -> emulated runtime -> host bridge -> physical USB Coral path is
now `14.32 FPS` for five measured invokes after one warmup, with 20 parsed
detections per frame.  No Renode or `tpu_posix_invoke_smoke` process remained
after the run.

### 2026-06-03 - low-level timing log at SendParameters / SendInputs boundary

Added `tpu_timing_repl`, a REPL autorun mission that:

- loads `/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite` from the
  emulated FileX FS;
- loads `/images/cat_640x480.bmp` from FileX into guest memory;
- starts the persistent host Coral session;
- measures the first invoke separately from steady-state invokes;
- records compact FlightRecorder files in the emulated FS at
  `/fr/events.csv` and `/fr/scalars.csv`;
- exposes bridge stats from the exact low-level boundaries:
  `SendParameters`, `SendInputs`, `SendInstructions`, `GetOutputs`,
  `ReadEvent`, plus POSIX/libusb byte/time counters.

Fresh run:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_timing_repl
iter82_renode_tpu_timing_repl_filex_physical_coral
pass=true
```

UART/timing summary:

```text
MODEL_STAGE_MS 1834
TPU_IMAGE_FS_TO_MEM_MS 72
IMAGE_MEM_SIZE 921654
TPU_FIRST_INVOKE 250
TPU_STEADY_INVOKE 0 70
TPU_STEADY_INVOKE 1 71
TPU_STEADY_INVOKE 2 71
TPU_STEADY_COMPLETED 3
TPU_STEADY_TOTAL_MS 212
TPU_STEADY_FPS 14.15
DETECTIONS_COUNT 20
FR_EVENTS_SIZE 1245
FR_SCALARS_SIZE 1314
```

First invoke, as expected, loads weights/parameters:

```text
TPU_STAGE_STATS params_calls=1 params_bytes=6703232
TPU_CALL_STATS send_params_calls=1 send_params_bytes=6703232
TPU_USB_STATS out_req=7238016 out_done=7238016 out_us=84887
```

Steady-state invokes reuse the loaded model and send only per-frame work:

```text
TPU_STAGE_STATS params_calls=0 params_bytes=0 ins_calls=1 ins_bytes=254304 input_calls=1 input_bytes=270000 output_calls=2 output_bytes=184040 event_calls=1
TPU_CALL_STATS send_params_calls=0 send_params_bytes=0 send_ins_calls=1 send_ins_bytes=254304 send_inputs_calls=1 send_inputs_bytes=270000
TPU_USB_STATS out_req=524320 out_done=524320 in_req=184616 in_done=184040 event_req=16 event_done=16 timeouts=0 failed=0
```

Important interpretation:

- `TPU_IMAGE_FS_TO_MEM_MS` is guest FileX -> guest memory staging.
- `HOST_PRELOAD_IMAGE_MS` is diagnostic bridge staging for the current
  emulator host process, not TPU wire time.
- The real TPU wire work is the `TPU_STAGE_STATS` / `TPU_CALL_STATS` /
  `TPU_USB_STATS` split above.
- This is not PyCoral.  The host bridge still uses the SentAI ARM-like
  `EdgeTpuManager -> EdgeTpuExecutable -> TpuDriver` path and the POSIX/libusb
  `USB_HostEdgeTpu*` backend.

While developing this target, a 5-invoke timing run exposed an intermittent
host-side server/USB stall on a later invoke.  The Renode bridge now has a
watchdog around persistent server commands and kills the host process by PID if
it stops producing `SERVER_DONE`, preventing dead Renode runs.  The clean
instrumented run above uses three steady invokes and leaves no lingering
Renode or `tpu_posix_invoke_smoke` processes.

### 2026-06-03 - persistent server polling fix and aggregate session FPS

The host-side `tpu_posix_invoke_smoke --server` command loop no longer uses
`vTaskDelay()` while idling on the command file.  That process is a host bridge
worker, not firmware being scheduled by the ARM emulator, so the idle polling
now uses a host sleep and prints explicit `SERVER_INVOKE_BEGIN` /
`SERVER_FPS_BEGIN` markers.  This keeps any future stall easy to classify:
before dispatch, inside one invoke, or inside an aggregate FPS run.

Direct host-only persistent smoke after the change:

```text
tpu_posix_invoke_smoke --server ... 10 x invoke
HOST_TEST_PASS completed 10 rc 0
first invoke: 266 ms, params_calls=1 params_bytes=6703232
steady invokes: 71..76 ms, params_calls=0, input_bytes=270000 each
USB timeouts=0 failed=0
```

Fresh emulator timing run:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_timing_repl
iter83_renode_tpu_timing_repl_filex_physical_coral
pass=true
```

Key timings:

```text
MODEL_STAGE_MS 1834
TPU_IMAGE_FS_TO_MEM_MS 72
TPU_FIRST_INVOKE 258
TPU_STEADY_INVOKE 0 76
TPU_STEADY_INVOKE 1 71
TPU_STEADY_INVOKE 2 72
TPU_STEADY_FPS 13.69
DETECTIONS_COUNT 20
```

The `tpu_fps_mem_session_repl` mission was then moved from a guest-side loop of
`sentai.tpu.invoke()` calls to the aggregate persistent-session command:

```text
sentai.tpu.start()
sentai.tpu.fps(5)
sentai.tpu.stop()
```

After restaging `/mission.py` into the FileX/LevelX NAND image:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target fs_stage_assets
iter86_renode_fs_stage_assets_filex_levelx_nand
pass=true, stage_bytes=15456

python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_fps_mem_session_repl
iter87_renode_tpu_fps_mem_session_repl_filex_physical_coral
pass=true
```

Bridge proof for iter87:

```text
server command fps 5 1
SERVER_FPS_BEGIN runs=5 warmup=1
FPS_BENCH runs=5 warmup=1 completed=5 measured_ms=376 fps_x100=1329 invoke_ms_sum=368
TPU_STAGE_STATS params_calls=0 params_bytes=0 ins_calls=5 ins_bytes=1271520 input_calls=5 input_bytes=1350000 output_calls=10 output_bytes=920200 event_calls=5
TPU_CALL_STATS send_params_calls=0 send_params_bytes=0 send_ins_calls=5 send_ins_bytes=1271520 send_inputs_calls=5 send_inputs_bytes=1350000
TPU_USB_STATS out_req=2621600 out_done=2621600 in_req=923080 in_done=920200 event_req=80 event_done=80 timeouts=0 failed=0
```

Conclusion: B8 currently has a complete REPL/FileX/emulator/host bridge/
physical USB Coral path for a COCO cat model.  The model is loaded once, then
steady-state detection sends per-frame instructions/input and reads output at
the `SendInstructions` / `SendInputs` / `GetOutputs` / `ReadEvent` boundary.
The latest aggregate session benchmark is `13.29 FPS` for five measured frames
after one warmup on the current host, with 20 parsed detections per measured
frame and no PyCoral in the bridge path.

### 2026-06-03 - guest EdgeTpuManager through physical USB Coral Send* bridge

Added and verified the lower-level guest-owned TPU path requested for B8.
Unlike the earlier `sentai.tpu` REPL bridge, this target keeps model parsing,
package registration, and invoke ownership inside the emulated ARM firmware:

```text
FileX guest FS
  -> guest EdgeTpuManager / EdgeTpuExecutable
  -> guest TpuDriver::SendParameters / SendInputs / SendInstructions /
     GetOutputs / ReadEvent
  -> Renode MMIO mailbox
  -> host tpu_posix_send_server
  -> POSIX/libusb USB_HostEdgeTpu* backend
  -> physical USB Coral
```

Fresh run:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_physical_send_smoke
iter91_renode_tpu_physical_send_smoke_filex_coral
pass=true
```

Guest UART proof:

```text
MODEL_BYTES=7077792
IMAGE_BYTES=921654
INPUT_BYTES=270000
OUTPUT_BYTES=182115
INVOKE 1 ms=26
INVOKE 2 ms=25
COMPLETED=2
OUTPUT_CHECKSUM=2273025060
BRIDGE_PARAMS_CALLS=1
BRIDGE_INPUT_CALLS=2
BRIDGE_INS_CALLS=3
BRIDGE_OUTPUT_CALLS=4
BRIDGE_EVENT_CALLS=3
BRIDGE_LAST_RESULT=0
TPU_PHYSICAL_SEND PASS
```

Host bridge proof from `tpu_send_bridge.log`:

```text
opened Coral USB interface=0 out=01,02,03 in=81,82 irq=83
SEND_SERVER_READY perf=low chunk_size=163840
cmd=ins    bytes=10448
cmd=params bytes=6703232
cmd=event
cmd=ins    bytes=254304
cmd=inputs bytes=270000
cmd=output bytes=7672
cmd=output bytes=176368
cmd=event
cmd=ins    bytes=254304
cmd=inputs bytes=270000
cmd=output bytes=7672
cmd=output bytes=176368
cmd=event
```

Conclusion: we now have a cap-to-cap guest-to-physical-USB-Coral smoke at the
same boundary as the ARM TPU driver.  The guest sends the real EdgeTPU package,
parameters, frame input, instructions, output reads, and events through the
physical Coral; the host side only forwards those `TpuDriver` boundary calls.
This path does not use PyCoral and does not run a high-level host invoke.

### 2026-06-03 - host wall-clock FPS for guest-to-physical Send* bridge

Added `tpu_physical_send_fps`, a 10-invoke guest-side benchmark using the same
low-level path as `tpu_physical_send_smoke`.  The runner now parses
`tpu_send_bridge.log` and writes `host_timing_summary.json` from real host
wall-clock timestamps, not from FreeRTOS/Renode guest ticks.

Fresh run:

```text
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_physical_send_fps
iter92_renode_tpu_physical_send_fps_filex_coral
pass=true
```

Guest sanity:

```text
tpu_physical_completed=10
tpu_bridge_params_calls=1
tpu_bridge_input_calls=10
tpu_bridge_ins_calls=11
tpu_bridge_output_calls=20
tpu_bridge_event_calls=11
tpu_bridge_last_result=0
```

Host wall-clock timing from the physical bridge:

```text
setup wall: 358 ms
setup params bytes: 6703232
first invoke wall: 110 ms
10 invokes measured wall: 1063 ms
FPS including first invoke: 9.41
steady invokes: 9
steady wall: 938 ms
steady FPS: 9.59
steady avg wall per invoke: 90.44 ms
steady avg bridge command sum: 88.22 ms
steady avg server/TpuDriver sum: 61.67 ms
steady avg ins bridge/server: 15.00 / 9.11 ms
steady avg inputs bridge/server: 15.44 / 9.00 ms
steady avg outputs bridge/server: 52.11 / 43.56 ms
steady avg event bridge/server: 5.67 / 0.00 ms
steady avg USB out/in/event: 6.32 / 41.88 / 0.06 ms
usb_failed=0
usb_timeouts=0
```

Interpretation:

- The reliable guest-to-physical-Coral FPS for this low-level bridge is
  `9.59 FPS` steady-state by host wall-clock, after weights are loaded once.
- The dominant steady-state cost is output readback: about `52 ms` bridge
  round-trip per frame, of which about `42 ms` is measured USB IN time.
- Guest tick timings still print in UART for smoke sanity, but they are not
  used for the FPS conclusion.

### 2026-06-03 - bulk-IN outfeed optimization

Investigated the low-level output-read bottleneck.  The initial physical Send*
benchmark used:

```text
outfeed_chunk_length = 0x20
bulkin_chunk_size = 256
```

That matches libedgetpu's USB2 High Speed short-packet workaround and ARM's
conservative RT1176/EHCI configuration, but it caused the 176368-byte output
tensor to be read as 692 bulk-IN transfers.

Two A/B runs:

```text
iter93: bulkin_chunk_size=32768, outfeed_chunk_length=0x20, pass=true
       steady FPS 8.33
       still 692 USB IN calls for the large output; worse due to oversized
       host requests against a device still outfeeding 256B chunks.

iter94: bulkin_chunk_size=1024, outfeed_chunk_length=0x80, pass=true
       steady FPS 11.75
       large output USB IN calls drop from 692 to 174.

iter98: same stable configuration after physical Coral replug from DFU mode,
       pass=true, steady FPS 11.81.  The setup wall time is longer because the
       host server performs DFU/app transition first; the steady FPS window is
       measured after params are loaded.
```

`iter94` host wall-clock summary:

```text
setup wall: 334 ms
first invoke wall: 83 ms
10 invokes measured wall: 864 ms
FPS including first invoke: 11.57
steady invokes: 9
steady wall: 766 ms
steady FPS: 11.75
steady avg wall per invoke: 71.44 ms
steady avg bridge command sum: 68.56 ms
steady avg server/TpuDriver sum: 42.44 ms
steady avg output bridge/server: 32.33 / 24.22 ms
steady avg USB out/in/event: 6.25 / 23.73 / 0.05 ms
usb_failed=0
usb_timeouts=0
```

Post-replug clean validation (`iter98`) is nearly identical:

```text
steady FPS: 11.81
steady avg wall per invoke: 71.44 ms
steady avg output bridge/server: 32.78 / 24.33 ms
steady avg USB out/in/event: 6.33 / 23.85 / 0.05 ms
usb_failed=0
usb_timeouts=0
```

The code now keeps ARM on `outfeed_chunk_length=0x20`, but lets the
POSIX/libusb server opt into libedgetpu's forced-largest bulk-IN path with:

```text
--outfeed-chunk-length 0x80 --bulkin-chunk-size 1024
```

Important failed candidates:

- `bulkin_queue_depth=4` hangs at the first output read.  Do not use it in this
  bridge yet.
- `perf=high` with `outfeed=0x80` destabilized the physical Coral after the
  first invoke; subsequent driver initialization failed at `read omc0_00`.
  The USB reset attempt also blocked, so a physical replug/restart is required
  before more Coral tests.
- The Renode bridge was fixed so `SEND_SERVER_DONE ... rc=1` is treated as
  failure instead of success.  Before this fix, a failed host command could be
  falsely acknowledged to the guest.

### 2026-06-03 - bulk-OUT chunk sweep against libedgetpu constants

Source constants checked:

- `coralmicro/libs/tpu/edgetpu_driver.cc` had `kMaxBulkBufferSize=32 KB`,
  ARM default `g_sentai_tpu_chunk_size=36 KB`, and SIM/POSIX default
  `160 KB`.
- `/home/bogdan/work/libedgetpu/driver/beagle/beagle_usb_driver_provider.cc`
  defaults `USB_MAX_BULK_OUT_TRANSFER=1 MB`, `USB_MAX_NUM_ASYNC_TRANSFERS=3`,
  `USB_BULK_IN_QUEUE_CAPACITY=32`, and has
  `USB_FORCE_LARGEST_BULK_IN_CHUNK_SIZE`.
- `/home/bogdan/work/libedgetpu/driver/usb/usb_driver.h` confirms the USB
  options: max bulk-out transfer, 1024-byte max bulk-IN chunk, queued bulk-IN,
  and overlapping bulk-in/out.

The B8 physical-send bridge is now environment-configurable:

```text
SENTAI_TPU_SEND_PERF
SENTAI_TPU_SEND_CHUNK_SIZE
SENTAI_TPU_SEND_BULKIN_CHUNK_SIZE
SENTAI_TPU_SEND_OUTFEED_CHUNK_LENGTH
SENTAI_TPU_SEND_BULKIN_QUEUE_DEPTH
```

Sweep setup: `perf=low`, `outfeed_chunk_length=0x80`,
`bulkin_chunk_size=1024`, 10 invokes after one parameter load.

```text
iter99   chunk=32 KB    setup=925 ms  steady= 7.60 FPS  usb_out=21.34 ms  usb_in=25.71 ms
iter100  chunk=64 KB    setup=578 ms  steady= 9.11 FPS  usb_out=13.51 ms  usb_in=25.57 ms
iter101  chunk=128 KB   setup=394 ms  steady=10.44 FPS  usb_out= 7.76 ms  usb_in=25.35 ms
iter105  chunk=160 KB   setup=356 ms  steady=10.99 FPS  usb_out= 6.28 ms  usb_in=23.77 ms
iter103  chunk=256 KB   setup=295 ms  steady=11.00 FPS  usb_out= 5.21 ms  usb_in=22.36 ms
iter104  chunk=512 KB   setup=247 ms  steady=11.14 FPS  usb_out= 4.11 ms  usb_in=23.11 ms
iter102  chunk=1 MB     setup=244 ms  steady=11.34 FPS  usb_out= 4.09 ms  usb_in=25.53 ms
iter106  chunk=1 MB     setup=197 ms  steady=12.10 FPS  usb_out= 3.98 ms  usb_in=24.79 ms
```

Earlier `iter98` with the old 160 KB default measured `11.81 FPS`; the rerun
shows host/USB jitter, so small differences among 160 KB / 256 KB / 512 KB /
1 MB are not decisive from a single pass.  The robust conclusions are:

- 32 KB and 64 KB are too small for the guest-to-host-to-physical-Coral bridge.
- 128 KB is acceptable but still pays avoidable bulk-out overhead.
- 1 MB matches libedgetpu, is stable in the physical test, and gives the
  fastest model setup plus the lowest bulk-out time.  The validated default
  run (`iter106`) reached `12.10 FPS` steady by host wall-clock, so the
  bridge/server default now uses 1 MB for SIM-only bulk-out sweeps.
- Steady FPS is still mostly limited by output readback (`usb_in` /
  output bridge time), not by bulk-out once chunks are at least ~160 KB.
- ARM defaults are unchanged; the 1 MB path is SIM/POSIX/host-bridge only.

Post-commit reproducibility check (`iter107`) after the WIP checkpoint push:

```text
setup wall: 212 ms
first invoke wall: 76 ms
10 invokes measured wall: 786 ms
FPS including first invoke: 12.72
steady invokes: 9
steady wall: 697 ms
steady FPS: 12.91
steady avg wall per invoke: 65.78 ms
steady avg bridge command sum: 63.33 ms
steady avg server/TpuDriver sum: 39.22 ms
steady avg SendInstructions bridge/server: 11.89 / 6.56 ms
steady avg SendInputs bridge/server: 11.00 / 6.44 ms
steady avg GetOutputs bridge/server: 35.11 / 26.22 ms
steady avg USB out/in/event: 4.88 / 26.03 / 0.05 ms
usb_failed=0
usb_timeouts=0
```

This reproduces the B8 physical Coral bridge result after commit/push.  The
stable ceiling for the current low-level guest `Send*` -> host POSIX/libusb ->
physical USB Coral path is now about `12-13 FPS` on the host wall clock.

### 2026-06-03 - B7 FlowTask end-to-end retest from B8 checkpoint

Goal: rerun the failed S209/B7-style SIM experiment with VirtualCameraTask,
PrepTask, InferTask/TPU, and FlowTask active after the B8 physical Coral bridge
checkpoint.

Runs:

```text
iter374_pipeline_5s_flow_tpu_retest
  command: run_s209_pipeline_5s.py --duration-ms 5000 --frames 80
  result : manual stop; trace stopped at after_pipe_start
  note   : pre-fix harness used subprocess stdout=PIPE

iter375_pipeline_3s_replay_flow_tpu_retest
  command: run_s209_pipeline_5s.py --duration-ms 3000 --frames 16 --replay
  result : manual stop; trace stopped at after_pipe_start
  note   : single-frame replay reproduced the same pre-fix blockage

iter376_pipeline_5s_flow_tpu_stdout_file
  command: run_s209_pipeline_5s.py --duration-ms 5000 --frames 80
  change : harness writes sentai_sim stdout directly to sim_stdout.log
  result : manual stop; trace reached after_cam_play and before_sleep
  note   : old CAM_PLAY blockage is gone; MP did not return from sleep_ms

iter377_pipeline_3s_replay_tpu_only_stdout_file
  command: run_s209_pipeline_5s.py --duration-ms 3000 --frames 16 --replay --no-flow
  result : manual stop; trace reached after_cam_replay and before_sleep
  note   : same sleep_ms blockage without FlowTask
```

Important observations:

- The pre-fix B7 retests were almost certainly affected by a harness-level
  stdout pipe deadlock: the runner captured `sentai_sim` stdout with an unread
  `subprocess.PIPE` while TPU/flight-recorder output was verbose.  The harness
  now streams stdout directly to `sim_stdout.log`.
- After that fix, the camera playback API returns: `after_cam_play` /
  `after_cam_replay` appear in the mission trace.
- VirtualCameraTask publishes frames, PrepTask consumes them, FlowTask publishes
  flow snapshots when enabled, and InferTask queues detections.  The logs prove
  the background tasks are alive.
- The remaining SIM blockage is later: MP does not reliably return from
  `sentai.rtos.sleep_ms()` under pipeline+TPU load.
- The TPU-only control (`--no-flow`) reproduces the post-fix `sleep_ms`
  blockage, so the current blocker is not FlowTask-specific.

Current B8 checkpoint conclusion:

- We have a working Renode guest -> host bridge -> physical USB Coral path at
  the `TpuDriver::SendParameters/SendInputs/SendInstructions/GetOutputs/
  ReadEvent` boundary.
- The path is not a PyCoral shortcut: the guest still emits EdgeTPU driver
  send/read operations, and the host process executes them against the physical
  Coral through the POSIX/libusb backend.
- The validated stable default is `perf=low`, bulk-out `1 MB`,
  `outfeed_chunk_length=0x80`, and bulk-IN request `1024`.
- Latest reproducible checkpoint after commit/push: `iter107`, pass=true,
  `12.91 FPS` steady by host wall-clock, `usb_failed=0`, `usb_timeouts=0`.
- The B7-style FlowTask end-to-end retest is still not a PASS in SIM.  We
  fixed the first harness deadlock (`stdout=PIPE`), but the next blocker is
  `sentai.rtos.sleep_ms()` not returning under pipeline+TPU load.  Since the
  no-Flow control reproduces it, the next debugging step should focus on the
  SIM scheduler/sleep/host-IO interaction around InferTask rather than
  FlowTask startup.

### 2026-06-04 - B8 end-to-end REPL mission through physical USB Coral

The B8 path to run the full emulator/REPL/guest-driver/physical-Coral timing
test is:

```bash
python3 examples/sentai_runtime/experiments/s213_arm_emulator_idle/run_s213.py --target tpu_timing_repl
```

This target is the B8 emulator path, not the B7 POSIX simulator path:

- Renode boots the ARM emulator target `sentai_emu_tpu_timing_repl`.
- The guest mounts the staged FileX/LevelX NAND image.
- MicroPython starts the REPL environment and autoruns `/mission.py` from the
  emulated filesystem.
- The mission loads the model and cat BMP from guest FS, loads the image into
  guest memory, starts the persistent TPU host session, then invokes through
  `sentai.tpu`.
- The guest-side driver boundary remains the EdgeTPU-style
  `SendParameters`, `SendInputs`, `SendInstructions`, `GetOutputs`, and
  `ReadEvent` flow.
- The host bridge forwards those calls to the physical USB Coral through the
  POSIX/libusb backend.  It is not a PyCoral shortcut.

`iter109_renode_tpu_timing_repl_filex_physical_coral` proved that the mission
completed (`MISSION_TPU_TIMING_DONE`) and the physical Coral returned cat
detections, but the runner still reported `pass=false` because Renode was
killed by the generic 120s timeout after the mission had already finished.  The
cause was host wall-clock time: `emulation RunFor "7.0s"` can take nearly two
minutes on the host with the TPU bridge and FileX/REPL workload.

The runner now streams Renode stdout/stderr to `renode.log` instead of keeping
Renode attached to an unread pipe, and `tpu_timing_repl` uses a 240s timeout.
The clean repro is:

```text
iter110_renode_tpu_timing_repl_filex_physical_coral
pass=true
renode returncode=0
boot_state=0x500
heartbeat=1
repl_lines=1
last_tick=2545
```

Guest/UART timing:

```text
MODEL_SIZE                 7077792 bytes
IMAGE_SIZE                  921654 bytes
MODEL_STAGE_MS                1835
TPU_IMAGE_FS_TO_MEM_MS          72
HOST_PRELOAD_IMAGE_MS            0
TPU_START_MS                     0
TPU_FIRST_INVOKE               243
TPU_STEADY_COMPLETED             3
TPU_STEADY_TOTAL_MS            212
TPU_STEADY_FPS_X100           1415
TPU_STEADY_FPS                14.15
DETECTIONS_COUNT                20
FR_EVENTS_SIZE                1245
FR_SCALARS_SIZE               1314
```

Host bridge timing for the first invoke:

```text
params_calls=1   params_bytes=6703232
input_calls=1    input_bytes=270000
ins_calls=2      ins_bytes=264752
output_calls=2   output_bytes=184040
event_calls=2
usb_out_us=79553
usb_in_us=46401
usb_event_us=66
usb_failed=0
usb_timeouts=0
```

Steady invokes no longer resend parameters:

```text
params_calls=0
input_calls=1    input_bytes=270000
ins_calls=1      ins_bytes=254304
output_calls=2   output_bytes=184040
event_calls=1
steady invoke times: 71 ms, 71 ms, 70 ms
```

Conclusion for B8:

- The end-to-end emulator path is now functional from REPL mission on guest FS
  to the physical USB Coral.
- The measured steady rate for this REPL/FileX/guest-memory/physical-Coral
  timing target is about `14.15 FPS`.
- The earlier `iter109` failure was a harness timeout, not a TPU communication
  failure.
- Future FlowTask end-to-end work should build on this B8 emulator path rather
  than the B7 SIM harness.

### 2026-06-04 - S214 production FlowTask-only benchmark in ARM emulator

Created a new experiment:

```bash
python3 examples/sentai_runtime/experiments/s214_arm_emulator_flow_task/run_s214.py
```

This is the production FlowTask path, not the earlier B8.7c/d toy estimator:

- binary: `sentai_emu_flow_task_runtime`
- linked runtime code:
  - `examples/sentai_runtime/flow_task.cc`
  - `examples/sentai_runtime/flow_phase_corr.cc`
  - `examples/sentai_runtime/sentai_prep.cc`
- data boundary: guest task publishes into
  `SENTAI_PREP_SLOT_FLOW_GRAY_80x60`, and FlowTask consumes that slot.
- algorithm: ARM phase-correlation/CMSIS-DSP path, not the SIM SAD fallback.
- frame source for this isolated gate: host prepares one 80x60 grayscale cat
  asset and Renode loads it into guest SDRAM; the guest generates shifted
  frames from that in memory.  This isolates FlowTask before reintroducing
  VirtualCameraTask/PrepTask/FS.

Implementation notes:

- `flow_task.cc` now uses a common `SENTAI_FLOW_USE_PREP_SLOT` macro for
  `SENTAI_PLATFORM_SIM || SENTAI_ARM_EMU`, so ARM-EMU can consume prep slots
  without enabling the SIM algorithm path.
- The ARM-EMU build still uses inline ARM `usada8` and `dmb`, matching the
  Cortex-M code shape.
- The harness stops FlowTask after the benchmark, avoiding a false
  `SERR_FLOW_NOTIFY_TIMEOUT` after no more frames are published.

Reproducible passing run:

```text
examples/sentai_runtime/experiments/s214_arm_emulator_flow_task/
  iter02_renode_flow_task_runtime
pass=true
flow_detect_ok=1
flow_fail_code=0
flow_completed=24
FLOW_FPS frames=24 elapsed_ms=504 fps_x100=4761
FlowTask-only FPS = 47.61
```

Strict offset validation, including unequal X/Y motion:

```text
frame  exp_dx exp_dy   got dx_q1000 dy_q1000  conf  match
1      0      0        0            0         0     1
2     +2      0        2001         9         255   1
3      0     +2       -23           2004      255   1
4     -3     -1       -3037        -992       255   1
5      0     -3       -24          -2995      255   1
6     +4      0        4032        -26        255   1
7     -2     +3       -2019         3007      255   1
8     -3     +2       -3027         2018      255   1
9     +2     -4        2024        -4005      255   1
```

What this proves:

- Production FlowTask, running as a FreeRTOS task in ARM emulation, correctly
  detects varied 2D cat-frame offsets rather than producing random/non-zero
  motion.
- FlowTask-only throughput in this isolated slot-fed benchmark is about
  `47.61 FPS` by guest FreeRTOS tick time.

Next gate:

- Wire `VirtualCameraTask -> PrepTask -> SENTAI_PREP_SLOT_FLOW_GRAY_80x60 ->
  FlowTask` in ARM-EMU, so the next measurement is camera/prep/flow end-to-end
  instead of direct slot injection.

### 2026-06-04 - S215 FlowTask + physical Coral Send* path in parallel

Created a new experiment:

```bash
python3 examples/sentai_runtime/experiments/s215_arm_emulator_flow_tpu_parallel/run_s215.py
```

New ARM-emulator target:

```text
build_emu/emu/sentai_emu_flow_tpu_parallel
emu/renode/sentai_emu_flow_tpu_parallel.resc
```

This is the B8 emulator path, not the B7 POSIX SIM path.  The guest binary runs:

- production `FlowTask`, consuming `SENTAI_PREP_SLOT_FLOW_GRAY_80x60`;
- guest-owned `EdgeTpuManager -> TpuDriver::SendParameters/SendInputs/
  SendInstructions/GetOutputs/ReadEvent`;
- Renode MMIO bridge to the host POSIX/libusb `tpu_posix_send_server`;
- physical USB Coral plugged into the host.

The emu profile defines `SENTAI_ARM_EMU_TPU_HOST_BRIDGE=1` so
`EdgeTpuManager` skips real-board `EdgeTpuTask` power toggling and the
ConsoleM7-backed mutex `CHECK()` path.  ARM hardware builds keep the real
power/USB task path.

Two consecutive runs passed:

```text
iter01_renode_flow_tpu_parallel_filex_coral
pass=True
boot_state=0x0B00
parallel_done=1
tpu_completed=5
tpu_fail_code=0
flow_completed=24
flow_detect_ok=1
flow_fail_code=0
flow_fps_x100=4761
physical_host_steady_fps=13.245
usb_failed=0
usb_timeouts=0

iter02_renode_flow_tpu_parallel_filex_coral
pass=True
boot_state=0x0B00
parallel_done=1
tpu_completed=5
tpu_fail_code=0
flow_completed=24
flow_detect_ok=1
flow_fail_code=0
flow_fps_x100=4761
physical_host_steady_fps=13.158
usb_failed=0
usb_timeouts=0
```

Representative UART evidence:

```text
TPU_PHYSICAL_SEND_PARALLEL BEGIN
RUNS=5
MODEL_BYTES=7077792
INPUT_BYTES=270000
INVOKE 1 ms=26
INVOKE 2 ms=25
INVOKE 3 ms=26
INVOKE 4 ms=26
INVOKE 5 ms=25
TPU_PHYSICAL_SEND PASS
FLOW_PARALLEL BEGIN
FLOW_VALIDATE_PASS
FLOW_FPS frames=24 elapsed_ms=504 fps_x100=4761 ...
FLOW_DONE
FLOW_STOP rc=0
```

Host wall-clock bridge timing stayed in the same range as B8.10f/g:

- steady physical Coral invoke rate: `~13.2 FPS`;
- average steady `SendInstructions` bridge time: `~11-13 ms`;
- average steady `SendInputs` bridge time: `~11 ms`;
- average steady `GetOutputs` bridge time: `~35-36 ms`;
- no USB failed transfers or timeouts.

Conclusion: FlowTask and the guest-to-physical-USB-Coral detection path now run
simultaneously in the ARM emulator without errors, across two consecutive runs.
This verifies the B8 direction for concurrent Flow + TPU in emulator and avoids
the B7 POSIX SIM dead end.
