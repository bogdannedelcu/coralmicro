# Prompt for implementing `sentai.diag`

Implement a new diagnostics and benchmarking module called `sentai.diag` for the SentAI MicroPython runtime.

The purpose of this module is to provide reproducible runtime diagnostics for the SentAI platform, with emphasis on **execution-time measurements, memory usage, task-level behavior, and subsystem-level performance** under multiple operating configurations. The module should support **ablation-style evaluation**, meaning that the same operation should be measured across several controlled scenarios in order to isolate the effects of camera selection, resolution, model choice, storage path, and runtime load.

The module should be designed so that it can be implemented first in **MicroPython** for accessibility and rapid iteration, while allowing the timing-critical inner parts to be moved later to **C/C++** if Python overhead becomes significant.

**Important constraint:** all experiments must be **started manually from the REPL** and must run in a way that allows the user to see when they begin and when they finish. Do **not** implement experiments that depend on boot-time execution, automatic `/main.py` startup, or any other pre-REPL behavior.

---

## General design requirements

- The module name must be `sentai.diag`.
- Each experiment must be implemented as a callable function.
- Each experiment should run for a configurable number of repetitions.
- Each experiment must record **raw per-run measurements**, not only aggregates.
- Each experiment must compute summary statistics:
  - mean
  - min
  - max
  - median
  - p95
  - standard deviation when feasible
- Each experiment should optionally save raw results and summary statistics as CSV files under `/diags/`.
- The module must create `/diags/` automatically if it does not already exist.
- Each experiment should optionally print a short human-readable summary to the REPL.
- Each experiment should print clear start and finish messages so the user knows when the run is active and when it has completed.
- Each experiment should include metadata in its outputs:
  - experiment name
  - timestamp or uptime
  - repetition count
  - active camera
  - resolution
  - model path
  - runtime state
- Use `sentai.rtos.ticks_ms()` for timing initially.
- Make the API clean, readable, and suitable for repeated use from the REPL.
- Add helper functions for:
  - statistics
  - CSV generation
  - metadata generation
  - scenario naming
- Prefer simple Python code first. If any benchmark appears dominated by Python overhead, structure it so that only the timing-critical inner loop could later be moved to C/C++.
- Do **not** require boot-time setup to execute any benchmark.
- If an experiment needs temporary files or test assets and they are not already present on the filesystem, the implementing agent may provision them itself through the available MCU HTTP/web interface rather than requiring manual creation from the user.

---

## Experiments to implement

### E1 — TPU invoke latency benchmark
Measure repeated execution time of `sentai.tpu.invoke()`.

**Goal:** characterize inference latency and its variability.

**Ablation factors:**
- model path
- input source (`flash_image`, `camera_tensor`)
- warm vs cold run
- repetition count

**Inputs:**
- `model_path`
- `image_path=None`
- `use_camera=False`
- `repetitions=100`

**Outputs:**
- raw per-run inference times in ms
- summary statistics
- metadata

---

### E2 — TPU model loading benchmark
Measure execution time and memory effect of `sentai.tpu.load(path)`.

**Goal:** quantify runtime model provisioning overhead.

**Ablation factors:**
- model size
- repeated reloads
- cold vs warm loads

**Inputs:**
- `model_path`
- `repetitions=10`

**Outputs:**
- load times
- memory before/after via `sentai.rtos.heap_info()`
- summary statistics

---

### E3 — Camera-to-tensor benchmark
Measure time required by `sentai.camera.to_tensor()`.

**Goal:** quantify camera capture + preprocess + tensor preparation path.

**Ablation factors:**
- resolution
- selected camera
- streaming mode if relevant

**Inputs:**
- `camera_id`
- `resolution`
- `repetitions`

**Outputs:**
- raw times
- frame counter deltas if helpful
- summary statistics

---

### E4 — JPEG capture benchmark
Measure execution time of:
- `sentai.camera.jpeg(quality)`
- `sentai.camera.save_jpeg(path, quality)`

**Goal:** quantify image capture and compression overhead.

**Ablation factors:**
- resolution
- JPEG quality
- active camera

**Inputs:**
- `camera_id`
- `resolution`
- `quality`
- `repetitions`

**Outputs:**
- times
- JPEG sizes
- summary statistics

---

### E5 — Camera switch benchmark
Measure the overhead of `sentai.camera.select(id)`.

**Goal:** quantify the runtime cost of switching between front and back cameras.

**Ablation factors:**
- switch direction
- wait interval after switch
- idle vs loaded state

**Inputs:**
- `from_camera`
- `to_camera`
- `wait_ms=200`
- `repetitions=20`

**Outputs:**
- switch call latency
- frame counter delta before/after
- optional first successful JPEG latency after switch
- summary statistics

---

### E6 — Filesystem read benchmark
Measure execution time of `sentai.fs.read(path)`.

**Goal:** characterize storage read overhead relevant to model and image loading.

**Ablation factors:**
- file size
- file type
- repeated reads

**Inputs:**
- `path`
- `repetitions=20`

**Outputs:**
- read times
- throughput estimate
- summary statistics

---

### E7 — Filesystem write benchmark
Measure execution time of `sentai.fs.write(path, data)`.

**Goal:** characterize data logging and persistence overhead.

**Ablation factors:**
- file size
- repeated writes

**Inputs:**
- `path`
- `data`
- `repetitions=20`

**Outputs:**
- write times
- throughput estimate
- summary statistics

---

### E8 — IMU read benchmark
Measure execution time of:
- `sentai.imu.read()`
- `sentai.imu.degrees()`
- `sentai.imu.radians()`

**Goal:** quantify sensor polling overhead.

**Ablation factors:**
- idle vs loaded runtime
- repetition count

**Inputs:**
- `repetitions=100`

**Outputs:**
- raw times for each IMU access function
- summary statistics

---

### E9 — Microphone overhead benchmark
Measure latency of:
- `sentai.mic.start(seconds)`
- `sentai.mic.level()`
- `sentai.mic.save_mp3()`

**Goal:** characterize audio subsystem cost.

**Ablation factors:**
- recording duration
- save interval
- idle vs loaded state

**Inputs:**
- `seconds`
- `repetitions`

**Outputs:**
- start latency
- level call latency
- save latency
- output file size if relevant
- summary statistics

---

### E10 — Memory-state benchmark
Collect memory snapshots with `sentai.rtos.heap_info()` under controlled runtime states.

**Goal:** quantify memory usage and fragmentation.

**Ablation runtime states:**
- idle after manual start from REPL
- after camera init
- after model load
- after repeated camera-to-tensor calls
- after repeated TPU invokes
- after JPEG captures
- after combined camera + TPU load

**Inputs:**
- scenario name

**Outputs:**
- heap snapshots
- state labels
- optional CSV table

---

### E11 — CPU/task-state benchmark
Collect:
- `sentai.rtos.cpu_usage()`
- `sentai.rtos.tasks()`

**Goal:** characterize scheduler-visible behavior under different runtime scenarios.

**Ablation scenarios:**
- idle
- camera active
- TPU active
- repeated JPEG capture
- repeated camera-to-tensor + invoke loop
- microphone active

**Inputs:**
- scenario function or setup
- measurement duration if needed

**Outputs:**
- CPU usage per task
- task states
- stack high-water marks
- optional CSV summary

---

### E12 — End-to-end live loop benchmark
Measure total loop time for a minimal perception cycle such as:

- `sentai.camera.to_tensor()`
- `sentai.tpu.invoke()`
- output read via `sentai.tpu.value(...)`

**Goal:** characterize user-visible runtime performance for a complete loop.

**Ablation factors:**
- camera selection
- resolution
- model choice
- repetitions

**Inputs:**
- `camera_id`
- `resolution`
- `model_path`
- `repetitions`

**Outputs:**
- per-loop latency
- effective FPS estimate
- summary statistics

---

## Shared ablation dimensions

Use the following shared ablation vocabulary across experiments where applicable.

| Ablation factor | Example levels | Why it matters |
|---|---|---|
| camera_id | `0`, `1` | isolates front/back camera behavior |
| resolution | `320x240`, `320x320`, `640x480`, `1280x720` | measures scaling and buffer effects |
| input_source | `flash_image`, `camera_tensor` | separates storage path from live capture path |
| model_path | small vs large model | measures sensitivity to model size/structure |
| run_state | `cold`, `warm` | separates initialization cost from steady-state behavior |
| workload | `isolated`, `combined` | measures subsystem interference |
| repetitions | `10`, `50`, `100` | supports stable statistics |
| file_size | small, medium, large | measures filesystem scaling |
| jpeg_quality | `50`, `75`, `95` | measures compression tradeoffs |

---

## Module API structure

Design `sentai.diag` with:

- one function per experiment
- shared helpers
- clear naming
- reusable scenario metadata

Suggested helpers:

- `stats(samples)`  
  Return summary stats dict.

- `save_csv(path, rows)`  
  Save rows to CSV.

- `ensure_dir(path="/diags")`  
  Create output directory if missing.

- `snapshot_meta()`  
  Return common metadata dictionary.

- `snapshot_heap()`  
  Wrapper over `sentai.rtos.heap_info()`.

- `snapshot_cpu()`  
  Wrapper over `sentai.rtos.cpu_usage()`.

- `snapshot_tasks()`  
  Wrapper over `sentai.rtos.tasks()`.

- `time_call(fn, *args, **kwargs)`  
  Measure elapsed ms using `sentai.rtos.ticks_ms()`.

- `percentile(samples, p)`  
  Compute pth percentile in pure Python.

---

## Output format expectations

Each experiment should return a dictionary with this general structure:

```python
{
    "experiment": "E1_tpu_invoke",
    "meta": {...},
    "params": {...},
    "samples": [...],
    "summary": {...},
    "extra": {...}
}