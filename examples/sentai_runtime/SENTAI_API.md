# SentAI MicroPython API Reference

MicroPython module `sentai` for SentAI board v1.0 (NXP i.MX RT1176 + EdgeTPU).  
Access via `import sentai` in the REPL.

## Module Structure

```
sentai
├── run(path)              # Execute .py script from flash
├── io                     # LED / GPIO
│   ├── led_on()
│   └── led_off()
├── rtos                   # FreeRTOS system
│   ├── sleep_ms(ms)
│   ├── ticks_ms()
│   ├── tasks()
│   ├── heap_info()
│   ├── cpu_usage()
│   ├── uptime()
│   ├── dmesg(max_bytes=16384) / dmesg_clear() / dmesg_stats()
│   ├── http_stats()       # (requests, hangs, healthy): healthy = no NCM TX drop in the last 5 s
│   └── suspend(name) / resume(name)   # protected tasks refused
├── sys                    # Reset / boot-mode diagnostics
│   ├── reset()
│   ├── uid()              # 64-bit chip id (= USB iSerial)
│   ├── peek32(addr)       # ARM only
│   └── poke32(addr, v)    # ARM only, bring-up use
├── tpu                    # EdgeTPU inference
│   ├── load(path)
│   ├── load_image(path)
│   ├── invoke()
│   ├── ready()
│   ├── num_outputs()
│   ├── output_size(idx)
│   ├── output(idx)
│   ├── output_dims(idx)
│   ├── output_type(idx)
│   ├── row(oidx, row)
│   ├── value(oidx, fi)
│   └── save_output(path)
├── fs                     # Filesystem (FileX / FAT32 on microSD)
│   ├── read(path)         # <= 512 KB
│   ├── read_str(path)
│   ├── read_base64(path)
│   ├── write(path, data)
│   ├── append(path, data)
│   ├── size(path)
│   ├── exists(path)
│   ├── format()
│   ├── remove(path)
│   ├── mkdir(path)
│   ├── sync()
│   ├── stats()            # HTTP file-API slot FSM counters
│   └── ls(path)
├── sdcard                 # Block-level microSD (USDHC1)
│   ├── init() / deinit()
│   ├── info()
│   ├── probe()
│   └── hexdump(lba=0)
├── camera                 # Camera capture
│   ├── init(streaming=1)
│   ├── stop()
│   ├── jpeg(quality=75)
│   ├── to_tensor()
│   ├── save_jpeg(path, q=75)
│   ├── resolution()
│   ├── set_resolution(w, h)
- `sentai.camera.set_hw(w, h)` → int. Select the SENSOR geometry before `init()`: `(640, 480)` (default) or `(1280, 960)` SXGA. 0 ok, -1 unsupported, -2 camera already initialised (needs `sys.reset()`). W24-T11.  SXGA fps keys for `init(1, fps)` = the frame rate the CSI frame counter measures (SDK patch 0007, re-keyed 2026-11-01): `15` → 15.0 fps, `22` → 22.6 fps (the pre-2026-11-01 "30" row; the only row the legacy 4 B/px capture may run), `30` → 30.0 fps (needs `camera.packed420(1)`; legacy at 30 returns **-12**, it starved the SDRAM and hung the board in EXP-s260).  `init()` codes: 0 ok, -10 unknown fps key, -11 already initialised at another fps, -12 legacy SXGA at 30.  Lockstep (EXP-s264, option B + OCRAM ring): key 15 → the TPU processes every camera frame (14.98/14.98, 1805/1806 in 120 s); key 22 → 16.1 of 22.5; key 30 → 16.3 of 30.  Logs from builds 1742–1755 use the old keys 36 (= 30.0 fps) and 18 (= 15.0 fps).
- `sentai.camera.csi_mode([bus, cr18_or, cr20_or, wdiv])` → tuple `(bus, cr18_or, CR1, CR3, CR18, cr20_or, wdiv)`. EXP-s259 capture-layer probe, set before `init()`: CSI receiver data bus 24 (default, 4 B/px XRGB/XYUV), 16 (2 B/px) or 8 (1 B/px); raw CSICR18 / CSICR20 bits OR'd in after `CSI_Init()`; `wdiv` 2 halves the receiver width for streams carrying two pixels per pixel-link cycle. Downstream consumers (PXP, JPEG, flow, pipeline) still assume 4 B/px — diagnostics only. W24-T14.
- `sentai.camera.csi_geom(w_cycles, h_lines)` → int. EXP-s259: receiver-only geometry override for the next `init()` (0, 0 = derive from the sensor geometry), used when the OV5640 ISP output size is re-programmed after init. W24-T14.
- `sentai.camera.csi_settle(v)` → int. EXP-s259: MIPI D-PHY T-HSSETTLE override for the next `init()` (0 = table lookup). W24-T14.
- `sentai.camera.packed420([on])` → int. EXP-s260 option B: capture the OV5640 "YUV420 for MIPI only" stream at 2 B/px (`[C,Y1,Y2,0]` per pixel pair; receiver at half width; sensor 0x4300=0x5F plus 0x460C/0x3824 on SXGA rows, written to the active camera after init) and let the detection PrepTask run the PXP byte-drop (`[C,Y1,Y2]`, 1.5 B/px) instead of the XYUV repack. Set before `init()`. Diagnostics / speed evaluation: JPEG, flow and markers assume XRGB/XYUV and are REFUSED in packed mode since build 1762 (W24-T20): `jpeg()` / `save_jpeg()` raise `cam capture fail` (firmware -7), `flow.start(0|1)` raises `flow.start failed (-4)` (SERR 0x0E21), `pipeline.start` with a single-input RGB model returns -9 (SERR 0x0448 val 1), the aux prep slots (aruco / whycon / places / prep-slot flow) are not produced (SERR 0x0448 val 2 once) and the zero-copy grey grab returns -1; `peek_col(x, 1)`, `save_raw`, `frame_log` and the dual-input YUV pipeline work. W24-T16.
- `sentai.camera.pxp_block16(on)` → int. PXP 16x16 processing blocks for the camera repack passes (default OFF = SDK 8x8). Faster repack under CSI load without the TPU (−30–40 %), slower USB transfer with the TPU streaming (invoke +10 %) — camera-only workloads. W24-T16.
- `sentai.camera.chase_test(n)` → tuple. EXP-s261: PXP repack that follows the CSI writer slice by slice inside one frame (pipeline stopped); returns `(ok, n, detect_us, eof_us, done_after_eof_us, cmp_equal, same_buffer, rows_per_slice, ready_us_k, pxp_us_k, ...)`. W24-T15.
- `sentai.pipeline.ocram_ring([on])` → int. EXP-s262: stream the packed-YUV420 SXGA TPU input through a 3-slot OCRAM ring (PXP producer slice by slice, USB consumer inside `SendInputs`) instead of the SDRAM staging buffer; one frame in flight; camera must be initialised. SXGA option B 13.0 → 15.6 fps, invoke 74 → 61 ms. Default off. W24-T15.
- `sentai.pipeline.chase_prep([on])` → int. EXP-s263: with the OCRAM ring on, PrepTask repacks the frame the CSI is still writing (slice markers) instead of the completed frame: last line → TPU input on the wire 75 → 20 ms, fps −17 % (USB paced by the camera). Default off. `sentai.pipeline.chase_stats()` → `(ok, begin_fail, slice_fail)`. W24-T15.
- `sentai.pipeline.ring_stats([reset])` → `(frames, urbs, consumer_wait_us, consumer_waits, consumer_timeouts, producer_timeouts)`. W24-T15.
- `sentai.pipeline.dump_staging(path, slot=0, nbytes=0)` → int. EXP-s260: write the last PXP output for the TPU (staging slot) to the user partition for layout checks. W24-T16.
- `sentai.camera.save_raw(path, offset=0, nbytes=w*h*4)` → int bytes written. EXP-s259: dump a slice of the latest raw frame buffer exactly as the CSI receiver stored it (XYUV, packed YUV420, ...) to the user partition; pull it with `GET /api/raw/<path>` (keep slices under 1 MB). W24-T14.
│   ├── native_res()
│   ├── select(id)
│   ├── frame_count() / sensor_frames()
│   ├── peek_col(x=40) / frame_log(n=64)   # EXP-s251 diagnostics
│   ├── pretrim() / genlock(sync_start=0) / phase()           # dual-sensor phase alignment (EXP-s251)
│   ├── genlock_auto([mode]) / autotune([on])                # init behaviour / on-the-fly re-alignment
│   ├── stats()            # switch / grab / stall counters
│   └── power_rails(on=1)  # PMIC camera LDOs (board rev 2026-10)
├── imu                     # LIS2DUX12 accelerometer
│   ├── init()
│   ├── read()
│   ├── degrees()
│   └── radians()
├── mic                     # PDM microphone recording (ring buffer)
│   ├── start(seconds=10)
│   ├── stop()
│   ├── recording()
│   ├── samples()
│   ├── level()
│   └── save_mp3()
└── usb                    # USB mass storage
    └── drive(on)
```

---

## Script Execution

### Importing `.py` files from flash

Python's standard `import` statement works with files stored on the LittleFS user partition.  
The interpreter searches `sys.path` which is set to `['/', '/lib']` at boot.

```python
# If /lib/utils.py exists on flash:
import utils
utils.my_function()

# Or from a subdirectory (package):
# /lib/mypackage/__init__.py
import mypackage
```

> **Note**: Imported modules are cached. Use `sentai.run()` to re-execute a file without caching.

### `sentai.run(path)` → None
Read a `.py` file from flash and execute it in the current REPL context.  
Unlike `import`, this always re-reads and re-executes the file (no caching).  
Variables and functions defined in the script become available in the REPL.
```python
>>> sentai.run("/scripts/setup.py")
>>> sentai.run("/test.py")
```

### Auto-run `/main.py` at boot

If a file named `/main.py` exists on the user partition, it is automatically executed at boot **before** the REPL starts. This is the standard MicroPython convention.

- The script runs with Ctrl+C support — press Ctrl+C to interrupt and drop to REPL.
- If `main.py` finishes normally, the REPL starts afterwards.
- If `main.py` has an infinite loop, Ctrl+C will break out of it.

```python
# Example /main.py — blink LED 5 times then drop to REPL
import sentai
for i in range(5):
    sentai.io.led_on()
    sentai.rtos.sleep_ms(200)
    sentai.io.led_off()
    sentai.rtos.sleep_ms(200)
print("Ready!")
```

To upload `main.py`, use USB mass storage or the filesystem API:
```python
>>> sentai.fs.write("/main.py", "import sentai\nsentai.io.led_on()\n")
True
```

To disable auto-run, delete the file:
```python
>>> sentai.fs.remove("/main.py")
True
```

---

## sentai.io — LED / GPIO

### `sentai.io.led_on()`
Turn on the user LED.
```python
>>> sentai.io.led_on()
```

### `sentai.io.led_off()`
Turn off the user LED.
```python
>>> sentai.io.led_off()
```

---

## sentai.rtos — FreeRTOS System

### `sentai.rtos.sleep_ms(ms)`
Sleep for `ms` milliseconds (FreeRTOS vTaskDelay).
```python
>>> sentai.rtos.sleep_ms(500)
```

### `sentai.rtos.ticks_ms()` → int
Get system uptime in milliseconds.
```python
>>> sentai.rtos.ticks_ms()
123456
```

### `sentai.rtos.tasks()` → list
List all FreeRTOS tasks. Returns list of `(name, state, priority, stack_hwm)` tuples.  
State: `"running"`, `"ready"`, `"blocked"`, `"suspended"`, `"deleted"`.  
`stack_hwm` = minimum free stack words since creation (high water mark).
```python
>>> sentai.rtos.tasks()
[('mp_repl', 'running', 6, 812), ('IDLE', 'ready', 0, 118), ('usb', 'blocked', 5, 340), ...]
>>> for name, state, pri, hwm in sentai.rtos.tasks():
...     print(f"{name:16s} {state:10s} pri={pri} hwm={hwm}")
mp_repl          running    pri=6 hwm=812
IDLE             ready      pri=0 hwm=118
usb              blocked    pri=5 hwm=340
```

### `sentai.rtos.heap_info()` → dict
Return memory usage information for both the FreeRTOS system heap (newlib malloc) and MicroPython's GC heap.
```python
>>> sentai.rtos.heap_info()
{'rtos_free': 1234567, 'gc_total': 65536, 'gc_used': 2048, 'gc_free': 63488,
 'gc_max_free': 63232, 'gc_max_block': 63232, 'gc_num_1block': 12}
```
| Key | Description |
|-----|-------------|
| `rtos_free` | Free bytes in FreeRTOS/newlib heap (pvPortMalloc arena + sbrk remainder) |
| `gc_total` | MicroPython GC heap total (64 KB) |
| `gc_used` | MicroPython GC heap in use |
| `gc_free` | MicroPython GC heap free |
| `gc_max_free` | Largest contiguous free GC block |
| `gc_max_block` | Maximum block size that can be allocated |
| `gc_num_1block` | Number of 1-block allocations |

### `sentai.rtos.cpu_usage()` → list
Return CPU usage per FreeRTOS task as a list of `(name, percent)` tuples.  
Uses FreeRTOS runtime stats (`configGENERATE_RUN_TIME_STATS=1`).  
Percent is integer 0-100, measured since boot.
```python
>>> sentai.rtos.cpu_usage()
[('IDLE', 97), ('mp_repl', 2), ('usb', 0), ('edgetpu', 0), ...]
>>> for name, pct in sentai.rtos.cpu_usage():
...     if pct > 0:
...         print(f"{name:16s} {pct}%")
IDLE             97%
mp_repl          2%
```

### `sentai.rtos.uptime()` → int
Return seconds since board boot (from `xTaskGetTickCount()`).
```python
>>> sentai.rtos.uptime()
42
```

---

## sentai.tpu — EdgeTPU Inference

### `sentai.tpu.load(path)` → int
Load a TFLite model from flash (LittleFS) and create the EdgeTPU interpreter.  
Can be called multiple times — reloading frees the previous model.  
Returns 0 on success, negative on error:
- **-1**: EdgeTPU not initialized
- **-2**: File not found / read error
- **-3**: AllocateTensors() failed
- **-4**: Model must have exactly one input tensor
```python
>>> sentai.tpu.load("/models/ssd_mobilenet_v2.tflite")
0
>>> sentai.tpu.ready()
True
```

### `sentai.tpu.urb_sync_stats()` → dict
  Since build 1708 the dict also carries `desync_pending/desync_count` (an invoke transfer failed; the next `tpu.load` power-cycles the TPU, SERR 0x0B76), `usb_attach/usb_detach`, `max_wait_us` per tag, `events` (ring of slow/timed-out/errored/refused URBs: seq, kind, tag, bytes, wait_us, usb_status, t_us, bytes_done) and `ehci` (qTD error bits from SDK patch 0006: xacterr, babble, buffer_err, missed_uf, halted_only, total, last_status, last_sofar, last_len, last_t_us).
Synchronous-URB counters of the Edge TPU USB driver (OP-S10-W24-T6, build 1702):
`entered` / `gave` (completion callbacks seen / semaphore gives), `stale`
(completions of orphaned, timed-out URBs that were ignored by the sequence
token), `take_failed` (URB waits that timed out = frames dropped, SERR 0x0B6x),
`take_ok`, `csr_timeouts` and `csr_mutex_timeouts` (control-transfer path).
After any `take_failed`/`csr_timeouts` increment the TPU usually needs
`sentai.tpu.recover()` before invokes succeed again.
```python
>>> sentai.tpu.urb_sync_stats()
{'entered': 81240, 'gave': 81240, 'stale': 0, 'take_failed': 0, 'take_ok': 81240, 'csr_timeouts': 0, 'csr_mutex_timeouts': 0}
```

### `sentai.tpu.load_image(path)` → int
Load an image from flash into the model's input tensor.  
Supports raw RGB and JPEG (auto-detected by file header `FF D8 FF`).  
If the image is larger than the tensor, it is cropped from the top-left corner. If smaller, the remaining area is zero-padded.
Returns 0 on success, negative on error:
- **-1**: Model not loaded
- **-2**: Invalid input tensor
- **-3**: File not found / read error
- **-4**: Invalid JPEG header
```python
>>> sentai.tpu.load_image("/images/test.jpg")
0
>>> sentai.tpu.load_image("/images/raw_320x320.rgb")
0
```

### `sentai.tpu.ready()` → bool
Check if EdgeTPU and model are loaded and ready.
```python
>>> sentai.tpu.ready()
True
```

### `sentai.tpu.invoke()` → int
Run inference on the current input tensor. Returns inference time in ms, -1 if not ready, -2 if invoke failed.
```python
>>> ms = sentai.tpu.invoke()
>>> print(f"Inference took {ms} ms")
Inference took 12 ms
```

### `sentai.tpu.num_outputs()` → int
Number of output tensors.
```python
>>> sentai.tpu.num_outputs()
4
```

### `sentai.tpu.output_size(idx)` → int
Size in bytes of output tensor `idx`.
```python
>>> sentai.tpu.output_size(0)
40
```

### `sentai.tpu.output(idx)` → bytes
Raw bytes of output tensor `idx`.
```python
>>> data = sentai.tpu.output(0)
>>> len(data)
40
```

### `sentai.tpu.output_dims(idx)` → tuple
Shape of output tensor `idx`.
```python
>>> sentai.tpu.output_dims(0)
(1, 10, 4)
```

### `sentai.tpu.output_type(idx)` → int
TfLiteType enum value for output tensor `idx`.  
Common values: 1=float32, 2=int32, 9=int8, 3=uint8.
```python
>>> sentai.tpu.output_type(0)
9
```

### `sentai.tpu.row(output_idx, row)` → tuple
Get a row from a 2D/3D output tensor. Returns tuple of ints (handles int8 sign extension).
```python
>>> sentai.tpu.row(0, 0)
(-12, 45, -3, 100)
```

### `sentai.tpu.value(output_idx, flat_index)` → int
Get a single value by flat index from output tensor. Handles int8 sign correctly.
```python
>>> sentai.tpu.value(0, 5)
-23
```

### `sentai.tpu.save_output(path)` → int
Save all output tensors to a CSV file on flash. Returns 0 on success.
```python
>>> sentai.tpu.save_output("/results/output.csv")
0
```

---

## sentai.fs — Filesystem (FileX / FAT32 on the microSD card)

Since 2026-10-30 the user filesystem lives on the microSD card (USDHC1,
FileX with the fault-tolerant journal, FAT32 32 KB clusters) — the NAND
user partition is gone; only the LittleFS *system* partition remains on
NAND.  `/main.py`, `/log`, `/diags`, captures and models are all on the
card.  The card is mounted after the scheduler starts (bounded 20 s
deadline); without a card the firmware boots normally and `sentai.fs.*`
raises `OSError`.  Block-level access is `sentai.sdcard.*` (below).

Whole-file reads are capped at **512 KB** (`read_base64`: 256 KB) because
the result lands on the 1 MB MicroPython heap; larger files go over HTTP
(`/api/raw/<path>`) or the USB drive (`sentai.usb.drive(1)`).

### `sentai.fs.read(path)` → bytes
Read entire file as bytes (≤ 512 KB, see above).
```python
>>> data = sentai.fs.read("/models/model.tflite")
>>> len(data)
524288
```

### `sentai.fs.read_str(path)` → str
Read entire file as string.
```python
>>> txt = sentai.fs.read_str("/config.txt")
>>> print(txt)
threshold=0.5
```

### `sentai.fs.read_base64(path)` → str
Read file and print base64 to console (76-char lines for easy copy-paste). Also returns the base64 string.
```python
>>> sentai.fs.read_base64("/photos/test.jpg")
/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAMCAgMCAgMDAwMEAwMEBQgF...
BQUFBQUFBQUFBQUFBQUFBQUFBQcHBwcHBwcHBwcHBwcHBwcHBwcHBw==
```

### `sentai.fs.write(path, data)` → bool
Write bytes or string to file. Returns True on success.
```python
>>> sentai.fs.write("/config.txt", "threshold=0.8\n")
True
>>> sentai.fs.write("/data/blob.bin", b'\x00\x01\x02')
True
```

### `sentai.fs.size(path)` → int
Get file size in bytes. Returns -1 if not found.
```python
>>> sentai.fs.size("/photos/test.jpg")
15234
```

### `sentai.fs.exists(path)` → bool
Check if file or directory exists.
```python
>>> sentai.fs.exists("/models")
True
>>> sentai.fs.exists("/nonexistent")
False
```

### `sentai.fs.remove(path)` → bool
Remove file or empty directory.
```python
>>> sentai.fs.remove("/old_file.txt")
True
```

### `sentai.fs.mkdir(path)` → bool
Create directories recursively (like `mkdir -p`).
```python
>>> sentai.fs.mkdir("/photos/2026/march")
True
```

### `sentai.fs.stats()` → tuple
Thirteen monotonic counters from the HTTP file-API slot state machine
(`sentai_fs_task.cc`), in this order: `served_ready_cached`,
`served_fast_raw`, `served_after_wait`, `busy_serving_prev`,
`busy_fast_raw_mutex`, `busy_slow_timeout`, `busy_slow_state_drift`,
`busy_path_mismatch`, `busy_not_inited`, `enqueue_ok`, `enqueue_fail`,
`stale_result`, `busy_fast_raw_pending`.  Every `{"error":"lfs_busy"}`
reply from `/api/ls` or `/api/raw` increments exactly one `busy_*`
counter, so a host script can tell which arm refused the request.
`stale_result` counts results the FS task computed for a request the
HTTP side had already abandoned (discarded, never served).
```python
>>> sentai.fs.stats()
(12, 40, 3, 0, 0, 0, 0, 0, 0, 15, 0, 0, 0)
```

### `sentai.fs.format()` → bool
Force-reformat the user FAT32 volume on the microSD card. **Erases all user data** (models, images, scripts).  
Use this to recover from filesystem corruption (mount failures, journal errors).
```python
>>> sentai.fs.format()
True
>>> sentai.fs.ls("/")
[]
```
> **Warning**: This permanently deletes everything on the user partition. System partition (`default.elf`) is not affected.

### `sentai.fs.ls(path)` → list
List directory contents. Returns list of `(name, type, size)` tuples.  
Type: 1=file, 2=directory.
```python
>>> sentai.fs.ls("/")
[('models', 2, 0), ('config.txt', 1, 42), ('photos', 2, 0)]
>>> for name, t, sz in sentai.fs.ls("/models"):
...     print(f"{'DIR' if t==2 else f'{sz}B':>8}  {name}")
  524288B  model.tflite
      DIR  backup
```

---

## sentai.sdcard — microSD block device (USDHC1)

Block-level access to the card under the FileX volume (`sentai.fs`).
Read-only except `init`/`deinit`, which the boot path already runs; use
these for bring-up and forensics, not for data.  Board rev 2026-10:
`SD_PWREN_B` GPIO powers the slot, `VSELECT` picks the 3.3 V pad rail.

### `sentai.sdcard.init()` → bool
Power the slot and run the SD init sequence.  Idempotent.

### `sentai.sys.health()` → dict — peripheral presence and the STAT-LED error code
Build ≥ 1681.  `{'sd': True/False/None, 'cam0', 'cam1', 'imu', 'tpu', 'led_codes': [...], 'led_ms_left': N}`;
`None` = not probed yet (cameras are probed at `camera.init()`).  The **STAT
LED** (SNVS_02) blinks, for 60 s after boot, the code of every missing
peripheral in turn: **1 = microSD / user FS, 2 = camera 0, 3 = camera 1,
4 = IMU, 5 = Edge TPU** (N × 150 ms flashes, 1 s pause between codes, 2 s
between rounds), then goes dark.  Everything present = three short flashes
once.  A peripheral found missing later (e.g. a camera at `camera.init()`)
re-opens the 60 s window with its code.  Each report also logs
`SERR_SYS_PERIPH_MISSING` (0x0F11, val = code).  The user LED is untouched.

### `sentai.imu.scan()` / `reg_read(reg)` / `reg_write(reg, val)`
I2C5 diagnostics for the LIS2DUX12 (build ≥ 1678): `scan()` lists the 7-bit
addresses that ACK (expected: 0x18 IMU once awake, 0x5B PMIC, plus 0x10 and
0x1F seen on rev 2026-10); `reg_read`/`reg_write` access the IMU at 0x19
raw (negative result = LPI2C status, 902 = NAK).  `init()` now tries 0x19
then 0x18, each preceded by the deep-power-down wake write (audit F18).

### `sentai.sdcard.init(deadline_ms=5000)` → bool  /  `sentai.sdcard.init_stage()` → int
Bounded (audit F14, build ≥ 1675): `SD_Init` runs in a static worker task
and the call returns `False` after `deadline_ms` if the card does not
answer (empty socket, unwired USDHC, or a card that stopped responding —
F17); dmesg gets `SERR_LFX_SD_INIT_TIMEOUT` (0x0D39).  The worker is left
running; a later `init()` returns its result once it finishes, or `False`
immediately while it is still inside `SD_Init`.  `init_stage()` is the
progress breadcrumb of the last attempt (1–8 host setup, 9 inside
`SD_Init`, 10 returned, 100 worker entered).

### `sentai.sdcard.deinit()` → None
Power the slot down.  Do not call while the FileX volume is mounted.

### `sentai.sdcard.info()` → tuple
`(blocks, block_size, capacity_mb, high_capacity, name)`.
```python
>>> sentai.sdcard.info()
(124735488, 512, 60906, 1, 'SD64G')
```

### `sentai.sdcard.probe()` → str
Identify the partition table / filesystem from LBA 0 (and the VBR).

### `sentai.sdcard.hexdump(lba=0)` → None
Print one 512-byte sector to the console.

---

## sentai.sys — Reset and boot-mode diagnostics

### `sentai.sys.reset()` → None
Software reset after flushing the boot log and syncing the card.  On a
RAM-loaded image the board returns to the ROM bootloader (SDP).

### `sentai.sys.uid()` → str
The chip's 64-bit UNIQUE_ID (OCOTP fuse words 0x900/0x910) as 16 lower-case
hex digits.  It is also the USB **iSerial** string, so `lsusb -v` and
`/dev/serial/by-id/usb-autonomous.ro_..._<uid>-if00` identify the same
board; `sentai.version()` and the USB product string carry its last four
digits (`board XXXX`) so a plain `lsusb` tells the bench boards apart.
```python
>>> sentai.sys.uid()
'0d1db80e829358eb'
>>> sentai.version()
'SentAI v1.1 build 1679 (2026-10-31 13:40:00) board 58eb'   # v1.1 = microSD-FS line (2026-10-31); v1.0 = NAND era
```

### `sentai.sys.peek32(addr)` → int
Volatile 32-bit read of a word-aligned address (peripheral registers or
memory).  Added for EXP-s248 to read the boot-mode straps latched in
`SRC_SBMR1/2` (`0x40C04008` / `0x40C0400C`).  ARM only; SIM raises `OSError`.
```python
>>> hex(sentai.sys.peek32(0x40C0400C))
'0x9'
```

### `sentai.sys.sysfs_size(path)` / `sysfs_read(path, off, n)` / `sysfs_find(path, needle, start=0)`
Diagnostics over the NAND **system** LittleFS partition (the one the ROM /
elf_loader boots from: `/default.elf`, `/usb_ip_address`, …), not the SD
user FS.  `sysfs_size` → bytes or a negative error; `sysfs_read` → `bytes`
(n ≤ 512); `sysfs_find` → first offset ≥ `start` of `needle` (≤ 64 bytes) or
−1, scanning in 4 KB chunks.  Static SDRAM buffers, bounded by the file
size.  Used by EXP-s253 to prove that a persistent `flashtool.py` run
really updates `/default.elf` on a board that cannot boot from flash.

### `sentai.sys.poke32(addr, value)` → None
Volatile 32-bit write.  **Bring-up diagnostic only** (pad / GPIO polarity
checks): a wrong address can crash or hang the running image.  ARM only.

---

## sentai.camera — Camera

### `sentai.camera.init(streaming=1)` → int
Initialize and start the camera. Returns 0 on success.  
`streaming=1`: continuous capture mode. `streaming=0`: trigger mode.
```python
>>> sentai.camera.init()      # streaming mode (default)
0
>>> sentai.camera.init(0)     # trigger mode
0
```

### `sentai.camera.stop()` → int
Stop camera and power off.
```python
>>> sentai.camera.stop()
0
```

### `sentai.camera.select(id)` → int
Select camera. `0`=front, `1`=back. Returns 0 on success.
```python
>>> sentai.camera.select(0)   # front camera
0
>>> sentai.camera.select(1)   # back camera
0
```

### `sentai.camera.power_rails(on=1)` → int
Switch the PMIC camera LDO rails (CAM1/CAM2 1V8 + 2V8; 2V8 first on
power-up).  On board rev 2026-10 the CAM1 I²C isolation switch is enabled
by `VDD_1V8_CAM1`, so camera 1 only answers on LPI2C1 while the rails are
on.  The rails default to on; `sentai.camera.init()` does not touch them.
Returns 0 on success.

### `sentai.camera.stats()` → dict
Monotonic fault counters since boot: `switch_ok_eof`, `switch_fallback`,
`drain_timeout`, `grab_retry`, `grab_fatal`, plus the watchdog liveness
observer: `stall_count` (times the CSI stopped delivering sensor frames
for ≥ 10 s while the camera was initialised) and `stalled` (1 while such
a stall is ongoing).  A stall is reported (`SERR_CAM_STALL` 0x0A0A in
`rtos.dmesg()`, a `CAM_STALL` row in `/log/crash_NNN.log`) but never acted
on; `SERR_CAM_STALL_END` 0x0A0B marks the recovery.
```python
>>> sentai.camera.stats()
{'stall_count': 0, 'stalled': 0, 'switch_ok_eof': 4, 'switch_fallback': 0,
 'drain_timeout': 0, 'grab_retry': 0, 'grab_fatal': 0,
 'phase_us': 6, 'genlock_ok': 1, 'genlock_steps': 1, 'stale_skips': 1,
 'phase_drift': 0, 'csi_queue_faults': 0, 'pretrim_lines': 18, 'autotrims': 0,
 'flips_deferred': 1340, 'flips_executed': 1340}
```

Dual-camera fields (build ≥ 1657, EXP-s251): `phase_us` = last measured
cam1 − cam0 frame-start offset (None until measured; refreshed by the
watchdog phase monitor once per kick while `ratio(a,b)` alternates),
`genlock_ok` / `genlock_steps` = result of the last `genlock()`,
`stale_skips` = queued buffers older than two frame periods dropped by the
grab path (first grab after idle), `phase_drift` = monitor excursions
(`SERR_CAM_PHASE_DRIFT`, |phase| > ~VBLANK/2), `csi_queue_faults` = CSI
driver empty-list underflow/overflow events caught by SDK patch 0005
(`SERR_CAM_CSI_QUEUE_FAULT`; expected 0), `pretrim_lines` = lines applied by
the init-time open-loop pre-trim (0 = none), `autotrims` = blind trims
applied by the `cam_tune` task (`SERR_CAM_PHASE_TRIM`).

### How the dual-camera phase is kept (build ≥ 1660)
Both OV5640s share one 24 MHz clock, so their frame phase φ (cam1 − cam0
frame start) is fixed until a sensor restarts or a timing register is
written.  A MUX flip is clean only when the new sensor is in vertical
blanking (543 µs at VGA45); at the as-found φ ≈ +400 µs (cam0 is started one
SCCB write before cam1) the flip towards cam0 leaves ~130 µs and the MIPI
receiver can wedge (audit F16).  The firmware therefore:
1. **pre-trims at `init()`** — delays cam0 by the known offset with one
   frame-synchronised `TIMING_VTS+d` write, no MUX flip (`pretrim()`,
   `genlock_auto` mode 1, default); on success `switch_drain(0)` and
   `dirty_skip_n(0)` are set so `ratio(1,1)` runs at the full sensor rate;
2. **measures φ for free** from the production flips (watchdog phase
   monitor, `stats()["phase_us"]`) and, on two consecutive samples with
   |φ| > ~VBLANK/2, re-arms `dirty_skip_n(1)` and wakes the `cam_tune` task,
   which applies one blind trim (edges predicted from the sensor's last
   FB-done timestamp, no MUX flip), rate-limited to one per 10 s
   (`autotune(1)`, default on); when |φ| is back below ~VBLANK/4 the dirty
   skip is released again.

### `sentai.camera.ratio_unit([n])` → int
Granularity of `ratio(a, b)` (build ≥ 1684): `1` = per sensor frame
(`ratio(1,1)` → cam0, cam1, cam0, cam1 … true frame interleave), `2` = legacy
FB2 pairs (cam0, cam0, cam1, cam1 …).  Set to 1 automatically when the sensors
are phase-aligned (pre-trim / genlock) and back to 2 (with dirty-skip) while
the drift monitor protects; `stats()["ratio_unit"]` shows the current value.

### `sentai.camera.flip_delay_us([us])` → int
Delay between the CSI FB-done interrupt and the MUX flip, executed on GPT1
output compare 2 (default 40 µs).  A flip executed immediately in the CSI
ISR races the old sensor's MIPI Frame-End packet and can wedge the CSI
receiver until a reset (audit F16); `0` restores that legacy behaviour for
diagnostics only.  `stats()["flips_deferred"]` / `["flips_executed"]` count
the arms and the executed flips (equal in steady state).

### `sentai.camera.pretrim()` → int
Apply the open-loop pre-trim now (cam0 delayed by one SCCB-write worth of
lines, ~18 at VGA45).  Returns the lines applied or a negative tweak code.
Runs automatically at `init()` in `genlock_auto` modes 1 and 2.

### `sentai.camera.autotune([on])` → int
Query or set the on-the-fly re-alignment (default 1).

### `sentai.camera.genlock(sync_start=0)` → int
Bring the two OV5640s in phase so `ratio(a,b)` alternation is clean at the
full sensor rate (both sensors share one 24 MHz XCLK, so the phase is fixed
until something restarts a sensor).  Measures the phase from the FB-done log
while alternating 1:1 for ~70 frames, then delays the leading sensor by
exactly the measured number of lines with a one-frame `TIMING_VTS+d` write
synchronised to that sensor's frame edge; up to 3 steps, tolerance 60 µs,
≤ 200 lines per step.  Returns the residual phase in µs; negative = error:
`-1` camera off, `-10` detection pipeline running, `-20` sensors did not
restart / no frame edge, `-30` phase not measurable, `-40` not converged,
`-50` SCCB write failed (sensor unchanged), `-51` VTS restore failed (that
sensor is left slower; reported).  `sync_start=1` first puts both sensors in software
standby and releases them back-to-back (phase ≈ one SCCB write ≈ 400 µs by
construction) before trimming.  Runs automatically at `camera.init()` only
in `genlock_auto` mode 2 (the measurement flips the MUX, which at an
unaligned phase wedged the CSI input on several cold boots, audit F16 — mode
1 pre-trims first, without flips); typically one step, 2–3 s; on success it
sets `switch_drain(0)` and `dirty_skip_n(0)` so `ratio(1,1)` runs at ~44.7
fps instead of 22.4.  Blocking; typically 2–3 s, worst case ~20 s.  Logs
`SERR_CAM_GENLOCK` (val = steps<<16 | residual) or `SERR_CAM_GENLOCK_FAIL`
(val = |code|).

### `sentai.camera.genlock_auto([mode])` → int
What `camera.init()` does when both sensors are detected: `0` nothing,
`1` (default) open-loop `pretrim()` only (no MUX flip), `2` pre-trim followed
by the measured `genlock()` (its flips then happen at an already-aligned
phase).

### `sentai.camera.phase()` → `(phase_us, T_us)` or negative int
One ~1.6 s measurement of the inter-sensor phase (alternates 1:1 while
dequeuing, then restores the caller's ratio and camera).  Errors: `-1`
camera off, `-10` detection pipeline running, `-30` not measurable.  `T_us` is the sensor frame
period (22 288 µs at VGA45).  Diagnostic; the production value is in
`stats()["phase_us"]`.

### `sentai.camera.peek_col(x=40)` → tuple
Diagnostic for the dual-camera work (EXP-s251): dequeues ONE frame and
returns `(tag_task, tag_isr, bytes[rows], idx, ts_us, now_us)` — the B-channel
column `x` over all rows of that buffer, both per-buffer camera tags, the
buffer slot and its FB-done time (GPT1 µs) next to the current time, so a
stale buffer is visible as `now_us - ts_us`.  `rows` is the runtime frame
height (480 at VGA, 960 at SXGA) and the row pitch follows the runtime
geometry and the packed YUV420 mode (build ≥ 1757, EXP-s266; before that the
VGA constants were used and the SXGA column was two columns interleaved over
half the frame).  In packed mode byte 0 of a pixel pair is chroma, not B.

Note on JPEG capture at SXGA (build ≥ 1757): `jpeg()` / `save_jpeg()` encode
the 1280x960 frame **halved to 640x480** (the RGB staging buffer and the
MicroPython heap are sized for VGA); before 1757 every SXGA capture failed
with `cam capture fail` (-5: the source size was the compile-time VGA
constant).  In packed YUV420 mode the capture returns -7 (the frame is not
XRGB); use `save_raw` there.

### `sentai.camera.frame_log(n=64)` → list
The last `n` CSI frame-complete events as `(ts_us, idx, cam, seq)`, oldest
first (64-entry ring written by the SDK-patch hook).  Only frames that the
driver actually queued are logged: with no consumer the 3-deep queue fills
and the log stops advancing, so measure while dequeuing.  Used by the
inter-sensor phase measurement (EXP-s251).

### `sentai.camera.frame_count()` → int
Return the hardware frame sequence counter. This is a `uint32_t` incremented by the
CSI DMA interrupt handler (ISR) on every completed frame — it counts at the native
sensor frame rate (15 fps) regardless of Python activity.

- **Monotonic** — never resets, even across camera switches. Use the delta
  between two readings to count elapsed frames.
- Internally, `select()` snapshots the counter before the MUX flip. The capture
  logic waits until `frame_count() - snapshot >= 2` to guarantee a clean image.
- Wraps at 2³² (≈9 years of continuous operation at 15 fps) — unsigned arithmetic
  is safe across wrap-around.

```python
>>> sentai.camera.init()
0
>>> before = sentai.camera.frame_count()
>>> sentai.camera.select(1)
0
>>> sentai.rtos.sleep_ms(200)   # wait ~3 frames
>>> sentai.camera.frame_count() - before
3
```

### `sentai.camera.resolution()` → tuple
Capture **output** size `(width, height)` used by `jpeg()` / `save_jpeg()` (set with
`set_resolution()`, defaults to the sensor geometry).  It never describes the frame
buffers: use `native_res()` for that.
```python
>>> sentai.camera.resolution()
(320, 240)
```

### `sentai.camera.set_resolution(w, h)` → int
Set the capture output size (PXP hardware scaler) for `jpeg()` / `save_jpeg()`.  Max = the
sensor geometry (`native_res()`).  Returns 0 on success, -1 if out of range.  Since build
1761 (W24-T20) this no longer affects the pipeline, flow or zero-copy grey paths, which
always read the frame buffers at the sensor geometry.
```python
>>> sentai.camera.set_resolution(320, 240)
0
>>> sentai.camera.set_resolution(640, 480)
0
```

### `sentai.camera.native_res()` → tuple
Sensor geometry of the frame buffers `(width, height)`: `(640, 480)` by default,
`(1280, 960)` after `set_hw(1280, 960)` (build ≥ 1761; earlier builds always reported
the compile-time 640x480).
```python
>>> sentai.camera.native_res()
(1280, 960)
```

### `sentai.camera.jpeg(quality=75)` → bytes
Capture frame and return JPEG bytes at current resolution.
```python
>>> jpg = sentai.camera.jpeg()        # default quality 75
>>> len(jpg)
12345
>>> jpg = sentai.camera.jpeg(90)      # higher quality
>>> len(jpg)
23456
```

### `sentai.camera.save_jpeg(path, quality=75)` → int
Capture frame, compress to JPEG, save to filesystem. Returns bytes written.
```python
>>> sentai.camera.save_jpeg("/photos/snap.jpg")
15234
>>> sentai.camera.save_jpeg("/photos/hq.jpg", 95)
28100
```

### `sentai.camera.to_tensor()` → int
Capture frame and load directly into TPU input tensor (PXP hardware scaled to model input dimensions). Returns 0 on success.
```python
>>> sentai.camera.to_tensor()
0
>>> ms = sentai.tpu.invoke()
>>> print(f"Inference: {ms} ms")
```

---

## sentai.imu — LIS2DUX12 Accelerometer

On-board STMicroelectronics **LIS2DUX12** 3-axis accelerometer (board rev
2026-10; WHO_AM_I 0x47, replaces the LIS2DU12 of the previous revision)
connected via I2C5 (address 0x19).  Driver: `libs/lis2dux12/` over the
vendored ST PID driver.  Returns acceleration in milli-g (mg) and
temperature in °C as native floats.  Same API as before.

### `sentai.imu.init()` → int
Initialize the LIS2DU12 accelerometer. Must be called once before `read()`.  
Safe to call multiple times — subsequent calls return 0 immediately.  
Returns 0 on success, -1 on failure (I2C error or sensor not found).
```python
>>> sentai.imu.init()
0
```

### `sentai.imu.read()` → dict or None
Read current acceleration and temperature.  
Returns a dict with keys `x`, `y`, `z` (milli-g) and `temp` (°C) as floats, or `None` if data not ready.

Values are **native floats**:
- `x`, `y`, `z`: acceleration in milli-g (mg). At rest with z pointing up: x≈0.0, y≈0.0, z≈981.0 (≈1g).
- `temp`: temperature in °C. Room temperature: ~25.3.

```python
>>> sentai.imu.init()
0
>>> sentai.imu.read()
{'x': -12.5, 'y': 3.2, 'z': 981.0, 'temp': 25.3}
>>> # -12.5 mg, 3.2 mg, 981.0 mg (≈1g gravity), 25.3°C
```

#### Continuous reading example
```python
import sentai

sentai.imu.init()
for i in range(10):
    d = sentai.imu.read()
    if d:
        print(f"x={d['x']} y={d['y']} z={d['z']} T={d['temp']}")
    sentai.rtos.sleep_ms(100)
```

### `sentai.imu.degrees()` → dict or None
Read current tilt angles computed from acceleration.
Returns a dict with `pitch`, `roll` (degrees, float) and `temp` (°C), or `None` if data not ready.

- **pitch**: tilt forward/back from horizon, range ±90°. Computed as `atan2(x, √(y²+z²))`.
- **roll**: tilt left/right, range ±90°. Computed as `atan2(y, √(x²+z²))`.
- At rest with z pointing up: pitch≈0°, roll≈0°.

```python
>>> sentai.imu.init()
0
>>> sentai.imu.degrees()
{'pitch': -43.9, 'roll': -5.0, 'temp': 25.3}
```

### `sentai.imu.radians()` → dict or None
Same as `degrees()` but pitch and roll are in **radians**.

```python
>>> sentai.imu.radians()
{'pitch': -0.766, 'roll': -0.087, 'temp': 25.3}
```

#### Tilt monitoring example
```python
import sentai

sentai.imu.init()
for i in range(10):
    d = sentai.imu.degrees()
    if d:
        print(f"pitch={d['pitch']:.1f}° roll={d['roll']:.1f}° T={d['temp']:.1f}")
    sentai.rtos.sleep_ms(100)
```

#### Motion detection
```python
import sentai

sentai.imu.init()
threshold = 100.0  # 100 mg

prev = sentai.imu.read()
while True:
    sentai.rtos.sleep_ms(50)
    cur = sentai.imu.read()
    if cur and prev:
        dx = cur['x'] - prev['x']
        dy = cur['y'] - prev['y']
        dz = cur['z'] - prev['z']
        # Simple magnitude squared (avoid sqrt)
        mag2 = dx*dx + dy*dy + dz*dz
        if mag2 > threshold * threshold:
            print("Motion detected!")
            sentai.io.led_on()
            sentai.rtos.sleep_ms(200)
            sentai.io.led_off()
    prev = cur
```

---

## sentai.mic — PDM Microphone Recording

On-board PDM microphone. Records 16-bit mono PCM at 16 kHz using DMA.
A background FreeRTOS task records continuously into a **ring buffer** that
keeps the last N seconds of audio (max 10 seconds = 160 000 samples = 320 KB).
Recording never stops automatically — it wraps around, always retaining the
most recent audio. Call `save_mp3()` at any time to grab what's in the ring.

All buffers are statically allocated in SDRAM (no heap usage for audio data).

### `sentai.mic.start(seconds=10)` → int
Start continuous ring-buffer recording. Duration sets ring size (1–10 seconds, default 10).
Recording wraps around — always keeps the last N seconds.
Returns: `0` = OK, `-2` = already recording, `-3` = audio driver error.

```python
>>> sentai.mic.start()     # 10-second ring (default)
0
>>> sentai.mic.start(5)    # 5-second ring
0
```

### `sentai.mic.stop()` → int
Stop mic and power off. Returns number of samples available in ring.

```python
>>> sentai.mic.stop()
80000
```

### `sentai.mic.recording()` → bool
Check if mic is running (recording into ring).

```python
>>> sentai.mic.recording()
True
```

### `sentai.mic.samples()` → int
Number of samples available in ring buffer.
If ring has wrapped, returns full ring size. Otherwise returns samples recorded so far.

```python
>>> sentai.mic.samples()
160000
```

### `sentai.mic.level()` → int
Current RMS level in centi-dB (e.g. 4500 = 45.00 dB).
Updated every 50 ms DMA block. Useful for voice activity detection.
**Auto-starts mic in monitor mode** if not already running — no need to call `start()` first.

```python
>>> sentai.mic.level()    # auto-starts mic
4500
>>> sentai.mic.level()
3200
```

### `sentai.mic.save_mp3()` → str or None
Save ring buffer contents as an MP3 file on flash using the **shine** fixed-point
MPEG Layer III encoder (64 kbps, mono). Files auto-increment: `rec000.mp3`, `rec001.mp3`, ...

**Can be called while recording** — pauses briefly during encoding, then resumes
with a fresh ring. No need to call `stop()` first.

Returns the filename string, or `None` on error.

```python
>>> sentai.mic.save_mp3()
'/audio/rec000.mp3'
>>> sentai.mic.save_mp3()    # save again (ring has new audio)
'/audio/rec001.mp3'
```

### Example: Continuous monitoring with periodic saves

```python
sentai.mic.start()                 # 10-second ring
for i in range(5):
    sentai.rtos.sleep_ms(10000)    # wait 10 seconds
    f = sentai.mic.save_mp3()       # save last 10s as MP3
    print('saved:', f)             # recording continues!
sentai.mic.stop()
```

### Example: Voice-activated save

```python
# level() auto-starts mic in monitor mode
while True:
    lev = sentai.mic.level()
    if lev > 4000:                     # voice detected (40 dB)
        sentai.mic.start(5)            # start 5s ring recording
        sentai.rtos.sleep_ms(5000)     # record for 5 seconds
        f = sentai.mic.save_mp3()       # save as MP3
        print('saved:', f)
    sentai.rtos.sleep_ms(100)
```

---

## sentai.usb — USB Mass Storage

### `sentai.usb.drive(on)` → int
Enable or disable USB mass storage. When enabled (1), the board's NAND flash filesystem appears as a USB drive on the host PC. When disabled (0), the drive is ejected and the LFS is remounted.
- **on**: 1 = enable (unmounts LFS, enables USB mass storage)
- **on**: 0 = disable (disables USB mass storage, remounts LFS)
- Returns: 1 if enabled, 0 if disabled
- **Note**: USB drive exposes only the **user partition** (~56 MB). System files (`default.elf`) are on a separate partition and cannot be corrupted from USB.

**Important**: While USB drive is active (`on=1`), all filesystem functions (`sentai.fs.read`, `sentai.fs.write`, `sentai.fs.ls`, `sentai.tpu.load`, etc.) will raise `OSError: flash busy: call sentai.usb.drive(0) first`. You must call `sentai.usb.drive(0)` after the host unmounts the drive before accessing files from Python.  
Exception: `sentai.fs.format()` auto-disables USB first if active.

```python
>>> sentai.usb.drive(1)       # unmounts LFS, drive visible on host
1
>>> sentai.fs.ls("/")         # ERROR while USB active
OSError: flash busy: call sentai.usb.drive(0) first
>>> sentai.usb.drive(0)       # eject, remounts LFS
0
>>> sentai.fs.ls("/")         # works — sees host's changes
[('a.txt', 1, 5), ('b.txt', 1, 7)]
```

---

## sentai.mesh — Meshtastic Mesh Radio (partial)

> Full mesh API documentation TBD. Key additions below.

### `sentai.mesh.set_pose(pitch_deg, roll_deg, altitude_cm=100, heading_deg=90)` → None
Set the sensor pose that is **automatically attached** to all subsequent `send_detection()` and `send_update()` calls as a `SensorPose` sub-message in the protobuf.

| Parameter | Type | Description |
|-----------|------|-------------|
| `pitch_deg` | int | Camera tilt forward/back from horizon (−90..+90°). From `imu.degrees()['pitch']` |
| `roll_deg` | int | Camera tilt left/right (−180..+180°). From `imu.degrees()['roll']` |
| `altitude_cm` | int | Camera height above ground in cm. Default: 100 (1 m) |
| `heading_deg` | int | Compass heading (0=N, 90=E, 180=S, 270=W). Default: 90 (East) |

```python
>>> import sentai
>>> sentai.imu.init()
0
>>> d = sentai.imu.degrees()
>>> sentai.mesh.set_pose(int(d['pitch']), int(d['roll']), 100, 90)
```

#### VisionMessage protobuf structure
```protobuf
message SensorPose {
  sint32 pitch_deg   = 1;
  sint32 roll_deg    = 2;
  uint32 altitude_cm = 3;
  uint32 heading_deg = 4;
}

message VisionMessage {
  // ... existing fields ...
  SensorPose pose = 20;  // auto-attached when set_pose() was called
}
```

#### `receive_vision()` now includes pose
When receiving a vision message that includes pose, the returned dict contains extra keys:
`pitch_deg`, `roll_deg`, `altitude_cm`, `heading_deg`.

```python
>>> msg = sentai.mesh.receive_vision(5000)
>>> msg['pitch_deg']   # -44
>>> msg['altitude_cm'] # 100
>>> msg['heading_deg'] # 90
```

#### Typical detection loop with pose
```python
import sentai

sentai.imu.init()
sentai.camera.init()
sentai.camera.set_resolution(320, 320)
sentai.mesh.init()

for i in range(100):
    # Update pose from IMU (altitude & heading are constants for now)
    d = sentai.imu.degrees()
    if d:
        sentai.mesh.set_pose(int(d['pitch']), int(d['roll']), 100, 90)
    
    sentai.camera.to_tensor()
    ms = sentai.tpu.invoke()
    # ... process detections and call send_detection() ...
```

---

## Typical Workflows

### Live Object Detection Loop
```python
import sentai

sentai.camera.init()
sentai.camera.set_resolution(320, 320)

for i in range(10):
    sentai.camera.to_tensor()
    ms = sentai.tpu.invoke()
    n = sentai.tpu.value(3, 0)  # number of detections
    print(f"Frame {i}: {ms}ms, {n} objects")
    for j in range(n):
        score = sentai.tpu.value(2, j)
        cls   = sentai.tpu.value(1, j)
        print(f"  obj{j}: class={cls} score={score}")

sentai.camera.stop()
```

### Capture and Download Image
```python
import sentai

sentai.camera.init()
sentai.camera.set_resolution(640, 480)
sentai.camera.save_jpeg("/photos/capture.jpg", 85)
sentai.camera.stop()

# Print base64 to terminal for copy-paste
sentai.fs.read_base64("/photos/capture.jpg")
```

### Switch Camera and Capture
```python
import sentai

sentai.camera.init()
sentai.camera.select(0)            # front camera
sentai.camera.save_jpeg("/front.jpg")
sentai.camera.select(1)            # back camera
sentai.camera.save_jpeg("/back.jpg")
sentai.camera.stop()
```

### Load Model and Image from Flash
```python
import sentai

# Upload files via USB drive first, then:
sentai.tpu.load("/models/ssd_mobilenet_v2.tflite")
sentai.tpu.load_image("/images/test.jpg")   # JPEG auto-decompressed
ms = sentai.tpu.invoke()
print(f"Inference: {ms} ms")
n = sentai.tpu.value(3, 0)
for j in range(n):
    score = sentai.tpu.value(2, j)
    cls   = sentai.tpu.value(1, j)
    print(f"  obj{j}: class={cls} score={score}")
```

### USB Drive: Save Photos then Browse from PC
```python
import sentai

sentai.camera.init()
sentai.camera.set_resolution(640, 480)
for i in range(5):
    sentai.camera.save_jpeg(f"/photos/img{i}.jpg", 85)
sentai.camera.stop()

# Enable USB drive so host PC can browse /photos/
sentai.usb.drive(1)
# ... browse files on PC ...
# When done, disable before writing again
sentai.usb.drive(0)
```

### Import and Run Scripts from Flash
```python
import sentai

# Write a utility module to flash
sentai.fs.write("/lib/detector.py", """
import sentai

def detect(n_frames=10):
    sentai.camera.init()
    sentai.camera.set_resolution(320, 320)
    for i in range(n_frames):
        sentai.camera.to_tensor()
        ms = sentai.tpu.invoke()
        count = sentai.tpu.value(3, 0)
        print(f"Frame {i}: {ms}ms, {count} objects")
    sentai.camera.stop()
""")

# Now import and use it
import detector
detector.detect(5)

# Or re-execute a script (no caching)
sentai.run("/lib/detector.py")
```

### System Monitoring
```python
import sentai

# Check memory
mem = sentai.rtos.heap_info()
print(f"RTOS free: {mem['rtos_free']} bytes")
print(f"GC free: {mem['gc_free']}/{mem['gc_total']} bytes")

# Check CPU usage
for name, pct in sentai.rtos.cpu_usage():
    if pct > 0:
        print(f"{name:16s} {pct}%")

# Uptime
print(f"Up {sentai.rtos.uptime()} seconds")

# List all tasks
for name, state, pri, hwm in sentai.rtos.tasks():
    print(f"{name:16s} {state:10s} pri={pri} hwm={hwm}")
```

### Auto-start with main.py
```python
import sentai

# Create a main.py that runs on every boot
sentai.fs.write("/main.py", """
import sentai
print("Booting...")
sentai.io.led_on()
sentai.rtos.sleep_ms(500)
sentai.io.led_off()
print("Ready!")
""")

# On next boot, this will run automatically before REPL
```

---

## Migration from `coral` → `sentai`

| Old API (`import coral`)       | New API (`import sentai`)          |
|-------------------------------|------------------------------------|
| `coral.led_on()`             | `sentai.io.led_on()`              |
| `coral.led_off()`            | `sentai.io.led_off()`             |
| `coral.sleep_ms(ms)`         | `sentai.rtos.sleep_ms(ms)`        |
| `coral.ticks_ms()`           | `sentai.rtos.ticks_ms()`          |
| `coral.tasks()`              | `sentai.rtos.tasks()`             |
| `coral.heap()`               | `sentai.rtos.heap_info()`         |
| `coral.cpu()`                | `sentai.rtos.cpu_usage()`         |
| `coral.uptime()`             | `sentai.rtos.uptime()`            |
| `coral.load_model(path)`     | `sentai.tpu.load(path)`           |
| `coral.load_image(path)`     | `sentai.tpu.load_image(path)`     |
| `coral.invoke()`             | `sentai.tpu.invoke()`             |
| `coral.is_ready()`           | `sentai.tpu.ready()`              |
| `coral.num_outputs()`        | `sentai.tpu.num_outputs()`        |
| `coral.output_size(idx)`     | `sentai.tpu.output_size(idx)`     |
| `coral.get_output(idx)`      | `sentai.tpu.output(idx)`          |
| `coral.output_dims(idx)`     | `sentai.tpu.output_dims(idx)`     |
| `coral.output_type(idx)`     | `sentai.tpu.output_type(idx)`     |
| `coral.get_row(o, r)`        | `sentai.tpu.row(o, r)`            |
| `coral.get_val(o, i)`        | `sentai.tpu.value(o, i)`          |
| `coral.save_output(path)`    | `sentai.tpu.save_output(path)`    |
| `coral.fs_read(path)`        | `sentai.fs.read(path)`            |
| `coral.fs_read_str(path)`    | `sentai.fs.read_str(path)`        |
| `coral.fs_read_base64(path)` | `sentai.fs.read_base64(path)`     |
| `coral.fs_write(path, data)` | `sentai.fs.write(path, data)`     |
| `coral.fs_size(path)`        | `sentai.fs.size(path)`            |
| `coral.fs_exists(path)`      | `sentai.fs.exists(path)`          |
| `coral.fs_format()`          | `sentai.fs.format()`              |
| `coral.fs_remove(path)`      | `sentai.fs.remove(path)`          |
| `coral.fs_mkdir(path)`       | `sentai.fs.mkdir(path)`           |
| `coral.ls(path)`             | `sentai.fs.ls(path)`              |
| `coral.cam_init(s)`          | `sentai.camera.init(s)`           |
| `coral.cam_stop()`           | `sentai.camera.stop()`            |
| `coral.cam_jpeg(q)`          | `sentai.camera.jpeg(q)`           |
| `coral.cam_to_tensor()`      | `sentai.camera.to_tensor()`       |
| `coral.cam_save_jpeg(p, q)`  | `sentai.camera.save_jpeg(p, q)`   |
| `coral.cam_res()`            | `sentai.camera.resolution()`      |
| `coral.cam_set_res(w, h)`    | `sentai.camera.set_resolution(w, h)` |
| `coral.cam_native_res()`     | `sentai.camera.native_res()`      |
| `coral.cam_switch(id)`       | `sentai.camera.select(id)`        |
| `coral.usb_drive(on)`        | `sentai.usb.drive(on)`            |
| `coral.run(path)`            | `sentai.run(path)`                |

---

## Mounting LittleFS on Linux

The board's NAND flash is split into two LittleFS partitions:

| Partition | Blocks     | Size   | Contents                         |
|-----------|------------|--------|----------------------------------|
| System    | 12–75      | 8 MB   | `default.elf`, system config     |
| **User**  | **76–523** | **~56 MB** | Models, images, user data     |

**Only the User partition is exposed via USB mass storage.** The system partition (with `default.elf`) is completely isolated and cannot be corrupted from the host.

You can mount the user partition on Linux using `littlefs-fuse`.

### Build littlefs-fuse

> **Important**: Use the littlefs-fuse copy bundled in the coralmicro repo — it has
> LittleFS **v2.0** on-disk format, matching the firmware. Do NOT use upstream
> `littlefs-fuse` v2.6+ which writes **v2.1** metadata that the board cannot mount.

```bash
sudo apt install libfuse-dev build-essential
cd /path/to/coralmicro/third_party/littlefs-fuse
make -j$(nproc)
sudo cp lfs /usr/local/bin/littlefs-fuse
```

### Identify the USB block device

Enable USB mass storage from the Python REPL:

```python
>>> import sentai
>>> sentai.usb.drive(1)
1
```

Then on the Linux host:

```bash
lsblk
# Look for a new device (usually /dev/sda) with no partitions.
# Size should be ~64 MB (the board's NAND flash).
# You can also check dmesg:
dmesg | tail -10
# Look for: "scsi ... Direct-Access NXP SEMIHOST ..."
```

### Mount

The LittleFS parameters must match the **user partition** configuration exactly:

| Parameter      | Value    |
|----------------|----------|
| block_size     | 131072   |
| read_size      | 2048     |
| prog_size      | 2048     |
| block_count    | **448**  |
| cache_size     | 2048     |
| lookahead_size | 2048     |

```bash
sudo mkdir -p /mnt/sentai

sudo littlefs-fuse \
  --block_size=131072 \
  --read_size=2048 \
  --prog_size=2048 \
  --block_count=448 \
  --cache_size=2048 \
  --lookahead_size=2048 \
  -o allow_other \
  /dev/sda /mnt/sentai
```

> **`-o allow_other`** allows non-root users to access the mount. Requires `user_allow_other` in `/etc/fuse.conf`.

Verify:

```bash
ls /mnt/sentai/
# Should show the board's filesystem contents
```

### Copy files

```bash
# Upload a model
sudo cp my_model.tflite /mnt/sentai/models/

# Upload test images
sudo mkdir -p /mnt/sentai/images
sudo cp test.jpg /mnt/sentai/images/

# Download logs
cp /mnt/sentai/log.txt ~/
```

### Unmount

```bash
sudo umount /mnt/sentai
```

Then disable USB drive from the REPL:

```python
>>> sentai.usb.drive(0)
0
```

### Quick-mount script

Save as `~/mount_sentai.sh`:

```bash
#!/bin/bash
DEV=${1:-/dev/sda}
MNT=${2:-/mnt/sentai}

sudo mkdir -p "$MNT"
sudo littlefs-fuse \
  --block_size=131072 \
  --read_size=2048 \
  --prog_size=2048 \
  --block_count=448 \
  --cache_size=2048 \
  --lookahead_size=2048 \
  -o allow_other \
  "$DEV" "$MNT"

echo "Mounted $DEV at $MNT"
ls "$MNT"
```

```bash
chmod +x ~/mount_sentai.sh
~/mount_sentai.sh              # uses /dev/sda, /mnt/sentai
~/mount_sentai.sh /dev/sdb     # custom device
```

### Troubleshooting

| Problem | Solution |
|---------|----------|
| `mount: wrong fs type` | Device is not LittleFS or parameters are wrong. Double-check `block_size` etc. |
| `Found older minor version v2.0 < v2.1` | You built littlefs-fuse from upstream (v2.1). **Use the copy in `third_party/littlefs-fuse/`** which is v2.0 matching the firmware. v2.1 writes metadata the board can't read → mount fails → data lost. |
| `Transport endpoint is not connected` | Board was reset while mounted. Unmount (`sudo umount -f /mnt/sentai`) and remount. |
| `Permission denied` on files | Use `sudo` for all operations, or add `user_allow_other` to `/etc/fuse.conf`. |
| Device not appearing | Check `sentai.usb.drive(1)` was called. Try a different USB cable/port. Check `dmesg`. |
| `FUSE: failed to exec fusermount` | Install fuse: `sudo apt install fuse` |
| `sentai.fs.ls("/")` returns `[]` after USB | Always `sudo umount /mnt/sentai` **before** calling `sentai.usb.drive(0)`. If data is lost, the LFS version mismatch is likely the cause — rebuild littlefs-fuse from the repo. To recover: `sentai.fs.format()`. |
| `OSError: flash busy` | Call `sentai.usb.drive(0)` first. All filesystem functions raise this error while USB drive is active. |

---

## Build Instructions

### ⚠️ CRITICAL: Regenerating MicroPython QSTR Headers

When you **add, rename, or remove** any `MP_QSTR_xxx` symbol or `MP_REGISTER_MODULE()` in
`modsentai.c` (or any other MicroPython C source), you **MUST** regenerate the QSTR headers
using the MicroPython embed build system. Otherwise `import` will fail with
`ImportError: module not found` because the QSTR binary-search pool won't contain the new strings.

#### Steps

```bash
# 1. Clean stale build-embed cache (IMPORTANT - old .qstr/.module files persist otherwise)
cd /home/bogdan/work/coralmicro/examples/sentai_runtime
rm -rf build-embed

# 2. Run the MicroPython embed Makefile to regenerate all headers
#    This scans modsentai.c (via USER_C_MODULES and modules/sentai/micropython.mk),
#    extracts Q(...) / MP_QSTR_xxx / MP_REGISTER_MODULE macros,
#    generates qstrdefs.generated.h (sorted QDEF1 pool), moduledefs.h, etc.,
#    and copies them into micropython_embed/genhdr/
make -f ../../third_party/micropython/ports/embed/embed.mk \
     MICROPYTHON_TOP=../../third_party/micropython \
     USER_C_MODULES=$(pwd)/modules \
     micropython-embed-package

# 3. Clean CMake build artifacts (CMake doesn't track #include changes in GLOB'd sources)
rm -rf ../../build/examples/sentai_runtime/CMakeFiles/libmicropython.dir/
rm -f  ../../build/examples/sentai_runtime/liblibmicropython.a

# 4. Rebuild
cd ../.. && cmake --build build -t sentai_runtime -j$(nproc)

# 5. Flash
python3 scripts/flashtool.py -e sentai_runtime
```

> **Note:** Step 2 overwrites `micropython_embed/port/` files from upstream, but this
> is harmless — our custom `mp_embed_exec_str_safe()` lives in its own file
> (`mp_embed_safe.c`) outside the embed port directory, so it survives regeneration.

#### Why This Is Needed

- `micropython_embed/genhdr/qstrdefs.generated.h` contains the **QDEF1 pool** — all
  user-defined QSTR strings sorted alphabetically (C locale / ASCII byte order).
- MicroPython's `qstr.c` declares this pool with `is_sorted = true`, meaning it uses
  **binary search** to find strings. If entries are not sorted or missing, `import` fails.
- The embed Makefile (`embed.mk`) runs `makeqstrdefs.py` + `makeqstrdata.py` which:
  1. Preprocesses all `.c` sources with `gcc -E -DNO_QSTR`
  2. Extracts `Q(...)` and `MP_QSTR_xxx` references
  3. Generates the sorted QDEF0/QDEF1 entries with correct hashes and lengths
- `moduledefs.h` is also generated — it lists all `MP_REGISTER_MODULE()` entries.
  If stale, the linker will fail with `undefined reference to mp_module_xxx`.

#### Key Files

| File | Purpose |
|------|---------|
| `modules/sentai/micropython.mk` | **CRITICAL** — tells `embed.mk` to scan `modsentai.c` for QSTRs. Without this file, only core MicroPython QSTRs are generated (~41 entries) and `import sentai` fails. |
| `modsentai.c` | Main C source scanned by embed build for QSTR extraction |
| `modsentai_camera.c` | Camera sub-module (included by `modsentai.c`, not scanned directly) |
| `mp_embed_safe.c` / `mp_embed_safe.h` | `mp_embed_exec_str_safe()` — safe exec wrapper (lives outside upstream, survives QSTR regen) |
| `build-embed/genhdr/` | Intermediate generated headers (cache — delete when stale) |
| `micropython_embed/genhdr/qstrdefs.generated.h` | Final QSTR pool (QDEF0 + sorted QDEF1, ~228 entries) |
| `micropython_embed/genhdr/moduledefs.h` | Module registration (`MICROPY_REGISTERED_MODULES`) |
| `micropython_embed/port/embed_util.c` | MicroPython embed runtime (overwritten by QSTR regen — no custom code here) |
| `micropython_embed/port/micropython_embed.h` | Embed public header (overwritten by QSTR regen — no custom code here) |
| `micropython_embed/py/qstr.c` | Includes the generated header; compiled into `libmicropython.a` |

#### Serial Console

The REPL is on the **UART** serial port, not the USB CDC port:
- **`/dev/ttyUSB0`** — UART console (REPL) — use `sudo screen /dev/ttyUSB0 115200`
- **`/dev/ttyACM0`** — USB CDC (LittleFS mass storage) — NOT the REPL
