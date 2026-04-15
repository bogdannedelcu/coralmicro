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
│   └── uptime()
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
├── fs                     # Filesystem (LittleFS)
│   ├── read(path)
│   ├── read_str(path)
│   ├── read_base64(path)
│   ├── write(path, data)
│   ├── size(path)
│   ├── exists(path)
│   ├── format()
│   ├── remove(path)
│   ├── mkdir(path)
│   └── ls(path)
├── camera                 # Camera capture
│   ├── init(streaming=1)
│   ├── stop()
│   ├── jpeg(quality=75)
│   ├── to_tensor()
│   ├── save_jpeg(path, q=75)
│   ├── resolution()
│   ├── set_resolution(w, h)
│   ├── native_res()
│   ├── select(id)
│   └── frame_count()
├── imu                     # LIS2DU12 accelerometer
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

## sentai.fs — Filesystem (LittleFS)

### `sentai.fs.read(path)` → bytes
Read entire file as bytes.
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

### `sentai.fs.format()` → bool
Force-reformat the user LittleFS partition. **Erases all user data** (models, images, scripts).  
Use this to recover from filesystem corruption (e.g., NAND read errors, mount failures).
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
Current capture output resolution `(width, height)`.
```python
>>> sentai.camera.resolution()
(320, 240)
```

### `sentai.camera.set_resolution(w, h)` → int
Set capture output resolution (PXP hardware scaler). Max = native sensor resolution. Returns 0 on success.
```python
>>> sentai.camera.set_resolution(320, 240)
0
>>> sentai.camera.set_resolution(640, 480)
0
```

### `sentai.camera.native_res()` → tuple
Native sensor resolution `(width, height)`.
```python
>>> sentai.camera.native_res()
(1280, 720)
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

## sentai.imu — LIS2DU12 Accelerometer

On-board STMicroelectronics LIS2DU12 3-axis accelerometer connected via I2C5 (address 0x19).  
Returns acceleration in milli-g (mg) and temperature in °C as native floats.

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
