# Coral MicroPython API Reference

MicroPython module `coral` for Coral Dev Board Micro.  
Access via `import coral` in the REPL.

---

## System

### `coral.led_on()`
Turn on the user LED.
```python
>>> coral.led_on()
```

### `coral.led_off()`
Turn off the user LED.
```python
>>> coral.led_off()
```

### `coral.sleep_ms(ms)`
Sleep for `ms` milliseconds (FreeRTOS vTaskDelay).
```python
>>> coral.sleep_ms(500)
```

### `coral.ticks_ms()` → int
Get system uptime in milliseconds.
```python
>>> coral.ticks_ms()
123456
```

---

## TPU / Inference

### `coral.is_ready()` → bool
Check if EdgeTPU and model are loaded and ready.
```python
>>> coral.is_ready()
True
```

### `coral.invoke()` → int
Run inference on the current input tensor. Returns inference time in ms, -1 if not ready, -2 if invoke failed.
```python
>>> ms = coral.invoke()
>>> print(f"Inference took {ms} ms")
Inference took 12 ms
```

### `coral.num_outputs()` → int
Number of output tensors.
```python
>>> coral.num_outputs()
4
```

### `coral.output_size(idx)` → int
Size in bytes of output tensor `idx`.
```python
>>> coral.output_size(0)
40
```

### `coral.get_output(idx)` → bytes
Raw bytes of output tensor `idx`.
```python
>>> data = coral.get_output(0)
>>> len(data)
40
```

### `coral.output_dims(idx)` → tuple
Shape of output tensor `idx`.
```python
>>> coral.output_dims(0)
(1, 10, 4)
```

### `coral.output_type(idx)` → int
TfLiteType enum value for output tensor `idx`.  
Common values: 1=float32, 2=int32, 9=int8, 3=uint8.
```python
>>> coral.output_type(0)
9
```

### `coral.get_row(output_idx, row)` → tuple
Get a row from a 2D/3D output tensor. Returns tuple of ints (handles int8 sign extension).
```python
>>> coral.get_row(0, 0)
(-12, 45, -3, 100)
```

### `coral.get_val(output_idx, flat_index)` → int
Get a single value by flat index from output tensor. Handles int8 sign correctly.
```python
>>> coral.get_val(0, 5)
-23
```

---

## Filesystem (LittleFS)

### `coral.fs_read(path)` → bytes
Read entire file as bytes.
```python
>>> data = coral.fs_read("/models/model.tflite")
>>> len(data)
524288
```

### `coral.fs_read_str(path)` → str
Read entire file as string.
```python
>>> txt = coral.fs_read_str("/config.txt")
>>> print(txt)
threshold=0.5
```

### `coral.fs_read_base64(path)` → str
Read file and print base64 to console (76-char lines for easy copy-paste). Also returns the base64 string.
```python
>>> coral.fs_read_base64("/photos/test.jpg")
/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAAMCAgMCAgMDAwMEAwMEBQgF...
BQUFBQUFBQUFBQUFBQUFBQUFBQcHBwcHBwcHBwcHBwcHBwcHBwcHBw==
```

### `coral.fs_write(path, data)` → bool
Write bytes or string to file. Returns True on success.
```python
>>> coral.fs_write("/config.txt", "threshold=0.8\n")
True
>>> coral.fs_write("/data/blob.bin", b'\x00\x01\x02')
True
```

### `coral.fs_size(path)` → int
Get file size in bytes. Returns -1 if not found.
```python
>>> coral.fs_size("/photos/test.jpg")
15234
```

### `coral.fs_exists(path)` → bool
Check if file or directory exists.
```python
>>> coral.fs_exists("/models")
True
>>> coral.fs_exists("/nonexistent")
False
```

### `coral.fs_remove(path)` → bool
Remove file or empty directory.
```python
>>> coral.fs_remove("/old_file.txt")
True
```

### `coral.fs_mkdir(path)` → bool
Create directories recursively (like `mkdir -p`).
```python
>>> coral.fs_mkdir("/photos/2026/march")
True
```

### `coral.ls(path)` → list
List directory contents. Returns list of `(name, type, size)` tuples.  
Type: 1=file, 2=directory.
```python
>>> coral.ls("/")
[('models', 2, 0), ('config.txt', 1, 42), ('photos', 2, 0)]
>>> for name, t, sz in coral.ls("/models"):
...     print(f"{'DIR' if t==2 else f'{sz}B':>8}  {name}")
  524288B  model.tflite
      DIR  backup
```

---

## Camera

### `coral.cam_init(streaming=1)` → int
Initialize and start the camera. Returns 0 on success.  
`streaming=1`: continuous capture mode. `streaming=0`: trigger mode.
```python
>>> coral.cam_init()      # streaming mode (default)
0
>>> coral.cam_init(0)     # trigger mode
0
```

### `coral.cam_stop()` → int
Stop camera and power off.
```python
>>> coral.cam_stop()
0
```

### `coral.cam_switch(id)` → int
Switch between cameras. `0`=front, `1`=back. Returns 0 on success.
```python
>>> coral.cam_switch(0)   # front camera
0
>>> coral.cam_switch(1)   # back camera
0
```

### `coral.cam_res()` → tuple
Current capture output resolution `(width, height)`.
```python
>>> coral.cam_res()
(320, 240)
```

### `coral.cam_set_res(w, h)` → int
Set capture output resolution (PXP hardware scaler). Max = native sensor resolution. Returns 0 on success.
```python
>>> coral.cam_set_res(320, 240)
0
>>> coral.cam_set_res(640, 480)
0
```

### `coral.cam_native_res()` → tuple
Native sensor resolution `(width, height)`.
```python
>>> coral.cam_native_res()
(1280, 720)
```

### `coral.cam_jpeg(quality=75)` → bytes
Capture frame and return JPEG bytes at current resolution.
```python
>>> jpg = coral.cam_jpeg()        # default quality 75
>>> len(jpg)
12345
>>> jpg = coral.cam_jpeg(90)      # higher quality
>>> len(jpg)
23456
```

### `coral.cam_save_jpeg(path, quality=75)` → int
Capture frame, compress to JPEG, save to filesystem. Returns bytes written.
```python
>>> coral.cam_save_jpeg("/photos/snap.jpg")
15234
>>> coral.cam_save_jpeg("/photos/hq.jpg", 95)
28100
```

### `coral.cam_to_tensor()` → int
Capture frame and load directly into TPU input tensor (PXP hardware scaled to model input dimensions). Returns 0 on success.
```python
>>> coral.cam_to_tensor()
0
>>> ms = coral.invoke()
>>> print(f"Inference: {ms} ms")
```

---

## USB

### `coral.usb_drive(on)` → int
Enable or disable USB mass storage. When enabled (1), the board's NAND flash filesystem appears as a USB drive on the host PC. When disabled (0), the drive is ejected.
- **on**: 1 = enable (drive visible), 0 = disable (ejected)
- Returns: 1 if enabled, 0 if disabled
- **Note**: While USB drive is enabled, avoid writing to the filesystem from Python (risk of corruption). Disable the drive before using `fs_write`, `cam_save_jpeg`, etc.
```python
>>> coral.usb_drive(1)       # mount drive on host
1
>>> coral.usb_drive(0)       # eject / unmount
0
```

---

## Typical Workflows

### Live Object Detection Loop
```python
import coral

coral.cam_init()
coral.cam_set_res(320, 320)

for i in range(10):
    coral.cam_to_tensor()
    ms = coral.invoke()
    n = coral.get_val(3, 0)  # number of detections
    print(f"Frame {i}: {ms}ms, {n} objects")
    for j in range(n):
        score = coral.get_val(2, j)
        cls   = coral.get_val(1, j)
        print(f"  obj{j}: class={cls} score={score}")

coral.cam_stop()
```

### Capture and Download Image
```python
import coral

coral.cam_init()
coral.cam_set_res(640, 480)
coral.cam_save_jpeg("/photos/capture.jpg", 85)
coral.cam_stop()

# Print base64 to terminal for copy-paste
coral.fs_read_base64("/photos/capture.jpg")
```

### Switch Camera and Capture
```python
import coral

coral.cam_init()
coral.cam_switch(0)            # front camera
coral.cam_save_jpeg("/front.jpg")
coral.cam_switch(1)            # back camera
coral.cam_save_jpeg("/back.jpg")
coral.cam_stop()
```

### USB Drive: Save Photos then Browse from PC
```python
import coral

coral.cam_init()
coral.cam_set_res(640, 480)
for i in range(5):
    coral.cam_save_jpeg(f"/photos/img{i}.jpg", 85)
coral.cam_stop()

# Enable USB drive so host PC can browse /photos/
coral.usb_drive(1)
# ... browse files on PC ...
# When done, disable before writing again
coral.usb_drive(0)
```
