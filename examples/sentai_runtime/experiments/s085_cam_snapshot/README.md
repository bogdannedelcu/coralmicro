# s085_cam_snapshot

Single-frame VGA capture from each of the two on-board OV5640 cameras
for visual identification.  Build #1139.

## Files

- `cam0_front.jpg` -- camera 0 (I2C bus 1, MUX low; "front").
- `cam1_back.jpg`  -- camera 1 (I2C bus 2, MUX high; "back").

The two scenes are visibly different (cam0 sees a drone + kitchen,
cam1 sees a desk + monitor).  Sizes differ (~39 KB vs ~33 KB) and md5
sums confirm the JPEGs are independent.

## Capture procedure (the simple version that works)

```python
sentai.camera.init(1)

sentai.camera.select(0)
sentai.rtos.sleep_ms(1500)               # let MUX flip + buffer queue rotate
sentai.camera.save_jpeg("cam0_front.jpg", 80)

sentai.camera.select(1)
sentai.rtos.sleep_ms(1500)
sentai.camera.save_jpeg("cam1_back.jpg", 80)
```

The 1.5 s sleep is the only thing that matters.  An earlier attempt
used a 4-tick `frame_count()` drain (~270 ms) plus an explicit "purge
buffer queue" loop calling `to_tensor()` 8x with 5 ms gaps -- it
produced **identical-looking JPEGs** because:

1. `frame_count()` is FB2-gated (half sensor rate); 4 ticks is only
   ~133 ms of real time.
2. `to_tensor()` reads the latest buffer but with 5 ms between calls
   only one fresh frame can land per pair -- the queue isn't actually
   rotated 8x as intended.

Both make the post-flip wait *too short* for the CameraTask buffer
pool to fully purge stale cam0 frames.  `save_jpeg` then PXP-scaled
and JPEG-encoded a stale buffer -> two near-identical photos.

A blunt `sleep_ms(1500)` is enough; sensor at 30 fps produces ~45
fresh frames in that window, far more than the 3-4 buffers in
CameraTask's pool.

## Notes on `grabbed_id()` post-flip

After `select(N)` + 1.5 s sleep + `to_tensor()`, `grabbed_id()` may
still return the *previous* camera id, because `to_tensor` returns the
buffer most-recently dequeued by the camera task -- which can pre-date
the MUX flip if no other consumer has touched the queue.  This is
diagnostic-only; the *next* `save_jpeg` does its own dequeue and gets
a fresh post-flip buffer.  Don't gate the save on `grabbed_id()`.

## Reproducing

```bash
cd examples/sentai_runtime
python3 diag/_host_upload_repl.py --file _t_cam_snapshot.py
# REPL: exec(sentai.fs.read_str("/lib/diag/_t_cam_snapshot.py"), globals())
# pull /diags/sNNN_cam/{cam0_FRONT,cam1_BACK}.jpg via http://10.0.0.1
```
