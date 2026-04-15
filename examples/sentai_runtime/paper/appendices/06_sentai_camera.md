# Appendix: sentai.camera

The `sentai.camera` namespace controls image capture and basic camera-side preprocessing. It provides the main bridge between the sensing layer and the inference pipeline, including frame capture, JPEG export, tensor feeding, camera selection, and frame-sequence inspection for dual-camera operation.

## Functions

- `sentai.camera.init(streaming)`: Starts the camera in continuous or trigger mode.
- `sentai.camera.stop()`: Stops camera capture.
- `sentai.camera.jpeg(quality)`: Captures a frame and returns JPEG bytes.
- `sentai.camera.save_jpeg(path, q)`: Captures and saves a JPEG file.
- `sentai.camera.to_tensor()`: Captures a frame and writes it into the TPU input tensor.
- `sentai.camera.resolution()`: Returns the current output resolution.
- `sentai.camera.set_resolution(w, h)`: Changes the capture resolution.
- `sentai.camera.native_res()`: Returns the sensor native resolution.
- `sentai.camera.select(id)`: Selects the active camera.
- `sentai.camera.frame_count()`: Returns the monotonic hardware frame counter.

## Example

```python
import sentai

sentai.camera.init()
sentai.camera.set_resolution(320, 320)
sentai.camera.select(0)
sentai.camera.save_jpeg('/front.jpg', 85)
sentai.camera.select(1)
sentai.camera.save_jpeg('/back.jpg', 85)
sentai.camera.stop()
```