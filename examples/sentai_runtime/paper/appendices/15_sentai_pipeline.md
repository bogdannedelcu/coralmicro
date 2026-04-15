# Appendix: sentai.pipeline

The `sentai.pipeline` namespace exposes the background detection pipeline and the SentAI-SORT tracker. It is the main high-level perception service in the runtime and packages detector scheduling, tracking, event reporting, camera geometry, and ground-plane projection into a single script-facing interface.

## Functions

- `sentai.pipeline.start(conf, iou, max, track)`: Starts continuous detection and optionally tracking.
- `sentai.pipeline.stop()`: Stops the pipeline.
- `sentai.pipeline.running()`: Reports whether the pipeline is active.
- `sentai.pipeline.get(timeout_ms)`: Returns the next detection frame.
- `sentai.pipeline.stats()`: Returns processed, dropped, and FPS statistics.
- `sentai.pipeline.tracks()`: Returns the current active-track snapshot.
- `sentai.pipeline.event(timeout_ms)`: Returns the next tracker event.
- `sentai.pipeline.track_config(...)`: Gets or sets tracker parameters.
- `sentai.pipeline.set_pose(altitude_cm, heading_deg, lat, lon)`: Sets camera pose for ground projection.
- `sentai.pipeline.camera_config(cam_id, fov_h, fov_v, mount_pitch, mount_roll, mount_yaw)`: Gets or sets per-camera geometry.

## Example

```python
import sentai

sentai.camera.init()
sentai.pipeline.camera_config(0, 70.8, 43.4, 0, 0, 0)
sentai.pipeline.set_pose(1000, 0, 44.4268, 26.1025)
sentai.pipeline.start(0.3, 0.45, 50, True)
while True:
    evt = sentai.pipeline.event(2000)
    if evt:
        print(evt)
```