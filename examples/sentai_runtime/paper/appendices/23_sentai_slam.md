# Appendix: sentai.slam

The `sentai.slam` namespace implements a compact detection-based EKF-SLAM system. It estimates a planar robot pose and maintains a landmark map using object detections or manually supplied observations. The interface is decoupled from the TPU, so it can consume detections from any compatible source.

## Functions

- `sentai.slam.init(fov_h_deg, img_w, img_h, max_lm, baseline_m)`: Initializes the SLAM model.
- `sentai.slam.update(detections)`: Updates pose and landmarks from monocular detections.
- `sentai.slam.update_stereo(dets_left, dets_right)`: Updates from stereo detections.
- `sentai.slam.observe(class_id, bearing_rad, range_m)`: Adds a manual landmark observation.
- `sentai.slam.predict(dx, dy, dtheta)`: Applies an explicit motion prediction.
- `sentai.slam.pose()`: Returns the current estimated pose.
- `sentai.slam.landmarks()`: Returns the active landmark set.
- `sentai.slam.noise(sigma_v, sigma_w, sigma_b, sigma_r)`: Gets or sets noise parameters.
- `sentai.slam.imu_correct(pitch_deg, roll_deg)`: Adjusts bearing noise using IMU tilt.
- `sentai.slam.clear()`: Resets pose and landmarks.
- `sentai.slam.save(path)`: Saves the map and pose.
- `sentai.slam.load(path)`: Loads the map and pose.
- `sentai.slam.info()`: Returns SLAM metadata and statistics.

## Example

```python
import sentai

sentai.slam.init(66.0, 320, 320)
sentai.tpu.load('/models/yolov8n_320.tflite')
sentai.camera.init()

sentai.camera.to_tensor()
sentai.tpu.invoke()
dets = sentai.tpu.detect(0.3, 0.45, 50)
sentai.slam.update(dets)
print(sentai.slam.pose(), sentai.slam.landmarks())
```