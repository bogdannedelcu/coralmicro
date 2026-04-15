# Appendix: sentai.imu

The `sentai.imu` namespace exposes the onboard LIS2DU12 accelerometer. It provides direct access to acceleration and derived tilt estimates and is commonly used for motion monitoring, camera-pose estimation, and simple event detection in scripts that combine perception with inertial context.

## Functions

- `sentai.imu.init()`: Initializes the accelerometer.
- `sentai.imu.read()`: Returns acceleration and temperature measurements.
- `sentai.imu.degrees()`: Returns pitch and roll in degrees.
- `sentai.imu.radians()`: Returns pitch and roll in radians.

## Example

```python
import sentai

sentai.imu.init()
for _ in range(10):
    d = sentai.imu.degrees()
    if d:
        print('pitch:', d['pitch'], 'roll:', d['roll'])
    sentai.rtos.sleep_ms(100)
```