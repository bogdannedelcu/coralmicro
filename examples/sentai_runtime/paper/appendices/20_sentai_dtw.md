# Appendix: sentai.dtw

The `sentai.dtw` namespace exposes dynamic time warping for template-based sequence matching. It is intended for short gesture or audio patterns where full trainable sequence models are unnecessary and a small number of stored templates is sufficient for online recognition.

## Functions

- `sentai.dtw.init(dim, max_len)`: Initializes the DTW engine.
- `sentai.dtw.record_start(label)`: Starts recording a named template.
- `sentai.dtw.record_add(frame)`: Adds one feature frame.
- `sentai.dtw.record_add_imu()`: Adds the current IMU sample as a frame.
- `sentai.dtw.record_add_mic()`: Adds the current microphone level as a frame.
- `sentai.dtw.record_end()`: Finalizes and stores the template.
- `sentai.dtw.match(sequence, threshold)`: Matches a supplied sequence against stored templates.
- `sentai.dtw.match_imu(n_frames, threshold, delay_ms)`: Captures IMU frames and matches them.
- `sentai.dtw.match_mic(n_frames, threshold, delay_ms)`: Captures microphone frames and matches them.
- `sentai.dtw.templates()`: Lists stored templates.
- `sentai.dtw.remove(label)`: Removes a template.
- `sentai.dtw.save(path)`: Saves all templates.
- `sentai.dtw.load(path)`: Loads saved templates.
- `sentai.dtw.info()`: Returns engine metadata.
- `sentai.dtw.clear()`: Clears all templates.

## Example

```python
import sentai

sentai.imu.init()
sentai.dtw.init(3, 100)
sentai.dtw.record_start('wave')
for _ in range(50):
    sentai.dtw.record_add_imu()
    sentai.rtos.sleep_ms(20)
sentai.dtw.record_end()
print(sentai.dtw.match_imu(50, 500.0))
```