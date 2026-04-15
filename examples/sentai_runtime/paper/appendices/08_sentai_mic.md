# Appendix: sentai.mic

The `sentai.mic` namespace manages the onboard PDM microphone through a ring-buffer recording model. It is designed for low-overhead acquisition, sound-level monitoring, and asynchronous MP3 export, allowing audio-triggered experiments without forcing the user to manage raw DMA buffers directly.

## Functions

- `sentai.mic.start(seconds)`: Starts ring-buffer recording for the specified window length.
- `sentai.mic.stop()`: Stops the microphone and returns the number of buffered samples.
- `sentai.mic.recording()`: Reports whether the microphone is active.
- `sentai.mic.samples()`: Returns the number of samples currently available.
- `sentai.mic.level()`: Returns the current RMS level in centi-decibels.
- `sentai.mic.save_mp3()`: Saves buffered audio as an MP3 file and returns the filename.

## Example

```python
import sentai

sentai.mic.start(5)
sentai.rtos.sleep_ms(5000)
name = sentai.mic.save_mp3()
print('saved:', name)
sentai.mic.stop()
```