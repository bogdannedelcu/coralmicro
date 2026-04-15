# Appendix: sentai.aifes

The `sentai.aifes` namespace integrates the AIfES library for on-device neural-network training and inference. It is intended for small multilayer perceptrons and compact transfer-learning workflows in which features extracted by the TPU or camera are fed into a trainable MCU-side head.

## Functions

- `sentai.aifes.load(path)`: Loads a model architecture from YAML.
- `sentai.aifes.load_weights(path)`: Loads pre-trained weights.
- `sentai.aifes.save_weights(path)`: Saves trained weights.
- `sentai.aifes.unload()`: Frees the loaded model.
- `sentai.aifes.ready()`: Reports whether the model is ready.
- `sentai.aifes.train(x_data, y_data, ...)`: Trains the model on supplied samples.
- `sentai.aifes.set_input(data)`: Sets the input vector from floats.
- `sentai.aifes.from_tpu(idx)`: Uses a TPU output tensor as AIfES input.
- `sentai.aifes.from_camera(w, h, gray)`: Uses a camera frame as normalized input.
- `sentai.aifes.from_mic(samples)`: Uses microphone samples as input.
- `sentai.aifes.invoke()`: Runs inference.
- `sentai.aifes.output()`: Returns the latest output vector.
- `sentai.aifes.mic_init()`: Initializes microphone capture for audio models.
- `sentai.aifes.mic_stop()`: Stops microphone capture.
- `sentai.aifes.predict(input)`: Legacy single-call inference helper.
- `sentai.aifes.info()`: Returns model metadata.

## Example

```python
import sentai

sentai.tpu.load('/models/mobilenet_features.tflite')
sentai.aifes.load('/models/classifier_head.yaml')
sentai.camera.init()

sentai.camera.to_tensor()
sentai.tpu.invoke()
sentai.aifes.from_tpu(0)
sentai.aifes.invoke()
print(sentai.aifes.output())
```