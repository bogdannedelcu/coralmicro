# Appendix: sentai.tfl

The `sentai.tfl` namespace provides CPU-side TensorFlow Lite Micro inference on the Cortex-M7. It complements `sentai.tpu` for models that are not compiled for the EdgeTPU, for smaller auxiliary models, or for experiments that require direct MCU execution with explicit tensor-arena sizing.

## Functions

- `sentai.tfl.load(path, arena_kb)`: Loads a `.tflite` model and allocates the tensor arena.
- `sentai.tfl.unload()`: Frees the interpreter and tensor arena.
- `sentai.tfl.invoke()`: Runs inference and returns elapsed time in milliseconds.
- `sentai.tfl.ready()`: Reports whether the interpreter is ready.
- `sentai.tfl.info()`: Prints model and arena information.
- `sentai.tfl.set_input(data)`: Writes bytes into the input tensor.
- `sentai.tfl.load_image(path)`: Loads a JPEG or raw RGB image into the input tensor.
- `sentai.tfl.input_size()`: Returns the input tensor size in bytes.
- `sentai.tfl.input_dims()`: Returns the input tensor shape.
- `sentai.tfl.input_type()`: Returns the input tensor type.
- `sentai.tfl.input_quant()`: Returns input quantization parameters.
- `sentai.tfl.num_outputs()`: Returns the number of output tensors.
- `sentai.tfl.output_size(idx)`: Returns output tensor size.
- `sentai.tfl.output(idx)`: Returns raw output bytes.
- `sentai.tfl.output_dims(idx)`: Returns the output tensor shape.
- `sentai.tfl.output_type(idx)`: Returns the output tensor type.
- `sentai.tfl.output_quant(idx)`: Returns output quantization parameters.
- `sentai.tfl.output_floats(idx)`: Returns dequantized output values.
- `sentai.tfl.row(idx, r)`: Returns one tensor row.
- `sentai.tfl.value(idx, flat_i)`: Returns one output value.
- `sentai.tfl.save_output(path)`: Saves all outputs to CSV.

## Example

```python
import sentai

sentai.tfl.load('/models/keyword_detect.tflite', 256)
sentai.tfl.set_input(feature_bytes)
ms = sentai.tfl.invoke()
probs = sentai.tfl.output_floats(0)
print('inference:', ms, 'ms', probs)
```