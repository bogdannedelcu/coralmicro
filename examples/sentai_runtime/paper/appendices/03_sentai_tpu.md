# Appendix: sentai.tpu

The `sentai.tpu` namespace is the main interface to EdgeTPU-backed inference. It covers model loading, input preparation, execution, tensor inspection, quantization helpers, and detector-oriented post-processing. In practice it is the default path for compact vision models compiled for the accelerator.

## Functions

- `sentai.tpu.load(path)`: Loads a `.tflite` model from flash and creates the EdgeTPU interpreter.
- `sentai.tpu.load_image(path)`: Loads a JPEG or raw RGB image into the input tensor.
- `sentai.tpu.invoke()`: Runs inference and returns elapsed time in milliseconds.
- `sentai.tpu.ready()`: Reports whether the model and interpreter are ready.
- `sentai.tpu.num_outputs()`: Returns the number of output tensors.
- `sentai.tpu.output_size(idx)`: Returns the byte size of an output tensor.
- `sentai.tpu.output(idx)`: Returns raw output bytes.
- `sentai.tpu.output_dims(idx)`: Returns the output tensor shape.
- `sentai.tpu.output_type(idx)`: Returns the tensor type.
- `sentai.tpu.row(idx, r)`: Returns a selected tensor row.
- `sentai.tpu.value(idx, flat_i)`: Returns a single flattened value.
- `sentai.tpu.save_output(path)`: Saves all outputs to CSV.
- `sentai.tpu.input_quant()`: Returns input quantization parameters.
- `sentai.tpu.output_quant(idx)`: Returns output quantization parameters.
- `sentai.tpu.output_floats(idx)`: Returns dequantized outputs as Python floats.
- `sentai.tpu.input_type()`: Returns the input tensor type.
- `sentai.tpu.detect(conf, iou, max)`: Runs YOLO-style non-maximum suppression on the last inference output.
- `sentai.tpu.draw(path, dets, quality)`: Draws detections on the last captured frame and saves a JPEG.

## Example

```python
import sentai

sentai.tpu.load('/models/yolov8n.tflite')
sentai.camera.init()
sentai.camera.to_tensor()
ms = sentai.tpu.invoke()
dets = sentai.tpu.detect(0.25, 0.45, 50)
print('ms:', ms, 'detections:', len(dets))
sentai.tpu.draw('/det.jpg', dets)
```