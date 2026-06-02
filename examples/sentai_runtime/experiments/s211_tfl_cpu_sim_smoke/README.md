# s211 - SIM TFLite Micro CPU COCO Smoke

Purpose: verify that `sentai.tfl` is no longer only a SIM namespace stub.  The
SIM build links the shared `sentai_tfl_bridge.cc` backend and runs TFLite Micro
on the host CPU.

Baseline convention: use the same COCO cat family as the EdgeTPU s209 smoke.
The model is the non-EdgeTPU CPU model:

```text
/models/tf2_ssd_mobilenet_v2_coco17_ptq.tflite
/images/cat_640x480.bmp
```

Repo fixture:

```text
models/tf2_ssd_mobilenet_v2_coco17_ptq.tflite
test_data/cat_640x480.bmp
```

Current passing run:

```text
iter03_coco_cat_tfl_cpu
load_rc=0
ready=True
input_dims=(1, 300, 300, 3)
load_image_rc=0
invoke_ms=482
num_outputs=4
```

This is an execution/backend smoke, not yet full bbox parity against s209.  It
proves the COCO model loads, the cat BMP is resized into the TFLM input tensor
inside C++, and inference runs without exposing image bytes to MicroPython.
