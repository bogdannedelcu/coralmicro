# s209 - Virtual Camera TPU E2E

Purpose: prove that a file-backed virtual camera can drive the normal SIM
camera/pipeline/TPU path and match a host-side PyCoral baseline using the same
USB EdgeTPU, model, and image.

Each run owns its artifacts:

```text
iterNN_<label>/
  fs_root/
    images/cat_640x480.bmp
    models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite
  helper.log
  sim_output.txt
  host_detections.txt
  sim_detections.txt
  detected_overlay.png
  summary.json
```

Run:

```bash
examples/sentai_runtime/experiments/s209_virtual_camera_tpu_e2e/run.sh
```

The test does not move image byte arrays through MicroPython.  MP only selects
the virtual camera, starts the TPU/pipeline path, and reads compact detection
tuples.
