#!/usr/bin/env python3
"""s238 host-side detection reference: run the 8 ReLU int8 models on CPU with the
same test image and the same decode as the board (raw grid cells with
sigmoid(obj)*sigmoid(cls) >= conf, both heads, pre-NMS), to check board parity.

Run:  venv-coral39/bin/python3 host_detect.py
"""
import numpy as np, tflite_runtime.interpreter as tfl, os

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, "test.bin")
MODELS = ["msblock", "c2f", "c3", "c2f_pan2", "gelan", "gelan_pan2", "c2f_deep", "c2f_thick"]


def sig(x):
    return 1.0 / (1.0 + np.exp(-x))


def count(model, img, conf=0.25):
    it = tfl.Interpreter(model_path=model)
    it.allocate_tensors()
    ind = it.get_input_details()[0]
    x = img.astype(ind["dtype"])
    if ind["dtype"] == np.int8:
        x = (img.astype(np.int16) - 128).astype(np.int8)
    it.set_tensor(ind["index"], x)
    it.invoke()
    n, mx = 0, 0.0
    for od in it.get_output_details():
        a = it.get_tensor(od["index"]).astype(np.float32).reshape(-1, 6)
        sc, zp = od["quantization"]
        real = (a - zp) * sc
        cf = sig(real[:, 4]) * sig(real[:, 5])
        n += int((cf >= conf).sum())
        mx = max(mx, float(cf.max()))
    return n, mx


def main():
    img = np.frombuffer(open(IMG, "rb").read(), dtype=np.uint8).reshape(1, 480, 640, 3)
    print("model        host_dets  host_maxconf")
    for m in MODELS:
        p = os.path.join(HERE, "host_int8_models", m + ".tflite")
        n, mx = count(p, img)
        print("%-11s  %6d      %.3f" % (m, n, mx))


if __name__ == "__main__":
    main()
