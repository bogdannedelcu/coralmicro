#!/usr/bin/env python3
"""EXP-s258 / OP-S10-W24-T11 -- build Edge TPU YUV420 detectors for the SXGA (1280x960)
camera, from the 640x480 dual-input YOLOv5n architecture used in s257.

The trained 640x480 models exist here only as compiled `*_edgetpu.tflite` blobs
(the sources live on MLflow, TLS-unreachable from this host), so the graph is
rebuilt in Keras from the architecture file
`yolov5n_p3p4_c2f_yuv420.yaml` (SPDConv Y stem + stride-1 UV stem, C2f backbone,
SPPF, P3/P4 head) with RANDOM weights.  Invoke time on the Edge TPU depends on
the graph + tensor shapes, not on the weight values, so this is a speed replica;
detections are meaningless (operator: "nu ma intereseaza detectiile").

First board result (build 1711): the trained c2f = 19 ms, this replica = 87 ms
at identical tensor shapes, so the export details matter a lot to the Edge TPU
compiler.  The flags below isolate them (one variant per flag set):

  --div8     channel widths rounded to multiples of 8 (yolov5 make_divisible)
  --nosplit  C2f split as two 1x1 convs instead of STRIDED_SLICE on channels
  --relu     ReLU6 instead of SiLU (LOGISTIC+MUL per layer)
  --nospd    stems as 3x3 stride-2 convs instead of SPACE_TO_DEPTH + conv

Resolutions: vga (Y 480x640 + UV 240x320), sxga (Y 960x1280 + UV 480x640 with a
3x3 stride-2 "downsampling stem" conv on each plane in front of the VGA net),
sxga_s2d (same, stem = space_to_depth(2) + 1x1 conv).

Pipeline: Keras -> int8 post-training quantization (random representative data,
uint8 I/O) -> edgetpu_compiler -> parse the log (every op must map to the Edge
TPU) -> models/<name>_edgetpu.tflite + models/<name>.log.

Usage: venv-coral312/bin/python build_model.py --res vga,sxga [--div8] [--nosplit]
          [--relu] [--nospd] [--front-ch 4] [--name tag]
Needs TensorFlow (venv-coral312 has 2.17) and /usr/bin/edgetpu_compiler (16.0).
"""
import argparse, os, re, subprocess, sys
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'models')

import numpy as np
import tensorflow as tf
from tensorflow import keras
L = keras.layers

NC = 1
WIDTH = 0.125
DEPTH = 0.33
FLAGS = dict(div8=False, nosplit=False, relu=False, nospd=False)


def ch(c):
    if FLAGS['div8']:
        return max(8, int(np.ceil(c * WIDTH / 8) * 8))   # yolov5 make_divisible(., 8)
    return max(1, int(round(c * WIDTH)))


def n_rep(n):
    return max(1, int(round(n * DEPTH)))


def conv(x, c, k, s, name):
    x = L.Conv2D(c, k, strides=s, padding='same', use_bias=False, name=name + '_conv')(x)
    x = L.BatchNormalization(name=name + '_bn')(x)
    if FLAGS['relu']:
        return L.ReLU(max_value=6.0, name=name + '_relu6')(x)
    return L.Activation('swish', name=name + '_silu')(x)   # SiLU == swish


def bottleneck(x, c, shortcut, name):
    y = conv(x, c, 3, 1, name + '_b1')
    y = conv(y, c, 3, 1, name + '_b2')
    return L.Add(name=name + '_add')([x, y]) if shortcut else y


def c2f(x, c, n, shortcut, name):
    hidden = max(1, c // 2)
    if FLAGS['nosplit']:
        a = conv(x, hidden, 1, 1, name + '_cv1a')
        b = conv(x, hidden, 1, 1, name + '_cv1b')
    else:
        y = conv(x, 2 * hidden, 1, 1, name + '_cv1')
        a = L.Lambda(lambda t: t[..., :hidden], name=name + '_split_a')(y)
        b = L.Lambda(lambda t: t[..., hidden:], name=name + '_split_b')(y)
    outs = [a, b]
    for i in range(n):
        b = bottleneck(b, hidden, shortcut, '%s_m%d' % (name, i))
        outs.append(b)
    y = L.Concatenate(name=name + '_cat')(outs)
    return conv(y, c, 1, 1, name + '_cv2')


def sppf(x, c, k, name):
    hidden = max(1, c // 2)
    y = conv(x, hidden, 1, 1, name + '_cv1')
    p1 = L.MaxPool2D(k, strides=1, padding='same', name=name + '_p1')(y)
    p2 = L.MaxPool2D(k, strides=1, padding='same', name=name + '_p2')(p1)
    p3 = L.MaxPool2D(k, strides=1, padding='same', name=name + '_p3')(p2)
    y = L.Concatenate(name=name + '_cat')([y, p1, p2, p3])
    return conv(y, c, 1, 1, name + '_cv2')


def spd_conv(x, c, k, name):
    if FLAGS['nospd']:
        return conv(x, c, 3, 2, name)
    y = L.Lambda(lambda t: tf.nn.space_to_depth(t, 2), name=name + '_s2d')(x)
    return conv(y, c, k, 1, name)


def build(res, front_ch, name):
    H, W = (480, 640) if res == 'vga' else (960, 1280)
    y_in = keras.Input((H, W, 1), name='y')
    uv_in = keras.Input((H // 2, W // 2, 2), name='uv')
    y, uv = y_in, uv_in
    if res == 'sxga':
        y = conv(y, front_ch, 3, 2, 'front_y')       # downsampling stems -> VGA geometry
        uv = conv(uv, front_ch, 3, 2, 'front_uv')
    elif res == 'sxga_s2d':
        y = spd_conv(y, front_ch, 1, 'front_y')
        uv = spd_conv(uv, front_ch, 1, 'front_uv')
    # --- backbone (yolov5n_p3p4_c2f_yuv420.yaml) ---
    l0 = spd_conv(y, ch(64), 3, 'L0_spd')            # Y stem  -> H/2
    l1 = conv(uv, ch(32), 3, 1, 'L1_uv')              # UV stem -> H/2
    l2 = L.Concatenate(name='L2_cat')([l0, l1])
    l3 = conv(l2, ch(128), 3, 2, 'L3')               # /4
    l4 = c2f(l3, ch(128), n_rep(3), True, 'L4')
    l5 = conv(l4, ch(256), 3, 2, 'L5')               # /8
    l6 = c2f(l5, ch(256), n_rep(6), True, 'L6')      # P3
    l7 = conv(l6, ch(512), 3, 2, 'L7')               # /16
    l8 = c2f(l7, ch(512), n_rep(9), True, 'L8')
    l9 = sppf(l8, ch(512), 5, 'L9')
    # --- head ---
    l10 = conv(l9, ch(256), 1, 1, 'L10')
    # Keras 3 UpSampling2D lowers to EXPAND_DIMS+TILE (v3), which the Edge TPU
    # compiler 16.0 cannot parse; a fixed-size nearest resize lowers to
    # RESIZE_NEAREST_NEIGHBOR, which maps to the TPU.
    up_h, up_w = l10.shape[1] * 2, l10.shape[2] * 2
    l11 = L.Lambda(lambda t: tf.image.resize(t, (up_h, up_w), method='nearest'),
                   name='L11_up')(l10)
    l12 = L.Concatenate(name='L12_cat')([l11, l6])
    l13 = c2f(l12, ch(256), n_rep(3), False, 'L13')   # P3 out
    l14 = conv(l13, ch(256), 3, 2, 'L14')
    l15 = L.Concatenate(name='L15_cat')([l14, l10])
    l16 = c2f(l15, ch(512), n_rep(3), False, 'L16')   # P4 out
    # Detect: one anchor per level -> (5 + nc) channels, as the s257 exports ([1,60,80,6], [1,30,40,6])
    d3 = L.Conv2D(5 + NC, 1, name='det_p3')(l13)
    d4 = L.Conv2D(5 + NC, 1, name='det_p4')(l16)
    return keras.Model([uv_in, y_in], [d4, d3], name=name), (H, W)


def quantize(model, H, W, path):
    def rep():
        rng = np.random.default_rng(0)
        for _ in range(8):
            yield [rng.random((1, H // 2, W // 2, 2), dtype=np.float32),
                   rng.random((1, H, W, 1), dtype=np.float32)]
    try:
        conv_ = tf.lite.TFLiteConverter.from_keras_model(model)
    except Exception as e:  # Keras 3 fallback: export a SavedModel first
        print('from_keras_model failed (%s); exporting SavedModel' % e)
        sm = path + '.savedmodel'
        model.export(sm)
        conv_ = tf.lite.TFLiteConverter.from_saved_model(sm)
    conv_.optimizations = [tf.lite.Optimize.DEFAULT]
    conv_.representative_dataset = rep
    conv_.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    conv_.inference_input_type = tf.uint8
    conv_.inference_output_type = tf.uint8
    tfl = conv_.convert()
    open(path, 'wb').write(tfl)
    return len(tfl)


def compile_edgetpu(path):
    r = subprocess.run(['edgetpu_compiler', '-s', '-o', OUT, path], capture_output=True, text=True)
    log = r.stdout + r.stderr
    open(path.replace('.tflite', '.log'), 'w').write(log)
    rows = re.findall(r'^([A-Z_0-9]+)\s+(\d+)\s+(.*)$', log, re.M)
    n_tpu = sum(int(n) for op, n, where in rows if 'Mapped to Edge TPU' in where)
    n_cpu = sum(int(n) for op, n, where in rows if 'Mapped to Edge TPU' not in where)
    cpu_ops = [(op, n, where.strip()) for op, n, where in rows if 'Mapped to Edge TPU' not in where]
    m_cache = re.search(r'On-chip memory used for caching model parameters:\s*([\d.]+\s*\w+)', log)
    m_off = re.search(r'Off-chip memory used for streaming uncached model parameters:\s*([\d.]+\s*\w+)', log)
    return (r.returncode, n_tpu, n_cpu, cpu_ops, m_cache.group(1) if m_cache else '?',
            m_off.group(1) if m_off else '?', log)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--res', default='vga,sxga')
    ap.add_argument('--front-ch', type=int, default=4)
    ap.add_argument('--name', default='')
    for f in FLAGS:
        ap.add_argument('--' + f, action='store_true')
    a = ap.parse_args()
    for f in FLAGS:
        FLAGS[f] = getattr(a, f)
    tag = a.name or ''.join('_' + f for f in FLAGS if FLAGS[f])
    os.makedirs(OUT, exist_ok=True)
    print('%-42s %10s %9s %5s %5s %10s %10s' % ('model', 'tflite B', 'params', 'tpu', 'cpu', 'cache', 'offchip'))
    for res in a.res.split(','):
        name = 'c2f_yuv420_%s%s_int8' % (res, tag)
        model, (H, W) = build(res, a.front_ch, name)
        path = os.path.join(OUT, name + '.tflite')
        size = quantize(model, H, W, path)
        rc, n_tpu, n_cpu, cpu_ops, cache, off, log = compile_edgetpu(path)
        if rc != 0:
            print('COMPILE FAILED for', name); print(log[-1500:]); continue
        comp = os.path.getsize(path.replace('.tflite', '_edgetpu.tflite'))
        print('%-42s %10d %9d %5d %5d %10s %10s  compiled %d B' %
              (name + '_edgetpu', size, model.count_params(), n_tpu, n_cpu, cache, off, comp))
        if cpu_ops:
            print('   CPU ops:', cpu_ops[:8])
    return 0


if __name__ == '__main__':
    sys.exit(main())
