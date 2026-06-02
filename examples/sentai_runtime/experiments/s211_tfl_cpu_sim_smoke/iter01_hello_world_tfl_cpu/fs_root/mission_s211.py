import sentai
import struct


MODEL = "/models/hello_world.tflite"
OUT = "/tfl_smoke.txt"


def _line(k, v):
    return str(k) + "=" + str(v) + "\n"


def run():
    text = ""
    rc = sentai.tfl.load(MODEL, 64)
    text += _line("load_rc", rc)
    text += _line("ready", sentai.tfl.ready())
    text += _line("input_size", sentai.tfl.input_size())
    text += _line("input_dims", sentai.tfl.input_dims())
    text += _line("num_outputs_before", sentai.tfl.num_outputs())
    if rc == 0 and sentai.tfl.input_size() == 4:
        text += _line("set_input_rc", sentai.tfl.set_input(struct.pack("<f", 0.0)))
        text += _line("invoke_ms", sentai.tfl.invoke())
        text += _line("num_outputs", sentai.tfl.num_outputs())
        text += _line("output0_size", sentai.tfl.output_size(0))
        text += _line("output0_dims", sentai.tfl.output_dims(0))
        text += _line("value0", sentai.tfl.value(0, 0))
    sentai.fs.write(OUT, text)
    print(text)
    return rc
