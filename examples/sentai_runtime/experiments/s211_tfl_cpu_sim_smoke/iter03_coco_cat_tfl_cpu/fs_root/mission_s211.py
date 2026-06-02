import sentai
MODEL = "/models/tf2_ssd_mobilenet_v2_coco17_ptq.tflite"
IMAGE = "/images/cat_640x480.bmp"
OUT = "/tfl_smoke.txt"


def _line(k, v):
    return str(k) + "=" + str(v) + "\n"


def run():
    text = ""
    rc = sentai.tfl.load(MODEL, 8192)
    text += _line("load_rc", rc)
    text += _line("ready", sentai.tfl.ready())
    text += _line("input_size", sentai.tfl.input_size())
    text += _line("input_dims", sentai.tfl.input_dims())
    text += _line("input_type", sentai.tfl.input_type())
    text += _line("num_outputs_before", sentai.tfl.num_outputs())
    if rc == 0:
        text += _line("load_image_rc", sentai.tfl.load_image(IMAGE))
        text += _line("invoke_ms", sentai.tfl.invoke())
        text += _line("num_outputs", sentai.tfl.num_outputs())
        for i in range(sentai.tfl.num_outputs()):
            text += _line("output" + str(i) + "_size", sentai.tfl.output_size(i))
            text += _line("output" + str(i) + "_dims", sentai.tfl.output_dims(i))
            text += _line("output" + str(i) + "_type", sentai.tfl.output_type(i))
            text += _line("output" + str(i) + "_v0", sentai.tfl.value(i, 0))
    sentai.fs.write(OUT, text)
    print(text)
    return rc
