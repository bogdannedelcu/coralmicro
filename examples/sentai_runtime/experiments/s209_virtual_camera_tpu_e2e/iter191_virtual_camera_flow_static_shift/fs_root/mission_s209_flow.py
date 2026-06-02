import sentai


BASE = "/images/cat_640x480.bmp"
SHIFT = "/images/cat_640x480_shift_x2.bmp"


def _compact(reading):
    return (
        bool(reading["alive"]),
        int(reading["state"]),
        int(reading["dx"]),
        int(reading["dy"]),
        int(reading["confidence"]),
        int(reading["frame_seq"]),
        int(reading["cam_id"]),
    )


def _sample(path):
    rc = sentai.camera.select(-1, path)
    return (rc, _compact(sentai.flow.read()))


def run():
    print("IMG_SIZE", sentai.fs.size(BASE))
    print("SHIFT_SIZE", sentai.fs.size(SHIFT))
    print("FLOW_START", sentai.flow.start(-1))
    prime = _sample(BASE)
    static_same = _sample(BASE)
    shifted = _sample(SHIFT)
    print("FLOW_STOP", sentai.flow.stop())
    result = {
        "prime": prime,
        "static_same": static_same,
        "shifted": shifted,
    }
    print("FLOW_RESULTS", repr(result))
    print("FLOW_RESULTS_WRITTEN",
          sentai.fs.write("/flow_results.txt", repr(result)))
    return 0
