# Read-only smoke check for the B7 namespace/model substrate.
#
# Intended to run inside the SentAI MicroPython runtime, either on SIM or board.
# It does not start flight/control tasks and does not mutate object/place maps.

import sentai


ROOT_REQUIRED = (
    "objects",
    "object_lifter",
    "places",
    "slam",
    "fr",
    "safety",
    "servo",
    "calib",
    "markers",
)

MODEL_CALLS = (
    ("objects", "count"),
    ("objects", "stats"),
    ("objects", "list"),
    ("object_lifter", "count"),
    ("object_lifter", "stats"),
    ("object_lifter", "list"),
    ("places", "count"),
    ("places", "stats"),
    ("places", "list"),
    ("places", "slam_stats"),
    ("places", "slam_current"),
    ("slam", "info"),
    ("slam", "landmarks"),
    ("slam", "pose"),
)


def _fail(msg):
    print("ERR " + msg)
    return False


def run():
    missing = []
    for name in ROOT_REQUIRED:
        if not hasattr(sentai, name):
            missing.append(name)
    if missing:
        return _fail("missing_root=" + ",".join(missing))

    for mod_name, fn_name in MODEL_CALLS:
        mod = getattr(sentai, mod_name)
        if not hasattr(mod, fn_name):
            return _fail("missing_call=" + mod_name + "." + fn_name)
        try:
            getattr(mod, fn_name)()
        except Exception as e:
            return _fail("call_failed=" + mod_name + "." + fn_name +
                         " err=" + str(e))

    print("OK namespace_model_smoke")
    return True


run()
