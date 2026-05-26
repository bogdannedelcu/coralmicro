# mission_s207 - TD-S10-B4 C++ marker-control migration workspace.
#
# B4 starts from the accepted strict calib.ini produced by B3/A3.  The runtime
# mission itself is the migrated B4 path: calibrated marker acquire, center
# hold, ExtPos/flow warmup, Generic Hover handoff, image-frame motion, and land.

import sentai


STATUS_NAME = "mission_s207_status.txt"
TICK_MS = 33


class MissionResults:
    def __init__(self):
        self.setup_ok = False
        self.acquire_ok = False
        self.center_hold_ok = False
        self.extpos_warmup_ok = False
        self.handoff_hover_ok = False
        self.axis_motion_ok = False
        self.land_ok = False

    def final_state(self):
        if not self.setup_ok:
            return "SETUP_FAIL", "setup_failed"
        if not self.acquire_ok:
            return "MARKER_ACQ_TIMEOUT", "marker_acquisition_timeout"
        if not self.center_hold_ok:
            return "CENTER_HOLD_FAIL", "center_hold_failed"
        if not self.extpos_warmup_ok:
            return "EXTPOS_WARMUP_FAIL", "extpos_warmup_failed"
        if not self.handoff_hover_ok:
            return "HANDOFF_HOVER_FAIL", "handoff_hover_failed"
        if not self.axis_motion_ok:
            return "IMAGE_AXIS_MOTION_FAIL", "image_axis_motion_failed"
        if not self.land_ok:
            return "LAND_FAIL", "land_failed"
        return "MARKER_CONTROL_OK", ""


def _kv_text(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, (int, float, str)):
        return str(v)
    return type(v).__name__


def _write_status(summary):
    lines = (
        "experiment=" + _kv_text(summary.get("experiment", "")),
        "task=" + _kv_text(summary.get("task", "")),
        "status=" + _kv_text(summary.get("status", "ERROR")),
        "phase=" + _kv_text(summary.get("phase", "")),
        "abort_reason=" + _kv_text(summary.get("abort_reason", "")),
    )
    sentai.fs.write(STATUS_NAME, "\n".join(lines) + "\n")


def _j(event, data=None):
    text = ""
    if isinstance(data, dict):
        parts = []
        for k, v in data.items():
            if isinstance(v, bool):
                v = 1 if v else 0
            parts.append(str(k) + "=" + str(v))
        text = " ".join(parts)
    elif data is not None:
        text = str(data)
    try:
        sentai.fr.push_event(str(event), text)
    except Exception:
        pass


def _set_phase(summary, phase, reason=""):
    prev = summary.get("phase", "")
    summary["phase"] = phase
    _j("phase_transition", {
        "from": prev,
        "to": phase,
        "reason": reason,
    })


def _orientation_wait_result():
    while not sentai.calib.orientation_task_is_done():
        sentai.rtos.sleep_ms(TICK_MS)
    return sentai.calib.orientation_result_tuple()


def _orientation_start_ok(rc):
    if rc != 0:
        return False
    result = _orientation_wait_result()
    if result is None:
        return False
    return bool(result[0])


def _marker_wait_result():
    while not sentai.servo.marker_task_is_done():
        sentai.rtos.sleep_ms(TICK_MS)
    return sentai.servo.marker_result_tuple()


def _marker_start_ok(rc):
    if rc != 0:
        return False
    result = _marker_wait_result()
    if result is None:
        return False
    return bool(result[0])


def _manual_descend():
    rc = sentai.calib.orientation_manual_descend_start()
    if rc == 0:
        _orientation_wait_result()


def run():
    summary = {
        "experiment": "s207_cpp_marker_control_task",
        "task": "TD-S10-B4",
        "status": "ERROR",
        "phase": "start",
        "abort_reason": "",
    }
    results = MissionResults()

    try:
        _set_phase(summary, "setup", "mission_start")
        results.setup_ok = _marker_start_ok(
            sentai.servo.marker_setup_start())
        if not results.setup_ok:
            summary["status"], summary["abort_reason"] = results.final_state()
            _write_status(summary)
            return summary

        _set_phase(summary, "marker_acquire", "setup_ok")
        results.acquire_ok = _marker_start_ok(
            sentai.servo.marker_acquire_start())

        if results.acquire_ok:
            _set_phase(summary, "center_hold", "marker_acquire_ok")
            results.center_hold_ok = _marker_start_ok(
                sentai.servo.marker_center_hold_start())

        if results.acquire_ok and results.center_hold_ok:
            _set_phase(summary, "extpos_warmup", "center_hold_ok")
            results.extpos_warmup_ok = _marker_start_ok(
                sentai.servo.marker_extpos_warmup_start())

        if (results.acquire_ok and results.center_hold_ok and
                results.extpos_warmup_ok):
            _set_phase(summary, "handoff_hover", "extpos_warmup_ok")
            results.handoff_hover_ok = _marker_start_ok(
                sentai.servo.marker_handoff_hover_start())

        if (results.acquire_ok and results.center_hold_ok and
                results.extpos_warmup_ok and results.handoff_hover_ok):
            _set_phase(summary, "image_axis_motion", "handoff_hover_ok")
            results.axis_motion_ok = _marker_start_ok(
                sentai.servo.marker_axis_motion_start())

        if (results.acquire_ok and results.center_hold_ok and
                results.extpos_warmup_ok and results.handoff_hover_ok and
                results.axis_motion_ok):
            _set_phase(summary, "land", "image_axis_motion_ok")
            results.land_ok = _marker_start_ok(
                sentai.servo.marker_land_start())
        else:
            _set_phase(summary, "manual_descend_disarm",
                       "b4_migration_slice_done")
            _manual_descend()

        _set_phase(summary, "post_acquisition", "flight_sequence_done")
        summary["status"], summary["abort_reason"] = results.final_state()
    except Exception as e:
        summary["status"] = "EXCEPTION"
        summary["exception"] = str(e)
        summary["abort_reason"] = "exception"
        _j("mission_exception", {"err": str(e)})
        try:
            sentai.servo.marker_task_stop()
            sentai.calib.orientation_emergency_stop()
        except Exception:
            pass

    _j("mission_done", {"status": summary["status"],
                         "phase": summary["phase"]})
    _write_status(summary)
    return summary
