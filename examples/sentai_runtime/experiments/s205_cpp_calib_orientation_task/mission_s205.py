# mission_s205 - TD-S10-A3 clean uncalibrated visual-servo calibration path.
#
# This mission is intentionally not a fork of s194's control algorithm.
# s194 remains useful as a negative-control experiment and launcher precedent.
# s205 starts from image-space WhyCon features and delays pose/body-frame
# decisions until the camera/body response has been measured.

import sentai


STATUS_NAME = "mission_s205_status.txt"

TICK_MS = 33


class MissionResults:
    def __init__(self):
        self.locked = False
        self.brake_ok = False
        self.zhold_ok = False
        self.axis_ok = False
        self.centroid_validation_ok = False
        self.candidate_ok = False
        self.optical_axis_ok = False
        self.final_validation_ok = False
        self.final_recenter_ok = False
        self.center_hold_descend_ok = False

    def final_state(self):
        if not self.locked:
            return "MARKER_ACQ_TIMEOUT", "marker_acquisition_timeout"
        if not self.brake_ok:
            return "POST_LOCK_BRAKE_FAIL", "post_lock_brake_failed"
        if not self.zhold_ok:
            return "VISUAL_Z_HOLD_FAIL", "visual_z_hold_failed"
        if not self.axis_ok:
            return "AXIS_RESPONSE_FAIL", "axis_response_failed"
        if not self.candidate_ok:
            return "CANDIDATE_SCORING_FAIL", "candidate_scoring_failed"
        if not self.optical_axis_ok:
            return ("OPTICAL_AXIS_VALIDATION_FAIL",
                    "optical_axis_validation_failed")
        if not self.final_validation_ok:
            return "FINAL_VALIDATION_FAIL", "final_candidate_validation_failed"
        if not self.final_recenter_ok:
            return "RETURN_TO_CENTER_FAIL", "return_to_center_failed"
        if not self.center_hold_descend_ok:
            return "CENTER_HOLD_DESCENT_FAIL", "center_hold_descend_failed"
        return "FINAL_VALIDATION_OK", ""


def _kv_text(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, (int, float, str)):
        return str(v)
    return type(v).__name__


def _write_summary(summary):
    lines = (
        "experiment=" + _kv_text(summary.get("experiment", "")),
        "task=" + _kv_text(summary.get("task", "")),
        "status=" + _kv_text(summary.get("status", "ERROR")),
        "phase=" + _kv_text(summary.get("phase", "")),
        "abort_reason=" + _kv_text(summary.get("abort_reason", "")),
    )
    sentai.fs.write(STATUS_NAME, "\n".join(lines) + "\n")


def _persist_calib_contract(summary):
    return bool(sentai.calib.orientation_save_contract(
        str(summary.get("status", "")),
        summary.get("status") == "FINAL_VALIDATION_OK"))


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


def _phase_setup():
    # A3/s205 is an online calibration run: do not load a previously persisted
    # calibration into the decision path. This mission writes its compact
    # artifact at the end, and a later task can decide how to reload it.
    rc_setup = sentai.calib.setup_defaults()

    sentai.safety.init()
    sentai.crazy.init()
    return rc_setup == 0


def _orientation_wait_result():
    while not sentai.calib.orientation_task_is_done():
        sentai.rtos.sleep_ms(TICK_MS)
    return sentai.calib.orientation_result_tuple()


def _orientation_result_from_start(rc):
    if rc != 0:
        return None
    return _orientation_wait_result()


def _orientation_start_ok(rc):
    result = _orientation_result_from_start(rc)
    if result is None:
        return False
    return bool(result[0])


def _preflight_features():
    rc = sentai.calib.orientation_task_start()
    if rc != 0:
        return False
    result = _orientation_wait_result()
    return result is not None


def _set_abort_reason(summary, reason):
    summary["abort_reason"] = reason
    _j("abort_decision", {
        "reason": reason,
        "phase": summary.get("phase", ""),
    })


def run():
    summary = {
        "experiment": "s205_sota_calib_orientation_guarded",
        "task": "TD-S10-B3",
        "status": "ERROR",
        "phase": "start",
        "abort_reason": "",
    }

    try:
        _set_phase(summary, "setup", "mission_start")
        if not _phase_setup():
            summary["status"] = "SETUP_FAIL"
            _write_summary(summary)
            return summary

        _set_phase(summary, "preflight_features", "setup_ok")
        if not _preflight_features():
            summary["status"] = "PREFLIGHT_FAIL"
            _write_summary(summary)
            return summary

        _set_phase(summary, "arm_zero_unlock", "preflight_sampled")
        rc = sentai.calib.orientation_arm_zero_start()
        arm_ok = _orientation_start_ok(rc)
        if not arm_ok:
            summary["status"] = "ARM_FAIL"
            _write_summary(summary)
            return summary

        _set_phase(summary, "thrust_only_marker_acquisition", "armed")
        results = MissionResults()
        rc = sentai.calib.orientation_marker_acquisition_start()
        results.locked = _orientation_start_ok(rc)

        if results.locked:
            _set_phase(summary, "post_lock_brake", "marker_lock_acquired")
            rc = sentai.calib.orientation_post_lock_brake_start()
            results.brake_ok = _orientation_start_ok(rc)

        if results.locked and results.brake_ok:
            _set_phase(summary, "visual_z_hold", "post_lock_brake_ok")
            rc = sentai.calib.orientation_visual_z_hold_start()
            results.zhold_ok = _orientation_start_ok(rc)

        if results.locked and results.brake_ok and results.zhold_ok:
            _set_phase(summary, "axis_response_smoke", "visual_z_hold_ok")
            rc = sentai.calib.orientation_axis_response_start()
            results.axis_ok = _orientation_start_ok(rc)

        if (results.locked and results.brake_ok and
                results.zhold_ok and results.axis_ok):
            _set_phase(summary, "centroid_pd_validation", "axis_response_ok")
            rc = sentai.calib.orientation_centroid_validation_start()
            results.centroid_validation_ok = _orientation_start_ok(rc)

        if (results.locked and results.brake_ok and
                results.zhold_ok and results.axis_ok):
            _set_phase(summary, "candidate_scoring", "axis_response_ok")
            results.candidate_ok = bool(
                sentai.calib.orientation_score_candidate_from_axis())

        if (results.locked and results.brake_ok and
                results.zhold_ok and results.axis_ok and
                results.candidate_ok):
            _set_phase(summary, "optical_axis_validation", "candidate_ok")
            results.optical_axis_ok = bool(
                sentai.calib.orientation_optical_axis_validate())

        if (results.locked and results.brake_ok and
                results.zhold_ok and results.axis_ok and
                results.candidate_ok and results.optical_axis_ok):
            _set_phase(summary, "final_candidate_validation",
                       "optical_axis_ok")
            rc = sentai.calib.orientation_final_candidate_validation_start()
            results.final_validation_ok = _orientation_start_ok(rc)

        if (results.locked and results.brake_ok and
                results.zhold_ok and results.axis_ok and
                results.candidate_ok and results.optical_axis_ok and
                results.final_validation_ok):
            _set_phase(summary, "final_centroid_recenter",
                       "final_candidate_validation_ok")
            rc = sentai.calib.orientation_final_recenter_start()
            results.final_recenter_ok = _orientation_start_ok(rc)
        _set_phase(summary, "manual_descend_disarm",
                   "select_recovery_or_landing")
        if results.final_recenter_ok:
            _set_phase(summary, "center_hold_descend_disarm",
                       "final_recenter_ok")
            rc = sentai.calib.orientation_center_hold_descend_start()
            center_result = _orientation_result_from_start(rc)
            if center_result is None:
                rc = sentai.calib.orientation_manual_descend_start()
                _orientation_result_from_start(rc)
                results.center_hold_descend_ok = False
            else:
                results.center_hold_descend_ok = bool(center_result[0])
        else:
            rc = sentai.calib.orientation_manual_descend_start()
            _orientation_result_from_start(rc)

        _set_phase(summary, "post_acquisition", "flight_sequence_done")
        summary["status"], abort_reason = results.final_state()
        if abort_reason:
            _set_abort_reason(summary, abort_reason)
        else:
            summary["abort_reason"] = ""
        if summary["status"] == "FINAL_VALIDATION_OK":
            _persist_calib_contract(summary)
    except Exception as e:
        summary["status"] = "EXCEPTION"
        summary["exception"] = str(e)
        summary["abort_reason"] = "exception"
        _j("mission_exception", {"err": str(e)})
        _j("abort_decision", {
            "reason": "exception",
            "phase": summary.get("phase", ""),
        })
        try:
            sentai.calib.orientation_emergency_stop()
        except Exception:
            pass

    _j("mission_done", {"status": summary["status"],
                         "phase": summary["phase"]})
    _write_summary(summary)
    return summary
