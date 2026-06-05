# s229 - ARM emulator + Gazebo WhyCon calib precursor.
#
# This mission is mirrored by SENTAI_EMU_CRAZY_CALIB_PRECURSOR_AUTORUN while
# B9 still uses fixed autorun targets.  Keep it here as the canonical mission
# text so the next step can stage it into FileX and run it with sentai.run().

import sentai


def run():
    print("CALIB_GZ_BEGIN")
    sentai.fr.init()
    sentai.fr.open("events", "/fr/s229_events.csv")
    sentai.fr.open("scalars", "/fr/s229_scalars.csv")
    sentai.fr.push_event("s229", "begin")

    sentai.crazy.debug(0)
    rc = sentai.crazy.init(576000)
    print("CALIB_GZ_INIT", rc)
    sentai.fr.push_scalar("crazy_init_rc", rc)

    pm = sentai.crazy.ping(1500)
    print("CALIB_GZ_PING_MS", pm)
    sentai.fr.push_scalar("crazy_ping_ms", pm)

    try:
        r = sentai.calib.get_R_cam_to_body()
        off = sentai.calib.get_cam_offset_B()
        is_cal = sentai.calib.is_calibrated()
        print("CALIB_GZ_CALIB_STUB", len(r), off, is_cal)
        sentai.fr.push_scalar("calib_stub_len_R", len(r))
    except Exception as exc:
        print("CALIB_GZ_CALIB_ERR", str(exc))

    sentai.fr.push_event("s229", "fly_start")
    frc = sentai.crazy.fly(0.35, 1800, 2200, 2200)
    print("CALIB_GZ_FLY_RC", frc)
    sentai.fr.push_scalar("crazy_fly_rc", frc)

    try:
        alt = sentai.crazy.altitude(400)
        print("CALIB_GZ_ALT_AFTER", alt)
        sentai.fr.push_scalar("alt_after", alt)
    except Exception as exc:
        print("CALIB_GZ_ALT_ERR", str(exc))

    sr = sentai.crazy.stop()
    print("CALIB_GZ_STOP", sr)
    sentai.fr.push_scalar("crazy_stop_rc", sr)
    sentai.fr.push_event("s229", "done")
    sentai.fr.task_stop()
    sentai.fs.mkdir("/b9")
    sentai.fs.write("/b9/s229_status.txt", "done\n")
    sentai.fs.sync()
    print("CALIB_GZ_DONE")


if __name__ == "__main__":
    run()
