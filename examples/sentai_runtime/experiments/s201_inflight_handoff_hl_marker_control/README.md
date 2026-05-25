# s201 - B4 In-Flight Handoff To HL Marker Control

Purpose: consume the accepted A3 calibration artifact, warm up marker/PnP pose,
then develop the B4 in-flight handoff sequence.

Current seed calibration:

```text
../s197_sota_calib_orientation_guarded/
  iter27_camera_landing_contract/
    mission_s197_calibration.json
```

`run.sh` converts that JSON into `/system/calib.ini`.

`/system/calib.ini` contains the runtime contract consumed by
`sentai.calib.load()` plus B4 metadata/provenance used by the REPL mission.
The C++ loader reads the known runtime keys and ignores unknown metadata keys:

```ini
schema=2
R_B_C=...
cam_offset_B=-0.04,0.0,-0.02
kp_x=-1.0
kp_y=-1.0
kp_yaw=-1.0
```

The first s201 implementation is deliberately a preflight/calibration-loading
gate.  It does not prove handoff yet.  It validates that the B4 mission starts
from a known accepted calibration and configures the WhyCon runtime with the
same marker layout used by A3.

Next phases:

- RPYT marker acquisition using the conservative A3 lock contract;
- PnP pose extraction with loaded `R_cam_to_body`;
- position-only ExtPos warmup;
- in-flight release from bootstrap RPYT without disarm;
- generic-hover hold-current proof;
- HL/generic motion proof: `-X`, `+X`, center, `-Y`, `+Y`, center;
- soft landing under post-handoff supervision.
