# s199 - calibration with visual axis-envelope survey

**Task**: TD-S10-A3 / TD-S10-B3

s199 starts from the successful s197 camera-only calibration baseline and adds
one more calibration phase before landing: an image-only axis-envelope survey.

The survey is still part of calibration, not general navigation.  After the
camera/body rotation is inferred, validated, and the marker-pad centroid is
hovered near image center, s199 probes the usable visual envelope:

1. move toward max image/camera X while keeping Y centered;
2. move toward min image/camera X while keeping Y centered;
3. return to center;
4. move toward max image/camera Y while keeping X centered;
5. move toward min image/camera Y while keeping X centered;
6. return to center;
7. run the camera-referenced slow landing.

Envelope extrema are detected from marker centers/radii against image bounds
(`fov_margin_px`) and smoothed marker-count evidence.  Gazebo GT is never used
inside the mission.

On success, s199 writes:

```text
mission_s199_journal.txt
mission_s199_summary.json
mission_s199_calibration.json
calib.ini
```

`calib.ini` is the compact persisted calibration surface intended for the drone
FS: rotation matrix, marker layout metadata, image intrinsics, landing marker
window, and envelope policy.

Run:

```bash
./run.sh iter1
```
