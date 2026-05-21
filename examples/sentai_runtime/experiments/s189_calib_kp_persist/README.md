# s189 — Kp persist round-trip (calib.ini schema v2 + commit_kp)

**WBS:** `OP-S10-W21-T3`.  Validates that the PID gains (Kp_x, Kp_y,
Kp_yaw) survive save / clear / load via `/system/calib.ini`.

| # | Scenario                                          | Pass criterion                |
|---|----------------------------------------------------|-------------------------------|
| T1 | commit_kp + save + clear + load round-trip        | All 3 axes restored exactly   |
| T2 | Default state (uncalibrated)                       | -1.0 sentinel returned        |
| T3 | Invalid axis / NaN / negative kp rejected          | commit_kp returns -1          |
| T4 | -1.0 sentinel accepted (commit_kp(axis, -1.0))     | get_persisted_kp returns -1.0 |
| T5 | INI on disk contains kp_x, kp_y, kp_yaw lines       | Format inspect               |

Pure synthetic.  Builds on the T2 INI parser tests in s188 (which
verify the parser-level forward-compat); s189 verifies the kp_*
specific commit/get API + persistence integration.
