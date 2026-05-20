# s181 — WhyCon SIM pose-estimation eval (MicroPython side).
#
# Runs inside `sentai_sim`.  Sweeps known world poses (X, Y, Z) over
# a pinhole camera, forward-projects each target to image space,
# synth-draws a Krajník-pattern marker at the predicted pixel location,
# runs WhyCon detect + closed-form PnP, and prints CSV rows with
# both ground-truth and estimated (X, Y, Z).
#
# WBS: OP-S10-W19-T4.
#
# Output: stdout CSV lines prefixed with "S181:" so the host harness
# can extract them while the rest of the REPL banner is noise.

import sentai

FX = 240.0
FY = 240.0
CX = 160.0
CY = 120.0
W_PX = 320
H_PX = 240
DIAM_M = 0.08          # WhyCon outer-ring diameter (8 cm)
R_PHYS = DIAM_M * 0.5  # 0.04 m radius


def setup():
    sentai.markers.init('whycon')
    sentai.markers.set_intrinsics(FX, FY, CX, CY)
    sentai.markers.set_marker_size(DIAM_M)


def project(X, Y, Z):
    # Pinhole forward projection: marker centre + outer-ring radius.
    cx = FX * X / Z + CX
    cy = FY * Y / Z + CY
    r_px = FX * R_PHYS / Z
    return int(cx + 0.5), int(cy + 0.5), int(r_px + 0.5)


def eval_one(X_gt, Y_gt, Z_gt, tag):
    cx_px, cy_px, r_px = project(X_gt, Y_gt, Z_gt)
    if r_px < 6:
        print('S181:%s,%.4f,%.4f,%.4f,%d,%d,%d,SKIP_R_LT_6,,,,,'
              % (tag, X_gt, Y_gt, Z_gt, cx_px, cy_px, r_px))
        return
    if cx_px < r_px or cx_px > W_PX - r_px or \
       cy_px < r_px or cy_px > H_PX - r_px:
        print('S181:%s,%.4f,%.4f,%.4f,%d,%d,%d,SKIP_OOB,,,,,'
              % (tag, X_gt, Y_gt, Z_gt, cx_px, cy_px, r_px))
        return
    n = sentai.markers.synth_one_whycon(cx_px, cy_px, r_px)
    if n < 1:
        print('S181:%s,%.4f,%.4f,%.4f,%d,%d,%d,NO_DETECT,,,,,'
              % (tag, X_gt, Y_gt, Z_gt, cx_px, cy_px, r_px))
        return
    t = sentai.markers.get_pose_tuple(0)
    if t is None:
        print('S181:%s,%.4f,%.4f,%.4f,%d,%d,%d,NO_POSE,,,,,'
              % (tag, X_gt, Y_gt, Z_gt, cx_px, cy_px, r_px))
        return
    # tuple = (id, cx, cy, tx, ty, tz, rx, ry, rz, reproj, backend, valid)
    print('S181:%s,%.4f,%.4f,%.4f,%d,%d,%d,OK,%.4f,%.4f,%.4f,%d,%.6f'
          % (tag, X_gt, Y_gt, Z_gt, cx_px, cy_px, r_px,
             t[3], t[4], t[5], t[11], t[9]))


def sweep_z():
    print('S181:#sweep_z X=0 Y=0 Z varying')
    for z_mm in range(200, 2001, 100):
        Z = z_mm * 0.001
        eval_one(0.0, 0.0, Z, 'Z')


def sweep_x():
    print('S181:#sweep_x Y=0 Z=0.5 X varying (lateral)')
    for x_mm in range(-200, 201, 25):
        X = x_mm * 0.001
        eval_one(X, 0.0, 0.5, 'X')


def sweep_y():
    print('S181:#sweep_y X=0 Z=0.5 Y varying (vertical)')
    for y_mm in range(-150, 151, 25):
        Y = y_mm * 0.001
        eval_one(0.0, Y, 0.5, 'Y')


def main():
    setup()
    print('S181:#begin')
    print('S181:#cols tag,Xgt,Ygt,Zgt,cx_px,cy_px,r_px,status,Xest,Yest,Zest,backend,reproj_px')
    sweep_z()
    sweep_x()
    sweep_y()
    print('S181:#end')


main()
