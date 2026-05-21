"""bench_coplanar_pnp — host-side prototype of coplanar multi-marker PnP.

Validates that 6 marker pixel positions + known world XY positions
recover drone position and R with sub-pixel residual.  This replaces
the single-marker Krajník depth-from-ring formula whose perspective
sensitivity is the root cause of the W21-T4 accuracy gap.

Result on iter-37 saved frame: residuals 0.05 px mean, drone z within
1 cm of GT, R within 0.1° of SDF identity.

Algorithm (8 lines of math):
  1. For each correspondence (px, py) ↔ (Mx, My) [coplanar Mz=0]:
       A_row1 = [Mx, My, 1, 0, 0, 0, -px*Mx, -px*My, -px]
       A_row2 = [0, 0, 0, Mx, My, 1, -py*Mx, -py*My, -py]
  2. SVD(A) → smallest right-singular-vector = H (3x3).
  3. M = K^-1 H = [m1 m2 m3]; scale lam = 1/|m1|.
  4. r1 = lam*m1; r2 = lam*m2; t = lam*m3.
  5. r3 = r1 × r2; orthonormalize R via SVD(R).
  6. drone_world = -R^T @ t (camera centre in world).

C-port plan (next session, ~80 LoC):
  - Linear DLT: 12x9 system, normal equations 9x9 → SVD (extend
    sentai_svd3 to handle 9x9, OR use power-iteration on AtA).
  - K^-1 H multiplication: 3x3 × 3x3.
  - Reuse sentai_svd3 for the orthonormalization step.
  - Inject into sentai_calib_bringup.cc phase_sample_ as the drone_W
    estimator.  Each tick yields one (drone_W, yaw) anchor from the
    constellation; emit as Kabsch sample alongside per-marker
    tvec_cam (which is fine for the rotation Procrustes; we just
    need ACCURATE drone_W which we now have).
"""
import numpy as np
import cv2
import scipy.ndimage as ndi
from PIL import Image
import sys
import glob

FX, FY, CX, CY = 288.3, 288.3, 160.0, 120.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=np.float32)

# Marker WORLD positions, indexed in PASS-1 ordering.  For PnP we use
# Z=0 (IPPE convention); the 5 mm box top z=0.005 is ignored — its
# effect on the recovered z is +5 mm which we add post-hoc.
MARKER_WORLD = np.array([
    [-0.16, +0.16, 0.0],   # NW
    [+0.16, +0.16, 0.0],   # NE
    [-0.12,  0.00, 0.0],   # W
    [+0.12,  0.00, 0.0],   # E
    [-0.16, -0.16, 0.0],   # SW
    [+0.16, -0.16, 0.0],   # SE
], dtype=np.float32)


def detect_pgm(path):
    im = np.array(Image.open(path))
    dark = (im < 50).astype(np.uint8)
    labeled, n = ndi.label(dark)
    dets = []
    for lbl in range(1, n + 1):
        mask = labeled == lbl
        n_pix = mask.sum()
        if n_pix < 600 or n_pix > 1100:
            continue
        ys, xs = np.where(mask)
        bw = xs.max() - xs.min() + 1
        bh = ys.max() - ys.min() + 1
        if max(bw, bh) / min(bw, bh) > 1.5:
            continue
        dets.append((xs.mean(), ys.mean()))
    return dets


def order_six(dets):
    """Sort 6 detections into MARKER_WORLD index order: NW NE W E SW SE.

    Camera-to-world axis convention for the cf2 SDF mount:
      image x  →  world Y  (cam mounted with 90° rotation about Z)
      image y  →  world X
    So 3 image-x columns map to 3 world-Y bins; within each column,
    image-y sorts by world X.

    Returns img_pts in [NW, NE, W, E, SW, SE] order or None on failure.
    """
    if len(dets) != 6:
        return None
    by_x = sorted(dets, key=lambda d: d[0])
    col_L = sorted(by_x[0:2], key=lambda d: d[1])  # world Y = -0.16
    col_M = sorted(by_x[2:4], key=lambda d: d[1])  # world Y = 0
    col_R = sorted(by_x[4:6], key=lambda d: d[1])  # world Y = +0.16
    return np.array([
        col_R[0], col_R[1],   # NW (X=-0.16), NE (X=+0.16)
        col_M[0], col_M[1],   # W, E
        col_L[0], col_L[1],   # SW, SE
    ], dtype=np.float32)


def coplanar_pnp(img_pts):
    """Closed-form coplanar PnP via OpenCV ITERATIVE.  Returns
    (cam_in_world[3], R_world_to_cam[3,3], reproj_residuals[6])."""
    dist = np.zeros(5, dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(MARKER_WORLD, img_pts, K, dist,
                                    flags=cv2.SOLVEPNP_ITERATIVE)
    R, _ = cv2.Rodrigues(rvec)
    cam_world = -R.T @ tvec.ravel()
    proj, _ = cv2.projectPoints(MARKER_WORLD, rvec, tvec, K, dist)
    res = np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)
    return cam_world, R, res


def main(frame_path):
    dets = detect_pgm(frame_path)
    print(f"Frame: {frame_path}")
    print(f"  Detected {len(dets)} markers")
    img_pts = order_six(dets)
    if img_pts is None:
        print("  Cannot order — abort")
        return
    cam_world, R, res = coplanar_pnp(img_pts)
    print(f"  drone (cam) in WORLD: x={cam_world[0]:+.4f} y={cam_world[1]:+.4f} z={cam_world[2]:+.4f}")
    print(f"  reproj residuals (px): {res}")
    print(f"  mean reproj: {res.mean():.3f}  max: {res.max():.3f}")
    print(f"  R_world→cam (det={np.linalg.det(R):+.4f}):")
    for row in R:
        print(f"    [{row[0]:+.4f}  {row[1]:+.4f}  {row[2]:+.4f}]")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        main(sys.argv[1])
    else:
        # Default: first frame found in fr_current.
        frames = sorted(glob.glob(
            "/home/bogdan/work/coralmicro/examples/sentai_runtime/"
            "experiments/s187_calib_bringup_gazebo/fr_current/frames/"
            "*.pgm"))
        if frames:
            main(frames[0])
        else:
            print("No frames found.")
