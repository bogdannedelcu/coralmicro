"""Camera-to-body rotation calibration (Kabsch 3D Procrustes).

PROBLEM: on real hardware, the OV5640 camera module's mount has
~±2-5° mechanical tolerance per drone unit; the camera-to-body
rotation `R_B_C` is NOT identical to the SDF value used in SIM.
Hardcoding R in firmware → biased landings, biased depth estimation.

SOLUTION: at takeoff, observe N≥4 fiducial markers (ArUco landing
pad), solve for R_B_C using closed-form Kabsch SVD, persist to
`/system/cam_calib.json` (FileX) — re-validate every takeoff,
re-calibrate on >3° drift, fall back to persisted value if marker
not visible (degraded mode).

This module is the HOST-SIDE Python reference. The on-board MP
binding `sentai.calib.cam_to_body_from_aruco(samples)` will mirror
this API.

API:
    R, info = kabsch_R_cam_to_body(samples)
        # samples: list of (tvec_cam[3], marker_world[3], drone_W[3], yaw_rad)
        # returns: R 3×3 rotation cam→body, plus quality metrics.

    save_cam_calib(path, R, schema_version=1)
    R = load_cam_calib(path)            # → None if missing/corrupt

    quality_ok(info) -> (bool, reason)  # implements §21.5 fault model

See `objects_plan.md §21` for the full design rationale + fault model.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


SCHEMA_VERSION = 1
QUALITY_DET_THRESHOLD = 0.99      # |det(R)| > 0.99 — proper rotation
QUALITY_RESIDUAL_DEG = 3.0        # mean residual angle < 3°
QUALITY_DRIFT_DEG = 10.0          # vs persisted; reject if exceeds


@dataclass
class CalibQuality:
    n_samples: int
    det_R: float                       # should be +1.0 for proper rotation
    mean_residual_deg: float           # mean angle between R·t_cam_unit and expected
    max_residual_deg: float
    drift_from_persisted_deg: Optional[float] = None
    accepted: bool = False
    reject_reason: Optional[str] = None


def _R_yaw(yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def kabsch_R_cam_to_body(samples,
                          persisted_R: Optional[np.ndarray] = None,
                          quality_det_thr: float = QUALITY_DET_THRESHOLD,
                          quality_residual_deg: float = QUALITY_RESIDUAL_DEG,
                          quality_drift_deg: float = QUALITY_DRIFT_DEG,
                          ) -> tuple[np.ndarray, CalibQuality]:
    """Solve for R_B_C via Kabsch 3D Procrustes.

    Each sample is `(tvec_cam, marker_world, drone_W, yaw_rad)`.

    The relationship:
        marker_world - drone_W = R_W_B(yaw) · cam_offset_B  +  R_W_C(yaw) · tvec_cam
                              = R_W_B(yaw) · (cam_offset_B + R_B_C · tvec_cam)

    Rearranged (drop cam_offset_B for the calibration purpose — solve
    for R_B_C from many samples; cam_offset_B cancels out in the
    centered Procrustes formulation):
        R_W_B(yaw)^T · (marker_world - drone_W - R_W_B · cam_offset_B)
          = R_B_C · tvec_cam

    Stack N samples → cam_vecs (3×N), body_vecs (3×N), solve
    `R_B_C = argmin ||body_vecs - R · cam_vecs||²` via SVD.

    NOTE: this simplified form ASSUMES cam_offset_B is known (from SDF
    / build sheet). For the auto-calibration use case, we can model
    cam_offset_B simultaneously by adding it as a translation in the
    Procrustes problem (rigid registration with translation). The
    current implementation centers both sets (removes mean), so the
    translation cancels — we recover R only.

    Args:
        samples: iterable of (tvec_cam, marker_W, drone_W, yaw_rad).
        persisted_R: optional previously-saved R for drift check.
        quality_*: thresholds for accept/reject.

    Returns:
        (R, CalibQuality). R may be returned even if accepted=False
        (caller can inspect quality for diagnostics).
    """
    n = len(list(samples))   # materialize for len; we walk twice
    if n < 3:
        # Degenerate — return identity + reject.
        R = np.eye(3, dtype=np.float64)
        q = CalibQuality(n_samples=n, det_R=1.0,
                         mean_residual_deg=180.0, max_residual_deg=180.0,
                         accepted=False, reject_reason="n_samples_lt_3")
        return R, q

    cam_vecs = np.zeros((3, n), dtype=np.float64)
    body_vecs = np.zeros((3, n), dtype=np.float64)
    for i, (tvec_cam, marker_W, drone_W, yaw_rad) in enumerate(samples):
        cam_vecs[:, i] = np.asarray(tvec_cam, dtype=np.float64).reshape(-1)
        # Body-frame expected: R_W_B^T · (marker_W - drone_W) - cam_offset_B.
        # We don't know cam_offset_B here; assume zero for this call and
        # let the caller pass it back in via subtraction in samples.
        delta_W = (np.asarray(marker_W, dtype=np.float64).reshape(-1)
                   - np.asarray(drone_W, dtype=np.float64).reshape(-1))
        R_W_B = _R_yaw(float(yaw_rad))
        body_vecs[:, i] = R_W_B.T @ delta_W

    # Center both sets (cancels translation / cam_offset_B).
    cam_centered = cam_vecs - cam_vecs.mean(axis=1, keepdims=True)
    body_centered = body_vecs - body_vecs.mean(axis=1, keepdims=True)

    # Kabsch SVD: H = body · cam^T, then R = U @ diag(1,1,sign(det(UV^T))) @ V^T.
    H = body_centered @ cam_centered.T
    U, _S, Vt = np.linalg.svd(H)
    d = float(np.sign(np.linalg.det(U @ Vt)))
    R = U @ np.diag([1.0, 1.0, d]) @ Vt
    det_R = float(np.linalg.det(R))

    # Residuals: per-sample angle between R·cam_unit and body_unit.
    residuals_deg = []
    for i in range(n):
        a = R @ cam_vecs[:, i]
        a_n = np.linalg.norm(a)
        b_n = np.linalg.norm(body_vecs[:, i])
        if a_n < 1e-6 or b_n < 1e-6:
            continue
        cos_t = float(np.clip(a @ body_vecs[:, i] / (a_n * b_n), -1.0, 1.0))
        residuals_deg.append(math.degrees(math.acos(cos_t)))
    mean_res = float(np.mean(residuals_deg)) if residuals_deg else 180.0
    max_res = float(np.max(residuals_deg)) if residuals_deg else 180.0

    drift_deg = None
    if persisted_R is not None:
        drift_deg = _rotation_angle_deg(R, persisted_R)

    reject_reason = None
    if abs(det_R) < quality_det_thr:
        reject_reason = f"det_R={det_R:.4f}<{quality_det_thr}"
    elif det_R < 0:
        reject_reason = f"det_R={det_R:.4f}<0_reflection_not_rotation"
    elif mean_res > quality_residual_deg:
        reject_reason = (f"mean_residual_deg={mean_res:.2f}"
                         f">{quality_residual_deg}")
    elif drift_deg is not None and drift_deg > quality_drift_deg:
        reject_reason = (f"drift_from_persisted_deg={drift_deg:.2f}"
                         f">{quality_drift_deg}")

    q = CalibQuality(
        n_samples=n, det_R=det_R,
        mean_residual_deg=mean_res, max_residual_deg=max_res,
        drift_from_persisted_deg=drift_deg,
        accepted=(reject_reason is None),
        reject_reason=reject_reason,
    )
    return R, q


def _rotation_angle_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    """Geodesic angle between two rotations (deg)."""
    M = R1.T @ R2
    cos_t = float(np.clip((np.trace(M) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cos_t))


def save_cam_calib(path: Path,
                    R: np.ndarray,
                    cam_offset_B: Optional[np.ndarray] = None,
                    schema_version: int = SCHEMA_VERSION) -> None:
    """Write cam_calib.json (schema-versioned).

    Schema 1:
        {
          "schema": 1,
          "R_B_C": [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]],
          "cam_offset_B": [x, y, z] (optional)
        }
    """
    if R.shape != (3, 3):
        raise ValueError(f"R must be 3x3, got {R.shape}")
    payload = {
        "schema": schema_version,
        "R_B_C": [[float(R[i, j]) for j in range(3)] for i in range(3)],
    }
    if cam_offset_B is not None:
        cam_offset_B = np.asarray(cam_offset_B, dtype=np.float64).reshape(-1)
        if cam_offset_B.shape != (3,):
            raise ValueError(f"cam_offset_B must be 3-vector, "
                             f"got {cam_offset_B.shape}")
        payload["cam_offset_B"] = [float(v) for v in cam_offset_B]
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(payload, indent=2))


def load_cam_calib(path: Path,
                    ) -> Optional[tuple[np.ndarray, Optional[np.ndarray]]]:
    """Load cam_calib.json; return (R, cam_offset_B) or None on any error.

    Errors are NOT raised — self-healing requirement per embeded.md
    section M (anti-brick): corrupt calibration must NOT brick boot.
    Caller can use default identity + emit `CAL_SCHEMA_FAIL` event.
    """
    p = Path(path)
    if not p.exists():
        return None
    try:
        payload = json.loads(p.read_text())
    except (json.JSONDecodeError, OSError):
        return None
    if payload.get("schema") != SCHEMA_VERSION:
        return None
    try:
        R = np.asarray(payload["R_B_C"], dtype=np.float64)
        if R.shape != (3, 3):
            return None
        if not np.all(np.isfinite(R)):
            return None
    except (KeyError, TypeError, ValueError):
        return None
    cam_offset_B = None
    if "cam_offset_B" in payload:
        try:
            cam_offset_B = np.asarray(payload["cam_offset_B"], dtype=np.float64)
            if cam_offset_B.shape != (3,):
                cam_offset_B = None
            elif not np.all(np.isfinite(cam_offset_B)):
                cam_offset_B = None
        except (TypeError, ValueError):
            cam_offset_B = None
    return R, cam_offset_B


def quality_ok(q: CalibQuality) -> tuple[bool, Optional[str]]:
    """Public predicate matching §21.5 fault model."""
    return bool(q.accepted), q.reject_reason


def default_R_B_C_sim() -> np.ndarray:
    """The hardcoded R_B_C used in s130 image-only nav.

    Empirically validated 2026-05-14 against sentai_crazysim Gazebo
    SDF + drone hover @ z=1 m. THIS is the value the on-board firmware
    should bootstrap to when no `cam_calib.json` exists yet.

    On hardware, the auto-calibration at takeoff will refine this.
    """
    return np.array([[0.0, 1.0,  0.0],
                     [1.0, 0.0,  0.0],
                     [0.0, 0.0, -1.0]], dtype=np.float64)


def default_cam_offset_B_sim() -> np.ndarray:
    """Camera optical-center offset from drone CoM, body frame (m).

    From sentai_crazysim SDF: cam mount at -0.04 X, 0 Y, -0.02 Z.
    """
    return np.array([-0.04, 0.0, -0.02], dtype=np.float64)
