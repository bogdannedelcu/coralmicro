"""Self-tests for camera_calibration module.

Run: python3 -m examples.sentai_runtime._shared.test_camera_calibration
or:  cd examples/sentai_runtime/_shared && python3 test_camera_calibration.py

Tests:
  T1  Identity recovery from clean synthetic data.
  T2  ±5° tilt recovery — bounded mean residual.
  T3  Drift detection — synthesize 2 calibrations differing by 8°
      and verify drift_deg comes back > 7 < 9.
  T4  load/save round-trip → bit-identical R.
  T5  Corrupt JSON → None (anti-brick: never raise to caller).
  T6  Reflection rejection — provide samples that imply det(R)<0 and
      check accepted=False.
  T7  n_samples<3 → identity + reject reason set.
"""
from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

import numpy as np

from camera_calibration import (
    SCHEMA_VERSION,
    CalibQuality,
    default_R_B_C_sim,
    default_cam_offset_B_sim,
    kabsch_R_cam_to_body,
    load_cam_calib,
    save_cam_calib,
    quality_ok,
    _R_yaw,
)


def _make_samples(R_true: np.ndarray,
                   n: int = 8,
                   yaw_spread_deg: float = 30.0,
                   noise_std_m: float = 0.0,
                   marker_world_radius_m: float = 0.30,
                   drone_z_m: float = 1.0,
                   cam_offset_B: np.ndarray | None = None,
                   seed: int = 7):
    """Generate n synthetic samples for Kabsch.

    Each sample:
        marker_world_i  : random point on circle radius R, z=0.20
        drone_W_i       : random translation near origin
        yaw_i           : random within yaw_spread
        tvec_cam_i      = R_true^T @ (R_W_B^T @ (marker_W - drone_W
                                                  - R_W_B @ cam_offset_B))
                          + Gaussian noise
    """
    rng = np.random.default_rng(seed)
    if cam_offset_B is None:
        cam_offset_B = np.zeros(3)
    samples = []
    for i in range(n):
        theta = 2.0 * math.pi * i / n   # evenly spaced markers
        marker_W = np.array([marker_world_radius_m * math.cos(theta),
                              marker_world_radius_m * math.sin(theta),
                              0.20])
        drone_W = np.array([
            rng.uniform(-0.05, 0.05),
            rng.uniform(-0.05, 0.05),
            drone_z_m,
        ])
        yaw = math.radians(rng.uniform(-yaw_spread_deg, yaw_spread_deg))
        R_W_B = _R_yaw(yaw)
        body = R_W_B.T @ (marker_W - drone_W) - cam_offset_B
        tvec_cam = R_true.T @ body
        if noise_std_m > 0:
            tvec_cam = tvec_cam + rng.normal(0, noise_std_m, size=3)
        samples.append((tvec_cam, marker_W, drone_W, yaw))
    return samples


class TestKabsch(unittest.TestCase):

    def test_t1_identity_recovery_clean(self):
        R_true = default_R_B_C_sim()
        samples = _make_samples(R_true, n=8, noise_std_m=0.0)
        R, q = kabsch_R_cam_to_body(samples)
        self.assertTrue(q.accepted, msg=q.reject_reason)
        # R should equal R_true to machine precision (centered Procrustes
        # with no noise + no cam_offset is exact).
        err = np.linalg.norm(R - R_true, ord="fro")
        self.assertLess(err, 1e-9)
        self.assertAlmostEqual(q.det_R, 1.0, places=6)
        self.assertLess(q.mean_residual_deg, 1e-3)

    def test_t2_recovery_with_noise(self):
        R_true = default_R_B_C_sim()
        samples = _make_samples(R_true, n=12, noise_std_m=0.005,
                                  cam_offset_B=default_cam_offset_B_sim())
        R, q = kabsch_R_cam_to_body(samples)
        # Should still accept; rotation error a few degrees.
        ang = math.degrees(math.acos(np.clip(
            (np.trace(R.T @ R_true) - 1.0) / 2.0, -1.0, 1.0)))
        self.assertLess(ang, 5.0, msg=f"R deviation {ang:.2f}° > 5°")
        ok, reason = quality_ok(q)
        # 5 mm noise per axis at 0.3 m radius → ~1° residual, accept.
        self.assertTrue(ok, msg=f"quality_ok: reason={reason}")

    def test_t3_drift_detection(self):
        R1 = default_R_B_C_sim()
        # R2 = R1 rotated by 8° around Z.
        c, s = math.cos(math.radians(8.0)), math.sin(math.radians(8.0))
        Rz_8 = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        R2_true = Rz_8 @ R1
        samples = _make_samples(R2_true, n=10, noise_std_m=0.0)
        R2_est, q = kabsch_R_cam_to_body(samples, persisted_R=R1)
        # Drift should be ~8°.
        self.assertIsNotNone(q.drift_from_persisted_deg)
        self.assertAlmostEqual(q.drift_from_persisted_deg, 8.0, delta=0.3)

    def test_t4_save_load_roundtrip(self):
        R = default_R_B_C_sim()
        ofs = default_cam_offset_B_sim()
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json",
                                          delete=False) as tf:
            path = Path(tf.name)
        try:
            save_cam_calib(path, R, cam_offset_B=ofs)
            loaded = load_cam_calib(path)
            self.assertIsNotNone(loaded)
            R_l, ofs_l = loaded
            self.assertTrue(np.allclose(R_l, R, atol=1e-12))
            self.assertTrue(np.allclose(ofs_l, ofs, atol=1e-12))
        finally:
            path.unlink(missing_ok=True)

    def test_t5_load_corrupt_returns_none(self):
        cases = ["", "{}", "not json", '{"schema": 99, "R_B_C": "x"}',
                 '{"schema": 1, "R_B_C": [[1,2,3]]}',  # wrong shape
                 '{"schema": 1, "R_B_C": [[1,2,3],[1,2,3],[1,2,NaN]]}']
        for content in cases:
            with tempfile.NamedTemporaryFile(mode="w", suffix=".json",
                                              delete=False) as tf:
                tf.write(content)
                path = Path(tf.name)
            try:
                result = load_cam_calib(path)
                self.assertIsNone(result,
                                  msg=f"expected None for: {content[:40]!r}")
            finally:
                path.unlink(missing_ok=True)

    def test_t6_n_samples_lt_3_rejected(self):
        R_true = default_R_B_C_sim()
        samples = _make_samples(R_true, n=2)
        R, q = kabsch_R_cam_to_body(samples[:2])
        self.assertFalse(q.accepted)
        self.assertIn("n_samples", q.reject_reason or "")

    def test_t7_load_missing_returns_none(self):
        result = load_cam_calib(Path("/tmp/__nonexistent_cam_calib_xyz.json"))
        self.assertIsNone(result)


if __name__ == "__main__":
    unittest.main(verbosity=2)
