"""Per-landmark inverse-depth EKF (Civera/Davison/Montiel TRO 2008).

Reference Python implementation; mirrors the math intended for
`sentai_object_lifter.cc` (L5 C++ port). Single-precision-friendly:
no library calls outside numpy linear algebra. Per-landmark 1-state
EKF (scalar ρ); anchor + world bearing are immutable constants.

State (per landmark):
    rho       : float  inverse depth (1/d), state estimate
    var_rho   : float  variance σ_ρ²
    anchor_w  : (3,)   camera world position at FIRST observation
    r_w       : (3,)   unit bearing in world frame (set at FIRST obs)
    n_obs     : int    number of observations integrated
    age_s     : float  elapsed time since anchor (for σ_ρ < 0.5/d² gate)

Constants (per camera):
    fx, fy, cx, cy : intrinsics
    R_B_C          : 3×3 body←camera rotation (from sentai.calib in
                     production; hardcoded for s130-compat in tests)
    cam_offset_B   : (3,) camera optical center offset from drone CoM,
                     body frame

External-facing API:
    L = Landmark(...)                      # init from class-prior pseudo-depth
    L.update(u_obs, v_obs, drone_w, yaw)   # one EKF measurement step
    L.ready() -> bool                      # σ_ρ < ε_lin · ρ²
    L.world_pos() -> (3,) or None          # current best L_W estimate
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# Civera linearization threshold: a landmark is "lifted" (depth
# reliable) when σ_d / d² < ε_lin ⇔ σ_ρ < ε_lin · ρ².
EPS_LIN_DEFAULT = 0.5

# Observation noise (1 px std for ArUco bbox center; tunable per class).
OBS_PX_STD_DEFAULT = 1.0

# Initial relative uncertainty on ρ (50%): σ_ρ₀ = ALPHA_INIT · ρ₀.
ALPHA_INIT_DEFAULT = 0.5


def _R_yaw(yaw: float) -> np.ndarray:
    """Body-to-world rotation for body-frame yaw (Z-up). 3×3."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=np.float64)


def bearing_from_pixel(u: float, v: float,
                       fx: float, fy: float,
                       cx: float, cy: float) -> np.ndarray:
    """Pixel center → unit bearing direction in CAMERA frame.

    Camera convention: +X right, +Y down, +Z forward (OpenCV).
    """
    bx = (u - cx) / fx
    by = (v - cy) / fy
    bz = 1.0
    v = np.array([bx, by, bz], dtype=np.float64)
    return v / np.linalg.norm(v)


def project_world_to_pixel(L_W: np.ndarray,
                           drone_W: np.ndarray,
                           yaw: float,
                           R_B_C: np.ndarray,
                           cam_offset_B: np.ndarray,
                           fx: float, fy: float,
                           cx: float, cy: float) -> Optional[tuple]:
    """Project a world point L_W into the current camera (u, v).

    Returns None if behind camera (δ_C.z <= 0).
    """
    R_W_B = _R_yaw(yaw)
    c_W = drone_W + R_W_B @ cam_offset_B
    delta_W = L_W - c_W
    # δ_C = R_C_W · δ_W = (R_W_B · R_B_C)^T · δ_W = R_B_C^T · R_W_B^T · δ_W
    delta_C = R_B_C.T @ (R_W_B.T @ delta_W)
    if delta_C[2] <= 1e-6:
        return None
    u = fx * delta_C[0] / delta_C[2] + cx
    v = fy * delta_C[1] / delta_C[2] + cy
    return float(u), float(v), delta_C  # delta_C returned for Jacobian


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    R_B_C: np.ndarray              # 3×3 body←camera (orthogonal)
    cam_offset_B: np.ndarray       # (3,)

    def project(self, L_W, drone_W, yaw):
        return project_world_to_pixel(
            L_W, drone_W, yaw,
            self.R_B_C, self.cam_offset_B,
            self.fx, self.fy, self.cx, self.cy,
        )


@dataclass
class Landmark:
    """One landmark tracked by inverse-depth EKF.

    Init via `Landmark.from_class_prior(...)` (uses bbox apparent size).
    Update via `.update(u_obs, v_obs, drone_W, yaw, dt_s, var_obs_px2)`.

    The (anchor_w, r_w) frame is the "anchored inverse-depth"
    parametrization from Civera 2008: r_w is the world-frame unit
    bearing at the first observation, captured from the anchor camera
    position. ρ is the inverse distance along r_w from anchor.

    Landmark world position estimate:
        L_W = anchor_w + (1/ρ) · r_w

    The world-frame bearing IS observable at the anchor instant (we
    know drone pose), and is therefore TREATED AS A CONSTANT (no
    state). Civera 2008 keeps (θ, φ) as state because the bearing has
    uncertainty too; here we simplify to scalar ρ since for our
    targets (ArUco markers + class-prior bboxes) the bearing
    uncertainty (~1 px) maps to ~1.7 mm at 1 m — negligible vs ρ
    uncertainty (50% initial).
    """

    rho: float
    var_rho: float
    anchor_w: np.ndarray             # (3,)
    r_w: np.ndarray                  # (3,) unit
    cam: CameraModel
    n_obs: int = 0
    age_s: float = 0.0
    rho_min: float = 0.05            # depth_max = 20 m cap
    rho_max: float = 20.0            # depth_min = 5 cm
    history: list = field(default_factory=list)  # debug trace

    @classmethod
    def from_class_prior(cls,
                         u_c: float, v_c: float,
                         bbox_w_px: float,
                         real_size_m: float,
                         drone_W: np.ndarray, yaw: float,
                         cam: CameraModel,
                         alpha_init: float = ALPHA_INIT_DEFAULT
                         ) -> "Landmark":
        """Init landmark from class-prior pseudo-depth.

        d₀  = fx · real_size_m / bbox_w_px
        ρ₀  = 1/d₀
        σ_ρ = α_init · ρ₀
        r_w = R_W_B(yaw) · R_B_C · bearing_C(u_c, v_c)
        anchor_w = drone_W + R_W_B · cam_offset_B
        """
        if bbox_w_px <= 1.0 or real_size_m <= 0.0:
            raise ValueError(f"invalid class-prior init: "
                             f"bbox_w_px={bbox_w_px}, "
                             f"real_size_m={real_size_m}")
        d0 = cam.fx * real_size_m / bbox_w_px
        rho0 = 1.0 / d0
        var0 = (alpha_init * rho0) ** 2

        b_C = bearing_from_pixel(u_c, v_c,
                                  cam.fx, cam.fy, cam.cx, cam.cy)
        R_W_B = _R_yaw(yaw)
        r_w = R_W_B @ (cam.R_B_C @ b_C)
        r_w /= np.linalg.norm(r_w)
        anchor_w = drone_W + R_W_B @ cam.cam_offset_B
        return cls(
            rho=rho0, var_rho=var0,
            anchor_w=anchor_w.astype(np.float64),
            r_w=r_w.astype(np.float64),
            cam=cam, n_obs=1, age_s=0.0,
        )

    def world_pos(self) -> Optional[np.ndarray]:
        if not (self.rho_min <= self.rho <= self.rho_max):
            return None
        return self.anchor_w + (1.0 / self.rho) * self.r_w

    def ready(self, eps_lin: float = EPS_LIN_DEFAULT) -> bool:
        """Civera linearization-validity gate: σ_ρ < ε_lin · ρ²."""
        return math.sqrt(max(self.var_rho, 0.0)) < eps_lin * (self.rho ** 2)

    # ─────────────────────────────────────────────────────────────
    # Measurement update
    # ─────────────────────────────────────────────────────────────
    def update(self,
               u_obs: float, v_obs: float,
               drone_W: np.ndarray, yaw: float,
               dt_s: float,
               var_obs_px2: float = OBS_PX_STD_DEFAULT ** 2
               ) -> dict:
        """One EKF measurement step.

        Returns a status dict with the post-update state + innovation
        magnitude. Errors are NON-RAISING: on invalid prediction (e.g.
        landmark behind camera, NaN δ_C) the state is left untouched
        and 'rejected': True is set. See embeded.md F (fault model):
        local recovery before subsystem restart.
        """
        status = {"rejected": False, "reason": None,
                  "innov_px": 0.0, "rho": self.rho, "var_rho": self.var_rho,
                  "n_obs": self.n_obs, "age_s": self.age_s}

        L_W = self.world_pos()
        if L_W is None:
            status["rejected"] = True
            status["reason"] = f"rho_out_of_bounds_{self.rho:.3g}"
            return status

        proj = self.cam.project(L_W, drone_W, yaw)
        if proj is None:
            status["rejected"] = True
            status["reason"] = "behind_camera"
            return status
        u_pred, v_pred, delta_C = proj
        # ----- Jacobian H = ∂(u, v)/∂ρ -----
        # L_W      = anchor + (1/ρ) · r_W
        # ∂L_W/∂ρ  = -(1/ρ²) · r_W
        # δ_W      = L_W - c_W
        # ∂δ_W/∂ρ  = ∂L_W/∂ρ
        # δ_C      = R_C_W · δ_W   (R_C_W constant)
        # ∂δ_C/∂ρ  = R_C_W · ∂δ_W/∂ρ
        R_W_B = _R_yaw(yaw)
        R_C_W = (R_W_B @ self.cam.R_B_C).T
        d_L_d_rho = -(1.0 / (self.rho ** 2)) * self.r_w
        d_dC_d_rho = R_C_W @ d_L_d_rho      # (3,)
        z = delta_C[2]
        # u = fx · δ_C.x / z + cx
        # ∂u/∂ρ = fx · (∂δ_C.x/∂ρ · z - δ_C.x · ∂z/∂ρ) / z²
        du_drho = self.cam.fx * (d_dC_d_rho[0] * z
                                 - delta_C[0] * d_dC_d_rho[2]) / (z * z)
        dv_drho = self.cam.fy * (d_dC_d_rho[1] * z
                                 - delta_C[1] * d_dC_d_rho[2]) / (z * z)
        H = np.array([du_drho, dv_drho], dtype=np.float64)  # (2,)

        innov = np.array([u_obs - u_pred, v_obs - v_pred], dtype=np.float64)
        # S = H · σ_ρ² · H^T + R_obs    (2×2 because innov is 2-vector)
        R_obs = var_obs_px2 * np.eye(2, dtype=np.float64)
        S = np.outer(H, H) * self.var_rho + R_obs
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            status["rejected"] = True
            status["reason"] = "S_singular"
            return status

        # K = σ_ρ² · H^T · S^-1     shape (2,)
        K = self.var_rho * (S_inv @ H)
        # Capture innovation magnitude BEFORE deciding to reject — useful
        # diagnostic when rho_clamp kicks in (innov was large enough to
        # overshoot bounds).
        status["innov_px"] = float(np.linalg.norm(innov))
        # rho update
        rho_new = self.rho + float(K @ innov)
        # Clamp ρ to valid range (Civera 2008 §V-B numerical robustness).
        if not (self.rho_min <= rho_new <= self.rho_max):
            status["rejected"] = True
            status["reason"] = f"rho_clamp_violated_{rho_new:.3g}"
            return status
        # Joseph form (numerically stable):
        # P_new = (1 - K·H) · P · (1 - K·H)^T + K · R · K^T
        # Here P is scalar (σ_ρ²), K is 2-vec, H is 2-vec → (1 - K·H) is
        # scalar via inner product since K and H are both 2-vec column*row:
        KH = float(K @ H)            # scalar (since K, H are both 2-vec)
        var_new_std = (1.0 - KH) ** 2 * self.var_rho
        var_new_joseph = var_new_std + float(K @ R_obs @ K)
        # Sanity check: variance must stay positive (always true for
        # Joseph form by construction).
        if not (var_new_joseph > 0.0 and math.isfinite(var_new_joseph)):
            status["rejected"] = True
            status["reason"] = f"var_invalid_{var_new_joseph:.3g}"
            return status

        # Commit state.
        self.rho = float(rho_new)
        self.var_rho = float(var_new_joseph)
        self.n_obs += 1
        self.age_s += float(dt_s)

        status.update({
            "rho": self.rho,
            "var_rho": self.var_rho,
            "n_obs": self.n_obs,
            "age_s": self.age_s,
            "innov_px": float(np.linalg.norm(innov)),
            "u_pred": float(u_pred),
            "v_pred": float(v_pred),
            "d_est_m": 1.0 / self.rho,
            "ready": self.ready(),
        })
        self.history.append(status.copy())
        return status

    # ─────────────────────────────────────────────────────────────
    # Diagnostics
    # ─────────────────────────────────────────────────────────────
    def status(self) -> dict:
        Lw = self.world_pos()
        return {
            "rho": self.rho,
            "var_rho": self.var_rho,
            "sigma_rho": math.sqrt(max(self.var_rho, 0.0)),
            "d_est_m": 1.0 / self.rho if self.rho_min <= self.rho <= self.rho_max
                       else float("nan"),
            "n_obs": self.n_obs,
            "age_s": self.age_s,
            "L_W": None if Lw is None else Lw.tolist(),
            "anchor_W": self.anchor_w.tolist(),
            "r_W": self.r_w.tolist(),
            "ready": self.ready(),
        }
