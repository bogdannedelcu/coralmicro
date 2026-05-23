#!/usr/bin/env python3
"""Validate synthetic WhyCon datasets against manifest ground truth."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import itertools
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
from typing import Any

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_RESULTS_ROOT = REPO_ROOT / "dataset" / "TD-S10-B2"


OPENCV_PARAMS = {
    "algorithm": "connected_component_concentric_whycon",
    "reference_notes": [
        "WhyCon/Krajnik-Nitsche-style concentric black-white circular marker detector",
        "Algorithm is aligned with the published WhyCon family: thresholding, connected components, concentric circular marker validation, and marker pose estimation",
        "OpenCV is used for image-processing primitives; OpenCV does not provide an official cv2.whycon module",
    ],
    "literature_references": [
        "Krajnik et al., External Localization System for Mobile Robotics, ICAR 2013",
        "Krajnik et al., A Practical Multirobot Localization System, JINT 2014",
        "Nitsche et al., WhyCon: An Efficient, Marker-based Localization System, 2015",
        "lrse/whycon open-source implementation and documentation",
    ],
    "threshold_values": [100, 130, 150],
    "threshold_type": "THRESH_BINARY_INV",
    "component_connectivity": 8,
    "outer_geometry": "connected component moments + external contour circularity",
    "dot_geometry": "small dark connected component inside outer bbox",
    "min_outer_radius_px": 4.0,
    "max_outer_radius_px": 36.0,
    "min_outer_area_px2": 30.0,
    "min_outer_circularity": 0.55,
    "max_center_offset_px": 4.0,
    "max_center_offset_radius_frac": 0.25,
    "min_dot_outer_radius_ratio": 0.12,
    "max_dot_outer_radius_ratio": 0.45,
    "angle_stability_min_axis_ratio": 1.10,
    "partial_edge_circle_fit": False,
    "partial_edge_circle_fit_below_circularity": 0.55,
    "partial_edge_circle_fit_min_radius_scale": 0.5,
    "partial_edge_circle_fit_max_radius_scale": 3.0,
    "ambiguity_reprojection_ratio": 1.25,
    "ambiguity_reprojection_delta_px": 1.0,
    "ambiguity_translation_delta_m": 0.05,
    "ambiguity_yaw_delta_deg": 15.0,
    "ambiguity_max_alternatives_recorded": 5,
    "ambiguity_max_permutation_markers": 7,
    "pose_max_reprojection_rmse_px": 0.5,
}


OPENCV_VARIANTS = {
    "paper": {},
    "edge_partial": {
        "algorithm": "connected_component_concentric_whycon_edge_partial_experiment",
        "variant_notes": [
            "Experimental recall variant for cropped/edge markers",
            "Lowers outer circularity gate and applies minEnclosingCircle to low-circularity edge candidates",
            "Not the default paper-backed parity baseline until ported and validated in sentai_sim",
        ],
        "min_outer_circularity": 0.15,
        "max_center_offset_px": 6.0,
        "partial_edge_circle_fit": True,
    },
}


def run_id() -> str:
    return dt.datetime.now().strftime("validation_%Y%m%d_%H%M%S")


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise RuntimeError(f"{path}:{line_no}: invalid JSON: {exc}") from exc
    return rows


def percentile(values: list[float], pct: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    k = (len(ordered) - 1) * (pct / 100.0)
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return ordered[lo]
    return ordered[lo] * (hi - k) + ordered[hi] * (k - lo)


def numeric_stats(values: list[float]) -> dict[str, float | int | None]:
    clean = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    if not clean:
        return {
            "n": 0,
            "mean": None,
            "median": None,
            "p95": None,
            "max": None,
            "mae": None,
            "bias": None,
            "rmse": None,
        }
    return {
        "n": len(clean),
        "mean": statistics.fmean(clean),
        "median": statistics.median(clean),
        "p95": percentile(clean, 95),
        "max": max(clean),
        "mae": statistics.fmean(abs(v) for v in clean),
        "bias": statistics.fmean(clean),
        "rmse": math.sqrt(statistics.fmean(v * v for v in clean)),
    }


def angle_delta_deg(a: float | None, b: float | None, period: float = 180.0) -> float | None:
    if a is None or b is None:
        return None
    delta = (float(a) - float(b) + period / 2.0) % period - period / 2.0
    return delta


def contour_circularity(contour: np.ndarray) -> float:
    area = cv2.contourArea(contour)
    peri = cv2.arcLength(contour, True)
    if peri <= 0.0:
        return 0.0
    return float(4.0 * math.pi * area / (peri * peri))


def contour_circle(contour: np.ndarray) -> tuple[float, float, float]:
    (x, y), radius = cv2.minEnclosingCircle(contour)
    return float(x), float(y), float(radius)


def contour_ellipse(contour: np.ndarray) -> dict[str, float | None]:
    if len(contour) < 5:
        return {"axis_a": None, "axis_b": None, "angle_deg": None}
    (cx, cy), (w, h), angle = cv2.fitEllipse(contour)
    major = max(float(w), float(h)) * 0.5
    minor = min(float(w), float(h)) * 0.5
    return {"axis_a": major, "axis_b": minor, "angle_deg": float(angle)}


def component_moment_geometry(
    labels: np.ndarray,
    label_idx: int,
    bbox: tuple[int, int, int, int],
    fallback_center: tuple[float, float],
    fallback_radius: float,
    contour: np.ndarray | None,
) -> dict[str, float | None]:
    x, y, w, h = bbox
    ys_local, xs_local = np.nonzero(labels[y:y + h, x:x + w] == label_idx)
    if len(xs_local) == 0:
        return {
            "center_x": fallback_center[0],
            "center_y": fallback_center[1],
            "axis_a": fallback_radius,
            "axis_b": fallback_radius,
            "angle_deg": None,
        }

    xs = xs_local.astype(np.float64) + float(x)
    ys = ys_local.astype(np.float64) + float(y)
    cx = float(fallback_center[0])
    cy = float(fallback_center[1])

    m00 = float(len(xs))
    mu20 = float(np.sum(xs * xs) / m00 - cx * cx)
    mu02 = float(np.sum(ys * ys) / m00 - cy * cy)
    mu11 = float(np.sum(xs * ys) / m00 - cx * cy)
    trace = mu20 + mu02
    det = mu20 * mu02 - mu11 * mu11
    disc = max(0.0, trace * trace * 0.25 - det)
    root = math.sqrt(disc)
    l1 = trace * 0.5 + root
    l2 = trace * 0.5 - root
    axis_a = 2.0 * math.sqrt(l1) if l1 > 0.0 else fallback_radius
    axis_b = 2.0 * math.sqrt(l2) if l2 > 0.0 else fallback_radius
    if axis_a < axis_b:
        axis_a, axis_b = axis_b, axis_a
    angle_deg = math.degrees(0.5 * math.atan2(2.0 * mu11, mu20 - mu02))

    return {
        "center_x": cx,
        "center_y": cy,
        "axis_a": float(axis_a),
        "axis_b": float(axis_b),
        "angle_deg": float(angle_deg),
    }


def contour_area(contours: list[np.ndarray], idx: int) -> float:
    return float(cv2.contourArea(contours[idx]))


def child_of(hierarchy: np.ndarray, idx: int) -> int:
    return int(hierarchy[idx][2])


def detect_opencv_whycon(gray: np.ndarray) -> list[dict[str, float]]:
    detections: list[dict[str, float]] = []
    for threshold_value in OPENCV_PARAMS["threshold_values"]:
        detections.extend(detect_opencv_whycon_at_threshold(gray, int(threshold_value)))
    return dedupe_detections(detections)


def detect_opencv_whycon_at_threshold(gray: np.ndarray, threshold_value: int) -> list[dict[str, float]]:
    _, binary = cv2.threshold(
        gray,
        threshold_value,
        255,
        cv2.THRESH_BINARY_INV,
    )
    n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary,
        connectivity=int(OPENCV_PARAMS["component_connectivity"]),
    )
    components = build_component_geometries(labels, stats, centroids, n_labels)
    detections: list[dict[str, float]] = []

    for idx, comp in components.items():
        area = comp["area"]
        if area < OPENCV_PARAMS["min_outer_area_px2"]:
            continue
        outer_x, outer_y = comp["center"]
        outer_r = comp["radius_outer"]
        if not (
            OPENCV_PARAMS["min_outer_radius_px"]
            <= outer_r
            <= OPENCV_PARAMS["max_outer_radius_px"]
        ):
            continue
        circularity = comp["circularity"]
        if circularity < OPENCV_PARAMS["min_outer_circularity"]:
            continue

        dot = find_concentric_dot(idx, comp, components)
        if dot is None:
            continue
        dot_x, dot_y = dot["center"]
        dot_r = dot["radius_outer"]
        center_offset = math.hypot(dot_x - outer_x, dot_y - outer_y)
        max_offset = max(
            OPENCV_PARAMS["max_center_offset_px"],
            OPENCV_PARAMS["max_center_offset_radius_frac"] * outer_r,
        )
        if center_offset > max_offset:
            continue

        dot_ratio = dot_r / outer_r if outer_r > 0.0 else 0.0
        if not (
            OPENCV_PARAMS["min_dot_outer_radius_ratio"]
            <= dot_ratio
            <= OPENCV_PARAMS["max_dot_outer_radius_ratio"]
        ):
            continue
        det_center = [outer_x, outer_y]
        det_radius = outer_r
        partial_circle = comp.get("min_enclosing_circle")
        if (
            OPENCV_PARAMS.get("partial_edge_circle_fit")
            and circularity < float(OPENCV_PARAMS["partial_edge_circle_fit_below_circularity"])
            and partial_circle is not None
        ):
            circle_x, circle_y, circle_r = partial_circle
            min_r = float(OPENCV_PARAMS["partial_edge_circle_fit_min_radius_scale"]) * outer_r
            max_r = float(OPENCV_PARAMS["partial_edge_circle_fit_max_radius_scale"]) * outer_r
            if min_r <= circle_r <= max_r:
                det_center = [circle_x, circle_y]
                det_radius = circle_r

        detections.append(
            {
                "center": det_center,
                "radius_outer": det_radius,
                "axis_a": comp["axis_a"],
                "axis_b": comp["axis_b"],
                "angle_deg": comp["angle_deg"],
                "circularity": circularity,
                "area": float(area),
                "dot_radius": dot_r,
                "dot_outer_radius_ratio": dot_ratio,
                "threshold_value": threshold_value,
            }
        )

    detections.sort(key=lambda d: (d["center"][1], d["center"][0]))
    return dedupe_detections(detections)


def build_component_geometries(
    labels: np.ndarray,
    stats: np.ndarray,
    centroids: np.ndarray,
    n_labels: int,
) -> dict[int, dict[str, Any]]:
    components: dict[int, dict[str, Any]] = {}
    for idx in range(1, n_labels):
        x = int(stats[idx, cv2.CC_STAT_LEFT])
        y = int(stats[idx, cv2.CC_STAT_TOP])
        w = int(stats[idx, cv2.CC_STAT_WIDTH])
        h = int(stats[idx, cv2.CC_STAT_HEIGHT])
        area_px = int(stats[idx, cv2.CC_STAT_AREA])
        if area_px <= 0 or w <= 0 or h <= 0:
            continue

        roi = (labels[y:y + h, x:x + w] == idx).astype(np.uint8) * 255
        contours_raw, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = list(contours_raw)
        contour = max(contours, key=cv2.contourArea) if contours else None
        contour_area_val = float(cv2.contourArea(contour)) if contour is not None else float(area_px)
        circularity = contour_circularity(contour) if contour is not None else 0.0
        min_enclosing_circle = None
        if contour is not None:
            (circle_x, circle_y), circle_r = cv2.minEnclosingCircle(contour)
            min_enclosing_circle = [
                float(circle_x) + float(x),
                float(circle_y) + float(y),
                float(circle_r),
            ]
        cx, cy = float(centroids[idx][0]), float(centroids[idx][1])
        radius_outer = max(w, h) * 0.5
        moment_geom = component_moment_geometry(
            labels,
            idx,
            (x, y, w, h),
            (cx, cy),
            float(radius_outer),
            contour,
        )
        components[idx] = {
            "label": idx,
            "bbox": [x, y, x + w - 1, y + h - 1],
            "center": [moment_geom["center_x"], moment_geom["center_y"]],
            "radius_outer": float(radius_outer),
            "axis_a": moment_geom["axis_a"] or float(radius_outer),
            "axis_b": moment_geom["axis_b"] or float(radius_outer),
            "angle_deg": moment_geom["angle_deg"],
            "circularity": float(circularity),
            "area": contour_area_val,
            "pixel_area": area_px,
            "min_enclosing_circle": min_enclosing_circle,
        }
    return components


def apply_opencv_variant(name: str) -> None:
    try:
        variant = OPENCV_VARIANTS[name]
    except KeyError as exc:
        choices = ", ".join(sorted(OPENCV_VARIANTS))
        raise SystemExit(f"unknown OpenCV variant '{name}', choose one of: {choices}") from exc
    OPENCV_PARAMS["variant"] = name
    OPENCV_PARAMS.update(variant)


def find_concentric_dot(
    outer_idx: int,
    outer: dict[str, Any],
    components: dict[int, dict[str, Any]],
) -> dict[str, Any] | None:
    ox, oy = outer["center"]
    outer_r = float(outer["radius_outer"])
    x0, y0, x1, y1 = outer["bbox"]
    best: dict[str, Any] | None = None
    best_dist = float("inf")
    for idx, comp in components.items():
        if idx == outer_idx:
            continue
        cx, cy = comp["center"]
        if cx < x0 or cx > x1 or cy < y0 or cy > y1:
            continue
        dot_r = float(comp["radius_outer"])
        dot_ratio = dot_r / outer_r if outer_r > 0.0 else 0.0
        if not (
            OPENCV_PARAMS["min_dot_outer_radius_ratio"]
            <= dot_ratio
            <= OPENCV_PARAMS["max_dot_outer_radius_ratio"]
        ):
            continue
        dist = math.hypot(float(cx) - float(ox), float(cy) - float(oy))
        max_offset = max(
            OPENCV_PARAMS["max_center_offset_px"],
            OPENCV_PARAMS["max_center_offset_radius_frac"] * outer_r,
        )
        if dist > max_offset:
            continue
        if dist < best_dist:
            best = comp
            best_dist = dist
    return best


def dedupe_detections(detections: list[dict[str, float]]) -> list[dict[str, float]]:
    kept: list[dict[str, float]] = []
    for det in sorted(detections, key=lambda d: d["radius_outer"], reverse=True):
        cx, cy = det["center"]
        duplicate = False
        for other in kept:
            ox, oy = other["center"]
            if math.hypot(cx - ox, cy - oy) < max(4.0, 0.25 * det["radius_outer"]):
                duplicate = True
                break
        if not duplicate:
            kept.append(det)
    kept.sort(key=lambda d: (d["center"][1], d["center"][0]))
    return kept


def visible_markers(
    row: dict[str, Any],
    include_partial_crop: bool = False,
) -> list[dict[str, Any]]:
    markers = []
    for marker in row["markers"]:
        reason = marker.get("visibility_reason")
        if reason == "visible" or marker.get("evaluation_visible"):
            markers.append(marker)
        elif include_partial_crop and reason == "partial_crop":
            markers.append(marker)
    return markers


def load_sentai_results(path: Path | None) -> dict[str, dict[str, Any]]:
    if path is None:
        return {}
    rows = load_jsonl(path)
    by_image: dict[str, dict[str, Any]] = {}
    for row in rows:
        frame = str(row.get("frame", ""))
        marker = "/frames/"
        if marker in frame:
            rel = "frames/" + frame.split(marker, 1)[1]
        else:
            rel = frame.lstrip("/")
        detections = []
        for det in row.get("detections", []):
            axis_a = float(det.get("axis_a") or 0.0)
            axis_b = float(det.get("axis_b") or 0.0)
            raw_radius = det.get("radius_outer")
            radius = float(raw_radius) if raw_radius is not None else None
            angle_rad = det.get("angle_rad")
            detections.append(
                {
                    "center": det["center"],
                    "radius_outer": radius,
                    "axis_a": axis_a,
                    "axis_b": axis_b,
                    "angle_rad": angle_rad,
                    "angle_deg": math.degrees(float(angle_rad)) if angle_rad is not None else None,
                    "comp_id": det.get("comp_id"),
                    "tvec_cam": det.get("tvec_cam"),
                    "rvec_cam": det.get("rvec_cam"),
                    "reproj_err_px": det.get("reproj_err_px"),
                    "backend": det.get("backend"),
                    "pose_valid": det.get("pose_valid"),
                    "geometry_valid": det.get("geometry_valid"),
                }
            )
        by_image[rel] = {
            "available": True,
            "source": str(path),
            "raw_rc": row.get("rc"),
            "detections": detections,
            "drone_pose_world": row.get("drone_pose_world"),
        }
    return by_image


def load_sentai_run_metadata(results_path: Path | None, summary_path: Path | None = None) -> dict[str, Any]:
    if results_path is None:
        return {"available": False, "reason": "sentai_sim result file was not supplied"}

    run_dir = results_path.resolve().parent
    summary_file = summary_path.resolve() if summary_path else run_dir / "sentai_sim_summary.json"
    log_file = run_dir / "sentai_sim_repl.log"
    metadata: dict[str, Any] = {
        "available": True,
        "results_path": str(results_path),
        "run_dir": str(run_dir),
    }

    if summary_file.exists():
        try:
            metadata["summary_path"] = str(summary_file)
            metadata["summary"] = json.loads(summary_file.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            metadata["summary_parse_error"] = str(exc)
    else:
        metadata["summary_missing"] = str(summary_file)

    if log_file.exists():
        metadata["repl_log_path"] = str(log_file)
        first_lines = log_file.read_text(encoding="utf-8", errors="replace").splitlines()[:20]
        build_line = next((line for line in first_lines if line.startswith("[sim] sentai_sim build")), None)
        metadata["sim_build_line"] = build_line
    return metadata


def git_commit_short() -> str | None:
    try:
        proc = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=REPO_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=False,
            timeout=5,
        )
    except Exception:
        return None
    if proc.returncode != 0:
        return None
    return proc.stdout.strip() or None


def match_detections(
    expected: list[dict[str, Any]],
    detections: list[dict[str, float]],
    ignored: list[dict[str, Any]] | None = None,
) -> tuple[list[dict[str, Any]], list[int], list[int], list[int]]:
    candidates = []
    for ei, marker in enumerate(expected):
        ex, ey = marker["pixel_center"]
        radius = float(marker.get("pixel_radius_outer") or 0.0)
        threshold = max(5.0, 0.25 * radius)
        for di, det in enumerate(detections):
            dx = det["center"][0] - ex
            dy = det["center"][1] - ey
            dist = math.hypot(dx, dy)
            if dist <= threshold:
                candidates.append((dist, ei, di, threshold))

    matches = []
    used_e: set[int] = set()
    used_d: set[int] = set()
    for dist, ei, di, threshold in sorted(candidates):
        if ei in used_e or di in used_d:
            continue
        marker = expected[ei]
        det = detections[di]
        matches.append(
            {
                "marker_id": marker["marker_id"],
                "expected_pixel": marker["pixel_center"],
                "detected_pixel": det["center"],
                "centroid_error_px": dist,
                "match_threshold_px": threshold,
                "expected_radius_outer": marker.get("pixel_radius_outer"),
                "detected_radius_outer": det.get("radius_outer"),
                "detected_axis_a": det["axis_a"],
                "detected_axis_b": det["axis_b"],
                "detected_angle_deg": det.get("angle_deg"),
                "axis_ratio": (
                    det["axis_a"] / det["axis_b"] if det["axis_b"] else None
                ),
            }
        )
        used_e.add(ei)
        used_d.add(di)

    ignored_d: set[int] = set()
    if ignored:
        ignored_candidates = []
        for ii, marker in enumerate(ignored):
            ex, ey = marker["pixel_center"]
            radius = float(marker.get("pixel_radius_outer") or 0.0)
            threshold = max(5.0, 0.25 * radius)
            for di, det in enumerate(detections):
                if di in used_d:
                    continue
                dx = det["center"][0] - ex
                dy = det["center"][1] - ey
                dist = math.hypot(dx, dy)
                if dist <= threshold:
                    ignored_candidates.append((dist, ii, di))
        used_i: set[int] = set()
        for dist, ii, di in sorted(ignored_candidates):
            if ii in used_i or di in used_d or di in ignored_d:
                continue
            ignored_d.add(di)
            used_i.add(ii)

    missed = [i for i in range(len(expected)) if i not in used_e]
    false_pos = [
        i for i in range(len(detections))
        if i not in used_d and i not in ignored_d
    ]
    return matches, missed, false_pos, sorted(ignored_d)


def yaw_bin_label(yaw_deg: float, width_deg: int = 45) -> str:
    yaw = float(yaw_deg) % 360.0
    lo = int(math.floor(yaw / width_deg) * width_deg)
    hi = lo + width_deg
    return f"{lo:03d}-{hi:03d}"


def roll_pitch_bin_label(roll_deg: float, pitch_deg: float) -> str:
    mag = math.hypot(float(roll_deg), float(pitch_deg))
    if mag < 5.0:
        return "00-05"
    if mag < 10.0:
        return "05-10"
    if mag < 15.0:
        return "10-15"
    return "15+"


def edge_visibility_category(row: dict[str, Any], config: dict[str, Any]) -> str:
    image_cfg = config.get("image", {})
    camera_cfg = config.get("camera", {})
    resolution = image_cfg.get("resolution") or camera_cfg.get("resolution") or [320, 240]
    width, height = float(resolution[0]), float(resolution[1])
    near_edge = False
    cropped = False
    for marker in row["markers"]:
        if marker.get("visibility_reason") not in ("visible", "partial_crop"):
            continue
        bbox = marker.get("projected_bbox")
        if not bbox:
            continue
        x0, y0, x1, y1 = [float(v) for v in bbox]
        if x0 < 0.0 or y0 < 0.0 or x1 > width or y1 > height:
            cropped = True
        if min(x0, y0, width - x1, height - y1) <= 5.0:
            near_edge = True
    if cropped:
        return "cropped"
    if near_edge:
        return "near_edge"
    return "interior"


def frame_status(missed: list[int], false_pos: list[int], matches: list[dict[str, Any]]) -> str:
    if missed:
        return "missed_marker"
    if false_pos:
        return "false_positive"
    return "pass"


def camera_matrix_from_config(config: dict[str, Any]) -> np.ndarray:
    cam = config["camera"]
    return np.array(
        [
            [float(cam["fx"]), 0.0, float(cam["cx"])],
            [0.0, float(cam["fy"]), float(cam["cy"])],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def rotation_matrix_to_euler_xyz_deg(rot: np.ndarray) -> tuple[float, float, float]:
    sy = math.sqrt(rot[0, 0] * rot[0, 0] + rot[1, 0] * rot[1, 0])
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def camera_world_to_manifest_body_rotation(r_c2w: np.ndarray) -> np.ndarray:
    # Must match generate_whycon_synthetic_dataset.py::camera_rotation_world:
    # r_wc = Rz(yaw) * Ry(pitch) * Rx(roll) * r0
    # where r0 columns are OpenCV camera axes in world coordinates at zero RPY.
    r0 = np.array(
        [
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ],
        dtype=np.float64,
    )
    return r_c2w @ r0.T


def pose_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> dict[str, Any]:
    r_w2c, _ = cv2.Rodrigues(rvec)
    r_c2w = r_w2c.T
    r_body = camera_world_to_manifest_body_rotation(r_c2w)
    cam_world = (-r_c2w @ tvec.reshape(3, 1)).reshape(3)
    roll_deg, pitch_deg, yaw_deg = rotation_matrix_to_euler_xyz_deg(r_body)
    return {
        "x": float(cam_world[0]),
        "y": float(cam_world[1]),
        "z": float(cam_world[2]),
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "yaw_deg": yaw_deg,
    }


def reprojection_rmse_px(
    obj: np.ndarray,
    img: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    camera_matrix: np.ndarray,
    dist: np.ndarray,
) -> float:
    projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist)
    projected = projected.reshape(-1, 2)
    return float(math.sqrt(np.mean(np.sum((projected - img) ** 2, axis=1))))


def min_camera_depth_m(obj: np.ndarray, rvec: np.ndarray, tvec: np.ndarray) -> float:
    r_w2c, _ = cv2.Rodrigues(rvec)
    points_cam = (r_w2c @ obj.T + tvec.reshape(3, 1)).T
    return float(np.min(points_cam[:, 2]))


def pose_delta_against(a: dict[str, Any], b: dict[str, Any]) -> dict[str, float | None]:
    x_delta = float(a["x"]) - float(b["x"])
    y_delta = float(a["y"]) - float(b["y"])
    z_delta = float(a["z"]) - float(b["z"])
    return {
        "translation_delta_m": math.sqrt(x_delta * x_delta + y_delta * y_delta + z_delta * z_delta),
        "x_delta_m": x_delta,
        "y_delta_m": y_delta,
        "z_delta_m": z_delta,
        "roll_delta_deg": angle_delta_deg(a.get("roll_deg"), b.get("roll_deg"), 360.0),
        "pitch_delta_deg": angle_delta_deg(a.get("pitch_deg"), b.get("pitch_deg"), 360.0),
        "yaw_delta_deg": angle_delta_deg(a.get("yaw_deg"), b.get("yaw_deg"), 360.0),
    }


def solve_pose_candidate(
    obj: np.ndarray,
    img: np.ndarray,
    camera_matrix: np.ndarray,
    dist: np.ndarray,
    flags: int,
) -> dict[str, Any] | None:
    ok, rvec, tvec = cv2.solvePnP(obj, img, camera_matrix, dist, flags=flags)
    if not ok:
        return None
    pose = pose_from_rvec_tvec(rvec, tvec)
    return {
        "pose": pose,
        "rvec": rvec,
        "tvec": tvec,
        "reprojection_rmse_px": reprojection_rmse_px(obj, img, rvec, tvec, camera_matrix, dist),
        "min_camera_depth_m": min_camera_depth_m(obj, rvec, tvec),
    }


def pose_candidate_record(
    kind: str,
    marker_assignment: list[str],
    candidate: dict[str, Any],
    primary_pose: dict[str, Any],
    primary_rmse: float,
) -> dict[str, Any]:
    delta = pose_delta_against(candidate["pose"], primary_pose)
    yaw_delta = delta.get("yaw_delta_deg")
    return {
        "kind": kind,
        "marker_assignment": marker_assignment,
        "reprojection_rmse_px": candidate["reprojection_rmse_px"],
        "reprojection_delta_px": candidate["reprojection_rmse_px"] - primary_rmse,
        "reprojection_ratio": (
            candidate["reprojection_rmse_px"] / primary_rmse
            if primary_rmse > 1e-9
            else None
        ),
        "min_camera_depth_m": candidate["min_camera_depth_m"],
        "translation_delta_m": delta["translation_delta_m"],
        "yaw_delta_deg": abs(float(yaw_delta)) if yaw_delta is not None else None,
        "pose_world_est": candidate["pose"],
    }


def evaluate_pose_ambiguity(
    marker_ids: list[str],
    obj: np.ndarray,
    img: np.ndarray,
    camera_matrix: np.ndarray,
    dist: np.ndarray,
    primary_rvec: np.ndarray,
    primary_tvec: np.ndarray,
    primary_pose: dict[str, Any],
) -> dict[str, Any]:
    primary_rmse = reprojection_rmse_px(obj, img, primary_rvec, primary_tvec, camera_matrix, dist)
    alternatives: list[dict[str, Any]] = []

    if len(marker_ids) >= 4:
        try:
            generic = cv2.solvePnPGeneric(
                obj,
                img,
                camera_matrix,
                dist,
                flags=cv2.SOLVEPNP_IPPE,
            )
        except cv2.error:
            generic = None
        if generic and generic[0]:
            for rvec, tvec in zip(generic[1], generic[2]):
                candidate = {
                    "pose": pose_from_rvec_tvec(rvec, tvec),
                    "rvec": rvec,
                    "tvec": tvec,
                    "reprojection_rmse_px": reprojection_rmse_px(obj, img, rvec, tvec, camera_matrix, dist),
                    "min_camera_depth_m": min_camera_depth_m(obj, rvec, tvec),
                }
                record = pose_candidate_record(
                    "ippe_solution",
                    marker_ids,
                    candidate,
                    primary_pose,
                    primary_rmse,
                )
                if record["translation_delta_m"] > 1e-6 or (record["yaw_delta_deg"] or 0.0) > 1e-6:
                    alternatives.append(record)

    max_perm_markers = int(OPENCV_PARAMS["ambiguity_max_permutation_markers"])
    max_perm = math.factorial(len(marker_ids))
    permutation_search_reason = None
    if len(marker_ids) <= max_perm_markers:
        identity = tuple(range(len(marker_ids)))
        for perm in itertools.permutations(range(len(marker_ids))):
            if perm == identity:
                continue
            perm_obj = obj[list(perm)]
            candidate = solve_pose_candidate(
                perm_obj,
                img,
                camera_matrix,
                dist,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if candidate is None:
                continue
            assignment = [marker_ids[i] for i in perm]
            alternatives.append(
                pose_candidate_record(
                    "correspondence_permutation",
                    assignment,
                    candidate,
                    primary_pose,
                    primary_rmse,
                )
            )
    else:
        permutation_search_reason = (
            f"skipped: {len(marker_ids)} markers exceeds "
            f"ambiguity_max_permutation_markers={max_perm_markers}"
        )

    alternatives = [
        alt for alt in alternatives
        if alt["min_camera_depth_m"] > 0.0 and alt["pose_world_est"]["z"] > 0.0
    ]
    alternatives.sort(key=lambda alt: (alt["reprojection_rmse_px"], alt["translation_delta_m"]))
    near = []
    for alt in alternatives:
        ratio = alt.get("reprojection_ratio")
        reproj_close = (
            alt["reprojection_delta_px"] <= float(OPENCV_PARAMS["ambiguity_reprojection_delta_px"])
            or (ratio is not None and ratio <= float(OPENCV_PARAMS["ambiguity_reprojection_ratio"]))
        )
        pose_different = (
            alt["translation_delta_m"] >= float(OPENCV_PARAMS["ambiguity_translation_delta_m"])
            or (alt.get("yaw_delta_deg") is not None and alt["yaw_delta_deg"] >= float(OPENCV_PARAMS["ambiguity_yaw_delta_deg"]))
        )
        if reproj_close and pose_different:
            near.append(alt)

    keep = int(OPENCV_PARAMS["ambiguity_max_alternatives_recorded"])
    return {
        "available": True,
        "primary_reprojection_rmse_px": primary_rmse,
        "candidate_count": len(alternatives),
        "permutation_candidate_count_possible": max_perm - 1,
        "permutation_search_reason": permutation_search_reason,
        "near_ambiguous_count": len(near),
        "has_near_ambiguous_solution": bool(near),
        "thresholds": {
            "reprojection_ratio": OPENCV_PARAMS["ambiguity_reprojection_ratio"],
            "reprojection_delta_px": OPENCV_PARAMS["ambiguity_reprojection_delta_px"],
            "translation_delta_m": OPENCV_PARAMS["ambiguity_translation_delta_m"],
            "yaw_delta_deg": OPENCV_PARAMS["ambiguity_yaw_delta_deg"],
        },
        "best_alternatives": alternatives[:keep],
        "near_ambiguous_alternatives": near[:keep],
    }


def estimate_camera_pose(
    row: dict[str, Any],
    matches: list[dict[str, Any]],
    config: dict[str, Any],
    evaluate_ambiguity: bool = True,
) -> dict[str, Any]:
    if len(matches) < 4:
        return {
            "available": False,
            "reason": "need at least 4 matched coplanar markers",
            "matched_for_pose": len(matches),
        }

    markers_by_id = {m["marker_id"]: m for m in row["markers"]}
    marker_ids = []
    object_points = []
    image_points = []
    for match in matches:
        marker = markers_by_id.get(match["marker_id"])
        if marker is None:
            continue
        marker_ids.append(match["marker_id"])
        object_points.append(marker["world_xyz"])
        image_points.append(match["detected_pixel"])
    if len(object_points) < 4:
        return {
            "available": False,
            "reason": "matched markers missing world coordinates",
            "matched_for_pose": len(object_points),
        }

    obj = np.asarray(object_points, dtype=np.float64)
    img = np.asarray(image_points, dtype=np.float64)
    camera_matrix = camera_matrix_from_config(config)
    dist = np.zeros((4, 1), dtype=np.float64)

    ok, rvec, tvec = cv2.solvePnP(
        obj,
        img,
        camera_matrix,
        dist,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return {
            "available": False,
            "reason": "cv2.solvePnP failed",
            "matched_for_pose": len(object_points),
        }

    reproj_rmse = reprojection_rmse_px(obj, img, rvec, tvec, camera_matrix, dist)
    max_reproj = float(OPENCV_PARAMS["pose_max_reprojection_rmse_px"])
    if reproj_rmse > max_reproj:
        return {
            "available": False,
            "reason": "pose reprojection RMSE above quality gate",
            "matched_for_pose": len(object_points),
            "reprojection_rmse_px": reproj_rmse,
            "max_reprojection_rmse_px": max_reproj,
        }

    est = pose_from_rvec_tvec(rvec, tvec)
    gt = row["pose_world_camera"]
    errors = {
        "x_error_m": est["x"] - float(gt["x"]),
        "y_error_m": est["y"] - float(gt["y"]),
        "z_error_m": est["z"] - float(gt["z"]),
        "roll_error_deg": angle_delta_deg(est["roll_deg"], float(gt["roll_deg"]), 360.0),
        "pitch_error_deg": angle_delta_deg(est["pitch_deg"], float(gt["pitch_deg"]), 360.0),
        "yaw_error_deg": angle_delta_deg(est["yaw_deg"], float(gt["yaw_deg"]), 360.0),
    }
    errors["translation_error_m"] = math.sqrt(
        errors["x_error_m"] ** 2 + errors["y_error_m"] ** 2 + errors["z_error_m"] ** 2
    )
    if evaluate_ambiguity:
        ambiguity_eval = evaluate_pose_ambiguity(
            marker_ids,
            obj,
            img,
            camera_matrix,
            dist,
            rvec,
            tvec,
            est,
        )
    else:
        ambiguity_eval = {
            "available": False,
            "reason": "disabled by validator --skip-ambiguity",
        }
    return {
        "available": True,
        "method": "cv2.solvePnP_ITERATIVE_world_markers",
        "matched_for_pose": len(object_points),
        "reprojection_rmse_px": reproj_rmse,
        "max_reprojection_rmse_px": max_reproj,
        "camera_pose_world_est": est,
        "camera_pose_world_gt": gt,
        "ambiguity_eval": ambiguity_eval,
        **errors,
    }


def compute_backend_delta(opencv: dict[str, Any], sentai: dict[str, Any]) -> dict[str, Any]:
    if not sentai.get("available"):
        return {"available": False, "reason": "sentai_sim unavailable"}

    delta: dict[str, Any] = {
        "available": True,
        "count_delta": opencv["detected_count"] - sentai["detected_count"],
        "matched_delta": opencv["matched_count"] - sentai["matched_count"],
        "false_positive_delta": opencv["false_positive_count"] - sentai["false_positive_count"],
        "missed_delta": opencv["missed_count"] - sentai["missed_count"],
        "marker_deltas": [],
    }

    opencv_by_id = {m["marker_id"]: m for m in opencv.get("matches", [])}
    sentai_by_id = {m["marker_id"]: m for m in sentai.get("matches", [])}
    for marker_id in sorted(set(opencv_by_id) & set(sentai_by_id)):
        o = opencv_by_id[marker_id]
        s = sentai_by_id[marker_id]
        ox, oy = o["detected_pixel"]
        sx, sy = s["detected_pixel"]
        o_axis_ratio = float(o.get("axis_ratio") or 1.0)
        s_axis_ratio = float(s.get("axis_ratio") or 1.0)
        angle_stable = (
            o_axis_ratio >= float(OPENCV_PARAMS["angle_stability_min_axis_ratio"])
            and s_axis_ratio >= float(OPENCV_PARAMS["angle_stability_min_axis_ratio"])
        )
        marker_delta = {
            "marker_id": marker_id,
            "centroid_delta_px": math.hypot(float(ox) - float(sx), float(oy) - float(sy)),
            "x_delta_px": float(ox) - float(sx),
            "y_delta_px": float(oy) - float(sy),
            "radius_delta_px": (
                float(o["detected_radius_outer"]) - float(s["detected_radius_outer"])
                if o.get("detected_radius_outer") is not None and s.get("detected_radius_outer") is not None
                else None
            ),
            "axis_a_delta_px": float(o["detected_axis_a"]) - float(s["detected_axis_a"]),
            "axis_b_delta_px": float(o["detected_axis_b"]) - float(s["detected_axis_b"]),
            "angle_delta_deg": angle_delta_deg(o.get("detected_angle_deg"), s.get("detected_angle_deg"), 180.0),
            "angle_stable": angle_stable,
            "opencv_axis_ratio": o_axis_ratio,
            "sentai_axis_ratio": s_axis_ratio,
        }
        delta["marker_deltas"].append(marker_delta)

    op_pose = opencv.get("pose_eval", {})
    se_pose = sentai.get("pose_eval", {})
    if op_pose.get("available") and se_pose.get("available"):
        op_est = op_pose["camera_pose_world_est"]
        se_est = se_pose["camera_pose_world_est"]
        pose_delta = {
            "available": True,
            "x_delta_m": float(op_est["x"]) - float(se_est["x"]),
            "y_delta_m": float(op_est["y"]) - float(se_est["y"]),
            "z_delta_m": float(op_est["z"]) - float(se_est["z"]),
            "roll_delta_deg": angle_delta_deg(op_est["roll_deg"], se_est["roll_deg"], 360.0),
            "pitch_delta_deg": angle_delta_deg(op_est["pitch_deg"], se_est["pitch_deg"], 360.0),
            "yaw_delta_deg": angle_delta_deg(op_est["yaw_deg"], se_est["yaw_deg"], 360.0),
        }
        pose_delta["translation_delta_m"] = math.sqrt(
            pose_delta["x_delta_m"] ** 2 + pose_delta["y_delta_m"] ** 2 + pose_delta["z_delta_m"] ** 2
        )
    else:
        pose_delta = {
            "available": False,
            "reason": "pose unavailable for one or both backends",
        }
    delta["pose_delta"] = pose_delta
    return delta


def draw_overlay(
    gray: np.ndarray,
    row: dict[str, Any],
    detections: list[dict[str, float]],
    matches: list[dict[str, Any]],
    missed_indices: list[int],
    false_pos_indices: list[int],
    out_path: Path,
    include_partial_crop: bool = False,
) -> None:
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    expected = visible_markers(row, include_partial_crop=include_partial_crop)

    for idx, marker in enumerate(expected):
        x, y = marker["pixel_center"]
        color = (0, 0, 255) if idx in missed_indices else (0, 180, 0)
        cv2.drawMarker(img, (round(x), round(y)), color, cv2.MARKER_CROSS, 10, 1)
        cv2.putText(
            img,
            marker["marker_id"],
            (round(x) + 4, round(y) - 4),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            color,
            1,
            cv2.LINE_AA,
        )

    for idx, det in enumerate(detections):
        x, y = det["center"]
        r = det["radius_outer"]
        color = (0, 165, 255) if idx in false_pos_indices else (255, 0, 0)
        cv2.circle(img, (round(x), round(y)), round(r), color, 1)
        cv2.circle(img, (round(x), round(y)), 2, color, -1)

    for match in matches:
        ex, ey = match["expected_pixel"]
        dx, dy = match["detected_pixel"]
        cv2.line(img, (round(ex), round(ey)), (round(dx), round(dy)), (255, 255, 0), 1)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)


def draw_detection_overlay(
    gray: np.ndarray,
    row: dict[str, Any],
    detections: list[dict[str, float]],
    out_path: Path,
) -> None:
    img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    red = (0, 0, 255)
    for det in detections:
        cx, cy = det["center"]
        radius = det["radius_outer"]
        cv2.circle(img, (round(cx), round(cy)), round(radius), red, 1)
        cv2.drawMarker(img, (round(cx), round(cy)), red, cv2.MARKER_CROSS, 10, 1)

        axis_a = float(det.get("axis_a") or radius)
        axis_b = float(det.get("axis_b") or radius)
        angle = det.get("angle_deg")
        if angle is not None:
            draw_axis_a = min(axis_a, radius * 1.25)
            draw_axis_b = min(axis_b, radius * 1.25)
            cv2.ellipse(
                img,
                (round(cx), round(cy)),
                (max(1, round(draw_axis_a)), max(1, round(draw_axis_b))),
                float(angle),
                0,
                360,
                red,
                1,
            )
            theta = math.radians(float(angle))
            x2 = cx + math.cos(theta) * draw_axis_a
            y2 = cy + math.sin(theta) * draw_axis_a
            cv2.line(img, (round(cx), round(cy)), (round(x2), round(y2)), red, 1)

    cv2.putText(
        img,
        f"frame {row['frame_id']} expected {row.get('markers_expected')} detected {len(detections)}",
        (5, 14),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        red,
        1,
        cv2.LINE_AA,
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), img)


def write_detection_contact_sheet(overlays_dir: Path, out_path: Path, max_tiles: int = 120) -> None:
    overlay_paths = sorted(overlays_dir.glob("frame_*.png"))[:max_tiles]
    if not overlay_paths:
        return
    thumbs = []
    for path in overlay_paths:
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            continue
        img = cv2.resize(img, (160, 120), interpolation=cv2.INTER_AREA)
        thumbs.append((path, img))
    if not thumbs:
        return

    cols = 5
    gutter = 2
    tile_w = 160
    tile_h = 120
    label_h = 25
    rows = math.ceil(len(thumbs) / cols)
    sheet = np.full(
        (rows * (tile_h + label_h) + (rows + 1) * gutter,
         cols * tile_w + (cols + 1) * gutter,
         3),
        255,
        dtype=np.uint8,
    )
    for i, (path, thumb) in enumerate(thumbs):
        col = i % cols
        row = i // cols
        x = gutter + col * (tile_w + gutter)
        y = gutter + row * (tile_h + label_h + gutter)
        sheet[y:y + tile_h, x:x + tile_w] = thumb
        cv2.putText(
            sheet,
            f"{i:02d}",
            (x + 4, y + tile_h + 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 0, 0),
            1,
            cv2.LINE_AA,
        )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), sheet)


def summarize(
    results: list[dict[str, Any]],
    source_dataset: Path,
    config: dict[str, Any],
    sentai_run_metadata: dict[str, Any],
    include_partial_crop: bool = False,
) -> dict[str, Any]:
    def summarize_backend(name: str) -> dict[str, Any]:
        backend_rows = [r["backends"][name] for r in results if r["backends"][name].get("available", True)]
        expected_total = sum(r["expected_count"] for r in results if r["backends"][name].get("available", True))
        matched_total = sum(b["matched_count"] for b in backend_rows)
        false_total = sum(b["false_positive_count"] for b in backend_rows)
        ignored_total = sum(b.get("ignored_detection_count", 0) for b in backend_rows)
        errors = [
            m["centroid_error_px"]
            for b in backend_rows
            for m in b["matches"]
        ]

        by_z: dict[str, dict[str, int]] = {}
        by_expected_count: dict[str, dict[str, int]] = {}
        by_yaw_bin: dict[str, dict[str, int]] = {}
        by_roll_pitch_bin: dict[str, dict[str, int]] = {}
        by_edge_category: dict[str, dict[str, int]] = {}
        for row in results:
            backend = row["backends"][name]
            if not backend.get("available", True):
                continue
            pose = row["pose_world_camera"]
            z_key = f"{pose['z']:.2f}"
            n_key = str(row["expected_count"])
            yaw_key = yaw_bin_label(float(pose["yaw_deg"]))
            rp_key = roll_pitch_bin_label(float(pose["roll_deg"]), float(pose["pitch_deg"]))
            edge_key = row.get("edge_visibility_category", "unknown")
            for group, key in (
                (by_z, z_key),
                (by_expected_count, n_key),
                (by_yaw_bin, yaw_key),
                (by_roll_pitch_bin, rp_key),
                (by_edge_category, edge_key),
            ):
                group.setdefault(key, {"frames": 0, "expected": 0, "matched": 0})
                group[key]["frames"] += 1
                group[key]["expected"] += row["expected_count"]
                group[key]["matched"] += backend["matched_count"]

        return {
            "available": bool(backend_rows),
            "frame_count": len(backend_rows),
            "expected_markers": expected_total,
            "matched_markers": matched_total,
            "marker_recall": matched_total / expected_total if expected_total else None,
            "false_positives_total": false_total,
            "ignored_detections_total": ignored_total,
            "false_positives_per_frame": false_total / len(backend_rows) if backend_rows else None,
            "complete_frame_success_rate": (
                sum(1 for r in results if r["backends"][name].get("status") == "pass") / len(backend_rows)
                if backend_rows
                else None
            ),
            "centroid_error_px": {
                "mean": statistics.fmean(errors) if errors else None,
                "median": statistics.median(errors) if errors else None,
                "p95": percentile(errors, 95),
                "max": max(errors) if errors else None,
            },
            "by_z": by_z,
            "by_expected_count": by_expected_count,
            "by_yaw_bin": by_yaw_bin,
            "by_roll_pitch_bin": by_roll_pitch_bin,
            "by_edge_category": by_edge_category,
            "geometry": geometry_summary(name),
        }

    def geometry_summary(name: str) -> dict[str, Any]:
        rows = [
            r["backends"][name]
            for r in results
            if r["backends"][name].get("available", True)
        ]
        matches = [m for b in rows for m in b.get("matches", [])]
        radius_errors = [
            float(m["detected_radius_outer"]) - float(m["expected_radius_outer"])
            for m in matches
            if m.get("detected_radius_outer") is not None and m.get("expected_radius_outer") is not None
        ]
        axis_a_errors = [
            float(m["detected_axis_a"]) - float(m["expected_radius_outer"])
            for m in matches
            if m.get("detected_axis_a") is not None and m.get("expected_radius_outer") is not None
        ]
        axis_b_errors = [
            float(m["detected_axis_b"]) - float(m["expected_radius_outer"])
            for m in matches
            if m.get("detected_axis_b") is not None and m.get("expected_radius_outer") is not None
        ]
        axis_ratios = [
            float(m["axis_ratio"])
            for m in matches
            if m.get("axis_ratio") is not None and math.isfinite(float(m["axis_ratio"]))
        ]
        angles = [
            float(m["detected_angle_deg"])
            for m in matches
            if m.get("detected_angle_deg") is not None and math.isfinite(float(m["detected_angle_deg"]))
        ]
        return {
            "matched_markers_with_geometry": len(matches),
            "radius_error_px": numeric_stats(radius_errors),
            "axis_a_error_px": numeric_stats(axis_a_errors),
            "axis_b_error_px": numeric_stats(axis_b_errors),
            "axis_ratio": numeric_stats(axis_ratios),
            "orientation_angle_deg": numeric_stats(angles),
            "orientation_error_deg": {
                **numeric_stats([]),
                "available": False,
                "reason": "manifest has no expected per-marker ellipse orientation",
            },
        }

    def pose_summary(name: str) -> dict[str, Any]:
        rows = [
            r["backends"][name]["pose_eval"]
            for r in results
            if r["backends"][name].get("available", True)
            and r["backends"][name].get("pose_eval", {}).get("available")
        ]
        ambiguity_rows = [
            r.get("ambiguity_eval", {})
            for r in rows
            if r.get("ambiguity_eval", {}).get("available")
        ]
        return {
            "available": bool(rows),
            "pose_valid_frames": len(rows),
            "translation_error_m": numeric_stats([r["translation_error_m"] for r in rows]),
            "x_error_m": numeric_stats([r["x_error_m"] for r in rows]),
            "y_error_m": numeric_stats([r["y_error_m"] for r in rows]),
            "z_error_m": numeric_stats([r["z_error_m"] for r in rows]),
            "roll_error_deg": numeric_stats([r["roll_error_deg"] for r in rows if r.get("roll_error_deg") is not None]),
            "pitch_error_deg": numeric_stats([r["pitch_error_deg"] for r in rows if r.get("pitch_error_deg") is not None]),
            "yaw_error_deg": numeric_stats([r["yaw_error_deg"] for r in rows if r.get("yaw_error_deg") is not None]),
            "ambiguity": {
                "available": bool(ambiguity_rows),
                "evaluated_frames": len(ambiguity_rows),
                "frames_with_near_ambiguous_solution": sum(
                    1 for r in ambiguity_rows if r.get("has_near_ambiguous_solution")
                ),
                "near_ambiguous_solution_count": sum(
                    int(r.get("near_ambiguous_count") or 0) for r in ambiguity_rows
                ),
                "candidate_count": numeric_stats([
                    int(r.get("candidate_count") or 0) for r in ambiguity_rows
                ]),
                "permutation_candidate_count_possible": numeric_stats([
                    int(r.get("permutation_candidate_count_possible") or 0)
                    for r in ambiguity_rows
                ]),
                "primary_reprojection_rmse_px": numeric_stats([
                    r["primary_reprojection_rmse_px"]
                    for r in ambiguity_rows
                    if r.get("primary_reprojection_rmse_px") is not None
                ]),
                "thresholds": ambiguity_rows[0].get("thresholds") if ambiguity_rows else None,
            },
        }

    opencv_summary = summarize_backend("opencv")
    opencv_summary.update({"version": cv2.__version__, "params": OPENCV_PARAMS})
    opencv_summary["pose"] = pose_summary("opencv")
    sentai_summary = summarize_backend("sentai_sim")
    sentai_summary["pose"] = pose_summary("sentai_sim")
    sentai_summary["run_metadata"] = sentai_run_metadata
    if not sentai_summary["available"]:
        sentai_summary["reason"] = "sentai_sim result file was not supplied"

    disagreements = []
    for row in results:
        sentai = row["backends"]["sentai_sim"]
        if not sentai.get("available"):
            continue
        opencv = row["backends"]["opencv"]
        if opencv["matched_count"] != sentai["matched_count"] or opencv["detected_count"] != sentai["detected_count"]:
            disagreements.append(
                {
                    "frame_id": row["frame_id"],
                    "image_path": row["image_path"],
                    "expected_count": row["expected_count"],
                    "opencv_detected": opencv["detected_count"],
                    "opencv_matched": opencv["matched_count"],
                    "sentai_detected": sentai["detected_count"],
                    "sentai_matched": sentai["matched_count"],
                }
            )

    marker_deltas = [
        md
        for row in results
        if row.get("backend_delta", {}).get("available")
        for md in row["backend_delta"].get("marker_deltas", [])
    ]
    pose_deltas = [
        row["backend_delta"]["pose_delta"]
        for row in results
        if row.get("backend_delta", {}).get("pose_delta", {}).get("available")
    ]

    return {
        "source_dataset": str(source_dataset),
        "dataset_id": source_dataset.name,
        "created_utc": dt.datetime.now(dt.UTC).isoformat(timespec="seconds"),
        "validator": "validate_whycon_synthetic_dataset.py",
        "git_commit_short": git_commit_short(),
        "opencv": opencv_summary,
        "sentai_sim": sentai_summary,
        "backend_disagreement": {
            "frames_with_count_or_match_disagreement": len(disagreements),
            "examples": disagreements[:50],
        },
        "backend_delta_stats": {
            "count_delta": numeric_stats([
                r["backend_delta"]["count_delta"]
                for r in results
                if r.get("backend_delta", {}).get("available")
            ]),
            "matched_delta": numeric_stats([
                r["backend_delta"]["matched_delta"]
                for r in results
                if r.get("backend_delta", {}).get("available")
            ]),
            "marker_delta_count": len(marker_deltas),
            "centroid_delta_px": numeric_stats([m["centroid_delta_px"] for m in marker_deltas]),
            "radius_delta_px": numeric_stats([
                m["radius_delta_px"] for m in marker_deltas if m.get("radius_delta_px") is not None
            ]),
            "axis_a_delta_px": numeric_stats([m["axis_a_delta_px"] for m in marker_deltas]),
            "axis_b_delta_px": numeric_stats([m["axis_b_delta_px"] for m in marker_deltas]),
            "angle_delta_deg": numeric_stats([
                m["angle_delta_deg"] for m in marker_deltas if m.get("angle_delta_deg") is not None
            ]),
            "angle_delta_stable_count": len([
                m for m in marker_deltas
                if m.get("angle_stable") and m.get("angle_delta_deg") is not None
            ]),
            "angle_delta_stable_deg": numeric_stats([
                m["angle_delta_deg"]
                for m in marker_deltas
                if m.get("angle_stable") and m.get("angle_delta_deg") is not None
            ]),
            "pose_delta_count": len(pose_deltas),
            "translation_delta_m": numeric_stats([p["translation_delta_m"] for p in pose_deltas]),
            "x_delta_m": numeric_stats([p["x_delta_m"] for p in pose_deltas]),
            "y_delta_m": numeric_stats([p["y_delta_m"] for p in pose_deltas]),
            "z_delta_m": numeric_stats([p["z_delta_m"] for p in pose_deltas]),
            "roll_delta_deg": numeric_stats([p["roll_delta_deg"] for p in pose_deltas if p.get("roll_delta_deg") is not None]),
            "pitch_delta_deg": numeric_stats([p["pitch_delta_deg"] for p in pose_deltas if p.get("pitch_delta_deg") is not None]),
            "yaw_delta_deg": numeric_stats([p["yaw_delta_deg"] for p in pose_deltas if p.get("yaw_delta_deg") is not None]),
        },
        "dataset_config": {
            "scene_id": config.get("scene_id"),
            "image": config.get("image"),
            "camera": config.get("camera"),
            "visibility_policy": config.get("visibility_policy"),
            "include_partial_crop": include_partial_crop,
        },
    }


def fmt_cell(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def write_table_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k) for k in fieldnames})


def write_table_md(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("| " + " | ".join(fieldnames) + " |\n")
        f.write("| " + " | ".join("---" for _ in fieldnames) + " |\n")
        for row in rows:
            f.write("| " + " | ".join(fmt_cell(row.get(k)) for k in fieldnames) + " |\n")


def write_report_tables(out_dir: Path, summary: dict[str, Any], results: list[dict[str, Any]]) -> None:
    tables_dir = out_dir / "report_tables"
    tables_dir.mkdir(parents=True, exist_ok=True)

    detection_fields = [
        "backend",
        "frames",
        "expected_markers",
        "matched_markers",
        "recall",
        "false_positives_per_frame",
        "complete_frame_success",
        "centroid_p95_px",
    ]
    detection_rows = []
    for backend_name in ("opencv", "sentai_sim"):
        backend = summary[backend_name]
        detection_rows.append(
            {
                "backend": backend_name,
                "frames": backend.get("frame_count"),
                "expected_markers": backend.get("expected_markers"),
                "matched_markers": backend.get("matched_markers"),
                "recall": backend.get("marker_recall"),
                "false_positives_per_frame": backend.get("false_positives_per_frame"),
                "complete_frame_success": backend.get("complete_frame_success_rate"),
                "centroid_p95_px": backend.get("centroid_error_px", {}).get("p95"),
            }
        )
    write_table_csv(tables_dir / "backend_detection_summary.csv", detection_rows, detection_fields)
    write_table_md(tables_dir / "backend_detection_summary.md", detection_rows, detection_fields)

    geometry_fields = [
        "backend",
        "matched_markers_with_geometry",
        "radius_error_mean_px",
        "radius_error_median_px",
        "radius_error_p95_px",
        "axis_a_error_mean_px",
        "axis_a_error_median_px",
        "axis_a_error_p95_px",
        "axis_b_error_mean_px",
        "axis_b_error_median_px",
        "axis_b_error_p95_px",
        "axis_ratio_mean",
        "axis_ratio_median",
        "axis_ratio_p95",
        "orientation_angle_mean_deg",
        "orientation_angle_median_deg",
        "orientation_angle_p95_deg",
        "orientation_error_available",
    ]
    geometry_rows = []
    for backend_name in ("opencv", "sentai_sim"):
        geom = summary[backend_name].get("geometry", {})
        geometry_rows.append(
            {
                "backend": backend_name,
                "matched_markers_with_geometry": geom.get("matched_markers_with_geometry"),
                "radius_error_mean_px": geom.get("radius_error_px", {}).get("mean"),
                "radius_error_median_px": geom.get("radius_error_px", {}).get("median"),
                "radius_error_p95_px": geom.get("radius_error_px", {}).get("p95"),
                "axis_a_error_mean_px": geom.get("axis_a_error_px", {}).get("mean"),
                "axis_a_error_median_px": geom.get("axis_a_error_px", {}).get("median"),
                "axis_a_error_p95_px": geom.get("axis_a_error_px", {}).get("p95"),
                "axis_b_error_mean_px": geom.get("axis_b_error_px", {}).get("mean"),
                "axis_b_error_median_px": geom.get("axis_b_error_px", {}).get("median"),
                "axis_b_error_p95_px": geom.get("axis_b_error_px", {}).get("p95"),
                "axis_ratio_mean": geom.get("axis_ratio", {}).get("mean"),
                "axis_ratio_median": geom.get("axis_ratio", {}).get("median"),
                "axis_ratio_p95": geom.get("axis_ratio", {}).get("p95"),
                "orientation_angle_mean_deg": geom.get("orientation_angle_deg", {}).get("mean"),
                "orientation_angle_median_deg": geom.get("orientation_angle_deg", {}).get("median"),
                "orientation_angle_p95_deg": geom.get("orientation_angle_deg", {}).get("p95"),
                "orientation_error_available": geom.get("orientation_error_deg", {}).get("available"),
            }
        )
    write_table_csv(tables_dir / "backend_geometry_summary.csv", geometry_rows, geometry_fields)
    write_table_md(tables_dir / "backend_geometry_summary.md", geometry_rows, geometry_fields)

    pose_fields = [
        "backend",
        "pose_valid_frames",
        "ambiguity_evaluated_frames",
        "frames_with_near_ambiguous_solution",
        "translation_rmse_m",
        "translation_p95_m",
        "x_mae_m",
        "y_mae_m",
        "z_mae_m",
        "roll_mae_deg",
        "pitch_mae_deg",
        "yaw_mae_deg",
        "yaw_p95_deg",
    ]
    pose_rows = []
    for backend_name in ("opencv", "sentai_sim"):
        pose = summary[backend_name].get("pose", {})
        pose_rows.append(
            {
                "backend": backend_name,
                "pose_valid_frames": pose.get("pose_valid_frames", 0),
                "ambiguity_evaluated_frames": pose.get("ambiguity", {}).get("evaluated_frames"),
                "frames_with_near_ambiguous_solution": pose.get("ambiguity", {}).get("frames_with_near_ambiguous_solution"),
                "translation_rmse_m": pose.get("translation_error_m", {}).get("rmse"),
                "translation_p95_m": pose.get("translation_error_m", {}).get("p95"),
                "x_mae_m": pose.get("x_error_m", {}).get("mae"),
                "y_mae_m": pose.get("y_error_m", {}).get("mae"),
                "z_mae_m": pose.get("z_error_m", {}).get("mae"),
                "roll_mae_deg": pose.get("roll_error_deg", {}).get("mae"),
                "pitch_mae_deg": pose.get("pitch_error_deg", {}).get("mae"),
                "yaw_mae_deg": pose.get("yaw_error_deg", {}).get("mae"),
                "yaw_p95_deg": pose.get("yaw_error_deg", {}).get("p95"),
            }
        )
    write_table_csv(tables_dir / "backend_pose_vs_ground_truth.csv", pose_rows, pose_fields)
    write_table_md(tables_dir / "backend_pose_vs_ground_truth.md", pose_rows, pose_fields)

    ambiguity_fields = [
        "frame_id",
        "image_path",
        "backend",
        "expected_count",
        "primary_reprojection_rmse_px",
        "candidate_count",
        "near_ambiguous_count",
        "best_kind",
        "best_reprojection_rmse_px",
        "best_reprojection_delta_px",
        "best_translation_delta_m",
        "best_yaw_delta_deg",
    ]
    ambiguity_rows = []
    for row in results:
        for backend_name in ("opencv", "sentai_sim"):
            pose = row["backends"][backend_name].get("pose_eval", {})
            ambiguity = pose.get("ambiguity_eval", {})
            if not ambiguity.get("has_near_ambiguous_solution"):
                continue
            best = (ambiguity.get("near_ambiguous_alternatives") or [{}])[0]
            ambiguity_rows.append(
                {
                    "frame_id": row["frame_id"],
                    "image_path": row["image_path"],
                    "backend": backend_name,
                    "expected_count": row["expected_count"],
                    "primary_reprojection_rmse_px": ambiguity.get("primary_reprojection_rmse_px"),
                    "candidate_count": ambiguity.get("candidate_count"),
                    "near_ambiguous_count": ambiguity.get("near_ambiguous_count"),
                    "best_kind": best.get("kind"),
                    "best_reprojection_rmse_px": best.get("reprojection_rmse_px"),
                    "best_reprojection_delta_px": best.get("reprojection_delta_px"),
                    "best_translation_delta_m": best.get("translation_delta_m"),
                    "best_yaw_delta_deg": best.get("yaw_delta_deg"),
                }
            )
    write_table_csv(tables_dir / "pose_ambiguity_candidates.csv", ambiguity_rows, ambiguity_fields)
    write_table_md(tables_dir / "pose_ambiguity_candidates.md", ambiguity_rows, ambiguity_fields)

    delta = summary.get("backend_delta_stats", {})
    ablation_fields = [
        "comparable_frames",
        "count_disagreement_frames",
        "marker_delta_count",
        "centroid_delta_p95_px",
        "axis_a_delta_p95_px",
        "axis_b_delta_p95_px",
        "angle_delta_p95_deg",
        "angle_stable_count",
        "angle_stable_p95_deg",
        "pose_delta_count",
        "translation_delta_mean_m",
        "translation_delta_p95_m",
        "translation_delta_max_m",
        "yaw_delta_mean_deg",
        "yaw_delta_p95_deg",
        "yaw_delta_max_deg",
    ]
    ablation_rows = [
        {
            "comparable_frames": summary.get("sentai_sim", {}).get("frame_count"),
            "count_disagreement_frames": summary.get("backend_disagreement", {}).get("frames_with_count_or_match_disagreement"),
            "marker_delta_count": delta.get("marker_delta_count"),
            "centroid_delta_p95_px": delta.get("centroid_delta_px", {}).get("p95"),
            "axis_a_delta_p95_px": delta.get("axis_a_delta_px", {}).get("p95"),
            "axis_b_delta_p95_px": delta.get("axis_b_delta_px", {}).get("p95"),
            "angle_delta_p95_deg": delta.get("angle_delta_deg", {}).get("p95"),
            "angle_stable_count": delta.get("angle_delta_stable_count"),
            "angle_stable_p95_deg": delta.get("angle_delta_stable_deg", {}).get("p95"),
            "pose_delta_count": delta.get("pose_delta_count"),
            "translation_delta_mean_m": delta.get("translation_delta_m", {}).get("mean"),
            "translation_delta_p95_m": delta.get("translation_delta_m", {}).get("p95"),
            "translation_delta_max_m": delta.get("translation_delta_m", {}).get("max"),
            "yaw_delta_mean_deg": delta.get("yaw_delta_deg", {}).get("mean"),
            "yaw_delta_p95_deg": delta.get("yaw_delta_deg", {}).get("p95"),
            "yaw_delta_max_deg": delta.get("yaw_delta_deg", {}).get("max"),
        }
    ]
    write_table_csv(tables_dir / "opencv_vs_sentai_ablation.csv", ablation_rows, ablation_fields)
    write_table_md(tables_dir / "opencv_vs_sentai_ablation.md", ablation_rows, ablation_fields)

    failure_fields = [
        "z_m",
        "expected_count",
        "frames",
        "opencv_matched",
        "sentai_matched",
        "count_or_match_disagreements",
    ]
    grouped: dict[tuple[str, str], dict[str, Any]] = {}
    for row in results:
        z_key = f"{row['pose_world_camera']['z']:.2f}"
        n_key = str(row["expected_count"])
        key = (z_key, n_key)
        grouped.setdefault(
            key,
            {
                "z_m": z_key,
                "expected_count": n_key,
                "frames": 0,
                "opencv_matched": 0,
                "sentai_matched": 0,
                "count_or_match_disagreements": 0,
            },
        )
        g = grouped[key]
        g["frames"] += 1
        g["opencv_matched"] += row["backends"]["opencv"]["matched_count"]
        sentai = row["backends"]["sentai_sim"]
        if sentai.get("available"):
            g["sentai_matched"] += sentai["matched_count"]
            if row.get("backend_delta", {}).get("count_delta") or row.get("backend_delta", {}).get("matched_delta"):
                g["count_or_match_disagreements"] += 1
    failure_rows = list(grouped.values())
    write_table_csv(tables_dir / "failure_mode_by_z_expected_count.csv", failure_rows, failure_fields)
    write_table_md(tables_dir / "failure_mode_by_z_expected_count.md", failure_rows, failure_fields)

    grouped_fields = [
        "group_type",
        "group",
        "backend",
        "frames",
        "expected",
        "matched",
        "recall",
    ]
    grouped_rows = []
    for backend_name in ("opencv", "sentai_sim"):
        backend = summary[backend_name]
        for group_type, key in (
            ("z", "by_z"),
            ("expected_count", "by_expected_count"),
            ("yaw_bin", "by_yaw_bin"),
            ("roll_pitch_bin", "by_roll_pitch_bin"),
            ("edge_category", "by_edge_category"),
        ):
            for group_name, values in sorted(backend.get(key, {}).items()):
                expected = values.get("expected", 0)
                matched = values.get("matched", 0)
                grouped_rows.append(
                    {
                        "group_type": group_type,
                        "group": group_name,
                        "backend": backend_name,
                        "frames": values.get("frames"),
                        "expected": expected,
                        "matched": matched,
                        "recall": matched / expected if expected else None,
                    }
                )
    write_table_csv(tables_dir / "backend_grouped_detection_summary.csv", grouped_rows, grouped_fields)
    write_table_md(tables_dir / "backend_grouped_detection_summary.md", grouped_rows, grouped_fields)

    disagreement_fields = [
        "frame_id",
        "image_path",
        "z_m",
        "yaw_bin",
        "roll_pitch_bin",
        "edge_category",
        "expected_count",
        "opencv_detected",
        "sentai_detected",
        "count_delta",
        "opencv_matched",
        "sentai_matched",
        "matched_delta",
        "mutual_marker_deltas",
        "pose_delta_available",
    ]
    disagreement_rows = []
    for row in results:
        delta = row.get("backend_delta", {})
        if not delta.get("available"):
            continue
        if not (delta.get("count_delta") or delta.get("matched_delta") or delta.get("marker_deltas")):
            continue
        disagreement_rows.append(
            {
                "frame_id": row["frame_id"],
                "image_path": row["image_path"],
                "z_m": row["pose_world_camera"]["z"],
                "yaw_bin": row.get("yaw_bin"),
                "roll_pitch_bin": row.get("roll_pitch_bin"),
                "edge_category": row.get("edge_visibility_category"),
                "expected_count": row["expected_count"],
                "opencv_detected": row["backends"]["opencv"]["detected_count"],
                "sentai_detected": row["backends"]["sentai_sim"].get("detected_count"),
                "count_delta": delta.get("count_delta"),
                "opencv_matched": row["backends"]["opencv"]["matched_count"],
                "sentai_matched": row["backends"]["sentai_sim"].get("matched_count"),
                "matched_delta": delta.get("matched_delta"),
                "mutual_marker_deltas": len(delta.get("marker_deltas", [])),
                "pose_delta_available": delta.get("pose_delta", {}).get("available"),
            }
        )
    write_table_csv(tables_dir / "opencv_vs_sentai_frame_disagreements.csv", disagreement_rows, disagreement_fields)
    write_table_md(tables_dir / "opencv_vs_sentai_frame_disagreements.md", disagreement_rows, disagreement_fields)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--results-root", type=Path, default=DEFAULT_RESULTS_ROOT)
    parser.add_argument("--overlay-limit", type=int, default=100)
    parser.add_argument("--sentai-results", type=Path, default=None)
    parser.add_argument("--sentai-summary", type=Path, default=None)
    parser.add_argument(
        "--opencv-variant",
        choices=sorted(OPENCV_VARIANTS),
        default="paper",
        help="OpenCV detector variant; default preserves paper-backed parity baseline",
    )
    parser.add_argument(
        "--include-partial-crop",
        action="store_true",
        help="Include partial_crop manifest markers in expected counts for stress/regression metrics",
    )
    parser.add_argument(
        "--skip-ambiguity",
        action="store_true",
        help="Skip exhaustive pose-ambiguity search for large ablation runs",
    )
    args = parser.parse_args()
    apply_opencv_variant(args.opencv_variant)

    dataset = args.dataset.resolve()
    config_path = dataset / "config.json"
    manifest_path = dataset / "manifest.jsonl"
    if not config_path.exists():
        raise SystemExit(f"missing config.json: {config_path}")
    if not manifest_path.exists():
        raise SystemExit(f"missing manifest.jsonl: {manifest_path}")

    config = json.loads(config_path.read_text())
    manifest = load_jsonl(manifest_path)
    sentai_by_image = load_sentai_results(args.sentai_results)
    sentai_run_metadata = load_sentai_run_metadata(args.sentai_results, args.sentai_summary)
    out_dir = args.results_root / dataset.name / run_id()
    overlays_dir = out_dir / "overlays"
    detected_overlays_dir = out_dir / "detected_overlays"
    sentai_overlays_dir = out_dir / "sentai_detected_overlays"
    out_dir.mkdir(parents=True, exist_ok=False)

    results_path = out_dir / "results.jsonl"
    failures_path = out_dir / "failures.csv"
    overlay_count = 0
    results: list[dict[str, Any]] = []

    with results_path.open("w", encoding="utf-8") as results_file, failures_path.open(
        "w", newline="", encoding="utf-8"
    ) as failures_file:
        failure_writer = csv.DictWriter(
            failures_file,
            fieldnames=[
                "frame_id",
                "image_path",
                "backend",
                "status",
                "expected_count",
                "detected_count",
                "matched_count",
                "max_centroid_error_px",
                "z_m",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
            ],
        )
        failure_writer.writeheader()

        for row in manifest:
            image_path = dataset / row["image_path"]
            gray = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise SystemExit(f"failed to read image: {image_path}")

            expected = visible_markers(row, include_partial_crop=args.include_partial_crop)
            ignored = [] if args.include_partial_crop else [
                marker for marker in row["markers"]
                if marker.get("visibility_reason") == "partial_crop"
            ]
            detections = detect_opencv_whycon(gray)
            matches, missed, false_pos, ignored_det = match_detections(expected, detections, ignored=ignored)
            status = frame_status(missed, false_pos, matches)
            max_error = max((m["centroid_error_px"] for m in matches), default=None)
            backend = {
                "detected_count": len(detections),
                "matched_count": len(matches),
                "false_positive_count": len(false_pos),
                "ignored_detection_count": len(ignored_det),
                "missed_count": len(missed),
                "matches": matches,
                "pose_eval": estimate_camera_pose(
                    row,
                    matches,
                    config,
                    evaluate_ambiguity=not args.skip_ambiguity,
                ),
                "detections": detections,
                "status": status,
            }
            sentai_record = sentai_by_image.get(row["image_path"])
            if sentai_record:
                sentai_detections = sentai_record["detections"]
                sentai_matches, sentai_missed, sentai_false_pos, sentai_ignored_det = match_detections(
                    expected,
                    sentai_detections,
                    ignored=ignored,
                )
                sentai_status = frame_status(sentai_missed, sentai_false_pos, sentai_matches)
                sentai_backend = {
                    "available": True,
                    "source": sentai_record["source"],
                    "raw_rc": sentai_record.get("raw_rc"),
                    "detected_count": len(sentai_detections),
                    "matched_count": len(sentai_matches),
                    "false_positive_count": len(sentai_false_pos),
                    "ignored_detection_count": len(sentai_ignored_det),
                    "missed_count": len(sentai_missed),
                    "matches": sentai_matches,
                    "pose_eval": estimate_camera_pose(
                        row,
                        sentai_matches,
                        config,
                        evaluate_ambiguity=not args.skip_ambiguity,
                    ),
                    "sentai_internal_drone_pose_world": sentai_record.get("drone_pose_world"),
                    "detections": sentai_detections,
                    "status": sentai_status,
                }
            else:
                sentai_backend = {"available": False}
            backend_delta = compute_backend_delta(backend, sentai_backend)
            result = {
                "frame_id": row["frame_id"],
                "image_path": row["image_path"],
                "scene_id": row.get("scene_id"),
                "expected_count": len(expected),
                "pose_world_camera": row["pose_world_camera"],
                "yaw_bin": yaw_bin_label(float(row["pose_world_camera"]["yaw_deg"])),
                "roll_pitch_bin": roll_pitch_bin_label(
                    float(row["pose_world_camera"]["roll_deg"]),
                    float(row["pose_world_camera"]["pitch_deg"]),
                ),
                "edge_visibility_category": edge_visibility_category(row, config),
                "backends": {
                    "opencv": backend,
                    "sentai_sim": sentai_backend,
                },
                "backend_delta": backend_delta,
                "status": status,
            }
            results.append(result)
            results_file.write(json.dumps(result, separators=(",", ":")) + "\n")

            if status != "pass":
                pose = row["pose_world_camera"]
                failure_writer.writerow(
                    {
                        "frame_id": row["frame_id"],
                        "image_path": row["image_path"],
                        "backend": "opencv",
                        "status": status,
                        "expected_count": len(expected),
                        "detected_count": len(detections),
                        "matched_count": len(matches),
                        "max_centroid_error_px": max_error,
                        "z_m": pose["z"],
                        "roll_deg": pose["roll_deg"],
                        "pitch_deg": pose["pitch_deg"],
                        "yaw_deg": pose["yaw_deg"],
                    }
                )
                if overlay_count < args.overlay_limit:
                    overlay_path = overlays_dir / f"frame_{row['frame_id']:06d}_{status}.png"
                    draw_overlay(
                        gray,
                        row,
                        detections,
                        matches,
                        missed,
                        false_pos,
                        overlay_path,
                        include_partial_crop=args.include_partial_crop,
                    )
                    overlay_count += 1

            if sentai_record and sentai_status != "pass":
                pose = row["pose_world_camera"]
                sentai_max_error = max((m["centroid_error_px"] for m in sentai_matches), default=None)
                failure_writer.writerow(
                    {
                        "frame_id": row["frame_id"],
                        "image_path": row["image_path"],
                        "backend": "sentai_sim",
                        "status": sentai_status,
                        "expected_count": len(expected),
                        "detected_count": len(sentai_detections),
                        "matched_count": len(sentai_matches),
                        "max_centroid_error_px": sentai_max_error,
                        "z_m": pose["z"],
                        "roll_deg": pose["roll_deg"],
                        "pitch_deg": pose["pitch_deg"],
                        "yaw_deg": pose["yaw_deg"],
                    }
                )

            detected_overlay_path = detected_overlays_dir / f"frame_{row['frame_id']:06d}_opencv_detected.png"
            draw_detection_overlay(gray, row, detections, detected_overlay_path)
            if sentai_record:
                sentai_overlay_path = sentai_overlays_dir / f"frame_{row['frame_id']:06d}_sentai_detected.png"
                draw_detection_overlay(gray, row, sentai_detections, sentai_overlay_path)

    summary = summarize(
        results,
        dataset,
        config,
        sentai_run_metadata,
        include_partial_crop=args.include_partial_crop,
    )
    summary["output_dir"] = str(out_dir)
    summary["overlay_count"] = overlay_count
    summary["detected_overlay_count"] = len(list(detected_overlays_dir.glob("*.png")))
    summary["sentai_detected_overlay_count"] = len(list(sentai_overlays_dir.glob("*.png")))
    write_detection_contact_sheet(
        detected_overlays_dir,
        out_dir / "opencv_detection_contact_sheet.png",
    )
    write_detection_contact_sheet(
        sentai_overlays_dir,
        out_dir / "sentai_detection_contact_sheet.png",
    )
    write_report_tables(out_dir, summary, results)
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
