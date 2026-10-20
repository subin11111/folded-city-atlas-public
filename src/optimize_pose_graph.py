#!/usr/bin/env python3
"""Conservatively optimize an RTAB-Map pose graph with floor-plane factors.

The database is opened read-only. Global loop closures are first verified with
RGB-D overlap and a small ICP refinement. The largest odometry component is
then optimized in gravity-aligned 4-DoF with robust least squares. Per-frame
floor planes constrain height, while one fixed local rotation corrects the
systematic sensor-frame tilt without replacing any measured point.
"""

from __future__ import annotations

import argparse
import csv
import json
import sqlite3
import zlib
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from export_rtabmap_db import (
    calibration_from_blob,
    rotation_matrix_to_quaternion,
    transform_from_blob,
)


@dataclass
class Edge:
    from_id: int
    to_id: int
    kind: str
    transform: np.ndarray
    sigma_translation: float
    sigma_rotation: float
    information_translation: float = 0.0
    information_rotation: float = 0.0
    overlap: float | None = None
    icp_rmse: float | None = None
    refinement_translation: float | None = None
    refinement_rotation_degrees: float | None = None


@dataclass
class FloorObservation:
    node_id: int
    local_normal: np.ndarray
    local_offset: float
    raw_height_at_pose: float
    raw_world_normal: np.ndarray
    raw_world_offset: float
    rms: float
    tilt_degrees: float
    inlier_count: int
    candidate_count: int
    camera_height: float
    selected: bool = False

    @property
    def inlier_ratio(self) -> float:
        return self.inlier_count / max(1, self.candidate_count)


def orthonormalize(rotation: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(rotation)
    result = u @ vt
    if np.linalg.det(result) < 0.0:
        u[:, -1] *= -1.0
        result = u @ vt
    return result


def invert_transform(transform: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def relative_error(measurement: np.ndarray, pose_a: np.ndarray, pose_b: np.ndarray) -> np.ndarray:
    predicted_rotation = pose_a[:3, :3].T @ pose_b[:3, :3]
    predicted_translation = pose_a[:3, :3].T @ (pose_b[:3, 3] - pose_a[:3, 3])
    error_rotation = measurement[:3, :3].T @ predicted_rotation
    error_translation = measurement[:3, :3].T @ (
        predicted_translation - measurement[:3, 3]
    )
    return np.concatenate(
        (error_translation, Rotation.from_matrix(error_rotation).as_rotvec())
    )


def unique_links(connection: sqlite3.Connection, types: tuple[int, ...]) -> list[tuple]:
    placeholders = ",".join("?" for _ in types)
    rows = connection.execute(
        f"""
        SELECT from_id, to_id, type, information_matrix, transform
        FROM Link
        WHERE type IN ({placeholders})
        ORDER BY type, from_id, to_id
        """,
        types,
    ).fetchall()
    links: list[tuple] = []
    seen: set[tuple[int, int, int]] = set()
    for from_id, to_id, link_type, information_blob, transform_blob in rows:
        key = (min(from_id, to_id), max(from_id, to_id), link_type)
        if key in seen:
            continue
        seen.add(key)
        if transform_blob is None:
            continue
        transform = transform_from_blob(transform_blob)
        transform[:3, :3] = orthonormalize(transform[:3, :3])
        information = np.frombuffer(information_blob, dtype="<f8", count=36).reshape(6, 6)
        links.append((from_id, to_id, link_type, information, transform))
    return links


def largest_component(node_ids: set[int], neighbor_links: list[tuple]) -> set[int]:
    adjacency = {node_id: set() for node_id in node_ids}
    for from_id, to_id, *_ in neighbor_links:
        if from_id in adjacency and to_id in adjacency:
            adjacency[from_id].add(to_id)
            adjacency[to_id].add(from_id)
    components: list[set[int]] = []
    unseen = set(node_ids)
    while unseen:
        root = next(iter(unseen))
        component = {root}
        stack = [root]
        unseen.remove(root)
        while stack:
            current = stack.pop()
            for neighbor in adjacency[current]:
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    component.add(neighbor)
                    stack.append(neighbor)
        components.append(component)
    return max(components, key=len)


def decode_local_cloud(
    row: tuple,
    pixel_stride: int,
    voxel_size: float,
    max_depth: float,
) -> np.ndarray:
    _node_id, depth_blob, confidence_blob, calibration_blob = row
    depth_bytes = cv2.imdecode(np.frombuffer(depth_blob, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if depth_bytes is None or depth_bytes.ndim != 3 or depth_bytes.shape[2] != 4:
        return np.empty((0, 3), dtype=np.float64)
    depth = np.ascontiguousarray(depth_bytes).view("<f4").reshape(depth_bytes.shape[:2])
    height, width = depth.shape
    confidence_raw = zlib.decompress(confidence_blob) if confidence_blob else b""
    if len(confidence_raw) == height * width:
        confidence = np.frombuffer(confidence_raw, dtype=np.uint8).reshape(height, width)
    else:
        confidence = np.full((height, width), 100, dtype=np.uint8)

    calibration_width, calibration_height, intrinsic, local_transform = calibration_from_blob(
        calibration_blob
    )
    scale_x = width / float(calibration_width)
    scale_y = height / float(calibration_height)
    fx, fy = intrinsic[0, 0] * scale_x, intrinsic[1, 1] * scale_y
    cx, cy = intrinsic[0, 2] * scale_x, intrinsic[1, 2] * scale_y
    stride = max(1, pixel_stride)
    vv, uu = np.mgrid[0:height:stride, 0:width:stride]
    sampled_depth = depth[::stride, ::stride]
    valid = (
        np.isfinite(sampled_depth)
        & (sampled_depth >= 0.25)
        & (sampled_depth <= max_depth)
        & (confidence[::stride, ::stride] >= 50)
    )
    if np.count_nonzero(valid) < 30:
        return np.empty((0, 3), dtype=np.float64)
    z = sampled_depth[valid].astype(np.float64)
    camera_points = np.column_stack(
        ((uu[valid] - cx) * z / fx, (vv[valid] - cy) * z / fy, z, np.ones_like(z))
    )
    points = (local_transform @ camera_points.T).T[:, :3]
    if voxel_size > 0.0 and len(points):
        keys = np.floor(points / voxel_size).astype(np.int32)
        _, indices = np.unique(keys, axis=0, return_index=True)
        points = points[indices]
    return points


def extract_floor_observation(
    row: tuple,
    raw_pose: np.ndarray,
    pixel_stride: int,
) -> FloorObservation | None:
    node_id = int(row[0])
    local_points = decode_local_cloud(row, pixel_stride=pixel_stride, voxel_size=0.0, max_depth=3.5)
    if len(local_points) < 150:
        return None
    world_points = (raw_pose[:3, :3] @ local_points.T).T + raw_pose[:3, 3]
    pose_height = float(raw_pose[2, 3])
    candidates = world_points[
        (world_points[:, 2] < pose_height - 0.35)
        & (world_points[:, 2] > pose_height - 2.30)
    ]
    if len(candidates) < 100:
        return None

    bins = np.arange(pose_height - 2.30, pose_height - 0.31, 0.03)
    histogram, edges = np.histogram(candidates[:, 2], bins)
    if not len(histogram) or int(histogram.max()) < 40:
        return None
    centers = (edges[:-1] + edges[1:]) * 0.5
    score = histogram * np.clip((pose_height - centers) / 1.0, 0.7, 1.5)
    mode = float(centers[int(np.argmax(score))])
    band = candidates[np.abs(candidates[:, 2] - mode) < 0.09]
    if len(band) < 80:
        return None

    design = np.column_stack((band[:, 0], band[:, 1], np.ones(len(band))))
    coefficients = np.linalg.lstsq(design, band[:, 2], rcond=None)[0]
    for _ in range(2):
        residual = band[:, 2] - design @ coefficients
        inliers = np.abs(residual) < 0.035
        if np.count_nonzero(inliers) < 60:
            return None
        coefficients = np.linalg.lstsq(design[inliers], band[inliers, 2], rcond=None)[0]
    residual = band[:, 2] - design @ coefficients
    inliers = np.abs(residual) < 0.035
    rms = float(np.sqrt(np.mean(np.square(residual[inliers]))))

    a, b, c = coefficients
    normalization = float(np.sqrt(a * a + b * b + 1.0))
    world_normal = np.asarray([-a, -b, 1.0], dtype=np.float64) / normalization
    world_offset = float(-c / normalization)
    tilt_degrees = float(
        np.degrees(np.arccos(np.clip(world_normal[2], -1.0, 1.0)))
    )
    floor_height = float(
        coefficients[0] * raw_pose[0, 3]
        + coefficients[1] * raw_pose[1, 3]
        + coefficients[2]
    )
    local_normal = raw_pose[:3, :3].T @ world_normal
    local_offset = float(world_normal @ raw_pose[:3, 3] + world_offset)
    return FloorObservation(
        node_id=node_id,
        local_normal=local_normal,
        local_offset=local_offset,
        raw_height_at_pose=floor_height,
        raw_world_normal=world_normal,
        raw_world_offset=world_offset,
        rms=rms,
        tilt_degrees=tilt_degrees,
        inlier_count=int(np.count_nonzero(inliers)),
        candidate_count=int(len(candidates)),
        camera_height=float(pose_height - floor_height),
    )


def estimate_local_floor_rotation(
    observations: list[FloorObservation], raw_poses: dict[int, np.ndarray]
) -> tuple[np.ndarray, dict[str, float | int | list[float]]]:
    """Estimate one fixed node-local rotation that aligns floor normals to gravity."""
    active = list(observations)
    rotation = Rotation.identity()
    for _ in range(3):
        measured = np.asarray([observation.local_normal for observation in active])
        expected = np.asarray(
            [
                raw_poses[observation.node_id][:3, :3].T
                @ np.asarray([0.0, 0.0, 1.0])
                for observation in active
            ]
        )
        rotation, _ = Rotation.align_vectors(expected, measured)
        corrected = rotation.apply(measured)
        angular_error = np.degrees(
            np.arccos(np.clip(np.sum(corrected * expected, axis=1), -1.0, 1.0))
        )
        if len(active) < 30:
            break
        cutoff = max(4.0, float(np.percentile(angular_error, 90)))
        filtered = [
            observation
            for observation, error in zip(active, angular_error)
            if error <= cutoff
        ]
        if len(filtered) == len(active):
            break
        active = filtered

    all_measured = np.asarray([observation.local_normal for observation in observations])
    all_expected = np.asarray(
        [
            raw_poses[observation.node_id][:3, :3].T
            @ np.asarray([0.0, 0.0, 1.0])
            for observation in observations
        ]
    )
    before_error = np.degrees(
        np.arccos(np.clip(np.sum(all_measured * all_expected, axis=1), -1.0, 1.0))
    )
    corrected_measured = rotation.apply(all_measured)
    after_error = np.degrees(
        np.arccos(
            np.clip(np.sum(corrected_measured * all_expected, axis=1), -1.0, 1.0)
        )
    )
    matrix = rotation.as_matrix()
    diagnostics: dict[str, float | int | list[float]] = {
        "input_floor_factors": len(observations),
        "robust_inliers": len(active),
        "rotation_vector_degrees": np.degrees(rotation.as_rotvec()).astype(float).tolist(),
        "rotation_angle_degrees": float(np.degrees(rotation.magnitude())),
        "normal_error_before_median_degrees": float(np.median(before_error)),
        "normal_error_before_p90_degrees": float(np.percentile(before_error, 90)),
        "normal_error_after_median_degrees": float(np.median(after_error)),
        "normal_error_after_p90_degrees": float(np.percentile(after_error, 90)),
    }
    return matrix, diagnostics


def refine_and_score_loop(
    target_points: np.ndarray,
    source_points: np.ndarray,
    initial_transform: np.ndarray,
    max_correspondence: float,
    iterations: int,
) -> tuple[np.ndarray, dict[str, float]]:
    if len(target_points) < 100 or len(source_points) < 100:
        return initial_transform, {
            "overlap": 0.0,
            "rmse": 99.0,
            "refinement_translation": 0.0,
            "refinement_rotation_degrees": 0.0,
        }
    target_tree = cKDTree(target_points)
    transform = initial_transform.copy()
    for _ in range(iterations):
        transformed = (
            transform[:3, :3] @ source_points.T
        ).T + transform[:3, 3]
        distance, indices = target_tree.query(transformed, k=1)
        selected = distance < max_correspondence
        if np.count_nonzero(selected) < 80:
            break
        trim_limit = np.percentile(distance[selected], 80)
        selected &= distance <= trim_limit
        moving = transformed[selected]
        fixed = target_points[indices[selected]]
        moving_center = moving.mean(axis=0)
        fixed_center = fixed.mean(axis=0)
        u, _, vt = np.linalg.svd((moving - moving_center).T @ (fixed - fixed_center))
        delta_rotation = vt.T @ u.T
        if np.linalg.det(delta_rotation) < 0.0:
            vt[-1] *= -1.0
            delta_rotation = vt.T @ u.T
        delta_translation = fixed_center - delta_rotation @ moving_center
        delta = np.eye(4, dtype=np.float64)
        delta[:3, :3] = delta_rotation
        delta[:3, 3] = delta_translation
        transform = delta @ transform
        if (
            np.linalg.norm(delta_translation) < 1e-4
            and np.linalg.norm(Rotation.from_matrix(delta_rotation).as_rotvec()) < 1e-4
        ):
            break

    transformed_source = (
        transform[:3, :3] @ source_points.T
    ).T + transform[:3, 3]
    source_distance, _ = target_tree.query(transformed_source, k=1)
    source_overlap = float(np.mean(source_distance < 0.15))
    source_tree = cKDTree(transformed_source)
    target_distance, _ = source_tree.query(target_points, k=1)
    target_overlap = float(np.mean(target_distance < 0.15))
    overlap = 2.0 * source_overlap * target_overlap / max(
        1e-9, source_overlap + target_overlap
    )
    inlier_distance = source_distance[source_distance < 0.15]
    rmse = (
        float(np.sqrt(np.mean(np.square(inlier_distance))))
        if len(inlier_distance)
        else 99.0
    )
    refinement = invert_transform(initial_transform) @ transform
    refinement_translation = float(np.linalg.norm(refinement[:3, 3]))
    refinement_rotation = float(
        np.degrees(
            np.linalg.norm(Rotation.from_matrix(refinement[:3, :3]).as_rotvec())
        )
    )
    return transform, {
        "overlap": overlap,
        "rmse": rmse,
        "refinement_translation": refinement_translation,
        "refinement_rotation_degrees": refinement_rotation,
    }


def write_trajectory(
    path: Path,
    ordered_ids: list[int],
    stamps: dict[int, float],
    poses: dict[int, np.ndarray],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["id", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for node_id in ordered_ids:
            pose = poses[node_id]
            qx, qy, qz, qw = rotation_matrix_to_quaternion(pose[:3, :3])
            writer.writerow(
                [node_id, stamps[node_id], *pose[:3, 3].tolist(), qx, qy, qz, qw]
            )


def error_statistics(edges: list[Edge], poses: dict[int, np.ndarray]) -> dict[str, float | int]:
    if not edges:
        return {"count": 0}
    translation = []
    rotation = []
    for edge in edges:
        error = relative_error(edge.transform, poses[edge.from_id], poses[edge.to_id])
        translation.append(float(np.linalg.norm(error[:3])))
        rotation.append(float(np.degrees(np.linalg.norm(error[3:]))))
    return {
        "count": len(edges),
        "translation_median_m": float(np.median(translation)),
        "translation_p90_m": float(np.percentile(translation, 90)),
        "translation_max_m": float(np.max(translation)),
        "rotation_median_degrees": float(np.median(rotation)),
        "rotation_p90_degrees": float(np.percentile(rotation, 90)),
        "rotation_max_degrees": float(np.max(rotation)),
    }


def floor_statistics(
    observations: list[FloorObservation],
    poses: dict[int, np.ndarray],
    local_correction_rotation: np.ndarray,
) -> dict[str, float | int]:
    heights = []
    tilts = []
    for observation in observations:
        if not observation.selected or observation.node_id not in poses:
            continue
        pose = poses[observation.node_id]
        normal = (
            pose[:3, :3]
            @ local_correction_rotation
            @ observation.local_normal
        )
        normal /= np.linalg.norm(normal)
        if normal[2] < 0.0:
            normal *= -1.0
        offset = observation.local_offset - float(normal @ pose[:3, 3])
        height_at_pose = -(
            normal[0] * pose[0, 3] + normal[1] * pose[1, 3] + offset
        ) / normal[2]
        heights.append(float(height_at_pose))
        tilts.append(float(np.degrees(np.arccos(np.clip(normal[2], -1.0, 1.0)))))
    return {
        "count": len(heights),
        "height_median_m": float(np.median(heights)),
        "height_std_m": float(np.std(heights)),
        "height_p05_m": float(np.percentile(heights, 5)),
        "height_p95_m": float(np.percentile(heights, 95)),
        "tilt_median_degrees": float(np.median(tilts)),
        "tilt_p90_degrees": float(np.percentile(tilts, 90)),
    }


def optimize(
    raw_poses: dict[int, np.ndarray],
    component: set[int],
    anchor_id: int,
    odometry_edges: list[Edge],
    loop_edges: list[Edge],
    floor_observations: list[FloorObservation],
    floor_height: float,
    floor_sigma_height: float,
    local_correction_rotation: np.ndarray,
    huber_scale: float,
    max_evaluations: int,
) -> tuple[dict[int, np.ndarray], dict]:
    ordered_component = sorted(component)
    variable_nodes = [node_id for node_id in ordered_component if node_id != anchor_id]
    node_slice = {
        node_id: slice(index * 4, index * 4 + 4)
        for index, node_id in enumerate(variable_nodes)
    }
    variable_count = len(variable_nodes) * 4
    initial = np.zeros(variable_count, dtype=np.float64)

    def current_poses(parameters: np.ndarray) -> dict[int, np.ndarray]:
        poses: dict[int, np.ndarray] = {anchor_id: raw_poses[anchor_id]}
        for node_id in variable_nodes:
            correction = parameters[node_slice[node_id]]
            pose = np.eye(4, dtype=np.float64)
            yaw_correction = Rotation.from_rotvec(
                np.asarray([0.0, 0.0, correction[3]], dtype=np.float64)
            ).as_matrix()
            pose[:3, :3] = yaw_correction @ raw_poses[node_id][:3, :3]
            pose[:3, 3] = raw_poses[node_id][:3, 3] + correction[:3]
            poses[node_id] = pose
        return poses

    all_edges = odometry_edges + loop_edges
    selected_floors = [
        observation
        for observation in floor_observations
        if observation.selected and observation.node_id != anchor_id
    ]
    def residual(parameters: np.ndarray) -> np.ndarray:
        poses = current_poses(parameters)
        values: list[np.ndarray] = []
        for edge in all_edges:
            error = relative_error(
                edge.transform, poses[edge.from_id], poses[edge.to_id]
            )
            values.append(
                np.concatenate(
                    (
                        error[:3] / edge.sigma_translation,
                        error[3:] / edge.sigma_rotation,
                    )
                )
            )
        for observation in selected_floors:
            pose = poses[observation.node_id]
            normal = (
                pose[:3, :3]
                @ local_correction_rotation
                @ observation.local_normal
            )
            normal /= np.linalg.norm(normal)
            if normal[2] < 0.0:
                normal *= -1.0
            height_at_pose = pose[2, 3] - observation.local_offset / normal[2]
            values.append(
                np.asarray([(height_at_pose - floor_height) / floor_sigma_height])
            )
        return np.concatenate(values)

    residual_count = len(all_edges) * 6 + len(selected_floors)
    sparsity = lil_matrix((residual_count, variable_count), dtype=np.int8)
    row = 0
    for edge in all_edges:
        for node_id in (edge.from_id, edge.to_id):
            if node_id in node_slice:
                sparsity[row : row + 6, node_slice[node_id]] = 1
        row += 6
    for observation in selected_floors:
        sparsity[row, node_slice[observation.node_id]] = 1
        row += 1

    lower = np.empty(variable_count, dtype=np.float64)
    upper = np.empty(variable_count, dtype=np.float64)
    for node_id in variable_nodes:
        current = node_slice[node_id]
        lower[current] = [-5.0, -5.0, -1.0, -0.65]
        upper[current] = [5.0, 5.0, 1.0, 0.65]

    result = least_squares(
        residual,
        initial,
        jac_sparsity=sparsity.tocsr(),
        bounds=(lower, upper),
        method="trf",
        tr_solver="lsmr",
        loss="huber",
        f_scale=huber_scale,
        x_scale="jac",
        max_nfev=max_evaluations,
        ftol=1e-7,
        xtol=1e-7,
        gtol=1e-7,
        verbose=2,
    )
    optimized = current_poses(result.x)
    corrections = np.asarray(
        [result.x[node_slice[node_id]] for node_id in variable_nodes], dtype=np.float64
    )
    diagnostics = {
        "success": bool(result.success),
        "status": int(result.status),
        "message": str(result.message),
        "function_evaluations": int(result.nfev),
        "initial_cost": float(0.5 * np.sum(np.square(residual(initial)))),
        "robust_final_cost": float(result.cost),
        "optimality": float(result.optimality),
        "translation_correction_median_m": float(
            np.median(np.linalg.norm(corrections[:, :3], axis=1))
        ),
        "translation_correction_p95_m": float(
            np.percentile(np.linalg.norm(corrections[:, :3], axis=1), 95)
        ),
        "translation_correction_max_m": float(
            np.max(np.linalg.norm(corrections[:, :3], axis=1))
        ),
        "rotation_correction_median_degrees": float(
            np.degrees(np.median(np.abs(corrections[:, 3])))
        ),
        "rotation_correction_p95_degrees": float(
            np.degrees(np.percentile(np.abs(corrections[:, 3]), 95))
        ),
        "rotation_correction_max_degrees": float(
            np.degrees(np.max(np.abs(corrections[:, 3])))
        ),
    }
    return optimized, diagnostics


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("db", type=Path)
    parser.add_argument("--raw-trajectory", type=Path, required=True)
    parser.add_argument("--optimized-trajectory", type=Path, required=True)
    parser.add_argument("--floor-observations", type=Path, required=True)
    parser.add_argument("--loop-report", type=Path, required=True)
    parser.add_argument("--local-correction", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--floor-pixel-stride", type=int, default=4)
    parser.add_argument("--floor-band", type=float, default=0.28)
    parser.add_argument("--floor-sigma-height", type=float, default=0.04)
    parser.add_argument("--huber-scale", type=float, default=3.0)
    parser.add_argument("--odom-sigma-translation", type=float, default=0.08)
    parser.add_argument("--odom-sigma-rotation", type=float, default=2.0)
    parser.add_argument("--loop-sigma-translation", type=float, default=0.12)
    parser.add_argument("--loop-sigma-rotation", type=float, default=3.0)
    parser.add_argument("--icp-overlap", type=float, default=0.45)
    parser.add_argument("--icp-rmse", type=float, default=0.07)
    parser.add_argument("--icp-refinement-translation", type=float, default=0.25)
    parser.add_argument("--icp-refinement-rotation", type=float, default=5.0)
    parser.add_argument("--max-evaluations", type=int, default=80)
    args = parser.parse_args()

    connection = sqlite3.connect(f"file:{args.db.resolve()}?mode=ro", uri=True)
    node_rows = connection.execute(
        "SELECT id, stamp, pose FROM Node WHERE pose IS NOT NULL ORDER BY id"
    ).fetchall()
    raw_poses = {node_id: transform_from_blob(blob) for node_id, _stamp, blob in node_rows}
    for pose in raw_poses.values():
        pose[:3, :3] = orthonormalize(pose[:3, :3])
    stamps = {node_id: float(stamp or 0.0) for node_id, stamp, _blob in node_rows}
    ordered_ids = [node_id for node_id, _stamp, _blob in node_rows]
    data_rows = {
        row[0]: row
        for row in connection.execute(
            """
            SELECT id, depth, depth_confidence, calibration
            FROM Data
            WHERE depth IS NOT NULL AND calibration IS NOT NULL
            ORDER BY id
            """
        ).fetchall()
    }
    neighbor_links = unique_links(connection, (0,))
    global_links = unique_links(connection, (1,))
    local_links = unique_links(connection, (2,))
    connection.close()

    component = largest_component(set(raw_poses), neighbor_links)
    print(
        f"Loaded {len(raw_poses)} poses; optimizing largest odometry component "
        f"with {len(component)} poses",
        flush=True,
    )

    floor_observations: list[FloorObservation] = []
    for index, node_id in enumerate(sorted(component), start=1):
        if node_id in data_rows:
            observation = extract_floor_observation(
                data_rows[node_id], raw_poses[node_id], args.floor_pixel_stride
            )
            if observation is not None:
                floor_observations.append(observation)
        if index % 50 == 0 or index == len(component):
            print(
                f"Floor analysis {index}/{len(component)}; "
                f"observations={len(floor_observations)}",
                flush=True,
            )

    quality_floors = [
        observation
        for observation in floor_observations
        if observation.rms <= 0.020
        and observation.tilt_degrees <= 10.0
        and observation.inlier_ratio >= 0.10
        and 0.65 <= observation.camera_height <= 1.90
    ]
    if len(quality_floors) < 20:
        raise RuntimeError(
            f"Only {len(quality_floors)} reliable floor observations were found"
        )
    provisional_height = float(
        np.median([observation.raw_height_at_pose for observation in quality_floors])
    )
    selected_floors = [
        observation
        for observation in quality_floors
        if abs(observation.raw_height_at_pose - provisional_height) <= args.floor_band
    ]
    floor_height = float(
        np.median([observation.raw_height_at_pose for observation in selected_floors])
    )
    for observation in selected_floors:
        observation.selected = True
    anchor_observation = min(
        selected_floors,
        key=lambda observation: (
            abs(observation.raw_height_at_pose - floor_height) / 0.05
            + observation.tilt_degrees / 2.0
            + observation.rms / 0.01
        ),
    )
    anchor_id = anchor_observation.node_id
    local_correction_rotation, local_correction_diagnostics = estimate_local_floor_rotation(
        selected_floors, raw_poses
    )
    print(
        f"Selected {len(selected_floors)}/{len(floor_observations)} floor factors; "
        f"reference height={floor_height:.3f} m; stable anchor={anchor_id}; "
        f"fixed local rotation={local_correction_diagnostics['rotation_angle_degrees']:.2f} deg",
        flush=True,
    )

    loop_node_ids = {
        node_id
        for from_id, to_id, _type, _information, transform in global_links
        if abs(np.linalg.det(transform[:3, :3])) > 0.1
        for node_id in (from_id, to_id)
        if node_id in component and node_id in data_rows
    }
    loop_clouds = {}
    for node_id in sorted(loop_node_ids):
        points = decode_local_cloud(
            data_rows[node_id], pixel_stride=2, voxel_size=0.07, max_depth=4.0
        )
        loop_clouds[node_id] = (local_correction_rotation @ points.T).T

    loop_rows: list[dict] = []
    loop_edges: list[Edge] = []
    for from_id, to_id, _type, information, transform in global_links:
        determinant = float(np.linalg.det(transform[:3, :3]))
        row = {
            "from_id": from_id,
            "to_id": to_id,
            "information_translation": float(np.median(np.diag(information)[:3])),
            "information_rotation": float(np.median(np.diag(information)[3:])),
            "valid_transform": abs(determinant) > 0.1,
            "accepted": False,
            "reason": "",
        }
        if from_id not in component or to_id not in component:
            row["reason"] = "outside main odometry component"
        elif abs(determinant) <= 0.1:
            row["reason"] = "singular merged-node transform"
        elif from_id not in loop_clouds or to_id not in loop_clouds:
            row["reason"] = "missing RGB-D cloud"
        else:
            refined, metrics = refine_and_score_loop(
                loop_clouds[from_id],
                loop_clouds[to_id],
                transform,
                max_correspondence=0.22,
                iterations=15,
            )
            row.update(metrics)
            accepted = (
                metrics["overlap"] >= args.icp_overlap
                and metrics["rmse"] <= args.icp_rmse
                and metrics["refinement_translation"]
                <= args.icp_refinement_translation
                and metrics["refinement_rotation_degrees"]
                <= args.icp_refinement_rotation
            )
            row["accepted"] = accepted
            row["reason"] = "passed RGB-D geometric verification" if accepted else "failed RGB-D geometric verification"
            if accepted:
                loop_edges.append(
                    Edge(
                        from_id=from_id,
                        to_id=to_id,
                        kind="global_loop",
                        transform=refined,
                        sigma_translation=args.loop_sigma_translation,
                        sigma_rotation=np.radians(args.loop_sigma_rotation),
                        information_translation=row["information_translation"],
                        information_rotation=row["information_rotation"],
                        overlap=metrics["overlap"],
                        icp_rmse=metrics["rmse"],
                        refinement_translation=metrics["refinement_translation"],
                        refinement_rotation_degrees=metrics[
                            "refinement_rotation_degrees"
                        ],
                    )
                )
        loop_rows.append(row)
    print(
        f"Accepted {len(loop_edges)}/{len(global_links)} stored global loop closures",
        flush=True,
    )

    odometry_edges: list[Edge] = []
    for from_id, to_id, _type, _information, transform in neighbor_links:
        if from_id in component and to_id in component:
            odometry_edges.append(
                Edge(
                    from_id=from_id,
                    to_id=to_id,
                    kind="odometry",
                    transform=transform,
                    sigma_translation=args.odom_sigma_translation,
                    sigma_rotation=np.radians(args.odom_sigma_rotation),
                )
            )
    for from_id, to_id, _type, information, transform in local_links:
        if from_id in component and to_id in component:
            odometry_edges.append(
                Edge(
                    from_id=from_id,
                    to_id=to_id,
                    kind="local_space",
                    transform=transform,
                    sigma_translation=0.08,
                    sigma_rotation=np.radians(2.0),
                    information_translation=float(np.median(np.diag(information)[:3])),
                    information_rotation=float(np.median(np.diag(information)[3:])),
                )
            )

    raw_component_poses = {node_id: raw_poses[node_id] for node_id in component}
    before = {
        "odometry": error_statistics(odometry_edges, raw_component_poses),
        "accepted_loops": error_statistics(loop_edges, raw_component_poses),
        "floor_without_local_correction": floor_statistics(
            floor_observations, raw_component_poses, np.eye(3, dtype=np.float64)
        ),
        "floor_with_local_correction": floor_statistics(
            floor_observations, raw_component_poses, local_correction_rotation
        ),
    }
    optimized_component, solver = optimize(
        raw_poses=raw_poses,
        component=component,
        anchor_id=anchor_id,
        odometry_edges=odometry_edges,
        loop_edges=loop_edges,
        floor_observations=floor_observations,
        floor_height=floor_height,
        floor_sigma_height=args.floor_sigma_height,
        local_correction_rotation=local_correction_rotation,
        huber_scale=args.huber_scale,
        max_evaluations=args.max_evaluations,
    )

    optimized_poses = dict(optimized_component)
    component_ids = np.asarray(sorted(component), dtype=np.int64)
    for node_id in ordered_ids:
        if node_id in optimized_poses:
            continue
        parent_id = int(component_ids[np.argmin(np.abs(component_ids - node_id))])
        correction = optimized_component[parent_id] @ invert_transform(raw_poses[parent_id])
        optimized_poses[node_id] = correction @ raw_poses[node_id]

    after = {
        "odometry": error_statistics(odometry_edges, optimized_component),
        "accepted_loops": error_statistics(loop_edges, optimized_component),
        "floor": floor_statistics(
            floor_observations, optimized_component, local_correction_rotation
        ),
    }

    write_trajectory(args.raw_trajectory, ordered_ids, stamps, raw_poses)
    write_trajectory(args.optimized_trajectory, ordered_ids, stamps, optimized_poses)

    args.floor_observations.parent.mkdir(parents=True, exist_ok=True)
    with args.floor_observations.open("w", encoding="utf-8", newline="") as stream:
        fieldnames = [
            "node_id",
            "selected",
            "raw_height_at_pose_m",
            "rms_m",
            "tilt_degrees",
            "inlier_count",
            "candidate_count",
            "inlier_ratio",
            "camera_height_m",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for observation in floor_observations:
            writer.writerow(
                {
                    "node_id": observation.node_id,
                    "selected": int(observation.selected),
                    "raw_height_at_pose_m": observation.raw_height_at_pose,
                    "rms_m": observation.rms,
                    "tilt_degrees": observation.tilt_degrees,
                    "inlier_count": observation.inlier_count,
                    "candidate_count": observation.candidate_count,
                    "inlier_ratio": observation.inlier_ratio,
                    "camera_height_m": observation.camera_height,
                }
            )

    args.loop_report.parent.mkdir(parents=True, exist_ok=True)
    loop_fieldnames = sorted({key for row in loop_rows for key in row})
    with args.loop_report.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=loop_fieldnames)
        writer.writeheader()
        writer.writerows(loop_rows)

    args.local_correction.parent.mkdir(parents=True, exist_ok=True)
    local_correction_payload = {
        "rotation_matrix": local_correction_rotation.astype(float).tolist(),
        **local_correction_diagnostics,
        "note": (
            "Fixed node-local rotation estimated from reliable floor normals. "
            "Apply between Node.pose and the camera local transform."
        ),
    }
    args.local_correction.write_text(
        json.dumps(local_correction_payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    parameter_report = {
        key: str(value) if isinstance(value, Path) else value
        for key, value in vars(args).items()
    }
    report = {
        "source_database": str(args.db.resolve()),
        "database_open_mode": "read-only",
        "method": "robust gravity-aligned 4-DoF pose graph with RGB-D verified loops and floor-plane factors",
        "total_poses": len(raw_poses),
        "optimized_component_poses": len(component),
        "unconnected_poses_propagated_from_nearest_component_pose": len(raw_poses) - len(component),
        "anchor_node": anchor_id,
        "odometry_and_local_edges": len(odometry_edges),
        "stored_global_loop_pairs": len(global_links),
        "accepted_global_loop_pairs": len(loop_edges),
        "floor_observations": len(floor_observations),
        "selected_floor_factors": len(selected_floors),
        "reference_floor_height_m": floor_height,
        "local_floor_rotation_correction": local_correction_diagnostics,
        "robust_loss": f"Huber (f_scale={args.huber_scale})",
        "before": before,
        "after": after,
        "solver": solver,
        "parameters": parameter_report,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
