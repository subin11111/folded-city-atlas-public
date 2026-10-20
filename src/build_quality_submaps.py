#!/usr/bin/env python3
"""Audit RGB-D frame quality and build overlapping, correlation-aware submaps.

This is a read-only preprocessing stage.  It does not optimize a pose, edit an
RTAB-Map database, or claim that a submap is geometrically correct.  Frozen app
baselines are used only to measure motion and define local submap coordinates.

The frame score is deterministic and deliberately records every threshold that
affected it.  Nearby frames are assigned to one correlation group; consequently
an interval with many almost-identical frames contributes one evidence unit,
not one independent observation per frame.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import sqlite3
import sys
import time
import zlib
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from export_rtabmap_db import (
    calibration_from_blob,
    load_pose_csv,
    voxel_reduce,
    write_binary_ply,
)


THRESHOLDS = {
    "depth_range_m": [0.25, 4.0],
    "usable_depth_fraction": {"A": 0.60, "B": 0.40, "minimum": 0.25},
    "confidence_100_fraction": {"A": 0.40, "B": 0.20, "minimum": 0.08},
    "depth_neighbor_fraction": {"A": 0.75, "B": 0.55, "minimum": 0.35},
    "blur_laplacian_variance_at_640px": {"A": 20.0, "B": 7.0, "minimum": 2.5},
    "clipped_exposure_fraction": {"A_max": 0.25, "B_max": 0.60, "maximum": 0.90},
    "intrinsic_jump_depth_pixels": {"A_max": 0.50, "B_max": 2.0, "maximum": 3.0},
    "linear_pose_increment_m": {"A_max": 0.35, "B_max": 0.75, "maximum": 1.0},
    "angular_pose_increment_deg": {"A_max": 12.0, "B_max": 25.0, "maximum": 35.0},
    "timestamp_increment_s": {"A_max": 2.5, "B_max": 5.0, "maximum": 5.0},
    "grade_score": {"A_min": 75, "B_min": 45},
    "correlation_group": {
        "translation_from_representative_max_m": 0.30,
        "rotation_from_representative_max_deg": 8.0,
        "time_from_representative_max_s": 3.0,
    },
    "submap": {"minimum_m": 2.0, "target_m": 3.5, "maximum_m": 5.0, "overlap_m": 1.0},
    "hard_boundary": {
        "linear_pose_increment_m": 1.0,
        "angular_pose_increment_deg": 75.0,
        "timestamp_increment_s": 5.0,
        "intrinsic_jump_depth_pixels": 3.0,
        "also": ["missing frozen pose", "depth/calibration decode error"],
    },
}

FRAME_COLUMNS = [
    "frame_index", "node_id", "stamp", "node_weight",
    "pose_x_m", "pose_y_m", "pose_z_m", "session_path_m",
    "linear_step_m", "angular_step_deg", "timestamp_step_s",
    "depth_width", "depth_height", "image_width", "image_height",
    "confidence_100_fraction", "confidence_50_fraction", "confidence_0_fraction",
    "confidence_ge50_fraction", "finite_in_range_depth_fraction", "usable_depth_fraction",
    "depth_neighbor_fraction", "depth_isolated_fraction", "depth_edge_fraction",
    "depth_neighbor_difference_p90_m", "depth_median_m",
    "blur_laplacian_variance_640px", "dark_clipped_fraction",
    "bright_clipped_fraction", "clipped_exposure_fraction", "mean_gray",
    "intrinsic_fx_depth_px", "intrinsic_fy_depth_px",
    "intrinsic_cx_depth_px", "intrinsic_cy_depth_px", "intrinsic_jump_depth_px",
    "quality_score", "quality_grade", "quality_reasons", "critical_reasons",
    "usable_for_submap", "hard_boundary", "boundary_reasons",
    "correlation_group_id", "correlation_representative_id", "evidence_weight",
    "membership_submap_ids", "primary_submap_id",
]


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rotation_difference_degrees(a: np.ndarray, b: np.ndarray) -> float:
    relative = a[:3, :3].T @ b[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def inverse_rigid(transform: np.ndarray) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def canonical_gray(image_blob: bytes | None) -> tuple[np.ndarray | None, dict]:
    if not image_blob:
        return None, {"image_width": None, "image_height": None}
    image = cv2.imdecode(np.frombuffer(image_blob, np.uint8), cv2.IMREAD_GRAYSCALE)
    if image is None:
        return None, {"image_width": None, "image_height": None}
    height, width = image.shape
    scale = 640.0 / max(height, width)
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(
        image,
        (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
        interpolation=interpolation,
    )
    return resized, {"image_width": width, "image_height": height}


def decode_depth(depth_blob: bytes) -> np.ndarray:
    encoded = cv2.imdecode(np.frombuffer(depth_blob, np.uint8), cv2.IMREAD_UNCHANGED)
    if encoded is None or encoded.ndim != 3 or encoded.shape[2] != 4:
        raise ValueError("unsupported_float_depth_encoding")
    encoded = np.ascontiguousarray(encoded)
    return encoded.view("<f4").reshape(encoded.shape[:2])


def decode_confidence(confidence_blob: bytes | None, pixel_count: int) -> np.ndarray:
    if not confidence_blob:
        raise ValueError("missing_depth_confidence")
    raw = zlib.decompress(confidence_blob)
    # iOS exports in this data set may append a 12-byte trailer.
    if len(raw) < pixel_count:
        raise ValueError(f"short_depth_confidence:{len(raw)}<{pixel_count}")
    return np.frombuffer(raw[:pixel_count], np.uint8)


def depth_quality_metrics(depth: np.ndarray, confidence: np.ndarray) -> dict:
    """Return full-image confidence and local depth-consistency diagnostics."""
    confidence = np.asarray(confidence, np.uint8).reshape(depth.shape)
    finite_range = np.isfinite(depth) & (depth >= THRESHOLDS["depth_range_m"][0]) & (
        depth <= THRESHOLDS["depth_range_m"][1]
    )
    usable = finite_range & (confidence >= 50)
    neighbor_count = np.zeros(depth.shape, np.uint8)
    neighbor_count[1:, :] += usable[:-1, :]
    neighbor_count[:-1, :] += usable[1:, :]
    neighbor_count[:, 1:] += usable[:, :-1]
    neighbor_count[:, :-1] += usable[:, 1:]
    usable_count = int(np.count_nonzero(usable))

    differences: list[np.ndarray] = []
    edge_flags: list[np.ndarray] = []
    for first, second, mask in (
        (depth[:, :-1], depth[:, 1:], usable[:, :-1] & usable[:, 1:]),
        (depth[:-1, :], depth[1:, :], usable[:-1, :] & usable[1:, :]),
    ):
        if np.any(mask):
            delta = np.abs(first[mask] - second[mask])
            reference = np.minimum(first[mask], second[mask])
            differences.append(delta)
            # Range-adaptive threshold: a real discontinuity must exceed sensor
            # noise both absolutely and relative to range.
            edge_flags.append(delta > np.maximum(0.08, 0.04 * reference))
    all_differences = np.concatenate(differences) if differences else np.empty(0)
    all_edges = np.concatenate(edge_flags) if edge_flags else np.empty(0, bool)
    total = depth.size
    return {
        "depth_width": int(depth.shape[1]),
        "depth_height": int(depth.shape[0]),
        "confidence_100_fraction": float(np.mean(confidence == 100)),
        "confidence_50_fraction": float(np.mean(confidence == 50)),
        "confidence_0_fraction": float(np.mean(confidence == 0)),
        "confidence_ge50_fraction": float(np.mean(confidence >= 50)),
        "finite_in_range_depth_fraction": float(np.mean(finite_range)),
        "usable_depth_fraction": float(usable_count / total),
        "depth_neighbor_fraction": (
            float(np.mean(neighbor_count[usable] >= 2)) if usable_count else 0.0
        ),
        "depth_isolated_fraction": (
            float(np.mean(neighbor_count[usable] == 0)) if usable_count else 1.0
        ),
        "depth_edge_fraction": float(np.mean(all_edges)) if len(all_edges) else 0.0,
        "depth_neighbor_difference_p90_m": (
            float(np.percentile(all_differences, 90)) if len(all_differences) else None
        ),
        "depth_median_m": float(np.median(depth[usable])) if usable_count else None,
    }


def rgb_quality_metrics(gray: np.ndarray | None) -> dict:
    if gray is None:
        return {
            "blur_laplacian_variance_640px": None,
            "dark_clipped_fraction": None,
            "bright_clipped_fraction": None,
            "clipped_exposure_fraction": None,
            "mean_gray": None,
        }
    dark = float(np.mean(gray <= 5))
    bright = float(np.mean(gray >= 250))
    return {
        "blur_laplacian_variance_640px": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        "dark_clipped_fraction": dark,
        "bright_clipped_fraction": bright,
        "clipped_exposure_fraction": dark + bright,
        "mean_gray": float(np.mean(gray)),
    }


def _high_is_good(value: float, excellent: float, moderate: float, minimum: float,
                  points: tuple[int, int, int], labels: tuple[str, str, str]) -> tuple[int, list[str], list[str]]:
    if value >= excellent:
        return points[0], [], []
    if value >= moderate:
        return points[1], [labels[0]], []
    if value >= minimum:
        return points[2], [labels[1]], []
    return 0, [labels[2]], [labels[2]]


def _low_is_good(value: float, excellent: float, moderate: float, maximum: float,
                 points: tuple[int, int, int], labels: tuple[str, str, str]) -> tuple[int, list[str], list[str]]:
    if value <= excellent:
        return points[0], [], []
    if value <= moderate:
        return points[1], [labels[0]], []
    if value <= maximum:
        return points[2], [labels[1]], []
    return 0, [labels[2]], [labels[2]]


def score_quality(metrics: dict) -> tuple[int, str, list[str], list[str]]:
    """Compute a 0--100 score; a critical threshold always forces grade C."""
    score = 0
    reasons: list[str] = []
    critical: list[str] = []

    components = [
        _high_is_good(metrics["usable_depth_fraction"], .60, .40, .25, (22, 15, 7),
                      ("moderate_usable_depth", "low_usable_depth", "critical_usable_depth")),
        _high_is_good(metrics["confidence_100_fraction"], .40, .20, .08, (18, 12, 5),
                      ("moderate_confidence100", "low_confidence100", "critical_confidence100")),
        _high_is_good(metrics["depth_neighbor_fraction"], .75, .55, .35, (10, 6, 2),
                      ("moderate_depth_neighborhood", "weak_depth_neighborhood", "critical_depth_neighborhood")),
    ]
    for points, why, severe in components:
        score += points
        reasons += why
        critical += severe

    blur = metrics.get("blur_laplacian_variance_640px")
    clipping = metrics.get("clipped_exposure_fraction")
    if blur is None or clipping is None:
        score += 16  # neutral, because RGB is diagnostic rather than required geometry
        reasons.append("rgb_unavailable_neutral_score")
    else:
        for result in (
            _high_is_good(blur, 20.0, 7.0, 2.5, (12, 8, 3),
                          ("moderate_blur", "strong_blur", "critical_blur")),
            _low_is_good(clipping, .25, .60, .90, (10, 6, 2),
                         ("moderate_exposure_clipping", "strong_exposure_clipping", "critical_exposure_clipping")),
        ):
            points, why, severe = result
            score += points
            reasons += why
            critical += severe

    jump = metrics.get("intrinsic_jump_depth_px")
    if jump is None:
        score += 8
        reasons.append("first_frame_no_intrinsic_delta")
    else:
        points, why, severe = _low_is_good(
            jump, .50, 2.0, 3.0, (8, 5, 2),
            ("moderate_intrinsic_jump", "large_intrinsic_jump", "critical_intrinsic_jump"),
        )
        score += points
        reasons += why
        critical += severe

    if metrics.get("linear_step_m") is None:
        score += 20
        reasons.append("first_frame_no_motion_delta")
    else:
        for result in (
            _low_is_good(metrics["linear_step_m"], .35, .75, 1.0, (7, 4, 2),
                         ("moderate_linear_step", "large_linear_step", "critical_linear_step")),
            _low_is_good(metrics["angular_step_deg"], 12.0, 25.0, 35.0, (7, 4, 2),
                         ("moderate_angular_step", "large_angular_step", "critical_angular_step")),
            _low_is_good(metrics["timestamp_step_s"], 2.5, 5.0, 5.0, (6, 3, 3),
                         ("moderate_timestamp_gap", "large_timestamp_gap", "critical_timestamp_gap")),
        ):
            points, why, severe = result
            score += points
            reasons += why
            critical += severe

    score = int(np.clip(score, 0, 100))
    if critical or score < THRESHOLDS["grade_score"]["B_min"]:
        grade = "C"
    elif score >= THRESHOLDS["grade_score"]["A_min"]:
        grade = "A"
    else:
        grade = "B"
    return score, grade, sorted(set(reasons)), sorted(set(critical))


def analyze_frame(
    row: tuple,
    pose: np.ndarray | None,
    previous_pose: np.ndarray | None,
    previous_stamp: float | None,
    previous_intrinsic: np.ndarray | None,
) -> tuple[dict, np.ndarray | None]:
    node_id, stamp, weight, image_blob, depth_blob, confidence_blob, calibration_blob = row
    stamp = float(stamp or 0.0)
    gray, dimensions = canonical_gray(image_blob)
    metrics = {**dimensions, **rgb_quality_metrics(gray)}
    try:
        depth = decode_depth(depth_blob)
        confidence = decode_confidence(confidence_blob, depth.size)
        metrics.update(depth_quality_metrics(depth, confidence))
        calibration_width, calibration_height, intrinsic, _local = calibration_from_blob(calibration_blob)
        if calibration_width <= 0 or calibration_height <= 0:
            raise ValueError("invalid_calibration_dimensions")
        intrinsic_depth = np.asarray([
            intrinsic[0, 0] * depth.shape[1] / calibration_width,
            intrinsic[1, 1] * depth.shape[0] / calibration_height,
            intrinsic[0, 2] * depth.shape[1] / calibration_width,
            intrinsic[1, 2] * depth.shape[0] / calibration_height,
        ], dtype=np.float64)
        metrics.update({
            "intrinsic_fx_depth_px": float(intrinsic_depth[0]),
            "intrinsic_fy_depth_px": float(intrinsic_depth[1]),
            "intrinsic_cx_depth_px": float(intrinsic_depth[2]),
            "intrinsic_cy_depth_px": float(intrinsic_depth[3]),
            "intrinsic_jump_depth_px": (
                None if previous_intrinsic is None
                else float(np.max(np.abs(intrinsic_depth - previous_intrinsic)))
            ),
        })
    except (ValueError, TypeError, zlib.error) as exc:
        intrinsic_depth = None
        metrics.update({
            "depth_width": None, "depth_height": None,
            "confidence_100_fraction": 0.0, "confidence_50_fraction": 0.0,
            "confidence_0_fraction": 1.0, "confidence_ge50_fraction": 0.0,
            "finite_in_range_depth_fraction": 0.0, "usable_depth_fraction": 0.0,
            "depth_neighbor_fraction": 0.0, "depth_isolated_fraction": 1.0,
            "depth_edge_fraction": 0.0, "depth_neighbor_difference_p90_m": None,
            "depth_median_m": None, "intrinsic_fx_depth_px": None,
            "intrinsic_fy_depth_px": None, "intrinsic_cx_depth_px": None,
            "intrinsic_cy_depth_px": None, "intrinsic_jump_depth_px": None,
            "decode_error": str(exc),
        })

    if previous_pose is None or pose is None:
        linear_step = angular_step = timestamp_step = None
    else:
        linear_step = float(np.linalg.norm(pose[:3, 3] - previous_pose[:3, 3]))
        angular_step = rotation_difference_degrees(previous_pose, pose)
        timestamp_step = max(0.0, stamp - float(previous_stamp))
    metrics.update({
        "linear_step_m": linear_step,
        "angular_step_deg": angular_step,
        "timestamp_step_s": timestamp_step,
    })
    score, grade, reasons, critical = score_quality(metrics)
    if pose is None:
        critical.append("missing_frozen_baseline_pose")
        reasons.append("missing_frozen_baseline_pose")
        grade = "C"
    if metrics.get("decode_error"):
        critical.append("depth_or_calibration_decode_error")
        reasons.append("depth_or_calibration_decode_error")
        grade = "C"
    # Quality C excludes a frame as registration evidence, but a blurred or
    # depth-poor frame alone does not prove tracking continuity was lost.  Hard
    # boundaries are restricted to discontinuities in pose/time/intrinsics or
    # an undecodable/missing-pose frame.  A >75 degree camera jump is used for
    # segmentation; the >35 degree quality gate remains a conservative C grade.
    hard_reasons = []
    if metrics.get("intrinsic_jump_depth_px") is not None and metrics["intrinsic_jump_depth_px"] > 3.0:
        hard_reasons.append("critical_intrinsic_jump")
    if linear_step is not None and linear_step > 1.0:
        hard_reasons.append("critical_linear_step")
    if angular_step is not None and angular_step > 75.0:
        hard_reasons.append("hard_angular_step_over_75deg")
    if timestamp_step is not None and timestamp_step > 5.0:
        hard_reasons.append("critical_timestamp_gap")
    if pose is None:
        hard_reasons.append("missing_frozen_baseline_pose")
    if metrics.get("decode_error"):
        hard_reasons.append("depth_or_calibration_decode_error")
    hard_reasons = sorted(set(hard_reasons))
    frame = {
        "node_id": int(node_id), "stamp": stamp, "node_weight": weight,
        "pose_x_m": None if pose is None else float(pose[0, 3]),
        "pose_y_m": None if pose is None else float(pose[1, 3]),
        "pose_z_m": None if pose is None else float(pose[2, 3]), "_pose": pose,
        **metrics,
        "quality_score": score, "quality_grade": grade,
        "quality_reasons": reasons, "critical_reasons": sorted(set(critical)),
        "usable_for_submap": grade in ("A", "B"),
        "hard_boundary": bool(hard_reasons),
        "boundary_reasons": hard_reasons,
    }
    return frame, intrinsic_depth


def assign_correlation_groups(frames: list[dict], session: str) -> None:
    """Assign a sequential physical-observation group and normalized weights."""
    limits = THRESHOLDS["correlation_group"]
    group_index = 0
    representative: dict | None = None
    members: dict[str, list[dict]] = defaultdict(list)
    for frame in frames:
        if not frame["usable_for_submap"]:
            frame["correlation_group_id"] = ""
            frame["correlation_representative_id"] = ""
            frame["evidence_weight"] = 0.0
            representative = None
            continue
        same_group = False
        if representative is not None:
            translation = float(np.linalg.norm(
                frame["_pose"][:3, 3] - representative["_pose"][:3, 3]
            ))
            rotation = rotation_difference_degrees(representative["_pose"], frame["_pose"])
            elapsed = max(0.0, frame["stamp"] - representative["stamp"])
            same_group = (
                translation <= limits["translation_from_representative_max_m"]
                and rotation <= limits["rotation_from_representative_max_deg"]
                and elapsed <= limits["time_from_representative_max_s"]
            )
        if representative is None or not same_group:
            group_index += 1
            representative = frame
        group_id = f"{session}:g{group_index:04d}"
        frame["correlation_group_id"] = group_id
        frame["correlation_representative_id"] = int(representative["node_id"])
        members[group_id].append(frame)
    for group in members.values():
        weight = 1.0 / len(group)
        for frame in group:
            frame["evidence_weight"] = weight


def contiguous_usable_runs(frames: list[dict]) -> list[list[dict]]:
    """Return usable evidence runs, bridging sensor-only C frames.

    A hard tracking/calibration boundary ends a run.  A non-hard C frame is
    omitted from registration evidence but does not fragment the trajectory.
    Gaps exceeding one maximum submap length are also split because no 2--5 m
    submap can bridge them without observations.
    """
    runs: list[list[dict]] = []
    current: list[dict] = []
    for frame in frames:
        if frame["hard_boundary"]:
            if current:
                runs.append(current)
                current = []
            continue
        if not frame["usable_for_submap"]:
            continue
        if current and frame["session_path_m"] - current[-1]["session_path_m"] > THRESHOLDS["submap"]["maximum_m"]:
            runs.append(current)
            current = []
        current.append(frame)
    if current:
        runs.append(current)
    return runs


def _choose_end(cumulative: np.ndarray, start: int, target: float, maximum: float) -> int:
    candidates = np.flatnonzero(cumulative >= cumulative[start] + target)
    end = int(candidates[0]) if len(candidates) else len(cumulative) - 1
    within_max = np.flatnonzero(cumulative <= cumulative[start] + maximum + 1e-12)
    last_allowed = int(within_max[-1]) if len(within_max) else start
    return max(start, min(end, last_allowed))


def build_submaps(frames: list[dict], session: str) -> list[dict]:
    params = THRESHOLDS["submap"]
    submaps: list[dict] = []
    all_frames_by_id = {frame["node_id"]: frame for frame in frames}
    for run_index, run in enumerate(contiguous_usable_runs(frames), start=1):
        cumulative = np.zeros(len(run), dtype=np.float64)
        for index in range(1, len(run)):
                # session_path_m includes motion through any sensor-only C
                # frames omitted between these two usable observations.
                cumulative[index] = cumulative[index - 1] + float(
                    run[index]["session_path_m"] - run[index - 1]["session_path_m"]
                )
        start = 0
        previous_end = -1
        while start < len(run):
            remaining = float(cumulative[-1] - cumulative[start])
            end = len(run) - 1 if remaining <= params["maximum_m"] else _choose_end(
                cumulative, start, params["target_m"], params["maximum_m"]
            )
            if end == start and end < len(run) - 1:
                end += 1
            selected = run[start:end + 1]
            groups = list(dict.fromkeys(f["correlation_group_id"] for f in selected))
            path_length = float(cumulative[end] - cumulative[start])
            overlap = 0.0
            overlap_frames = 0
            if previous_end >= start:
                overlap = float(cumulative[previous_end] - cumulative[start])
                overlap_frames = previous_end - start + 1
            quality_counts = Counter(f["quality_grade"] for f in selected)
            short_reason = ""
            if path_length < params["minimum_m"]:
                if run_index > 1 or (run and run[0]["frame_index"] > 0):
                    short_reason = "short_run_adjacent_to_quality_boundary"
                else:
                    short_reason = "short_session_endpoint_run"
            submap = {
                "submap_id": f"{session}:s{len(submaps) + 1:04d}",
                "run_id": f"{session}:r{run_index:03d}",
                "start_node_id": int(selected[0]["node_id"]),
                "end_node_id": int(selected[-1]["node_id"]),
                "anchor_node_id": int(selected[0]["correlation_representative_id"]),
                "initial_anchor_pose_matrix": all_frames_by_id[
                    int(selected[0]["correlation_representative_id"])
                ]["_pose"].tolist(),
                "start_stamp": float(selected[0]["stamp"]),
                "end_stamp": float(selected[-1]["stamp"]),
                "node_ids": [int(f["node_id"]) for f in selected],
                "correlation_group_ids": groups,
                "primary_correlation_group_ids": [],
                "path_length_m": path_length,
                "session_path_center_m": float(
                    (selected[0]["session_path_m"] + selected[-1]["session_path_m"]) * 0.5
                ),
                "raw_frame_count": len(selected),
                "effective_evidence_groups": len(groups),
                "quality_counts": dict(sorted(quality_counts.items())),
                "overlap_previous_m": overlap,
                "overlap_previous_frames": overlap_frames,
                "short_reason": short_reason,
                "within_nominal_2_to_5m": params["minimum_m"] <= path_length <= params["maximum_m"] + 1e-9,
            }
            submaps.append(submap)
            if end >= len(run) - 1:
                break
            previous_end = end
            next_start = end
            for candidate in range(end, start, -1):
                if cumulative[end] - cumulative[candidate] >= params["overlap_m"]:
                    next_start = candidate
                    break
            if next_start <= start:
                next_start = start + 1
            start = next_start
    return submaps


def assign_submap_memberships(frames: list[dict], submaps: list[dict]) -> None:
    """Give each correlated observation one primary submap despite overlap."""
    by_group: dict[str, list[dict]] = defaultdict(list)
    for frame in frames:
        frame["membership_submap_ids"] = []
        frame["primary_submap_id"] = ""
        if frame["correlation_group_id"]:
            by_group[frame["correlation_group_id"]].append(frame)
    submap_by_id = {submap["submap_id"]: submap for submap in submaps}
    group_memberships: dict[str, set[str]] = defaultdict(set)
    for submap in submaps:
        for group_id in submap["correlation_group_ids"]:
            group_memberships[group_id].add(submap["submap_id"])
    for group_id, group_frames in by_group.items():
        memberships = sorted(group_memberships[group_id])
        if not memberships:
            raise RuntimeError(f"Correlation group has no submap membership: {group_id}")
        representative = next(
            frame for frame in group_frames
            if frame["node_id"] == frame["correlation_representative_id"]
        )
        position = float(representative["session_path_m"])
        primary = min(
            memberships,
            key=lambda submap_id: (
                abs(submap_by_id[submap_id]["session_path_center_m"] - position), submap_id
            ),
        )
        submap_by_id[primary]["primary_correlation_group_ids"].append(group_id)
        for frame in group_frames:
            frame["membership_submap_ids"] = memberships
            frame["primary_submap_id"] = primary
    for submap in submaps:
        submap["primary_correlation_group_ids"].sort()
        submap["primary_evidence_groups"] = len(submap["primary_correlation_group_ids"])


def flatten_submap(row: dict) -> dict:
    return {
        k: v for k, v in row.items()
        if k not in (
            "node_ids", "correlation_group_ids", "primary_correlation_group_ids",
            "quality_counts", "initial_anchor_pose_matrix",
        )
    } | {
        "node_ids": ";".join(map(str, row["node_ids"])),
        "correlation_group_ids": ";".join(row["correlation_group_ids"]),
        "primary_correlation_group_ids": ";".join(row["primary_correlation_group_ids"]),
        "initial_anchor_pose_matrix": json.dumps(row["initial_anchor_pose_matrix"], separators=(",", ":")),
        "quality_A_frames": row["quality_counts"].get("A", 0),
        "quality_B_frames": row["quality_counts"].get("B", 0),
    }


def write_csv(path: Path, rows: Iterable[dict], fieldnames: list[str] | None = None) -> None:
    rows = list(rows)
    if fieldnames is None:
        fieldnames = list(rows[0]) if rows else []
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            converted = {
                key: (";".join(map(str, value)) if isinstance(value, list) else value)
                for key, value in row.items()
            }
            writer.writerow(converted)


def _cloud_points_for_frame(connection: sqlite3.Connection, frame: dict, anchor_inverse: np.ndarray,
                            pixel_stride: int) -> tuple[np.ndarray, np.ndarray]:
    row = connection.execute(
        "SELECT image,depth,depth_confidence,calibration FROM Data WHERE id=?", (frame["node_id"],)
    ).fetchone()
    if row is None:
        return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)
    image_blob, depth_blob, confidence_blob, calibration_blob = row
    depth = decode_depth(depth_blob)
    confidence = decode_confidence(confidence_blob, depth.size).reshape(depth.shape)
    cw, ch, intrinsic, local = calibration_from_blob(calibration_blob)
    color = cv2.imdecode(np.frombuffer(image_blob, np.uint8), cv2.IMREAD_COLOR)
    if color is None:
        return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)
    height, width = depth.shape
    vv, uu = np.mgrid[0:height:pixel_stride, 0:width:pixel_stride]
    zz = depth[::pixel_stride, ::pixel_stride]
    conf = confidence[::pixel_stride, ::pixel_stride]
    valid = np.isfinite(zz) & (zz >= .25) & (zz <= 4.0) & (conf >= 50)
    if not np.any(valid):
        return np.empty((0, 3), np.float32), np.empty((0, 3), np.uint8)
    z = zz[valid].astype(np.float64)
    fx, fy = intrinsic[0, 0] * width / cw, intrinsic[1, 1] * height / ch
    cx, cy = intrinsic[0, 2] * width / cw, intrinsic[1, 2] * height / ch
    points = np.column_stack(((uu[valid] - cx) * z / fx, (vv[valid] - cy) * z / fy, z, np.ones_like(z)))
    transform = anchor_inverse @ frame["_pose"] @ local
    points = (transform @ points.T).T[:, :3].astype(np.float32)
    rgb = cv2.cvtColor(cv2.resize(color, (width, height), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
    colors = rgb[::pixel_stride, ::pixel_stride][valid]
    return points, colors


def write_submap_clouds(connection: sqlite3.Connection, frames: list[dict], submaps: list[dict],
                        output: Path, pixel_stride: int, voxel: float) -> dict:
    by_id = {f["node_id"]: f for f in frames}
    cloud_dir = output / "clouds"
    cloud_dir.mkdir()
    records = {}
    for index, submap in enumerate(submaps, start=1):
        anchor_inverse = inverse_rigid(by_id[submap["anchor_node_id"]]["_pose"])
        representative_ids = {
            by_id[node]["correlation_representative_id"] for node in submap["node_ids"]
        }
        points, colors = [], []
        for node_id in sorted(representative_ids):
            p, c = _cloud_points_for_frame(connection, by_id[node_id], anchor_inverse, pixel_stride)
            if len(p):
                points.append(p)
                colors.append(c)
        if not points:
            records[submap["submap_id"]] = {"error": "no_points"}
            continue
        reduced, reduced_colors, _ = voxel_reduce(np.concatenate(points), np.concatenate(colors), voxel)
        path = cloud_dir / f"submap_{index:04d}.ply"
        write_binary_ply(path, reduced, reduced_colors)
        records[submap["submap_id"]] = {
            "path": str(path.resolve()), "sha256": sha256_file(path),
            "points": int(len(reduced)), "pixel_stride": pixel_stride, "voxel_m": voxel,
            "coordinate_frame": f"anchor_node_{submap['anchor_node_id']}",
            "frame_sampling": "one representative per correlation group",
        }
    return records


def aggregate_numeric(frames: list[dict], key: str) -> dict:
    values = np.asarray([f[key] for f in frames if f.get(key) is not None], dtype=np.float64)
    if not len(values):
        return {"count": 0}
    return {
        "count": int(len(values)), "min": float(values.min()),
        "median": float(np.median(values)), "p90": float(np.percentile(values, 90)),
        "max": float(values.max()),
    }


def audit_session(name: str, entry: dict, baseline: dict, output: Path,
                  project_root: Path, write_clouds: bool, cloud_pixel_stride: int,
                  cloud_voxel: float) -> dict:
    database = Path(entry["database"])
    if not database.is_absolute():
        database = project_root / database
    database = database.resolve()
    pose_csv = Path(baseline["selected_pose_csv"]).resolve()
    poses = load_pose_csv(pose_csv)
    if not poses:
        raise ValueError(f"{name}: frozen pose CSV is empty")
    database_hash_before = sha256_file(database)
    expected_hash = baseline.get("database_sha256")
    if expected_hash and database_hash_before != expected_hash:
        raise ValueError(f"{name}: database hash differs from frozen baseline report")

    session_output = output / name
    session_output.mkdir()
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    query = (
        "SELECT n.id,n.stamp,n.weight,d.image,d.depth,d.depth_confidence,d.calibration "
        "FROM Node n JOIN Data d ON d.id=n.id WHERE n.pose IS NOT NULL ORDER BY n.id"
    )
    frames: list[dict] = []
    previous_pose = None
    previous_stamp = None
    previous_intrinsic = None
    cumulative_path = 0.0
    for frame_index, row in enumerate(connection.execute(query)):
        node_id = int(row[0])
        pose = poses.get(node_id)
        frame, intrinsic = analyze_frame(row, pose, previous_pose, previous_stamp, previous_intrinsic)
        if frame["linear_step_m"] is not None:
            cumulative_path += frame["linear_step_m"]
        frame["frame_index"] = frame_index
        frame["session_path_m"] = cumulative_path
        frames.append(frame)
        previous_pose, previous_stamp = pose, frame["stamp"]
        if intrinsic is not None:
            previous_intrinsic = intrinsic
        if (frame_index + 1) % 200 == 0:
            print(f"{name}: audited {frame_index + 1} frames", flush=True)
    if set(poses) - {f["node_id"] for f in frames}:
        extra = sorted(set(poses) - {f["node_id"] for f in frames})
        raise ValueError(f"{name}: pose CSV contains nodes without complete DB rows: {extra[:10]}")

    assign_correlation_groups(frames, name)
    submaps = build_submaps(frames, name)
    assign_submap_memberships(frames, submaps)
    cloud_records = (
        write_submap_clouds(connection, frames, submaps, session_output, cloud_pixel_stride, cloud_voxel)
        if write_clouds else {}
    )
    connection.close()

    frame_rows = []
    for frame in frames:
        row = {key: frame.get(key) for key in FRAME_COLUMNS}
        frame_rows.append(row)
    write_csv(session_output / "frames.csv", frame_rows, FRAME_COLUMNS)
    write_csv(session_output / "submaps.csv", [flatten_submap(s) for s in submaps])
    (session_output / "submaps.json").write_text(json.dumps({
        "session": name, "parameters": THRESHOLDS["submap"],
        "correlation_rule": THRESHOLDS["correlation_group"], "submaps": submaps,
    }, indent=2) + "\n", encoding="utf-8")

    grade_counts = Counter(f["quality_grade"] for f in frames)
    reason_counts = Counter(reason for f in frames for reason in f["quality_reasons"])
    boundary_counts = Counter(reason for f in frames for reason in f["boundary_reasons"])
    groups = defaultdict(list)
    for frame in frames:
        if frame["correlation_group_id"]:
            groups[frame["correlation_group_id"]].append(frame)
    weight_errors = {
        group_id: float(sum(f["evidence_weight"] for f in group) - 1.0)
        for group_id, group in groups.items()
        if abs(sum(f["evidence_weight"] for f in group) - 1.0) > 1e-12
    }
    if weight_errors:
        raise RuntimeError(f"{name}: correlation group weights do not sum to one")
    database_hash_after = sha256_file(database)
    if database_hash_after != database_hash_before:
        raise RuntimeError(f"{name}: source database changed during read-only audit")

    summary = {
        "session": name,
        "status": "QUALITY_AUDIT_AND_SUBMAP_PROPOSAL_NOT_OPTIMIZED_NOT_VALIDATED_MAP",
        "source_database": str(database), "database_open_mode": "read-only",
        "database_sha256_before": database_hash_before,
        "database_sha256_after": database_hash_after, "database_unchanged": True,
        "frozen_pose_csv": str(pose_csv), "frozen_pose_sha256": sha256_file(pose_csv),
        "baseline_selected_label": baseline.get("selected_label"),
        "frame_count": len(frames), "quality_grade_counts": dict(sorted(grade_counts.items())),
        "missing_frozen_pose_frame_count": sum(f["_pose"] is None for f in frames),
        "usable_frame_count": sum(f["usable_for_submap"] for f in frames),
        "hard_boundary_frame_count": sum(f["hard_boundary"] for f in frames),
        "correlation_group_count": len(groups),
        "correlation_weights_all_sum_to_one": True,
        "submap_count": len(submaps),
        "nominal_2_to_5m_submap_count": sum(s["within_nominal_2_to_5m"] for s in submaps),
        "short_submap_count": sum(bool(s["short_reason"]) for s in submaps),
        "session_path_m": cumulative_path,
        "reason_counts": dict(reason_counts.most_common()),
        "boundary_reason_counts": dict(boundary_counts.most_common()),
        "metric_summaries": {
            key: aggregate_numeric(frames, key) for key in (
                "quality_score", "usable_depth_fraction", "confidence_100_fraction",
                "confidence_50_fraction", "depth_neighbor_fraction", "depth_edge_fraction",
                "blur_laplacian_variance_640px", "clipped_exposure_fraction",
                "intrinsic_jump_depth_px", "linear_step_m", "angular_step_deg",
            )
        },
        "artifacts": {
            "frames_csv": str((session_output / "frames.csv").resolve()),
            "submaps_csv": str((session_output / "submaps.csv").resolve()),
            "submaps_json": str((session_output / "submaps.json").resolve()),
            "clouds": cloud_records,
        },
        "limitations": [
            "A/B/C grades are deterministic engineering gates, not calibrated probabilities of correct tracking.",
            "Blur and exposure thresholds are evaluated on a 640-pixel-long-side grayscale image.",
            "Depth confidence and local smoothness do not detect a globally wrong camera pose.",
            "Correlation groups prevent frame-count inflation but are not statistically independent ground truth.",
            "Submap boundaries and 2-5 m lengths are proposals for later registration and region holdout; no pose was optimized here.",
        ],
    }
    (session_output / "session_manifest.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(
        f"{name}: frames={len(frames)}, grades={dict(grade_counts)}, "
        f"groups={len(groups)}, submaps={len(submaps)}",
        flush=True,
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config/student_union_all_sessions.json"))
    parser.add_argument(
        "--baselines", type=Path,
        default=Path("outputs/student_union_20261017_geometry_review/baselines/report.json"),
    )
    parser.add_argument(
        "--output", type=Path,
        default=Path("outputs/student_union_20261017_submap_quality"),
    )
    parser.add_argument("--write-clouds", action="store_true",
                        help="Write a small local PLY from one frame per correlation group")
    parser.add_argument("--cloud-pixel-stride", type=int, default=8)
    parser.add_argument("--cloud-voxel", type=float, default=0.05)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {args.output}")
    if args.cloud_pixel_stride < 1 or args.cloud_voxel <= 0:
        raise ValueError("Cloud pixel stride and voxel size must be positive")
    config_path = args.config.resolve()
    baseline_path = args.baselines.resolve()
    project_root = config_path.parent.parent
    config = json.loads(config_path.read_text(encoding="utf-8"))
    baseline_report = json.loads(baseline_path.read_text(encoding="utf-8"))
    sessions = config["sessions"]
    if set(sessions) != set(baseline_report["sessions"]):
        raise ValueError("Config sessions and frozen baseline report sessions differ")

    start = time.monotonic()
    args.output.mkdir(parents=True)
    manifest = {
        "created_utc": utc_now(),
        "status": "QUALITY_AUDIT_AND_SUBMAP_PROPOSAL_NOT_OPTIMIZED_NOT_VALIDATED_MAP",
        "source": str(Path(__file__).resolve()), "source_sha256": sha256_file(Path(__file__)),
        "argv": sys.argv, "config": str(config_path), "config_sha256": sha256_file(config_path),
        "baseline_report": str(baseline_path), "baseline_report_sha256": sha256_file(baseline_path),
        "thresholds": THRESHOLDS,
        "algorithms": [
            "Read-only RTAB-Map access; iOS four-channel PNG bytes reinterpreted as little-endian float32 depth.",
            "Per-frame confidence 100/50/0 ratios and valid 0.25-4.0 m confidence>=50 depth fraction.",
            "Depth neighborhood coverage, isolation, range-adaptive discontinuity fraction, and neighbor-difference p90.",
            "Long-side-640 grayscale Laplacian variance plus <=5/>=250 clipped exposure fractions.",
            "Per-frame scaled fx/fy/cx/cy jump, frozen-pose translation/rotation increment, and timestamp gap.",
            "Explicit 100-point A/B/C gate; any critical quality threshold forces C and excludes that frame from registration evidence.",
            "Hard boundaries are limited to >1 m or >75 deg pose jumps, >5 s gaps, >3 depth-pixel intrinsic jumps, missing frozen poses, or depth/calibration decode failures; sensor-only C frames are skipped without asserting tracking loss.",
            "Sequential correlation grouping by <=0.30 m, <=8 deg, <=3 s from representative; group frame weights sum to one.",
            "Greedy 3.5 m submaps bounded at 5 m, targeting 1 m overlap inside each uninterrupted A/B run; short boundary fragments retained and labeled.",
        ],
        "independence_statement": (
            "Frames in one correlation_group_id are one evidence unit. Overlapping submaps are also not "
            "independent validation regions; downstream holdout must group shared correlation IDs."
        ),
        "thread_environment": {key: os.environ.get(key) for key in (
            "OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"
        )},
        "sessions": {},
    }
    for name, entry in sessions.items():
        manifest["sessions"][name] = audit_session(
            name, entry, baseline_report["sessions"][name], args.output,
            project_root, args.write_clouds, args.cloud_pixel_stride, args.cloud_voxel,
        )
        (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    manifest["finished_utc"] = utc_now()
    manifest["runtime_seconds"] = float(time.monotonic() - start)
    manifest["all_databases_unchanged"] = all(
        session["database_unchanged"] for session in manifest["sessions"].values()
    )
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(args.output.resolve()), "runtime_seconds": manifest["runtime_seconds"],
        "all_databases_unchanged": manifest["all_databases_unchanged"],
        "sessions": {name: {
            "frames": result["frame_count"], "groups": result["correlation_group_count"],
            "submaps": result["submap_count"], "grades": result["quality_grade_counts"],
        } for name, result in manifest["sessions"].items()},
    }, indent=2), flush=True)


if __name__ == "__main__":
    main()
