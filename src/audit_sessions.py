#!/usr/bin/env python3
"""Read-only RTAB-Map session audit; exports are observations, not GNSS priors.

Reproduce with /usr/bin/python3 src/audit_sessions.py DB [DB ...]
    --output OUTPUT --sessions NAME [NAME ...].
Exactly two DBs without --sessions retain the legacy names outside/lobby;
other counts default to sanitized database stems.
Node.pose is an odometry baseline, Admin.opt_poses is a saved app result.
No claim of independently measured ground truth or actual GNSS fusion is made.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import re
import sqlite3
import struct
import sys
import zlib
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from export_rtabmap_db import calibration_from_blob, rotation_matrix_to_quaternion

SOURCE_URLS = {
    "db_serialization": "https://github.com/introlab/rtabmap/blob/master/corelib/src/DBDriverSqlite3.cpp",
    "compressed_matrix": "https://github.com/introlab/rtabmap/blob/master/corelib/src/Compression.cpp",
    "link_types": "https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Link.h",
    "parameter_semantics": "https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h",
}
LINK_TYPES = {0: "neighbor", 1: "global_closure", 2: "local_space_closure", 3: "local_time_closure",
              4: "user_closure", 5: "virtual_closure", 6: "neighbor_merged", 7: "pose_prior", 8: "landmark", 9: "gravity"}
GPS_FIELDS = ["session", "node_id", "node_stamp", "gnss_stamp", "longitude_deg", "latitude_deg",
              "altitude_raw_m", "reported_accuracy_m", "bearing_deg", "east_m", "north_m", "surface_up_m",
              "age_s", "duplicate_timestamp", "duplicate_coordinate", "previous_unique_dt_s",
              "previous_unique_jump_m", "previous_unique_speed_mps", "coarse_view_eligible",
              "quality_exclusion_reasons", "optimization_prior_applied", "optimization_prior_policy"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stats(values) -> dict:
    array = np.asarray(list(values), dtype=float)
    array = array[np.isfinite(array)]
    if not array.size:
        return {"count": 0}
    return {"count": int(array.size), "min": float(array.min()), "p05": float(np.percentile(array, 5)),
            "median": float(np.median(array)), "p90": float(np.percentile(array, 90)),
            "p95": float(np.percentile(array, 95)), "max": float(array.max()),
            "mean": float(array.mean()), "std": float(array.std())}


def decode_matrix(blob: bytes) -> np.ndarray:
    """RTAB-Map zlib stream followed by little-endian rows, cols, OpenCV type."""
    if not blob or len(blob) < 13:
        raise ValueError("compressed matrix missing/truncated")
    rows, cols, kind = struct.unpack("<iii", blob[-12:])
    types = {0: "u1", 1: "i1", 2: "<u2", 3: "<i2", 4: "<i4", 5: "<f4", 6: "<f8"}
    depth, channels = kind & 7, (kind >> 3) + 1
    if rows < 1 or cols < 1 or depth not in types or channels > 512:
        raise ValueError("invalid compressed matrix metadata")
    dtype = np.dtype(types[depth])
    raw = zlib.decompress(blob[:-12])
    if len(raw) != rows * cols * channels * dtype.itemsize:
        raise ValueError("compressed matrix payload/metadata size mismatch")
    shape = (rows, cols) if channels == 1 else (rows, cols, channels)
    return np.frombuffer(raw, dtype=dtype).reshape(shape)


def decode_pose(blob: bytes) -> np.ndarray:
    if blob is None or len(blob) != 48:
        raise ValueError("expected 48-byte float32 3x4 transform")
    pose = np.eye(4)
    pose[:3, :] = np.frombuffer(blob, "<f4").reshape(3, 4)
    return pose


def pose_quality(pose: np.ndarray) -> dict:
    finite = bool(np.isfinite(pose).all())
    if not finite:
        return {"valid": False, "finite": False, "determinant": None, "orthogonality_error": None}
    rotation = pose[:3, :3]
    determinant = float(np.linalg.det(rotation))
    orthogonality = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    return {"valid": abs(determinant - 1) < 1e-3 and orthogonality < 1e-3,
            "finite": True, "determinant": determinant, "orthogonality_error": orthogonality}


def decode_gps(blob: bytes) -> tuple:
    if blob is None or len(blob) != 48:
        raise ValueError("expected stamp,lon,lat,altitude,accuracy,bearing as 6 little-endian float64")
    return struct.unpack("<6d", blob)


def valid_gps_coordinate(gps: tuple) -> bool:
    stamp, lon, lat, *_ = gps
    return bool(np.isfinite(gps).all() and -90 <= lat <= 90 and -180 <= lon <= 180 and stamp > 0)


def find_enu_origin(paths: list[Path]) -> dict | None:
    """First decodable finite GNSS coordinate in input order, or no origin.

    This is a mathematical reference, not a surveyed point or a quality claim.
    A DB with no fixes does not prevent a later DB supplying the common origin.
    """
    for path in paths:
        with sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True) as connection:
            for node_id, blob in connection.execute("SELECT id,gps FROM Node WHERE gps IS NOT NULL ORDER BY id"):
                try:
                    gps = decode_gps(blob)
                except ValueError:
                    continue
                if valid_gps_coordinate(gps):
                    return {"latitude_deg": gps[2], "longitude_deg": gps[1], "ellipsoid_height_m": 0,
                            "source_database": str(path.resolve()), "source_node_id": node_id,
                            "source_gnss_stamp": gps[0], "selection": "first_valid_coordinate_in_input_order_not_surveyed"}
    return None


def ecef(latitude_degrees, longitude_degrees, height=0.0) -> np.ndarray:
    """Exact WGS84 geodetic-to-ECEF; height is explicitly ellipsoidal."""
    latitude, longitude = np.deg2rad(latitude_degrees), np.deg2rad(longitude_degrees)
    a, inverse_flattening = 6378137.0, 298.257223563
    flattening = 1 / inverse_flattening
    eccentricity2 = flattening * (2 - flattening)
    n = a / np.sqrt(1 - eccentricity2 * np.sin(latitude) ** 2)
    return np.stack(((n + height) * np.cos(latitude) * np.cos(longitude),
                     (n + height) * np.cos(latitude) * np.sin(longitude),
                     (n * (1 - eccentricity2) + height) * np.sin(latitude)), axis=-1)


def horizontal_enu(latitude, longitude, origin: tuple[float, float]) -> np.ndarray:
    """Project WGS84 surface positions to a common ENU; stored altitude unused.

    Database altitude datum is unspecified, so assigning it ellipsoidal height
    would be unjustified. U here is ellipsoid curvature, not measured height.
    """
    lat0, lon0 = np.deg2rad(origin)
    rotation = np.array([[-np.sin(lon0), np.cos(lon0), 0],
                         [-np.sin(lat0) * np.cos(lon0), -np.sin(lat0) * np.sin(lon0), np.cos(lat0)],
                         [np.cos(lat0) * np.cos(lon0), np.cos(lat0) * np.sin(lon0), np.sin(lat0)]])
    return (ecef(latitude, longitude) - ecef(*origin)) @ rotation.T


def connected_components(node_ids: set[int], edges: list[tuple[int, int]]) -> list[list[int]]:
    adjacency = {node_id: set() for node_id in node_ids}
    for first, second in edges:
        if first in adjacency and second in adjacency and first != second:
            adjacency[first].add(second)
            adjacency[second].add(first)
    unseen, components = set(node_ids), []
    while unseen:
        root = min(unseen)
        stack, component = [root], []
        unseen.remove(root)
        while stack:
            current = stack.pop()
            component.append(current)
            for neighbor in sorted(adjacency[current]):
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    stack.append(neighbor)
        components.append(sorted(component))
    return sorted(components, key=lambda component: (-len(component), component[0]))


def write_csv(path: Path, rows: list[dict], fields: list[str] | None = None) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields or (list(rows[0]) if rows else []))
        writer.writeheader()
        writer.writerows(rows)


def pose_rows(poses: dict[int, np.ndarray], stamps: dict[int, float]) -> list[dict]:
    result = []
    for node_id, pose in sorted(poses.items()):
        if not pose_quality(pose)["valid"]:
            continue
        result.append(dict(zip(["id", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw"],
                               [node_id, stamps[node_id], *pose[:3, 3],
                                *rotation_matrix_to_quaternion(pose[:3, :3])])) )
    return result


def gps_rows(nodes: list[tuple], session: str, origin: tuple[float, float] | None) -> tuple[list[dict], list[dict]]:
    result, errors, stamps_seen, xy_seen = [], [], set(), set()
    previous_unique = None
    for node_id, _, node_stamp, _, _, gps, *_ in nodes:
        if not gps:
            continue
        try:
            stamp, lon, lat, altitude, accuracy, bearing = decode_gps(gps)
        except ValueError as error:
            errors.append({"node_id": node_id, "error": str(error)})
            continue
        finite = bool(np.isfinite([stamp, lon, lat, altitude, accuracy, bearing]).all())
        coordinate_valid = valid_gps_coordinate((stamp, lon, lat, altitude, accuracy, bearing))
        east, north, surface_up = horizontal_enu(lat, lon, origin) if coordinate_valid and origin is not None else (math.nan,) * 3
        age = node_stamp - stamp
        duplicate_stamp, duplicate_xy = stamp in stamps_seen, (lat, lon) in xy_seen
        jump, speed, dt = math.nan, math.nan, math.nan
        if not duplicate_stamp and coordinate_valid and origin is not None:
            if previous_unique is not None:
                dt = stamp - previous_unique[0]
                jump = math.hypot(east - previous_unique[1], north - previous_unique[2])
                if dt > 0:
                    speed = jump / dt
            previous_unique = (stamp, east, north)
        reasons = []
        if origin is None:
            reasons.append("no_valid_common_enu_origin")
        if not coordinate_valid:
            reasons.append("invalid_coordinate_or_timestamp")
        if not finite or accuracy <= 0 or accuracy > 10:
            reasons.append("reported_accuracy_above_10m_or_invalid")
        if not np.isfinite(age) or age < -0.1 or age > 1.5:
            reasons.append("measurement_age_outside_minus0p1_to_1p5s")
        if duplicate_stamp:
            reasons.append("repeated_measurement_timestamp")
        if duplicate_xy:
            reasons.append("repeated_coordinate")
        if np.isfinite(speed) and speed > 3:
            reasons.append("unique_fix_speed_above_3m_per_s")
        if np.isfinite(dt) and dt <= 0:
            reasons.append("nonmonotonic_unique_gnss_timestamp")
        result.append({"session": session, "node_id": node_id, "node_stamp": node_stamp,
                       "gnss_stamp": stamp, "longitude_deg": lon, "latitude_deg": lat,
                       "altitude_raw_m": altitude, "reported_accuracy_m": accuracy, "bearing_deg": bearing,
                       "east_m": east, "north_m": north, "surface_up_m": surface_up,
                       "age_s": age, "duplicate_timestamp": duplicate_stamp, "duplicate_coordinate": duplicate_xy,
                       "previous_unique_dt_s": dt, "previous_unique_jump_m": jump, "previous_unique_speed_mps": speed,
                       "coarse_view_eligible": not reasons, "quality_exclusion_reasons": ";".join(reasons),
                       "optimization_prior_applied": False,
                       "optimization_prior_policy": "not_applied_unvalidated_sensor_uncertainty"})
        stamps_seen.add(stamp)
        xy_seen.add((lat, lon))
    return result, errors


def audit_db(path: Path, session: str, output: Path, origin: tuple[float, float] | None) -> dict:
    print(f"Auditing {session}: {path}", flush=True)
    checksum_before, stat_before = sha256(path), path.stat()
    connection = sqlite3.connect(path.resolve().as_uri() + "?mode=ro", uri=True)
    connection.execute("PRAGMA query_only=ON")
    report = {"session": session, "db_path": str(path.resolve()), "bytes": stat_before.st_size,
              "sha256_before": checksum_before, "quick_check": [r[0] for r in connection.execute("PRAGMA quick_check")],
              "schema": dict(connection.execute("SELECT name,sql FROM sqlite_master WHERE type='table' ORDER BY name")),
              "source_urls": SOURCE_URLS}
    report["table_counts"] = {name: connection.execute(f'SELECT count(*) FROM "{name}"').fetchone()[0]
                              for name in report["schema"]}
    report["stored_parameter_snapshots"] = [{"time_enter": timestamp, "parameters": dict(
        pair.split(":", 1) for pair in params.split(";") if ":" in pair)}
        for timestamp, params in connection.execute("SELECT time_enter,parameters FROM Info ORDER BY time_enter")]
    latest_parameters = report["stored_parameter_snapshots"][-1]["parameters"] if report["stored_parameter_snapshots"] else {}
    report["relevant_stored_parameters"] = {key: latest_parameters.get(key) for key in (
        "Reg/Force3DoF", "RGBD/ForceOdom3DoF", "Optimizer/Strategy", "Optimizer/Iterations",
        "Optimizer/Robust", "Optimizer/PriorsIgnored", "Optimizer/GravitySigma", "Rtabmap/LoopGPS")}
    report["force_3dof_interpretation"] = "RGBD/ForceOdom3DoF is conditional on Reg/Force3DoF=true; its true value alone does not prove z was flattened."
    nodes = connection.execute("SELECT id,map_id,stamp,pose,ground_truth_pose,gps,velocity,env_sensors FROM Node ORDER BY id").fetchall()
    stamps = {row[0]: row[2] for row in nodes}
    poses, truth, pose_errors, null_truth_ids = {}, {}, [], []
    for node_id, _, _, pose, gt, *_ in nodes:
        for kind, blob, target in (("Node.pose", pose, poses), ("ground_truth_pose", gt, truth)):
            if not blob:
                continue
            try:
                decoded = decode_pose(blob)
                if kind == "ground_truth_pose" and not np.any(decoded[:3]):
                    null_truth_ids.append(node_id)
                    continue
                quality = pose_quality(decoded)
                if not quality["valid"]:
                    pose_errors.append({"node_id": node_id, "field": kind, **quality})
                target[node_id] = decoded
            except ValueError as error:
                pose_errors.append({"node_id": node_id, "field": kind, "error": str(error)})
    (output / session).mkdir(exist_ok=True)
    write_csv(output / session / "raw_trajectory.csv", pose_rows(poses, stamps))
    positions = np.asarray([pose[:3, 3] for pose in poses.values()])
    stamp_delta = np.diff(list(stamps.values()))
    report["nodes"] = {"count": len(nodes), "map_id_counts": dict(Counter(row[1] for row in nodes)),
                       "pose_count": len(poses), "ground_truth_nonnull_blob_count": sum(bool(row[4]) for row in nodes),
                       "ground_truth_valid_pose_count": sum(pose_quality(pose)["valid"] for pose in truth.values()),
                       "ground_truth_null_transform_count": len(null_truth_ids), "ground_truth_null_transform_node_ids": null_truth_ids,
                       "pose_errors": pose_errors,
                       "raw_position_extent_xyz_m": np.ptp(positions, axis=0).tolist(),
                       "raw_path_length_m": float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum()),
                       "raw_step_m": stats(np.linalg.norm(np.diff(positions, axis=0), axis=1)),
                       "stamp_start_utc": datetime.fromtimestamp(min(stamps.values()), timezone.utc).isoformat(),
                       "stamp_end_utc": datetime.fromtimestamp(max(stamps.values()), timezone.utc).isoformat(),
                       "duration_s": max(stamps.values()) - min(stamps.values()),
                       "node_stamp_step_s": stats(stamp_delta), "nonpositive_stamp_steps": int(np.count_nonzero(stamp_delta <= 0)),
                       "timestamp_limit": "Data has no independent RGB/depth stamps; association by node id cannot verify acquisition synchronization.",
                       "ground_truth_limit": "A named ground_truth_pose field is not evidence of independent surveyed truth; no accuracy scoring against it.",
                       "imu_limit": "Gravity links are available; no standalone timestamped accelerometer/gyroscope samples in this schema."}
    admin = connection.execute("SELECT version,opt_ids,opt_poses FROM Admin").fetchone()
    app_poses = {}
    report["app_optimized_poses"] = {"db_version": admin[0], "present": bool(admin[1] and admin[2])}
    if admin[1] and admin[2]:
        try:
            ids, transforms = decode_matrix(admin[1]).ravel(), decode_matrix(admin[2]).reshape(-1, 3, 4)
            if len(ids) != len(transforms):
                raise ValueError("optimized id/pose count mismatch")
            for node_id, transform in zip(ids, transforms):
                pose = np.eye(4)
                pose[:3] = transform
                app_poses[int(node_id)] = pose
            common = sorted(set(app_poses) & set(poses))
            displacements = [np.linalg.norm(app_poses[i][:3, 3] - poses[i][:3, 3]) for i in common]
            rotation_differences = [math.degrees(math.acos(float(np.clip((np.trace(app_poses[i][:3, :3].T @ poses[i][:3, :3]) - 1) / 2, -1, 1)))) for i in common]
            report["app_optimized_poses"].update({"count": len(app_poses), "missing_node_ids": sorted(set(poses) - set(app_poses)),
                "invalid_pose_ids": [i for i, p in app_poses.items() if not pose_quality(p)["valid"]],
                "node_pose_displacement_m_no_alignment": stats(displacements), "node_pose_rotation_difference_deg_no_alignment": stats(rotation_differences),
                "first_common_node_translation_difference_m": float(displacements[0]),
                "interpretation": "Saved app optimized-pose artifact exists and differs from Node.pose; its historical optimizer execution and constraints are not fully inferable from stored settings."})
            write_csv(output / session / "app_optimized_trajectory.csv", pose_rows(app_poses, stamps))
        except (ValueError, zlib.error) as error:
            report["app_optimized_poses"]["decode_error"] = str(error)
    links, valid_neighbor, valid_all, link_errors = [], [], [], []
    for first, second, kind, transform, information in connection.execute("SELECT from_id,to_id,type,transform,information_matrix FROM Link ORDER BY type,from_id,to_id"):
        row = {"from_id": first, "to_id": second, "type": kind, "type_name": LINK_TYPES.get(kind, "unknown")}
        try:
            quality = pose_quality(decode_pose(transform))
            matrix = np.frombuffer(information, "<f8").reshape(6, 6)
            symmetric = np.isfinite(matrix).all() and np.max(np.abs(matrix - matrix.T)) < 1e-6
            eigenvalues = np.linalg.eigvalsh((matrix + matrix.T) / 2) if symmetric else np.array([math.nan])
            positive = bool(symmetric and eigenvalues.min() > 0)
            row.update({"transform_valid": quality["valid"], "rotation_determinant": quality["determinant"],
                        "rotation_orthogonality_error": quality["orthogonality_error"], "information_spd": positive,
                        "information_min_eigenvalue": float(eigenvalues.min()),
                        "information_max_eigenvalue": float(eigenvalues.max()),
                        "valid": quality["valid"] and positive})
            if row["valid"] and first != second:
                valid_all.append((first, second))
                if kind in (0, 6):
                    valid_neighbor.append((first, second))
            if not row["valid"]:
                link_errors.append(row.copy())
        except (ValueError, np.linalg.LinAlgError) as error:
            row.update({"valid": False, "error": str(error)})
            link_errors.append(row.copy())
        links.append(row)
    write_csv(output / f"{session}_link_audit.csv", links, sorted(set().union(*(row.keys() for row in links))))
    neighbor_components = connected_components(set(poses), valid_neighbor)
    all_components = connected_components(set(poses), valid_all)
    report["links"] = {"counts": dict(Counter(row["type"] for row in links)), "invalid_links": link_errors,
                       "valid_odometry_component_sizes": [len(c) for c in neighbor_components],
                       "valid_odometry_components": neighbor_components,
                       "valid_all_relative_component_sizes": [len(c) for c in all_components],
                       "valid_all_relative_components": all_components,
                       "warning": "Zero/singular transforms must be rejected before rotation projection/SVD; pose priors and gravity self-links do not connect graph components."}
    frame_rows, frame_errors, rgb_shapes, depth_shapes, confidence_shapes = [], [], Counter(), Counter(), Counter()
    confidence_values, calibration_dimensions, intrinsics, local_transforms = Counter(), Counter(), [], []
    for node_id, image, depth_blob, confidence_blob, calibration_blob in connection.execute("SELECT id,image,depth,depth_confidence,calibration FROM Data ORDER BY id"):
        frame = {"node_id": node_id, "rgb_ok": False, "depth_ok": False, "confidence_ok": False, "calibration_ok": False}
        errors = []
        try:
            rgb = cv2.imdecode(np.frombuffer(image or b"", np.uint8), cv2.IMREAD_UNCHANGED)
            if rgb is None or rgb.ndim not in (2, 3):
                raise ValueError("RGB decode failed")
            rgb_shapes[str(rgb.shape)] += 1
            frame["rgb_ok"] = True
        except (ValueError, cv2.error) as error:
            errors.append(str(error))
        depth = None
        try:
            packed = cv2.imdecode(np.frombuffer(depth_blob or b"", np.uint8), cv2.IMREAD_UNCHANGED)
            if packed is not None and packed.dtype == np.uint8 and packed.ndim == 3 and packed.shape[2] == 4:
                depth = np.ascontiguousarray(packed).view("<f4").reshape(packed.shape[:2])
            elif packed is not None and packed.dtype == np.uint16 and packed.ndim == 2:
                depth = packed.astype(float) / 1000
            else:
                raise ValueError("unsupported depth encoding or decode failure")
            depth_shapes[str(depth.shape)] += 1
            frame.update({"depth_ok": True, "depth_finite_fraction": float(np.isfinite(depth).mean()),
                          "depth_positive_fraction": float((np.isfinite(depth) & (depth > 0)).mean()),
                          "depth_0p25_to_5m_fraction": float((np.isfinite(depth) & (depth >= .25) & (depth <= 5)).mean())})
        except (ValueError, cv2.error) as error:
            errors.append(str(error))
        try:
            confidence = decode_matrix(confidence_blob)
            confidence_shapes[str(confidence.shape)] += 1
            values, counts = np.unique(confidence, return_counts=True)
            confidence_values.update(dict(zip(map(int, values), map(int, counts))))
            if depth is None or confidence.shape != depth.shape or confidence.dtype != np.uint8:
                raise ValueError("confidence/depth shape or confidence dtype mismatch")
            if not np.isin(values, [0, 50, 100]).all():
                errors.append("confidence levels differ from expected iOS 0/50/100")
            frame.update({"confidence_ok": True, "usable_depth_conf50_fraction": float((np.isfinite(depth) & (depth >= .25) & (depth <= 5) & (confidence >= 50)).mean()),
                          "usable_depth_conf100_fraction": float((np.isfinite(depth) & (depth >= .25) & (depth <= 5) & (confidence >= 100)).mean())})
        except (ValueError, TypeError, zlib.error) as error:
            errors.append(str(error))
        try:
            width, height, intrinsic, local = calibration_from_blob(calibration_blob)
            calibration_dimensions[f"{width}x{height}"] += 1
            intrinsics.append(intrinsic.ravel())
            local_transforms.append(local)
            quality = pose_quality(local)
            if not quality["valid"] or not np.isfinite(intrinsic).all() or min(width, height) <= 0 or min(intrinsic[0, 0], intrinsic[1, 1]) <= 0:
                raise ValueError("invalid calibration matrix or local transform")
            if frame["rgb_ok"] and rgb.shape[:2] != (height, width):
                errors.append("RGB resolution differs from calibration dimensions")
            frame["calibration_ok"] = True
        except (ValueError, TypeError, struct.error) as error:
            errors.append(str(error))
        frame["errors"] = ";".join(errors)
        if errors:
            frame_errors.append({"node_id": node_id, "errors": errors})
        frame_rows.append(frame)
    write_csv(output / f"{session}_frame_audit.csv", frame_rows, sorted(set().union(*(row.keys() for row in frame_rows))))
    report["frames"] = {"count": len(frame_rows), "all_frames_decoded": not frame_errors, "errors": frame_errors,
                        "rgb_shapes": dict(rgb_shapes), "depth_shapes": dict(depth_shapes),
                        "confidence_shapes": dict(confidence_shapes), "confidence_histogram_pixels": dict(confidence_values),
                        "calibration_dimensions": dict(calibration_dimensions),
                        "intrinsics_min_row_major": np.min(intrinsics, axis=0).tolist(), "intrinsics_max_row_major": np.max(intrinsics, axis=0).tolist(),
                        "local_transform_max_deviation_from_first": float(np.max(np.abs(np.asarray(local_transforms) - local_transforms[0]))),
                        "first_local_transform": local_transforms[0].tolist(),
                        "usable_depth_conf50_fraction_per_frame": stats(row.get("usable_depth_conf50_fraction", math.nan) for row in frame_rows),
                        "usable_depth_conf100_fraction_per_frame": stats(row.get("usable_depth_conf100_fraction", math.nan) for row in frame_rows)}
    gps, gps_errors = gps_rows(nodes, session, origin)
    write_csv(output / f"{session}_gnss_raw.csv", gps, GPS_FIELDS)
    filtered = [row for row in gps if row["coarse_view_eligible"]]
    write_csv(output / f"{session}_gnss_coarse_filtered.csv", filtered, GPS_FIELDS)
    report["gnss"] = {"records": len(gps), "missing_node_ids": [row[0] for row in nodes if not row[5]],
                      "decode_errors": gps_errors, "unique_timestamps": len(set(row["gnss_stamp"] for row in gps)),
                      "unique_coordinates": len(set((row["latitude_deg"], row["longitude_deg"]) for row in gps)),
                      "reported_accuracy_m": stats(row["reported_accuracy_m"] for row in gps),
                      "measurement_age_s": stats(row["age_s"] for row in gps),
                      "altitude_raw_m": stats(row["altitude_raw_m"] for row in gps),
                      "unique_fix_speed_mps": stats(row["previous_unique_speed_mps"] for row in gps),
                      "coarse_view_eligible_count": len(filtered),
                      "unique_fix_jumps_over_10m_node_ids": [row["node_id"] for row in gps if row["previous_unique_jump_m"] > 10],
                      "exclusion_counts": dict(Counter(reason for row in gps for reason in row["quality_exclusion_reasons"].split(";") if reason)),
                      "all_bearings_zero": all(row["bearing_deg"] == 0 for row in gps) if gps else None,
                      "common_enu_origin_available": origin is not None,
                      "optimization_priors_applied": 0,
                      "accuracy_limit": "Reported sensor accuracy/uncertainty, NOT measured positioning error, and not proven 1-sigma covariance.",
                      "correlation_limit": "Repeated timestamps/positions and temporally correlated observations are not independent priors.",
                      "coordinate_limit": "Common WGS84 ENU from lat/lon at zero ellipsoid height. surface_up is geometric curvature only. Raw GNSS altitude datum unspecified and never a z target.",
                      "stored_settings_limit": "Optimizer/PriorsIgnored records a configuration snapshot, not proof of all historical optimizer/GNSS behavior."}
    connection.close()
    checksum_after = sha256(path)
    report["preservation"] = {"sha256_after": checksum_after, "sha256_unchanged": checksum_after == checksum_before,
                              "mtime_ns_unchanged": path.stat().st_mtime_ns == stat_before.st_mtime_ns, "opened_read_only": True}
    (output / f"{session}_audit.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    print(f"{session}: {len(nodes)} nodes, {len(app_poses)} app poses, odometry components {[len(c) for c in neighbor_components]}, {len(frame_errors)} frame errors, {len(filtered)} GNSS coarse samples", flush=True)
    return report


def plot_overview(output: Path, sessions: list[str]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    columns = min(3, max(2, len(sessions)))
    rows = math.ceil((len(sessions) + 2) / columns)
    figure, axes = plt.subplots(rows, columns, figsize=(6.5 * columns, 5 * rows), squeeze=False)
    panels = axes.ravel()
    enu_axis, accuracy_axis = panels[len(sessions):len(sessions) + 2]
    for index, session in enumerate(sessions):
        axis = panels[index]
        for suffix, label in (("raw_trajectory", "Node.pose"), ("app_optimized_trajectory", "saved app poses")):
            path = output / session / f"{suffix}.csv"
            if not path.exists():
                continue
            with path.open() as stream:
                trajectory = list(csv.DictReader(stream))
            axis.plot([float(r["x"]) for r in trajectory], [float(r["y"]) for r in trajectory], lw=.9, label=label)
        axis.set(title=f"{session}: own session frame (not common ENU)", xlabel="x [m]", ylabel="y [m]")
        axis.axis("equal")
        with (output / f"{session}_gnss_raw.csv").open() as stream:
            gps = list(csv.DictReader(stream))
        if not gps:
            continue
        unique = [row for row in gps if row["duplicate_timestamp"] == "False"
                  and np.isfinite([float(row["east_m"]), float(row["north_m"])]).all()]
        if unique:
            enu_axis.plot([float(r["east_m"]) for r in unique], [float(r["north_m"]) for r in unique], ".-", markersize=2, lw=.5, label=session)
        start = float(gps[0]["node_stamp"])
        accuracy_axis.plot([float(r["node_stamp"]) - start for r in gps], [float(r["reported_accuracy_m"]) for r in gps], lw=.8, label=session)
    enu_axis.set(title="GNSS in common WGS84 ENU; no map alignment inferred", xlabel="east [m]", ylabel="north [m]")
    enu_axis.axis("equal")
    accuracy_axis.set(title="Sensor-reported accuracy, NOT measured error", xlabel="time from session start [s]", ylabel="reported accuracy [m]")
    for axis in panels[:len(sessions) + 2]:
        axis.grid(alpha=.25)
        if axis.get_legend_handles_labels()[0]:
            axis.legend()
    for axis in (enu_axis, accuracy_axis):
        if not axis.lines:
            axis.text(.5, .5, "No usable GNSS observations", ha="center", va="center", transform=axis.transAxes)
    for axis in panels[len(sessions) + 2:]:
        axis.set_visible(False)
    figure.tight_layout()
    figure.savefig(output / "session_and_gnss_overview.png", dpi=160)
    plt.close(figure)


def session_names(paths: list[Path], requested: list[str] | None) -> list[str]:
    if requested is None:
        names = ["outside", "lobby"] if len(paths) == 2 else [
            re.sub(r"[^A-Za-z0-9_-]", "_", path.stem).strip("_-") for path in paths]
    else:
        names = requested
    if len(names) != len(paths):
        raise ValueError("--sessions must provide exactly one name per database")
    if any(not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", name) for name in names):
        raise ValueError("session names must be 1-80 ASCII letters/digits/underscore/hyphen, starting with a letter or digit")
    if len(set(names)) != len(names):
        raise ValueError("session names must be unique to prevent output collisions")
    return names


def parse_arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dbs", nargs="+", type=Path, help="databases in audit order; list before --sessions")
    parser.add_argument("--sessions", nargs="+", help="one safe unique output name per database; put after database arguments")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--skip-plots", action="store_true")
    args = parser.parse_args(argv)
    try:
        args.sessions = session_names(args.dbs, args.sessions)
    except ValueError as error:
        parser.error(str(error))
    return args


def main() -> None:
    args = parse_arguments()
    args.output.mkdir(parents=True, exist_ok=True)
    origin_metadata = find_enu_origin(args.dbs)
    origin = (origin_metadata["latitude_deg"], origin_metadata["longitude_deg"]) if origin_metadata else None
    metadata = {"created_utc": datetime.now(timezone.utc).isoformat(), "command": sys.argv,
                "audit_script_sha256": sha256(Path(__file__)),
                "versions": {"python": platform.python_version(), "numpy": np.__version__, "opencv": cv2.__version__, "sqlite": sqlite3.sqlite_version},
                "sessions": [{"name": name, "database": str(path.resolve())} for name, path in zip(args.sessions, args.dbs)],
                "common_enu_origin": origin_metadata,
                "common_enu_status": "available" if origin_metadata else "unavailable_no_valid_gnss_coordinates",
                "gnss_coarse_view_policy": {"maximum_reported_accuracy_m": 10, "age_s": [-.1, 1.5], "maximum_unique_speed_mps": 3,
                    "reject_repeated_timestamp": True, "reject_repeated_coordinate": True, "used_as_optimization_prior": False},
                "source_urls": SOURCE_URLS}
    (args.output / "audit_manifest.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    for session, path in zip(args.sessions, args.dbs):
        audit_db(path, session, args.output, origin)
    if not args.skip_plots:
        plot_overview(args.output, args.sessions)


if __name__ == "__main__":
    main()
