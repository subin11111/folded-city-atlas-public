#!/usr/bin/env python3
"""Export a colored point cloud and trajectory from an RTAB-Map SQLite DB.

The input database is opened read-only. iOS/ARKit float depth is stored as a
four-channel PNG; reinterpreting the four bytes as little-endian float32
restores the metric depth image.
"""

from __future__ import annotations

import argparse
import csv
import json
import sqlite3
import struct
import zlib
from pathlib import Path

import cv2
import numpy as np


def transform_from_blob(blob: bytes) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :4] = np.frombuffer(blob, dtype="<f4", count=12).reshape(3, 4)
    return transform


def calibration_from_blob(blob: bytes) -> tuple[int, int, np.ndarray, np.ndarray]:
    if len(blob) < 164:
        raise ValueError(f"Unexpected calibration blob length: {len(blob)}")
    width, height = struct.unpack_from("<II", blob, 16)
    intrinsic = np.frombuffer(blob, dtype="<f8", count=9, offset=44).reshape(3, 3)
    local_transform = np.eye(4, dtype=np.float64)
    local_transform[:3, :4] = np.frombuffer(
        blob, dtype="<f4", count=12, offset=116
    ).reshape(3, 4)
    return width, height, intrinsic, local_transform


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif axis == 1:
            scale = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
    return float(qx), float(qy), float(qz), float(qw)


def quaternion_to_rotation(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def load_pose_csv(path: Path) -> dict[int, np.ndarray]:
    poses: dict[int, np.ndarray] = {}
    with path.open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            pose = np.eye(4, dtype=np.float64)
            pose[:3, :3] = quaternion_to_rotation(
                float(row["qx"]),
                float(row["qy"]),
                float(row["qz"]),
                float(row["qw"]),
            )
            pose[:3, 3] = [float(row["x"]), float(row["y"]), float(row["z"])]
            poses[int(row["id"])] = pose
    return poses


def load_local_correction(path: Path) -> np.ndarray:
    payload = json.loads(path.read_text(encoding="utf-8"))
    rotation = np.asarray(payload["rotation_matrix"], dtype=np.float64)
    if rotation.shape != (3, 3):
        raise ValueError("Local correction rotation_matrix must be 3x3")
    correction = np.eye(4, dtype=np.float64)
    correction[:3, :3] = rotation
    return correction


def voxel_reduce(
    points: np.ndarray,
    colors: np.ndarray,
    voxel_size: float,
    weights: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if not len(points):
        return points, colors, np.empty((0,), dtype=np.float64)
    if weights is None:
        weights = np.ones((len(points),), dtype=np.float64)
    keys = np.floor(points / voxel_size).astype(np.int32)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    count = np.bincount(inverse, weights=weights)
    point_sums = np.column_stack(
        [np.bincount(inverse, weights=points[:, index] * weights) for index in range(3)]
    )
    color_sums = np.column_stack(
        [np.bincount(inverse, weights=colors[:, index] * weights) for index in range(3)]
    )
    reduced_points = (point_sums / count[:, None]).astype(np.float32)
    reduced_colors = np.clip(np.rint(color_sums / count[:, None]), 0, 255).astype(np.uint8)
    return reduced_points, reduced_colors, count


def write_binary_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    vertex_dtype = np.dtype(
        [
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
        align=False,
    )
    vertices = np.empty((len(points),), dtype=vertex_dtype)
    vertices["x"], vertices["y"], vertices["z"] = points.T
    vertices["red"], vertices["green"], vertices["blue"] = colors.T
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        "comment Folded City Atlas RTAB-Map export\n"
        f"element vertex {len(points)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    ).encode("ascii")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as stream:
        stream.write(header)
        stream.write(vertices.tobytes())


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("db", type=Path)
    parser.add_argument("--cloud", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--frame-stride", type=int, default=2)
    parser.add_argument("--pixel-stride", type=int, default=1)
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--min-depth", type=float, default=0.20)
    parser.add_argument("--max-depth", type=float, default=5.0)
    parser.add_argument("--min-confidence", type=int, default=50)
    parser.add_argument("--batch-frames", type=int, default=20)
    parser.add_argument("--mask-dir", type=Path, help="RGB-size PNG masks by node id; nonzero pixels excluded")
    parser.add_argument("--missing-pose-policy", choices=["error", "skip"], default="error",
                        help="Use skip only for an explicitly incomplete reference trajectory")
    parser.add_argument(
        "--pose-csv",
        type=Path,
        help="Optional pose CSV whose poses replace Node.pose while rebuilding the cloud",
    )
    parser.add_argument(
        "--local-correction-json",
        type=Path,
        help="Optional fixed node-local rotation correction estimated from floor planes",
    )
    args = parser.parse_args()

    db_uri = f"file:{args.db.resolve()}?mode=ro"
    connection = sqlite3.connect(db_uri, uri=True)
    query = """
        SELECT n.id, n.stamp, n.pose, d.image, d.depth,
               d.depth_confidence, d.calibration
        FROM Node AS n
        JOIN Data AS d ON d.id = n.id
        WHERE n.pose IS NOT NULL
          AND d.image IS NOT NULL
          AND d.depth IS NOT NULL
          AND d.calibration IS NOT NULL
        ORDER BY n.id
    """
    rows = connection.execute(query).fetchall()
    connection.close()
    pose_overrides = load_pose_csv(args.pose_csv) if args.pose_csv else {}
    if args.pose_csv and not pose_overrides:
        raise ValueError("Pose CSV is empty")
    local_correction = (
        load_local_correction(args.local_correction_json)
        if args.local_correction_json
        else np.eye(4, dtype=np.float64)
    )

    missing_pose_ids = [row[0] for row in rows if pose_overrides and row[0] not in pose_overrides]
    if missing_pose_ids and args.missing_pose_policy == "error":
        raise ValueError(
            f"Pose CSV is missing {len(missing_pose_ids)} node ids; first missing ids: "
            f"{missing_pose_ids[:10]}"
        )
    if missing_pose_ids:
        rows = [row for row in rows if row[0] in pose_overrides]
    selected_rows = rows[:: max(1, args.frame_stride)]

    trajectory_rows: list[list[float | int]] = []
    for node_id, stamp, pose_blob, *_ in rows:
        pose = pose_overrides.get(node_id, transform_from_blob(pose_blob))
        qx, qy, qz, qw = rotation_matrix_to_quaternion(pose[:3, :3])
        trajectory_rows.append(
            [
                node_id,
                float(stamp or 0.0),
                *pose[:3, 3].tolist(),
                qx,
                qy,
                qz,
                qw,
            ]
        )

    args.trajectory.parent.mkdir(parents=True, exist_ok=True)
    with args.trajectory.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["id", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writerows(trajectory_rows)

    batch_points: list[np.ndarray] = []
    batch_colors: list[np.ndarray] = []
    reduced_points: list[np.ndarray] = []
    reduced_colors: list[np.ndarray] = []
    reduced_counts: list[np.ndarray] = []
    decoded_frames = 0
    valid_input_points = 0
    masked_valid_points = 0

    def flush_batch() -> None:
        nonlocal batch_points, batch_colors
        if not batch_points:
            return
        points = np.concatenate(batch_points, axis=0)
        colors = np.concatenate(batch_colors, axis=0)
        points, colors, counts = voxel_reduce(points, colors, args.voxel)
        reduced_points.append(points)
        reduced_colors.append(colors)
        reduced_counts.append(counts)
        batch_points, batch_colors = [], []

    for frame_index, row in enumerate(selected_rows, start=1):
        node_id, _stamp, pose_blob, image_blob, depth_blob, confidence_blob, calibration_blob = row
        color_bgr = cv2.imdecode(np.frombuffer(image_blob, dtype=np.uint8), cv2.IMREAD_COLOR)
        depth_bytes = cv2.imdecode(np.frombuffer(depth_blob, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        if color_bgr is None or depth_bytes is None or depth_bytes.ndim != 3 or depth_bytes.shape[2] != 4:
            print(f"Skipping node {node_id}: unsupported image/depth encoding", flush=True)
            continue

        depth_bytes = np.ascontiguousarray(depth_bytes)
        height, width = depth_bytes.shape[:2]
        depth = depth_bytes.view("<f4").reshape(height, width)
        confidence_raw = zlib.decompress(confidence_blob) if confidence_blob else b""
        if len(confidence_raw) == height * width:
            confidence = np.frombuffer(confidence_raw, dtype=np.uint8).reshape(height, width)
        else:
            confidence = np.full((height, width), 100, dtype=np.uint8)

        calibration_width, calibration_height, intrinsic, local_transform = calibration_from_blob(calibration_blob)
        scale_x = width / float(calibration_width)
        scale_y = height / float(calibration_height)
        fx, fy = intrinsic[0, 0] * scale_x, intrinsic[1, 1] * scale_y
        cx, cy = intrinsic[0, 2] * scale_x, intrinsic[1, 2] * scale_y

        pixel_stride = max(1, args.pixel_stride)
        vv, uu = np.mgrid[0:height:pixel_stride, 0:width:pixel_stride]
        sampled_depth = depth[::pixel_stride, ::pixel_stride]
        sampled_confidence = confidence[::pixel_stride, ::pixel_stride]
        valid = (
            np.isfinite(sampled_depth)
            & (sampled_depth >= args.min_depth)
            & (sampled_depth <= args.max_depth)
            & (sampled_confidence >= args.min_confidence)
        )
        if args.mask_dir:
            mask_path = args.mask_dir / f"{node_id}.png"
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if mask is None or mask.shape != color_bgr.shape[:2]:
                raise ValueError(f"Missing or wrong-sized mask for node {node_id}: {mask_path}")
            excluded = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST)[::pixel_stride, ::pixel_stride] > 0
            masked_valid_points += int(np.count_nonzero(valid & excluded))
            valid &= ~excluded
        if not np.any(valid):
            continue

        z = sampled_depth[valid].astype(np.float64)
        x = (uu[valid] - cx) * z / fx
        y = (vv[valid] - cy) * z / fy
        camera_points = np.column_stack((x, y, z, np.ones_like(z)))
        map_transform = (
            pose_overrides.get(node_id, transform_from_blob(pose_blob))
            @ local_correction
            @ local_transform
        )
        map_points = (map_transform @ camera_points.T).T[:, :3].astype(np.float32)

        resized_bgr = cv2.resize(color_bgr, (width, height), interpolation=cv2.INTER_AREA)
        resized_rgb = cv2.cvtColor(resized_bgr, cv2.COLOR_BGR2RGB)
        colors = resized_rgb[::pixel_stride, ::pixel_stride][valid]

        batch_points.append(map_points)
        batch_colors.append(colors)
        decoded_frames += 1
        valid_input_points += len(map_points)
        if len(batch_points) >= args.batch_frames:
            flush_batch()
        if frame_index % 25 == 0 or frame_index == len(selected_rows):
            print(
                f"Decoded {frame_index}/{len(selected_rows)} selected frames; "
                f"valid depth points={valid_input_points:,}",
                flush=True,
            )

    flush_batch()
    if not reduced_points:
        raise ValueError("No valid points survived the export filters")
    points = np.concatenate(reduced_points, axis=0)
    colors = np.concatenate(reduced_colors, axis=0)
    counts = np.concatenate(reduced_counts, axis=0)
    points, colors, counts = voxel_reduce(points, colors, args.voxel, counts)
    write_binary_ply(args.cloud, points, colors)

    trajectory_xyz = np.asarray([row[2:5] for row in trajectory_rows], dtype=np.float64)
    summary = {
        "source_database": str(args.db.resolve()),
        "database_open_mode": "read-only",
        "pose_source": str(args.pose_csv.resolve()) if args.pose_csv else "database Node.pose",
        "local_correction_source": (
            str(args.local_correction_json.resolve())
            if args.local_correction_json
            else "identity"
        ),
        "total_pose_frames": len(rows),
        "missing_pose_policy": args.missing_pose_policy,
        "skipped_missing_pose_ids": missing_pose_ids,
        "mask_directory": str(args.mask_dir.resolve()) if args.mask_dir else None,
        "masked_valid_depth_points": masked_valid_points,
        "selected_depth_frames": len(selected_rows),
        "decoded_depth_frames": decoded_frames,
        "frame_stride": args.frame_stride,
        "pixel_stride": args.pixel_stride,
        "confidence_threshold": args.min_confidence,
        "depth_range_m": [args.min_depth, args.max_depth],
        "voxel_size_m": args.voxel,
        "valid_input_points": valid_input_points,
        "output_points": int(len(points)),
        "cloud_bounds_min_m": points.min(axis=0).astype(float).tolist(),
        "cloud_bounds_max_m": points.max(axis=0).astype(float).tolist(),
        "cloud_extent_m": np.ptp(points, axis=0).astype(float).tolist(),
        "trajectory_length_m": float(np.linalg.norm(np.diff(trajectory_xyz, axis=0), axis=1).sum()),
        "trajectory_extent_m": np.ptp(trajectory_xyz, axis=0).astype(float).tolist(),
    }
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
