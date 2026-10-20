#!/usr/bin/env python3
"""Split a colored PLY into dominant ground-plane and non-ground clouds."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from export_rtabmap_db import write_binary_ply


PLY_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "red", "green", "blue"],
        "formats": ["<f4", "<f4", "<f4", "u1", "u1", "u1"],
        "offsets": [0, 4, 8, 12, 13, 14],
        "itemsize": 15,
    }
)


def read_binary_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open("rb") as stream:
        vertex_count = None
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PLY header ended unexpectedly")
            decoded = line.decode("ascii").strip()
            if decoded.startswith("format ") and decoded != "format binary_little_endian 1.0":
                raise ValueError(f"Unsupported PLY format: {decoded}")
            if decoded.startswith("element vertex "):
                vertex_count = int(decoded.split()[-1])
            if decoded == "end_header":
                break
        if vertex_count is None:
            raise ValueError("PLY vertex count is missing")
        cloud = np.fromfile(stream, dtype=PLY_DTYPE, count=vertex_count)
    points = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float64)
    colors = np.column_stack((cloud["red"], cloud["green"], cloud["blue"])).astype(np.uint8)
    return points, colors


def fit_dominant_horizontal_plane(
    points: np.ndarray,
    threshold: float,
    iterations: int,
    max_tilt_degrees: float,
    sample_limit: int,
    seed: int,
) -> tuple[np.ndarray, float]:
    rng = np.random.default_rng(seed)
    if len(points) > sample_limit:
        sample = points[rng.choice(len(points), sample_limit, replace=False)]
    else:
        sample = points

    min_vertical_component = np.cos(np.deg2rad(max_tilt_degrees))
    best_count = -1
    best_normal = None
    best_offset = None

    for _ in range(iterations):
        p0, p1, p2 = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        if normal[2] < 0:
            normal *= -1.0
        if normal[2] < min_vertical_component:
            continue
        offset = -float(np.dot(normal, p0))
        count = int(np.count_nonzero(np.abs(sample @ normal + offset) <= threshold))
        if count > best_count:
            best_count = count
            best_normal = normal.copy()
            best_offset = offset

    if best_normal is None or best_offset is None:
        raise RuntimeError("No near-horizontal RANSAC plane was found")

    normal, offset = best_normal, best_offset
    for _ in range(3):
        inliers = np.abs(points @ normal + offset) <= threshold
        inlier_points = points[inliers]
        if len(inlier_points) < 3:
            break
        # z = ax + by + c is stable because the plane is constrained near horizontal.
        design = np.column_stack((inlier_points[:, 0], inlier_points[:, 1], np.ones(len(inlier_points))))
        a, b, c = np.linalg.lstsq(design, inlier_points[:, 2], rcond=None)[0]
        normal = np.array([-a, -b, 1.0], dtype=np.float64)
        normal /= np.linalg.norm(normal)
        offset = -c / np.sqrt(a * a + b * b + 1.0)

    return normal, float(offset)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("--ground", type=Path, required=True)
    parser.add_argument("--non-ground", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--threshold", type=float, default=0.06)
    parser.add_argument("--iterations", type=int, default=600)
    parser.add_argument("--max-tilt", type=float, default=15.0)
    parser.add_argument("--sample-limit", type=int, default=100000)
    parser.add_argument("--seed", type=int, default=20261007)
    args = parser.parse_args()

    points, colors = read_binary_ply(args.input)
    normal, offset = fit_dominant_horizontal_plane(
        points,
        threshold=args.threshold,
        iterations=args.iterations,
        max_tilt_degrees=args.max_tilt,
        sample_limit=args.sample_limit,
        seed=args.seed,
    )
    distance = np.abs(points @ normal + offset)
    ground_mask = distance <= args.threshold

    ground_points, ground_colors = points[ground_mask], colors[ground_mask]
    non_ground_points, non_ground_colors = points[~ground_mask], colors[~ground_mask]
    write_binary_ply(args.ground, ground_points.astype(np.float32), ground_colors)
    write_binary_ply(args.non_ground, non_ground_points.astype(np.float32), non_ground_colors)

    tilt = float(np.rad2deg(np.arccos(np.clip(normal[2], -1.0, 1.0))))
    report = {
        "input": str(args.input.resolve()),
        "method": "gravity-constrained dominant-plane RANSAC",
        "distance_threshold_m": args.threshold,
        "max_plane_tilt_degrees": args.max_tilt,
        "iterations": args.iterations,
        "plane": {
            "normal": normal.astype(float).tolist(),
            "offset": offset,
            "equation": "normal_x*x + normal_y*y + normal_z*z + offset = 0",
            "tilt_degrees": tilt,
            "height_at_origin_m": float(-offset / normal[2]),
        },
        "input_points": int(len(points)),
        "ground_points": int(len(ground_points)),
        "non_ground_points": int(len(non_ground_points)),
        "ground_ratio": float(np.mean(ground_mask)),
        "note": "Only observed points inside the plane distance threshold were removed; no unobserved area was filled.",
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
