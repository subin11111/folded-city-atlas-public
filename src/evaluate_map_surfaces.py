#!/usr/bin/env python3
"""Read-only, finite-patch surface diagnostics; never flatten or optimize a map.

Plane extraction uses confidence-filtered local depth, training-only normals and
RANSAC, finite-support checks and spatially withheld depth samples. A horizontal
patch is NOT necessarily a floor: desks, stair treads, ramps and other surfaces
remain possible. Temporal height bands are motion proxies, not semantic floors.

Validated stored loop measurements establish tentative surface correspondences,
independently of the trajectory being evaluated. These correspondences and the
withheld points are not independent ground truth. Signed plane residuals are
computed on the same local observations for every trajectory. No actual floor
height, building level, staircase identity, or global accuracy is asserted.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import sqlite3
import sys
import time

import cv2
import numpy as np
import scipy
from scipy.spatial import cKDTree

from export_rtabmap_db import load_pose_csv
from optimize_session import (Clouds, apply, checked_transform, inverse,
                              sha256_file)


@dataclass(frozen=True)
class Settings:
    frame_stride: int = 5
    normal_neighbors: int = 16
    normal_radius_m: float = .30
    normal_planarity_max: float = .08
    floor_tilt_degrees: float = 12.
    wall_tilt_tolerance_degrees: float = 15.
    normal_agreement_degrees: float = 18.
    ransac_iterations: int = 160
    plane_threshold_m: float = .03
    min_training_inliers: int = 40
    min_test_inliers: int = 12
    min_tangent_extent_m: float = .18
    min_patch_area_m2: float = .08
    max_training_rms_m: float = .02
    max_test_rms_m: float = .025
    min_test_inlier_fraction: float = .65
    test_support_radius_m: float = .15
    max_planes_per_kind: int = 3
    correspondence_plane_distance_m: float = .08
    correspondence_normal_degrees: float = 12.
    correspondence_support_radius_m: float = .20
    correspondence_min_support_fraction: float = .25
    temporal_window_s: float = 15.
    temporal_height_change_m: float = .35
    seed: int = 4217


def stats(values):
    values = np.asarray(list(values), dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"count": 0}
    return {"count": len(values), "median": float(np.median(values)),
            "p10": float(np.percentile(values, 10)), "p90": float(np.percentile(values, 90)),
            "min": float(values.min()), "max": float(values.max())}


def write_csv(path, rows):
    fields = sorted({key for row in rows for key in row}) or ["no_rows"]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fields)
        writer.writeheader()
        writer.writerows(rows)


def point_normals(points, settings):
    """PCA normals from training points only; reject sparse/nonplanar neighborhoods."""
    if len(points) < settings.normal_neighbors:
        return np.zeros_like(points), np.zeros(len(points), dtype=bool)
    distances, indices = cKDTree(points).query(points, k=settings.normal_neighbors, workers=1)
    neighborhoods = points[indices]
    centered = neighborhoods - neighborhoods.mean(axis=1, keepdims=True)
    covariance = np.einsum("nki,nkj->nij", centered, centered) / settings.normal_neighbors
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    normals = eigenvectors[:, :, 0]
    valid = ((distances[:, -1] <= settings.normal_radius_m)
             & (eigenvalues[:, 1] > 1e-5)
             & (eigenvalues[:, 0] / np.maximum(eigenvalues.sum(axis=1), 1e-12)
                <= settings.normal_planarity_max))
    return normals, valid


def normal_kind(normal, kind, settings):
    z = abs(float(normal[2]))
    return (z >= np.cos(np.radians(settings.floor_tilt_degrees)) if kind == "horizontal"
            else z <= np.sin(np.radians(settings.wall_tilt_tolerance_degrees)))


def svd_plane(points):
    center = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - center, full_matrices=False)
    normal = vt[-1]
    if normal[np.argmax(np.abs(normal))] < 0:
        normal = -normal
    return normal, -float(normal @ center), center, vt[:2]


def fit_planes(train, test, kind, settings, seed):
    """Fit planes in a gravity-aligned camera-centered frame, not world XY heights."""
    normals, normal_valid = point_normals(train, settings)
    z = np.abs(normals[:, 2])
    if kind == "horizontal":
        candidates = normal_valid & (z >= np.cos(np.radians(settings.floor_tilt_degrees)))
        # Exclude ceilings; retain separate below-camera treads, not one height band.
        candidates &= (train[:, 2] < -.4) & (train[:, 2] > -2.5)
    else:
        candidates = normal_valid & (z <= np.sin(np.radians(settings.wall_tilt_tolerance_degrees)))
    remaining = np.flatnonzero(candidates)
    rng = np.random.default_rng(seed)
    accepted, diagnostics = [], []
    cosine = np.cos(np.radians(settings.normal_agreement_degrees))
    for plane_index in range(settings.max_planes_per_kind):
        if len(remaining) < settings.min_training_inliers:
            break
        points, pnormals = train[remaining], normals[remaining]
        best = np.zeros(len(points), dtype=bool)
        for _ in range(settings.ransac_iterations):
            three = points[rng.choice(len(points), 3, replace=False)]
            normal = np.cross(three[1] - three[0], three[2] - three[0])
            magnitude = np.linalg.norm(normal)
            if magnitude < 1e-6:
                continue
            normal /= magnitude
            if not normal_kind(normal, kind, settings):
                continue
            selected = ((np.abs((points - three[0]) @ normal) <= settings.plane_threshold_m)
                        & (np.abs(pnormals @ normal) >= cosine))
            if selected.sum() > best.sum():
                best = selected
        if best.sum() < settings.min_training_inliers:
            break
        for _ in range(3):
            normal, offset, center, basis = svd_plane(points[best])
            updated = ((np.abs(points @ normal + offset) <= settings.plane_threshold_m)
                       & (np.abs(pnormals @ normal) >= cosine))
            if updated.sum() < settings.min_training_inliers:
                break
            best = updated
        normal, offset, center, basis = svd_plane(points[best])
        selected = points[best]
        remaining = remaining[~best]
        residuals = selected @ normal + offset
        tangent = (selected - center) @ basis.T
        extents = np.percentile(tangent, 90, axis=0) - np.percentile(tangent, 10, axis=0)
        # Only nearby finite-patch held-out points are eligible for this local test.
        # The .15 m support gate is wider than the .03 m inlier test. It is not a
        # claim about all points in a frame, nor fully independent sensor evidence.
        test_near = (cKDTree(selected).query(test, workers=1)[0] <= settings.test_support_radius_m
                     if len(test) else np.zeros(0, dtype=bool))
        testing = test[test_near]
        test_residuals = testing @ normal + offset
        test_inliers = np.abs(test_residuals) <= settings.plane_threshold_m
        training_rms = float(np.sqrt(np.mean(residuals**2)))
        test_rms = (float(np.sqrt(np.mean(test_residuals[test_inliers]**2)))
                    if test_inliers.any() else None)
        fraction = float(test_inliers.mean()) if len(test_inliers) else 0.
        reasons = []
        if not normal_kind(normal, kind, settings): reasons.append("normal_orientation")
        if extents.min() < settings.min_tangent_extent_m: reasons.append("narrow_support")
        if extents.prod() < settings.min_patch_area_m2: reasons.append("small_support")
        if training_rms > settings.max_training_rms_m: reasons.append("training_rms")
        if test_inliers.sum() < settings.min_test_inliers: reasons.append("too_few_heldout_inliers")
        if fraction < settings.min_test_inlier_fraction: reasons.append("heldout_inlier_fraction")
        if test_rms is None or test_rms > settings.max_test_rms_m: reasons.append("heldout_rms")
        row = {"kind": kind, "plane_index": plane_index, "accepted": not reasons,
               "rejection_reasons": ";".join(reasons), "candidate_count": len(points),
               "training_inliers": len(selected), "heldout_near_patch": len(testing),
               "heldout_inliers": int(test_inliers.sum()), "heldout_inlier_fraction": fraction,
               "training_rms_m": training_rms, "heldout_inlier_rms_m": test_rms,
               "heldout_all_near_patch_abs_p90_m": (float(np.percentile(np.abs(test_residuals), 90))
                                                     if len(test_residuals) else None),
               "extent_1_m": float(extents[0]), "extent_2_m": float(extents[1]),
               "normal_tilt_from_up_degrees": float(np.degrees(np.arccos(abs(normal[2]))))}
        diagnostics.append(row)
        if not reasons:
            accepted.append({"kind": kind, "plane_index": plane_index, "normal": normal,
                             "center": center, "training": selected,
                             "testing": testing[test_inliers], "diagnostic": row})
    return accepted, diagnostics


def anchor_align(raw, other):
    ids = sorted(set(raw) & set(other))
    if not ids:
        raise ValueError("No common node for gauge alignment")
    anchor = ids[0]
    gauge = raw[anchor] @ inverse(other[anchor])
    return {node: gauge @ pose for node, pose in other.items()}, anchor, gauge


def temporal_bands(node_ids, stamps, poses, settings):
    """Raw-trajectory height-change bins; never infer floor/stair identity."""
    times = np.array([stamps[i] for i in node_ids])
    heights = np.array([poses[i][2, 3] for i in node_ids])
    delta = []
    for j, t in enumerate(times):
        before = heights[(times >= t - settings.temporal_window_s) & (times <= t)]
        after = heights[(times >= t) & (times <= t + settings.temporal_window_s)]
        delta.append(float(np.median(after) - np.median(before)))
    labels = np.where(np.asarray(delta) > settings.temporal_height_change_m, "height_increase",
                      np.where(np.asarray(delta) < -settings.temporal_height_change_m,
                               "height_decrease", "locally_stable_height"))
    rows, bands = [], []
    band = -1
    previous = None
    for node, stamp, height, change, label in zip(node_ids, times, heights, delta, labels):
        if label != previous:
            band += 1
            bands.append({"band_id": band, "proxy_label": str(label), "node_ids": []})
        bands[-1]["node_ids"].append(node)
        rows.append({"node_id": node, "stamp": float(stamp), "band_id": band,
                     "proxy_label": str(label), "raw_camera_z_m": float(height),
                     "local_median_height_change_m": change})
        previous = label
    return rows, bands


def patch_correspondence(a, b, measurement, settings):
    """Candidate pair under a stored loop; no trajectory-derived correspondence."""
    if a["kind"] != b["kind"]:
        return None
    nb = measurement[:3, :3] @ b["normal"]
    angle = float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a["normal"], nb)),
                                       abs(a["normal"] @ nb))))
    moved = apply(b["training"], measurement)
    normal_distance = float(np.median(np.abs((moved - a["center"]) @ a["normal"])))
    distances = cKDTree(a["training"]).query(moved, workers=1)[0]
    reverse = cKDTree(moved).query(a["training"], workers=1)[0]
    support = min(float(np.mean(distances <= settings.correspondence_support_radius_m)),
                  float(np.mean(reverse <= settings.correspondence_support_radius_m)))
    if (angle > settings.correspondence_normal_degrees
            or normal_distance > settings.correspondence_plane_distance_m
            or support < settings.correspondence_min_support_fraction):
        return None
    return {"measurement_normal_degrees": angle, "measurement_plane_abs_m": normal_distance,
            "measurement_min_patch_support_fraction": support}


def pair_metrics(a, b, pose_a, pose_b):
    # Relative frame computation is invariant under an arbitrary shared SE(3) gauge.
    transform = inverse(pose_a) @ pose_b
    moved = apply(b["testing"], transform)
    forward = (moved - a["center"]) @ a["normal"]
    reverse = (apply(a["testing"], inverse(transform)) - b["center"]) @ b["normal"]
    residuals = np.r_[forward, reverse]
    moved_normal = transform[:3, :3] @ b["normal"]
    angle = np.degrees(np.arctan2(np.linalg.norm(np.cross(a["normal"], moved_normal)),
                                abs(a["normal"] @ moved_normal)))
    distances = cKDTree(a["training"]).query(moved, workers=1)[0]
    return {"symmetric_plane_abs_median_m": float(np.median(np.abs(residuals))),
            "symmetric_plane_rms_m": float(np.sqrt(np.mean(residuals**2))),
            "symmetric_plane_abs_p90_m": float(np.percentile(np.abs(residuals), 90)),
            "forward_signed_median_m": float(np.median(forward)),
            "normal_difference_degrees": float(angle),
            "forward_heldout_within_20cm_patch_fraction": float(np.mean(distances < .20)),
            "heldout_points": len(residuals)}


def parse_overrides(values):
    result = {}
    for value in values:
        parts = value.split(":", 2)
        if len(parts) != 3 or not all(re.fullmatch(r"[a-z][a-z0-9_]*", x) for x in parts[:2]):
            raise ValueError("--trajectory requires SESSION:LABEL:CSV with safe names")
        session, label, path = parts
        if label in {"raw", "independent", "app"} or (session, label) in result:
            raise ValueError("Trajectory labels must be unique and not replace raw/independent/app")
        result[session, label] = Path(path).resolve()
    return result


def evaluate_session(name, database, run, output, settings, overrides):
    started = time.monotonic()
    before_hash = sha256_file(database)
    directory = output / name
    directory.mkdir(parents=True, exist_ok=True)
    source_paths = {"raw": run / name / "raw_main_trajectory.csv",
                    "independent": run / name / "optimized_main_trajectory.csv",
                    "app": run / "audit" / name / "app_optimized_trajectory.csv"}
    source_paths.update({label: path for (session, label), path in overrides.items() if session == name})
    trajectories = {label: load_pose_csv(path) for label, path in source_paths.items()}
    raw = trajectories["raw"]
    all_ids = sorted(raw)
    if not all_ids:
        raise ValueError(f"Empty main component: {name}")
    alignment = {}
    for label, poses in list(trajectories.items()):
        aligned, anchor, gauge = anchor_align(raw, poses)
        trajectories[label] = aligned
        alignment[label] = {"anchor_id": anchor, "left_gauge_transform": gauge.tolist(),
                            "missing_main_ids": sorted(set(raw) - set(poses)),
                            "extra_ids_not_used": sorted(set(poses) - set(raw))}
    connection = sqlite3.connect(f"file:{database.resolve()}?mode=ro", uri=True)
    stamps = {node: stamp for node, stamp in connection.execute("SELECT id,stamp FROM Node")}
    validation_path = run / name / "loop_validation.csv"
    with validation_path.open() as stream:
        loops = [row for row in csv.DictReader(stream) if row["accepted"] == "True"]
    node_ids = set(all_ids[::settings.frame_stride]) | {all_ids[-1]}
    node_ids |= {int(row[key]) for row in loops for key in ("from_id", "to_id")}
    node_ids &= set(raw)
    clouds = Clouds(connection, None)
    patches, diagnostics, surfaces = {}, [], []
    bands_rows, bands = temporal_bands(all_ids, stamps, raw, settings)
    band_lookup = {row["node_id"]: row for row in bands_rows}
    for count, node in enumerate(sorted(node_ids), 1):
        train, test = clouds.get(node)
        rotation = raw[node][:3, :3]
        # Rotation only centers each frame at its camera/node origin; avoids plane
        # extrapolation at world XY origin or solving z = -(nx*x+ny*y+d)/nz.
        train_g, test_g = train @ rotation.T, test @ rotation.T
        local_patches = []
        for kind, seed_offset in [("horizontal", 0), ("wall", 100000)]:
            planes, rejected = fit_planes(train_g, test_g, kind, settings, settings.seed + node + seed_offset)
            diagnostics += [{"node_id": node, **row} for row in rejected]
            for plane in planes:
                plane["normal"] = rotation.T @ plane["normal"]
                plane["center"] = rotation.T @ plane["center"]
                plane["training"] = plane["training"] @ rotation
                plane["testing"] = plane["testing"] @ rotation
                local_patches.append(plane)
                for label, poses in trajectories.items():
                    if node not in poses:
                        continue
                    center = apply(plane["center"][None], poses[node])[0]
                    normal = poses[node][:3, :3] @ plane["normal"]
                    anchor_height = raw[min(raw)][2, 3]
                    surfaces.append({"node_id": node, "stamp": stamps[node], "trajectory": label,
                                     "kind": kind, "plane_index": plane["plane_index"],
                                     "band_id": band_lookup[node]["band_id"],
                                     "temporal_proxy": band_lookup[node]["proxy_label"],
                                     "center_x_m": float(center[0]), "center_y_m": float(center[1]),
                                     "center_z_from_anchor_m": float(center[2] - anchor_height),
                                     "normal_x": float(normal[0]), "normal_y": float(normal[1]),
                                     "normal_z": float(normal[2]),
                                     "tilt_from_up_degrees": float(np.degrees(np.arccos(np.clip(abs(normal[2]), 0., 1.)))),
                                     "is_physical_floor_verified": False})
        patches[node] = local_patches
        clouds.cache.pop(node, None)
        if count % 25 == 0:
            print(f"{name}: extracted {count}/{len(node_ids)} sampled nodes", flush=True)
    pairs, unmatched = [], []
    for loop in loops:
        a, b = int(loop["from_id"]), int(loop["to_id"])
        stored = connection.execute("SELECT transform FROM Link WHERE from_id=? AND to_id=? AND type=?",
                                    (a, b, int(loop["type"]))).fetchone()
        if stored is None:
            raise ValueError("Accepted loop has no matching database measurement")
        measurement = checked_transform(stored[0])
        matches = []
        for pa in patches.get(a, []):
            for pb in patches.get(b, []):
                match = patch_correspondence(pa, pb, measurement, settings)
                if match:
                    matches.append((match["measurement_plane_abs_m"], pa, pb, match))
        # One-to-one patch matches prevent several nearby treads sharing one plane.
        used_a, used_b = set(), set()
        selected = 0
        for _, pa, pb, match in sorted(matches, key=lambda value: value[0]):
            ka, kb = (pa["kind"], pa["plane_index"]), (pb["kind"], pb["plane_index"])
            if ka in used_a or kb in used_b:
                continue
            used_a.add(ka); used_b.add(kb); selected += 1
            for label, poses in trajectories.items():
                if a not in poses or b not in poses:
                    continue
                pairs.append({"from_id": a, "to_id": b, "kind": pa["kind"],
                              "from_plane": pa["plane_index"], "to_plane": pb["plane_index"],
                              "loop_held_out_from_independent_optimizer": loop["held_out"],
                              "trajectory": label, **match,
                              **pair_metrics(pa, pb, poses[a], poses[b])})
        if not selected:
            unmatched.append({"from_id": a, "to_id": b, "reason": "no_valid_overlapping_plane_pair"})
    connection.close()
    summaries = []
    for band in bands:
        for label, poses in trajectories.items():
            ids = [node for node in band["node_ids"] if node in poses]
            if not ids:
                continue
            observations = [r for r in surfaces if r["trajectory"] == label and r["kind"] == "horizontal"
                            and r["band_id"] == band["band_id"]]
            per_node = {}
            for row in observations:
                per_node.setdefault(row["node_id"], []).append(row["center_z_from_anchor_m"])
            # Keep distinct planes in detailed CSV; this median is only a motion/
            # surface-height proxy. It is expressly not a flatness or level metric.
            heights = [float(np.median(v)) for v in per_node.values()]
            times = [stamps[node] for node in per_node]
            slope = None
            if len(times) >= 8 and max(times) - min(times) >= 20:
                # A robust endpoint trend, avoiding a false high-precision linear fit.
                order = np.argsort(times)
                n = max(2, len(times) // 4)
                dt = np.median(np.asarray(times)[order[-n:]]) - np.median(np.asarray(times)[order[:n]])
                if dt > 0:
                    slope = float((np.median(np.asarray(heights)[order[-n:]]) -
                                   np.median(np.asarray(heights)[order[:n]])) / dt * 60)
            summaries.append({"band_id": band["band_id"], "proxy_label": band["proxy_label"],
                              "trajectory": label, "first_node": ids[0], "last_node": ids[-1],
                              "duration_s": stamps[ids[-1]] - stamps[ids[0]],
                              "camera_z_span_m": float(np.ptp([poses[i][2, 3] for i in ids])),
                              "horizontal_patch_count": len(observations),
                              "horizontal_patch_node_count": len(per_node),
                              "surface_height_proxy_m": stats(heights),
                              "surface_height_proxy_endpoint_trend_m_per_minute": slope,
                              "same_physical_floor_established": False})
    after_hash = sha256_file(database)
    if before_hash != after_hash:
        raise RuntimeError(f"Database changed during read-only evaluation: {database}")
    aggregate = {}
    for label in trajectories:
        aggregate[label] = {}
        for kind in ("horizontal", "wall"):
            for heldout in (False, True):
                selected = [r for r in pairs if r["trajectory"] == label and r["kind"] == kind
                            and (not heldout or r["loop_held_out_from_independent_optimizer"] == "True")]
                aggregate[label][kind + ("_heldout_loops" if heldout else "_all_loops")] = {
                    "patch_pair_count": len(selected),
                    "unique_loop_pair_count": len({(r["from_id"], r["to_id"]) for r in selected}),
                    "plane_abs_median_m": stats(r["symmetric_plane_abs_median_m"] for r in selected),
                    "normal_difference_degrees": stats(r["normal_difference_degrees"] for r in selected)}
    report = {"session": name, "database": str(database), "database_sha256_before": before_hash,
              "database_sha256_after": after_hash, "original_database_unchanged": True,
              "settings": asdict(settings), "main_component_nodes": len(raw),
              "sampled_nodes": sorted(node_ids), "accepted_horizontal_patches": sum(p["kind"] == "horizontal" for v in patches.values() for p in v),
              "accepted_wall_patches": sum(p["kind"] == "wall" for v in patches.values() for p in v),
              "rejected_plane_proposals": sum(not r["accepted"] for r in diagnostics),
              "source_trajectories": {label: {"path": str(path.resolve()), "sha256": sha256_file(path)} for label, path in source_paths.items()},
              "source_loop_validation": {"path": str(validation_path.resolve()), "sha256": sha256_file(validation_path)},
              "gauge_alignment": alignment, "pair_aggregates": aggregate,
              "temporal_bands": summaries, "loops_without_surface_correspondence": unmatched,
              "elapsed_seconds": time.monotonic() - started}
    write_csv(directory / "plane_fit_validation.csv", diagnostics)
    write_csv(directory / "surface_observations.csv", surfaces)
    write_csv(directory / "plane_pair_metrics.csv", pairs)
    write_csv(directory / "temporal_height_proxies.csv", bands_rows)
    (directory / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"{name}: complete; horizontal={report['accepted_horizontal_patches']}, walls={report['accepted_wall_patches']}, correspondences={len(pairs)//len(trajectories)}", flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config/student_union_added_sessions.json"))
    parser.add_argument("--run", type=Path, default=Path("outputs/student_union_20261017_added_reviewed"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sessions", nargs="+")
    parser.add_argument("--trajectory", action="append", default=[], metavar="SESSION:LABEL:CSV")
    parser.add_argument("--frame-stride", type=int, default=5)
    args = parser.parse_args()
    workspace = Path(__file__).resolve().parents[1]
    configuration = json.loads(args.config.read_text())
    sessions = configuration.get("sessions", {})
    names = args.sessions or list(sessions)
    if not names or set(names) - set(sessions) or len(set(names)) != len(names):
        parser.error("--sessions must be unique configured names")
    if args.frame_stride < 1:
        parser.error("--frame-stride must be positive")
    try:
        overrides = parse_overrides(args.trajectory)
    except ValueError as exc:
        parser.error(str(exc))
    if {session for session, _ in overrides} - set(names):
        parser.error("Trajectory override refers to an unselected session")
    for name in names:
        if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
            parser.error("Unsafe session name")
    output = args.output.resolve()
    if (output / "manifest.json").exists():
        parser.error("Use a fresh output directory; existing evidence is not overwritten")
    output.mkdir(parents=True, exist_ok=True)
    settings = Settings(frame_stride=args.frame_stride)
    reports = [evaluate_session(name, (workspace / sessions[name]["database"]).resolve(),
                                args.run.resolve(), output, settings, overrides) for name in names]
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(), "argv": sys.argv,
                "cwd": str(Path.cwd()), "source_sha256": sha256_file(Path(__file__)),
                "imported_source_sha256": {name: sha256_file(Path(__file__).with_name(name))
                                          for name in ("optimize_session.py", "export_rtabmap_db.py")},
                "config_sha256": sha256_file(args.config), "settings": asdict(settings),
                "versions": {"numpy": np.__version__, "scipy": scipy.__version__, "opencv": cv2.__version__},
                "thread_environment": {key: os.environ.get(key) for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS")},
                "session_reports": [str((output / r["session"] / "report.json").resolve()) for r in reports],
                "limitations": ["No optimization, floor flattening, GNSS/plane prior, dynamic semantic mask or DB modification.",
                    "Horizontal patches are not verified floors; wall-like patches are not verified structural walls.",
                    "Distinct plane patches are retained; temporal z bands are not physical building levels or stair labels.",
                    "Within-frame held-out points are conditional near-patch validation from the same sensor, not independent ground truth.",
                    "Loop patch correspondences use stored validated relative transforms, not trajectories under evaluation; wrong loops can still mislead.",
                    "Global normal/height proxies use first-common-node SE(3) gauge alignment; relative pair metrics are gauge invariant.",
                    "Patch statistics weight patches/pairs, not independent traversals; adjacent frames and shared loop endpoints are correlated.",
                    "No same-floor flatness or absolute map accuracy is claimed; changes in proxy height may reflect real geometry."]}
    (output / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
