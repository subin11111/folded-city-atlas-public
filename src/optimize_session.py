#!/usr/bin/env python3
"""Conservative, independently anchored RTAB-Map session optimization.

Coordinate convention: T_w_i maps node-local points to session world; a Link
from i to j measures inv(T_w_i) @ T_w_j, mapping points in j into i.  Residual
E = inv(Z_i_j) @ inv(T_w_i) @ T_w_j is [E.translation, log(E.rotation)].
Stored information is used in that six-dimensional translation/rotation-vector
tangent approximation; this is not a reproduction of RTAB-Map's native solver.

Only x/y/z and world-Z yaw corrections vary. R'_i = Rz(delta_yaw_i) R_i
preserves R_i.T @ world_up exactly. No floor or extrinsic calibration correction
is estimated. Every odometry component has its own fixed anchor; unconnected
nodes never inherit a correction from an arbitrary nearby node ID.

Both type 1 and type 2 loops are geometrically verified. Trimmed point-to-point
ICP runs independently in both directions; its transforms are diagnostics, not
substituted for the stored measurement with an unjustified covariance. One in
five reliable loop pairs is held out when at least ten are available. Each
whitened six-vector has block Huber loss (not independent scalar Huber losses).
Original full covariance is retained; the effective odometry covariance adds
an explicit VIO model-noise diagonal, recorded separately for reproducibility.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shlex
import sqlite3
import sys
import time
import zlib
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import scipy
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from export_rtabmap_db import calibration_from_blob, load_pose_csv, transform_from_blob


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def checked_transform(blob: bytes) -> np.ndarray:
    """Reject null/singular matrices *before* correcting floating point drift."""
    if blob is None or len(blob) != 48:
        raise ValueError("missing_or_wrong_size_transform")
    transform = transform_from_blob(blob)
    rotation = transform[:3, :3]
    if not np.all(np.isfinite(transform)):
        raise ValueError("nonfinite_transform")
    if abs(np.linalg.det(rotation) - 1.0) > 1e-3:
        raise ValueError("singular_or_invalid_rotation")
    if np.linalg.norm(rotation.T @ rotation - np.eye(3)) > 1e-3:
        raise ValueError("nonorthogonal_rotation")
    u, _, vt = np.linalg.svd(rotation)
    transform[:3, :3] = u @ vt
    return transform


def checked_information(blob: bytes) -> np.ndarray:
    if blob is None or len(blob) != 288:
        raise ValueError("missing_or_wrong_size_information")
    information = np.frombuffer(blob, dtype="<f8").reshape(6, 6).copy()
    if not np.all(np.isfinite(information)):
        raise ValueError("nonfinite_information")
    if not np.allclose(information, information.T, rtol=1e-6, atol=1e-8):
        raise ValueError("asymmetric_information")
    information = (information + information.T) * 0.5
    if np.linalg.eigvalsh(information).min() <= 0.0:
        raise ValueError("nonpositive_information")
    return information


def inverse(transform: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def relative_error(measurement: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    error = inverse(measurement) @ inverse(a) @ b
    return np.r_[error[:3, 3], Rotation.from_matrix(error[:3, :3]).as_rotvec()]


def apply(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


@dataclass
class Edge:
    a: int
    b: int
    type: int
    measurement: np.ndarray
    information: np.ndarray
    effective_information: np.ndarray
    held_out: bool = False

    def serializable(self) -> dict:
        return {
            "from_id": self.a, "to_id": self.b, "type": self.type,
            "measurement": self.measurement.tolist(),
            "information": self.information.tolist(),
            "effective_information": self.effective_information.tolist(),
            "held_out": self.held_out,
        }


def connected_components(node_ids: list[int], edges: list[Edge]) -> list[list[int]]:
    adjacency = {i: set() for i in node_ids}
    for edge in edges:
        adjacency[edge.a].add(edge.b)
        adjacency[edge.b].add(edge.a)
    unseen = set(node_ids)
    components = []
    while unseen:
        root = min(unseen)
        stack, component = [root], []
        unseen.remove(root)
        while stack:
            node = stack.pop()
            component.append(node)
            for neighbor in sorted(adjacency[node] & unseen):
                unseen.remove(neighbor)
                stack.append(neighbor)
        components.append(sorted(component))
    return sorted(components, key=lambda ids: (-len(ids), ids[0]))


def read_edges(connection, node_ids, odom_sigma_m, odom_sigma_degrees):
    grouped = {}
    rejected = []
    for a, b, kind, iblob, tblob in connection.execute(
        "SELECT from_id,to_id,type,information_matrix,transform FROM Link "
        "WHERE type IN (0,1,2) ORDER BY type,from_id,to_id"
    ):
        try:
            transform = checked_transform(tblob)
            information = checked_information(iblob)
            if a not in node_ids or b not in node_ids or a == b:
                raise ValueError("invalid_or_missing_endpoint")
        except ValueError as exc:
            rejected.append({"from_id": a, "to_id": b, "type": kind, "reason": str(exc)})
            continue
        key = (kind, min(a, b), max(a, b))
        if key in grouped:
            # Keep one native direction: inverse covariance need not be guessed.
            continue
        covariance = np.linalg.inv(information)
        if kind == 0:
            covariance += np.diag([odom_sigma_m**2] * 3 + [np.radians(odom_sigma_degrees)**2] * 3)
        grouped[key] = Edge(a, b, kind, transform, information, np.linalg.inv(covariance))
    return list(grouped.values()), rejected


def saved_app_poses(connection):
    """Read the optional app-saved solution without fabricating missing IDs."""
    row = connection.execute("SELECT opt_ids,opt_poses FROM Admin LIMIT 1").fetchone()
    if row is None or not row[0] or not row[1]:
        return {}
    ids = np.frombuffer(zlib.decompress(row[0]), dtype="<i4")
    pose_bytes = zlib.decompress(row[1])
    if len(pose_bytes) != len(ids) * 48:
        raise ValueError("App optimized pose IDs/matrices have inconsistent lengths")
    return {int(node): checked_transform(pose_bytes[k * 48:(k + 1) * 48]) for k, node in enumerate(ids)}


class Clouds:
    def __init__(self, connection, mask_dir: Path | None, max_depth=4.0):
        self.connection = connection
        self.mask_dir = mask_dir
        self.max_depth = max_depth
        self.cache = {}
        self.diagnostics = {}

    def get(self, node_id: int) -> tuple[np.ndarray, np.ndarray]:
        if node_id in self.cache:
            return self.cache[node_id]
        row = self.connection.execute(
            "SELECT depth,depth_confidence,calibration FROM Data WHERE id=?", (node_id,)
        ).fetchone()
        if row is None or row[0] is None or row[2] is None:
            self.diagnostics[node_id] = {"error": "missing_depth_or_calibration"}
            self.cache[node_id] = (np.empty((0, 3)), np.empty((0, 3)))
            return self.cache[node_id]
        depth_bytes = cv2.imdecode(np.frombuffer(row[0], np.uint8), cv2.IMREAD_UNCHANGED)
        if depth_bytes is None or depth_bytes.ndim != 3 or depth_bytes.shape[2] != 4:
            raise ValueError(f"Unsupported depth encoding at node {node_id}")
        depth = np.ascontiguousarray(depth_bytes).view("<f4").reshape(depth_bytes.shape[:2])
        height, width = depth.shape
        confidence_raw = zlib.decompress(row[1]) if row[1] else b""
        if len(confidence_raw) not in (height * width, height * width + 12):
            raise ValueError(f"Unsupported/missing confidence encoding at node {node_id}: {len(confidence_raw)}")
        confidence = np.frombuffer(confidence_raw[:height * width], np.uint8).reshape(height, width)
        cw, ch, intrinsic, local = calibration_from_blob(row[2])
        if cw <= 0 or ch <= 0:
            raise ValueError(f"Invalid calibration dimensions at node {node_id}")
        valid = np.isfinite(depth) & (depth >= .25) & (depth <= self.max_depth) & (confidence >= 50)
        masked_pixels = 0
        if self.mask_dir is not None:
            mask_path = self.mask_dir / f"{node_id}.png"
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
            if mask is None:
                raise ValueError(f"Requested dynamic mask missing: {mask_path}")
            excluded = cv2.resize(mask, (width, height), interpolation=cv2.INTER_NEAREST) > 0
            masked_pixels = int(np.count_nonzero(valid & excluded))
            valid &= ~excluded
        vv, uu = np.mgrid[:height:2, :width:2]
        valid = valid[::2, ::2]
        zz = depth[::2, ::2][valid].astype(float)
        fx, fy = intrinsic[0, 0] * width / cw, intrinsic[1, 1] * height / ch
        cx, cy = intrinsic[0, 2] * width / cw, intrinsic[1, 2] * height / ch
        points = np.column_stack(((uu[valid] - cx) * zz / fx, (vv[valid] - cy) * zz / fy, zz))
        points = apply(points, local)
        # Spatially deterministic split; validation samples never enter ICP fitting.
        voxels = np.floor(points / .06).astype(np.int64)
        _, indices = np.unique(voxels, axis=0, return_index=True)
        points, voxels = points[indices], voxels[indices]
        held_out = np.mod(voxels @ np.array([73856093, 19349663, 83492791]), 5) == 0
        train, test = points[~held_out], points[held_out]
        self.cache[node_id] = (train, test)
        self.diagnostics[node_id] = {"training_points": len(train), "held_out_points": len(test),
                                     "dynamic_masked_depth_pixels": masked_pixels,
                                     "confidence_minimum": 50}
        return train, test


def geometric_score(a: np.ndarray, b: np.ndarray, transform: np.ndarray, threshold=.15) -> dict:
    if min(len(a), len(b)) < 30:
        return {"overlap": 0.0, "rmse_m": None, "clipped_rmse_m": threshold,
                "source_overlap": 0.0, "target_overlap": 0.0, "pairs": 0}
    moved = apply(b, transform)
    da = cKDTree(moved).query(a, workers=1)[0]
    db = cKDTree(a).query(moved, workers=1)[0]
    oa, ob = np.mean(da < threshold), np.mean(db < threshold)
    inliers = np.r_[da[da < threshold], db[db < threshold]]
    return {
        "overlap": float(2 * oa * ob / max(oa + ob, 1e-12)),
        "source_overlap": float(ob), "target_overlap": float(oa),
        "rmse_m": float(np.sqrt(np.mean(inliers**2))) if len(inliers) else None,
        "clipped_rmse_m": float(np.sqrt(np.mean(np.minimum(np.r_[da, db], threshold)**2))),
        "pairs": int(len(inliers)),
    }


def trimmed_icp(target, source, initial, iterations=20):
    transform = initial.copy()
    tree = cKDTree(target)
    performed = 0
    if min(len(target), len(source)) < 80:
        return transform, performed
    for k in range(iterations):
        moved = apply(source, transform)
        distances, indices = tree.query(moved, workers=1)
        valid = distances < .22
        if np.count_nonzero(valid) < 80:
            break
        valid &= distances <= np.percentile(distances[valid], 80)
        moving, fixed = moved[valid], target[indices[valid]]
        cm, cf = moving.mean(axis=0), fixed.mean(axis=0)
        u, _, vt = np.linalg.svd((moving - cm).T @ (fixed - cf))
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0:
            vt[-1] *= -1
            rotation = vt.T @ u.T
        delta = np.eye(4)
        delta[:3, :3], delta[:3, 3] = rotation, cf - rotation @ cm
        transform = delta @ transform
        performed = k + 1
        if np.linalg.norm(delta[:3, 3]) < 1e-5 and Rotation.from_matrix(rotation).magnitude() < 1e-5:
            break
    return transform, performed


def verify_loop(edge, clouds, component_lookup):
    row = {"from_id": edge.a, "to_id": edge.b, "type": edge.type, "accepted": False,
           "held_out": False, "reason": ""}
    if component_lookup[edge.a] != component_lookup[edge.b]:
        row["reason"] = "different_odometry_components_not_merged"
        return row
    a, _ = clouds.get(edge.a)
    b, _ = clouds.get(edge.b)
    if min(len(a), len(b)) < 150:
        row["reason"] = "insufficient_static_depth_points"
        return row
    refined, forward_n = trimmed_icp(a, b, edge.measurement)
    reverse, reverse_n = trimmed_icp(b, a, inverse(edge.measurement))
    score = geometric_score(a, b, refined)
    reverse_score = geometric_score(b, a, reverse)
    refinement = inverse(edge.measurement) @ refined
    cycle = refined @ reverse
    row.update({"overlap": score["overlap"], "rmse_m": score["rmse_m"],
                "reverse_overlap": reverse_score["overlap"], "reverse_rmse_m": reverse_score["rmse_m"],
                "initial_overlap": geometric_score(a, b, edge.measurement)["overlap"],
                "refinement_translation_m": float(np.linalg.norm(refinement[:3, 3])),
                "refinement_rotation_degrees": float(np.degrees(Rotation.from_matrix(refinement[:3, :3]).magnitude())),
                "cycle_translation_m": float(np.linalg.norm(cycle[:3, 3])),
                "cycle_rotation_degrees": float(np.degrees(Rotation.from_matrix(cycle[:3, :3]).magnitude())),
                "forward_iterations": forward_n, "reverse_iterations": reverse_n})
    failures = []
    for ok, reason in [
        (min(score["overlap"], reverse_score["overlap"]) >= .45, "overlap_below_0.45"),
        (score["rmse_m"] is not None and score["rmse_m"] <= .075, "forward_rmse_above_0.075m"),
        (reverse_score["rmse_m"] is not None and reverse_score["rmse_m"] <= .075, "reverse_rmse_above_0.075m"),
        (row["refinement_translation_m"] <= .25, "refinement_above_0.25m"),
        (row["refinement_rotation_degrees"] <= 5, "refinement_above_5deg"),
        (row["cycle_translation_m"] <= .10, "cycle_above_0.10m"),
        (row["cycle_rotation_degrees"] <= 2, "cycle_above_2deg"),
    ]:
        if not ok:
            failures.append(reason)
    row["accepted"] = not failures
    row["reason"] = ";".join(failures) if failures else "passed_bidirectional_trimmed_point_to_point_icp"
    return row


def block_huber(residuals, delta):
    norms = np.linalg.norm(residuals, axis=1)
    weights = np.ones(len(norms))
    selected = norms > delta
    weights[selected] = np.sqrt(2 * delta * norms[selected] - delta**2) / norms[selected]
    return residuals * weights[:, None]


def correction_poses(raw, ordered_ids, variable_ids, values):
    result = {i: raw[i].copy() for i in ordered_ids}
    for i, delta in zip(variable_ids, values.reshape(-1, 4)):
        result[i][:3, 3] += delta[:3]
        result[i][:3, :3] = Rotation.from_rotvec([0, 0, delta[3]]).as_matrix() @ raw[i][:3, :3]
    return result


def optimize(raw, odometry, loops, components, huber_delta=3., max_evaluations=300):
    ids = sorted(raw)
    index = {n: k for k, n in enumerate(ids)}
    anchors = {component[0] for component in components}
    variables = [n for n in ids if n not in anchors]
    variable_index = {n: k for k, n in enumerate(variables)}
    active = odometry + [edge for edge in loops if not edge.held_out]
    ri = np.asarray([raw[n][:3, :3] for n in ids])
    ti = np.asarray([raw[n][:3, 3] for n in ids])
    ai, bi = np.asarray([index[e.a] for e in active]), np.asarray([index[e.b] for e in active])
    mi = np.asarray([e.measurement[:3, :3].T for e in active])
    mt = np.asarray([e.measurement[:3, 3] for e in active])
    whitening = np.asarray([np.linalg.cholesky(e.effective_information).T for e in active])
    vi = np.asarray([index[n] for n in variables])

    def residual(parameters):
        correction = np.zeros((len(ids), 4))
        correction[vi] = parameters.reshape(-1, 4)
        rz = Rotation.from_rotvec(np.column_stack((np.zeros((len(ids), 2)), correction[:, 3]))).as_matrix()
        rotations, translations = rz @ ri, ti + correction[:, :3]
        inverse_a = rotations[ai].transpose(0, 2, 1)
        pr = inverse_a @ rotations[bi]
        pt = np.einsum("nij,nj->ni", inverse_a, translations[bi] - translations[ai])
        er = mi @ pr
        et = np.einsum("nij,nj->ni", mi, pt - mt)
        errors = np.column_stack((et, Rotation.from_matrix(er).as_rotvec()))
        whitened = np.einsum("nij,nj->ni", whitening, errors)
        return block_huber(whitened, huber_delta).ravel()

    sparsity = lil_matrix((len(active) * 6, len(variables) * 4), dtype=np.int8)
    for k, edge in enumerate(active):
        for n in (edge.a, edge.b):
            if n in variable_index:
                j = variable_index[n] * 4
                sparsity[k * 6:(k + 1) * 6, j:j + 4] = 1
    initial = np.zeros(len(variables) * 4)
    limits = np.tile([3., 3., 1., np.radians(20.)], len(variables))
    initial_cost = float(.5 * np.sum(residual(initial)**2))
    result = least_squares(residual, initial, jac_sparsity=sparsity.tocsr(),
                           bounds=(-limits, limits), method="trf", tr_solver="lsmr", loss="linear",
                           x_scale="jac", max_nfev=max_evaluations, ftol=1e-6, xtol=1e-7,
                           gtol=1e-6, verbose=1, tr_options={"atol": 1e-7, "btol": 1e-7, "maxiter": 300})
    poses = correction_poses(raw, ids, variables, result.x)
    return poses, {"success": bool(result.success), "status": int(result.status),
                   "message": str(result.message), "nfev": int(result.nfev), "njev": int(result.njev),
                   "initial_block_huber_cost": initial_cost, "final_block_huber_cost": float(result.cost),
                   "optimality": float(result.optimality), "active_bounds": int(np.count_nonzero(result.active_mask)),
                   "component_anchors": sorted(anchors), "variable_nodes": len(variables),
                   "accepted_for_output": False}


def error_statistics(edges, poses):
    errors = [relative_error(e.measurement, poses[e.a], poses[e.b]) for e in edges if e.a in poses and e.b in poses]
    if not errors:
        return {"count": 0}
    errors = np.asarray(errors)
    translation = np.linalg.norm(errors[:, :3], axis=1)
    rotation = np.degrees(np.linalg.norm(errors[:, 3:], axis=1))
    return {"count": len(errors), "translation_median_m": float(np.median(translation)),
            "translation_p90_m": float(np.percentile(translation, 90)), "translation_max_m": float(max(translation)),
            "rotation_median_degrees": float(np.median(rotation)), "rotation_p90_degrees": float(np.percentile(rotation, 90))}


def geometry_metrics(pairs, poses, clouds):
    rows = []
    for a, b in pairs:
        if a not in poses or b not in poses:
            continue
        _, aa = clouds.get(a)
        _, bb = clouds.get(b)
        score = geometric_score(aa, bb, inverse(poses[a]) @ poses[b], threshold=.20)
        rows.append({"from_id": a, "to_id": b, **score})
    if not rows:
        return {"count": 0}, rows
    return {"count": len(rows), "overlap_median": float(np.median([r["overlap"] for r in rows])),
            "clipped_rmse_median_m": float(np.median([r["clipped_rmse_m"] for r in rows])),
            "threshold_m": .20, "note": "Held-out voxel samples; geometry is not surveyed ground truth."}, rows


def floor_observations(node_ids, raw, optimized, clouds):
    """Diagnostics only: deterministic robust least-squares height-band plane fits."""
    rows = []
    for node_id in node_ids:
        train, test = clouds.get(node_id)
        points = np.vstack((train, test))
        world = apply(points, raw[node_id])
        pz = raw[node_id][2, 3]
        candidates = world[(world[:, 2] < pz - .4) & (world[:, 2] > pz - 2.2)]
        if len(candidates) < 80:
            continue
        hist, bins = np.histogram(candidates[:, 2], bins=np.arange(pz - 2.2, pz - .35, .04))
        height = .5 * (bins[np.argmax(hist)] + bins[np.argmax(hist) + 1])
        band = candidates[np.abs(candidates[:, 2] - height) < .08]
        if len(band) < 60:
            continue
        design = np.column_stack((band[:, :2], np.ones(len(band))))
        coefficients = np.linalg.lstsq(design, band[:, 2], rcond=None)[0]
        for _ in range(3):
            mask = np.abs(design @ coefficients - band[:, 2]) < .03
            if np.count_nonzero(mask) < 40:
                break
            coefficients = np.linalg.lstsq(design[mask], band[mask, 2], rcond=None)[0]
        mask = np.abs(design @ coefficients - band[:, 2]) < .03
        if np.count_nonzero(mask) < 40:
            continue
        normal = np.r_[-coefficients[:2], 1.]
        norm = np.linalg.norm(normal)
        normal /= norm
        offset = -coefficients[2] / norm
        local_normal = raw[node_id][:3, :3].T @ normal
        local_offset = offset + normal @ raw[node_id][:3, 3]
        corrected_normal = optimized[node_id][:3, :3] @ local_normal
        before_height = raw[node_id][2, 3] - local_offset / normal[2]
        after_height = optimized[node_id][2, 3] - local_offset / corrected_normal[2]
        rows.append({"node_id": node_id, "used_as_factor": False,
                     "raw_height_at_pose_m": float(before_height), "optimized_height_at_pose_m": float(after_height),
                     "tilt_degrees": float(np.degrees(np.arccos(np.clip(normal[2], -1, 1)))),
                     "rms_m": float(np.sqrt(np.mean((design[mask] @ coefficients - band[mask, 2])**2))),
                     "inlier_count": int(np.count_nonzero(mask)), "candidate_count": len(candidates),
                     "camera_height_m": float(pz - before_height)})
    return rows


def write_csv(path, rows, fields=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = fields or sorted({key for row in rows for key in row}) or ["no_rows"]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def trajectory(path, poses, stamps):
    rows = []
    for node_id in sorted(poses):
        pose = poses[node_id]
        q = Rotation.from_matrix(pose[:3, :3]).as_quat()
        rows.append(dict(zip(["id", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw"],
                             [node_id, stamps[node_id], *pose[:3, 3], *q])))
    write_csv(path, rows, ["id", "stamp", "x", "y", "z", "qx", "qy", "qz", "qw"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("db", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mask-dir", type=Path)
    parser.add_argument("--reference-trajectory", type=Path)
    parser.add_argument("--odom-model-sigma-m", type=float, default=.03)
    parser.add_argument("--odom-model-sigma-degrees", type=float, default=.5)
    parser.add_argument("--max-evaluations", type=int, default=300)
    args = parser.parse_args()
    started, start_time = utc_now(), time.monotonic()
    args.output.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(f"file:{args.db.resolve()}?mode=ro", uri=True)
    raw, stamps = {}, {}
    for node_id, stamp, blob in connection.execute("SELECT id,stamp,pose FROM Node ORDER BY id"):
        raw[node_id], stamps[node_id] = checked_transform(blob), float(stamp)
    edges, invalid = read_edges(connection, set(raw), args.odom_model_sigma_m, args.odom_model_sigma_degrees)
    odometry = [e for e in edges if e.type == 0]
    components = connected_components(sorted(raw), odometry)
    lookup = {node: k for k, component in enumerate(components) for node in component}
    clouds = Clouds(connection, args.mask_dir)
    print(f"{started}: {len(raw)} nodes, {len(odometry)} odometry pairs; component sizes {[len(c) for c in components]}", flush=True)
    loops, validation = [], []
    for k, edge in enumerate(e for e in edges if e.type in (1, 2)):
        row = verify_loop(edge, clouds, lookup)
        validation.append(row)
        if row["accepted"]:
            loops.append(edge)
        print(f"Loop {k + 1}: {edge.a}->{edge.b} type {edge.type}: {row['reason']}", flush=True)
    # Reserve complete pairs, not inverse duplicates. Selection is deterministic.
    if len(loops) >= 10:
        for k, edge in enumerate(loops):
            edge.held_out = k % 5 == 4
        for row in validation:
            row["held_out"] = any(e.a == row["from_id"] and e.b == row["to_id"] and e.type == row["type"] and e.held_out for e in loops)
    optimized, solver = optimize(raw, odometry, loops, components, max_evaluations=args.max_evaluations)
    holdout_pairs = [(e.a, e.b) for e in loops if e.held_out]
    all_loop_pairs = [(e.a, e.b) for e in loops]
    adjacent_pairs = [(e.a, e.b) for k, e in enumerate(odometry) if k % max(1, len(odometry) // 60) == 0]
    metrics = {}
    geometry_rows = []
    references = {"node_pose_baseline": raw, "candidate_optimized": optimized}
    if args.reference_trajectory:
        references["app_saved_optimized"] = load_pose_csv(args.reference_trajectory)
    else:
        app_poses = saved_app_poses(connection)
        if app_poses:
            references["app_saved_optimized"] = app_poses
    for label, poses in references.items():
        entry = {"odometry": error_statistics(odometry, poses),
                 "training_loops": error_statistics([e for e in loops if not e.held_out], poses),
                 "held_out_loops": error_statistics([e for e in loops if e.held_out], poses)}
        for name, pairs in [("held_out_loop_geometry", holdout_pairs), ("all_verified_loop_geometry", all_loop_pairs), ("adjacent_geometry", adjacent_pairs)]:
            entry[name], rows = geometry_metrics(pairs, poses, clouds)
            geometry_rows.extend({"trajectory": label, "pair_group": name, **row} for row in rows)
        metrics[label] = entry
    # Fail closed on nonconvergence, constrained solutions, or geometric regression.
    guards = []
    if not solver["success"]:
        guards.append("solver_not_converged")
    if solver["active_bounds"]:
        guards.append("correction_reached_bound")
    if solver["final_block_huber_cost"] > solver["initial_block_huber_cost"] + 1e-8:
        guards.append("objective_increased")
    before, after = metrics["node_pose_baseline"], metrics["candidate_optimized"]
    for name in ["held_out_loop_geometry", "adjacent_geometry"]:
        if before[name]["count"] and (after[name]["clipped_rmse_median_m"] > before[name]["clipped_rmse_median_m"] + .005
                                      or after[name]["overlap_median"] < before[name]["overlap_median"] - .03):
            guards.append(name + "_regression")
    solver["rejection_reasons"] = guards
    solver["accepted_for_output"] = not guards
    trajectory(args.output / "candidate_trajectory.csv", optimized, stamps)
    if guards:
        optimized = {node: pose.copy() for node, pose in raw.items()}
    trajectory(args.output / "raw_trajectory.csv", raw, stamps)
    trajectory(args.output / "optimized_trajectory.csv", optimized, stamps)
    trajectory(args.output / "raw_main_trajectory.csv", {i: raw[i] for i in components[0]}, stamps)
    trajectory(args.output / "optimized_main_trajectory.csv", {i: optimized[i] for i in components[0]}, stamps)
    corrections = []
    for node_id in sorted(raw):
        delta = optimized[node_id][:3, 3] - raw[node_id][:3, 3]
        angle = Rotation.from_matrix(optimized[node_id][:3, :3] @ raw[node_id][:3, :3].T).magnitude()
        corrections.append({"node_id": node_id, "stamp": stamps[node_id], "component": lookup[node_id],
                            "anchor": node_id == components[lookup[node_id]][0], "dx_m": float(delta[0]),
                            "dy_m": float(delta[1]), "dz_m": float(delta[2]),
                            "translation_m": float(np.linalg.norm(delta)), "rotation_degrees": float(np.degrees(angle))})
    floor_rows = floor_observations(sorted(raw), raw, optimized, clouds)
    write_csv(args.output / "floor_observations.csv", floor_rows)
    write_csv(args.output / "loop_validation.csv", validation)
    write_csv(args.output / "geometry_validation.csv", geometry_rows)
    write_csv(args.output / "pose_corrections.csv", corrections)
    write_csv(args.output / "cloud_decoding.csv", [{"node_id": i, **d} for i, d in sorted(clouds.diagnostics.items())])
    (args.output / "validated_edges.json").write_text(json.dumps({"odometry": [e.serializable() for e in odometry],
        "verified_loops": [e.serializable() for e in loops], "invalid_rows": invalid}, indent=2) + "\n")
    app_comparison = {}
    if "app_saved_optimized" in metrics:
        app_metrics = metrics["app_saved_optimized"]
        for name in ["held_out_loop_geometry", "all_verified_loop_geometry", "adjacent_geometry"]:
            if after[name]["count"] and app_metrics[name]["count"]:
                app_comparison[name] = {
                    "candidate_pairs": after[name]["count"], "app_pairs": app_metrics[name]["count"],
                    "candidate_minus_app_clipped_rmse_m": after[name]["clipped_rmse_median_m"] - app_metrics[name]["clipped_rmse_median_m"],
                    "candidate_minus_app_overlap": after[name]["overlap_median"] - app_metrics[name]["overlap_median"],
                }
        app_comparison["interpretation"] = (
            "This candidate is experimental. Acceptance guards compare with Node.pose, not a best-map selection. "
            "Positive RMSE difference favors the saved app solution; mixed or small changes do not establish superiority. "
            "No automated selection of a universally best solution from this limited internal geometry test."
        )
    source_files = [Path(__file__).resolve(), Path(__file__).resolve().with_name("export_rtabmap_db.py")]
    report = {
        "started_utc": started, "finished_utc": utc_now(), "elapsed_seconds": time.monotonic() - start_time,
        "source_database": str(args.db.resolve()), "database_open_mode": "read-only", "baseline": "Node.pose (stored ARKit/VIO odometry); app optimized graph is separately benchmarked when supplied",
        "provenance": {"command": shlex.join([sys.executable, *sys.argv]), "working_directory": str(Path.cwd()),
            "python": sys.version, "numpy": np.__version__, "scipy": scipy.__version__, "opencv": cv2.__version__,
            "thread_environment": {name: os.environ.get(name) for name in ["OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"]},
            "source_sha256": {str(path): sha256_file(path) for path in source_files},
            "database_sha256": sha256_file(args.db),
            "mask_report_sha256": sha256_file(args.mask_dir / "report.json") if args.mask_dir and (args.mask_dir / "report.json").exists() else None},
        "method": "gravity-preserving 4-DoF pose graph; bidirectional trimmed point-to-point ICP loop verification",
        "parameters": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        "coordinate_convention": "T_w_i maps node i to world; Z_i_j maps node j to i. E=inv(Z_i_j)@inv(T_w_i)@T_w_j; residual=[E.translation,SO3_log(E.rotation)]",
        "robust_loss": "block Huber delta=3 on each full-covariance-whitened 6D edge; odometry and training loops; not per-scalar Huber",
        "covariance_model": "Original full matrices retained. Odometry effective covariance = original covariance + diag(model_translation_sigma^2 x3, model_rotation_sigma^2 x3). Verified loops retain original measurement and full covariance; refined ICP transforms are not substituted.",
        "loop_validation_thresholds": {"minimum_bidirectional_overlap": .45, "maximum_bidirectional_rmse_m": .075,
            "maximum_refinement_m": .25, "maximum_refinement_degrees": 5, "maximum_cycle_m": .10,
            "maximum_cycle_degrees": 2, "icp_correspondence_m": .22, "icp_trim_fraction": .8,
            "overlap_distance_m": .15, "icp_max_iterations_per_direction": 20},
        "total_poses": len(raw), "odometry_pairs": len(odometry), "stored_valid_loop_pairs": len(validation),
        "verified_loop_pairs": len(loops), "training_loop_pairs": sum(not e.held_out for e in loops),
        "held_out_loop_pairs": sum(e.held_out for e in loops), "invalid_link_rows": invalid,
        "components": [{"id": k, "size": len(c), "anchor": c[0], "node_ids": c} for k, c in enumerate(components)],
        "solver": solver, "metrics": metrics,
        "app_saved_solution_comparison": app_comparison,
        "output_status": "experimental_candidate_passed_Node_pose_consistency_guards" if not guards else "Node_pose_fallback_candidate_failed_guards",
        "best_solution_claim": False,
        "main_component_export": {"node_count": len(components[0]), "raw_trajectory": "raw_main_trajectory.csv",
            "optimized_trajectory": "optimized_main_trajectory.csv", "excluded_node_ids": sorted(set(raw) - set(components[0])),
            "note": "Full CSVs retain every node; isolated nodes remain exactly at their own raw pose. Main-component CSVs exclude them for map export without arbitrary correction propagation."},
        "corrections": {"translation_median_m": float(np.median([r["translation_m"] for r in corrections])),
            "translation_p95_m": float(np.percentile([r["translation_m"] for r in corrections], 95)),
            "translation_max_m": max(r["translation_m"] for r in corrections),
            "rotation_max_degrees": max(r["rotation_degrees"] for r in corrections)},
        "floor_observation_count": len(floor_rows), "floor_factors": 0, "extrinsic_correction": "none",
        "gnss_factors": 0, "mask_directory": str(args.mask_dir) if args.mask_dir else None,
        "limitations": ["No independent surveyed ground truth; held-out depth geometry is an internal consistency test.",
            "4-DoF preserves measured gravity attitude; does not repair roll/pitch bias.",
            "No global floor-height or normal factor, preserving outdoor slopes/stairs.",
            "No new visual loop discovery, feature RANSAC, adjacent ICP refinement, GNSS prior, TSDF or cross-session merge in this stage.",
            "Per-edge covariance is a local tangent approximation; optional odometry model noise is a modeling assumption, not calibrated uncertainty.",
            "With no mask directory, moving objects have not been semantically removed."]}
    (args.output / "optimization.json").write_text(json.dumps(report, indent=2) + "\n")
    connection.close()
    print(json.dumps({"output": str(args.output), "solver": solver, "corrections": report["corrections"],
                      "verified_loops": len(loops), "elapsed_seconds": report["elapsed_seconds"]}, indent=2), flush=True)


if __name__ == "__main__":
    main()
