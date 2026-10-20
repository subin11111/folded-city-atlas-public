#!/usr/bin/env python3
"""Generic, auditable robust pose-graph backend for pre-built submaps.

This module intentionally knows nothing about RTAB-Map databases or this
project's session names.  It consumes a small JSON contract containing
submap poses and relative-pose measurements and compares three backends:

* ``baseline_huber``: block-Huber IRLS on non-odometry constraints;
* ``dcs``: Dynamic Covariance Scaling on non-odometry constraints; and
* ``group_switch``: one alternating switch shared by every physical-region
  group, so correlated frames from one doorway cannot vote independently.

For every backend, the information assigned to a training non-odometry
region is averaged over its members.  This bounds repeated observations from
one declared region to one information unit.  It is a correlation heuristic,
not a replacement for a calibrated joint covariance or correct region labels.

Odometry is always quadratic with weight one.  Held-out edges are never
placed in an objective, never connect graph components, and are evaluated
only after fitting.  One deterministic gauge is fixed in every connected
component.  Outputs are experimental diagnostics and can never grant
navigation approval.

Pose convention
---------------
``pose`` is ``T_world_submap``.  An edge measurement is
``T_from_to = inv(T_world_from) @ T_world_to``.  A pose/measurement may be a
flat ``[x,y,z,qx,qy,qz,qw]`` vector or a 4x4 nested list.  Information and
covariance matrices use tangent order ``[tx,ty,tz,rx,ry,rz]``.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
from typing import Iterable

import numpy as np
from scipy import __version__ as scipy_version
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation


SCHEMA_VERSION = 1
METHODS = ("baseline_huber", "dcs", "group_switch")
SWITCHABLE_TYPES = {"loop", "cross_session", "structural", "registration"}


@dataclass(frozen=True)
class Node:
    id: str
    session_id: str
    initial: np.ndarray
    gauge_requested: bool = False


@dataclass(frozen=True)
class Edge:
    id: str
    source: str
    target: str
    measurement: np.ndarray
    information: np.ndarray
    edge_type: str
    role: str
    region_group: str
    information_diagnostic: dict

    @property
    def is_odometry(self) -> bool:
        return self.edge_type == "odometry"

    @property
    def used_for_fit(self) -> bool:
        return self.role == "train"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _as_transform(value, field: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape == (7,):
        if not np.isfinite(array).all():
            raise ValueError(f"{field} contains a non-finite value")
        norm = np.linalg.norm(array[3:])
        if norm < 1e-12:
            raise ValueError(f"{field} has a zero quaternion")
        result = np.eye(4)
        result[:3, :3] = Rotation.from_quat(array[3:] / norm).as_matrix()
        result[:3, 3] = array[:3]
        return result
    if array.shape != (4, 4) or not np.isfinite(array).all():
        raise ValueError(f"{field} must be a finite length-7 pose or 4x4 matrix")
    if not np.allclose(array[3], [0., 0., 0., 1.], atol=1e-8):
        raise ValueError(f"{field} has an invalid homogeneous last row")
    error = np.linalg.norm(array[:3, :3].T @ array[:3, :3] - np.eye(3))
    determinant = np.linalg.det(array[:3, :3])
    if error > 1e-5 or determinant < .999 or determinant > 1.001:
        raise ValueError(f"{field} rotation is not in SO(3)")
    return array.copy()


def transform_record(transform: np.ndarray) -> list[float]:
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return [float(v) for v in np.r_[transform[:3, 3], quaternion]]


def inverse(transform: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def _clipped_spd(matrix, field: str, minimum: float, maximum: float) -> tuple[np.ndarray, dict]:
    value = np.asarray(matrix, dtype=float)
    if value.shape != (6, 6) or not np.isfinite(value).all():
        raise ValueError(f"{field} must be a finite 6x6 matrix")
    value = (value + value.T) / 2.
    eigenvalues, eigenvectors = np.linalg.eigh(value)
    if eigenvalues[-1] <= 0:
        raise ValueError(f"{field} has no positive eigenvalue")
    clipped = np.clip(eigenvalues, minimum, maximum)
    used = (eigenvectors * clipped) @ eigenvectors.T
    diagnostic = {
        "raw_eigenvalues": [float(v) for v in eigenvalues],
        "used_eigenvalues": [float(v) for v in clipped],
        "eigenvalue_floor": float(minimum),
        "eigenvalue_ceiling": float(maximum),
        "was_clipped": bool(not np.allclose(eigenvalues, clipped, rtol=1e-12, atol=1e-15)),
        "condition_number_used": float(clipped[-1] / clipped[0]),
    }
    return used, diagnostic


def _information_from_row(row: dict, edge_id: str, minimum: float, maximum: float) -> tuple[np.ndarray, dict]:
    has_information = "information" in row
    has_covariance = "covariance" in row
    if has_information == has_covariance:
        raise ValueError(f"edge {edge_id} must contain exactly one of information or covariance")
    if has_information:
        information, diagnostic = _clipped_spd(
            row["information"], f"edge {edge_id} information", minimum, maximum)
        diagnostic["input_kind"] = "information"
        return information, diagnostic

    covariance_floor = 1. / maximum
    covariance_ceiling = 1. / minimum
    covariance, covariance_diagnostic = _clipped_spd(
        row["covariance"], f"edge {edge_id} covariance", covariance_floor, covariance_ceiling)
    information = np.linalg.inv(covariance)
    # Re-clip after inversion to remove harmless numerical asymmetry.
    information, information_diagnostic = _clipped_spd(
        information, f"edge {edge_id} inverted covariance", minimum, maximum)
    information_diagnostic.update({
        "input_kind": "covariance",
        "covariance_raw_eigenvalues": covariance_diagnostic["raw_eigenvalues"],
        "covariance_used_eigenvalues": covariance_diagnostic["used_eigenvalues"],
        "was_clipped": bool(covariance_diagnostic["was_clipped"] or information_diagnostic["was_clipped"]),
    })
    return information, information_diagnostic


def parse_manifest(manifest: dict, information_floor: float = 1e-6,
                   information_ceiling: float = 1e8) -> tuple[list[Node], list[Edge]]:
    if manifest.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"schema_version must equal {SCHEMA_VERSION}")
    if information_floor <= 0 or information_ceiling <= information_floor:
        raise ValueError("invalid information eigenvalue limits")
    if not isinstance(manifest.get("submaps"), list) or not manifest["submaps"]:
        raise ValueError("submaps must be a non-empty list")
    if not isinstance(manifest.get("edges"), list):
        raise ValueError("edges must be a list")

    nodes: list[Node] = []
    ids: set[str] = set()
    for index, row in enumerate(manifest["submaps"]):
        node_id = str(row.get("id", ""))
        if not node_id or node_id in ids:
            raise ValueError(f"submap id is empty or duplicated at index {index}: {node_id!r}")
        ids.add(node_id)
        nodes.append(Node(node_id, str(row.get("session_id", "")),
                          _as_transform(row.get("pose"), f"submap {node_id} pose"),
                          bool(row.get("gauge", False))))

    edges: list[Edge] = []
    edge_ids: set[str] = set()
    for index, row in enumerate(manifest["edges"]):
        edge_id = str(row.get("id", ""))
        if not edge_id or edge_id in edge_ids:
            raise ValueError(f"edge id is empty or duplicated at index {index}: {edge_id!r}")
        edge_ids.add(edge_id)
        source, target = str(row.get("from", "")), str(row.get("to", ""))
        if source not in ids or target not in ids or source == target:
            raise ValueError(f"edge {edge_id} has invalid endpoints {source!r}, {target!r}")
        edge_type = str(row.get("type", ""))
        if edge_type not in {"odometry", *SWITCHABLE_TYPES}:
            raise ValueError(f"edge {edge_id} has unsupported type {edge_type!r}")
        role = str(row.get("role", ""))
        if role not in {"train", "heldout"}:
            raise ValueError(f"edge {edge_id} role must be train or heldout")
        information, diagnostic = _information_from_row(
            row, edge_id, information_floor, information_ceiling)
        region_group = str(row.get("region_group") or f"edge:{edge_id}")
        edges.append(Edge(edge_id, source, target,
                          _as_transform(row.get("measurement"), f"edge {edge_id} measurement"),
                          information, edge_type, role, region_group, diagnostic))
    return nodes, edges


def connected_components(node_ids: Iterable[str], edges: Iterable[Edge]) -> list[list[str]]:
    adjacency = {node_id: set() for node_id in node_ids}
    for edge in edges:
        if edge.used_for_fit:
            adjacency[edge.source].add(edge.target)
            adjacency[edge.target].add(edge.source)
    result: list[list[str]] = []
    remaining = set(adjacency)
    while remaining:
        root = min(remaining)
        queue = [root]
        remaining.remove(root)
        component: list[str] = []
        while queue:
            current = queue.pop()
            component.append(current)
            for neighbor in sorted(adjacency[current]):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
        result.append(sorted(component))
    return sorted(result, key=lambda component: component[0])


def select_gauges(nodes: list[Node], components: list[list[str]]) -> dict[str, str]:
    requested = {node.id for node in nodes if node.gauge_requested}
    gauges: dict[str, str] = {}
    for component in components:
        candidates = sorted(requested & set(component))
        if len(candidates) > 1:
            raise ValueError(f"component has multiple requested gauges: {candidates}")
        gauges[component[0]] = candidates[0] if candidates else component[0]
    return gauges


def pose_error(measurement: np.ndarray, pose_a: np.ndarray, pose_b: np.ndarray) -> np.ndarray:
    predicted = inverse(pose_a) @ pose_b
    error = inverse(measurement) @ predicted
    return np.r_[error[:3, 3], Rotation.from_matrix(error[:3, :3]).as_rotvec()]


def _edge_chi2(edge: Edge, poses: dict[str, np.ndarray]) -> tuple[np.ndarray, float]:
    error = pose_error(edge.measurement, poses[edge.source], poses[edge.target])
    return error, float(error @ edge.information @ error)


def _robust_weights(method: str, train_edges: list[Edge], poses: dict[str, np.ndarray],
                    huber_delta: float, dcs_phi: float,
                    group_switch_prior: float) -> tuple[dict[str, float], dict[str, dict]]:
    weights = {edge.id: 1. for edge in train_edges}
    group_report: dict[str, dict] = {}
    switchable = [edge for edge in train_edges if not edge.is_odometry]
    chi2 = {edge.id: _edge_chi2(edge, poses)[1] for edge in switchable}

    if method == "baseline_huber":
        for edge in switchable:
            norm = math.sqrt(max(0., chi2[edge.id]))
            weights[edge.id] = 1. if norm <= huber_delta else huber_delta / norm
        for group in sorted({edge.region_group for edge in switchable}):
            members = [edge for edge in switchable if edge.region_group == group]
            group_report[group] = {
                "switch_scale": None,
                "information_weight": None,
                "member_edge_ids": [edge.id for edge in members],
                "policy": "independent block-Huber weights; no shared switch",
            }
    elif method == "dcs":
        for edge in switchable:
            scale = min(1., 2. * dcs_phi / (dcs_phi + chi2[edge.id]))
            weights[edge.id] = scale * scale
        for group in sorted({edge.region_group for edge in switchable}):
            members = [edge for edge in switchable if edge.region_group == group]
            group_report[group] = {
                "switch_scale": None,
                "information_weight": None,
                "member_edge_ids": [edge.id for edge in members],
                "policy": "per-edge Dynamic Covariance Scaling",
            }
    elif method == "group_switch":
        groups: dict[str, list[Edge]] = {}
        for edge in switchable:
            groups.setdefault(edge.region_group, []).append(edge)
        for group, members in sorted(groups.items()):
            statistic = float(np.median([chi2[edge.id] for edge in members]))
            scale = group_switch_prior / (group_switch_prior + statistic)
            information_weight = scale * scale
            for edge in members:
                weights[edge.id] = information_weight
            group_report[group] = {
                "switch_scale": float(scale),
                "information_weight": float(information_weight),
                "member_edge_ids": [edge.id for edge in members],
                "aggregate": "median member chi-square (duplicates in a region do not add votes)",
                "aggregate_chi2": statistic,
                "switch_prior": float(group_switch_prior),
                "formula": "s=lambda/(lambda+median_chi2), information_weight=s^2",
            }
    else:
        raise ValueError(f"unknown method {method!r}")

    # Average the member objectives within each declared physical region.
    # In particular, copying one identical measurement N times leaves both
    # its total information and its effect on the optimized poses unchanged.
    # Heldouts and odometry do not participate in this normalization.
    members_by_group: dict[str, list[Edge]] = {}
    for edge in switchable:
        members_by_group.setdefault(edge.region_group, []).append(edge)
    for group, members in members_by_group.items():
        normalization = 1. / len(members)
        for edge in members:
            weights[edge.id] *= normalization
        row = group_report[group]
        row["correlation_member_count"] = len(members)
        row["correlation_information_normalization"] = normalization
        row["normalization_policy"] = "average member information objectives within each declared physical region"
        if row.get("information_weight") is not None:
            row["robust_information_weight_before_group_normalization"] = row["information_weight"]
            row["information_weight"] *= normalization

    # Every method exposes a group-level summary, even when its robust policy
    # assigns independent member weights rather than one shared switch.
    for group, row in group_report.items():
        member_weights = [float(weights[edge_id]) for edge_id in row["member_edge_ids"]]
        row["member_information_weights"] = dict(zip(row["member_edge_ids"], member_weights))
        row["final_weight_summary"] = {
            "minimum": float(np.min(member_weights)),
            "median": float(np.median(member_weights)),
            "maximum": float(np.max(member_weights)),
            "all_members_share_one_weight": bool(np.ptp(member_weights) <= 1e-15),
        }

    # This invariant is deliberately checked after every update, not merely
    # implied by which edges were placed in `switchable`.
    for edge in train_edges:
        if edge.is_odometry and weights[edge.id] != 1.:
            raise AssertionError("odometry must never be switched or robustly downweighted")
    return weights, group_report


def _decode_corrections(x: np.ndarray, nodes: list[Node], variable_ids: list[str]) -> dict[str, np.ndarray]:
    corrections = {node_id: x[index * 6:(index + 1) * 6]
                   for index, node_id in enumerate(variable_ids)}
    result: dict[str, np.ndarray] = {}
    for node in nodes:
        correction = corrections.get(node.id)
        if correction is None:
            result[node.id] = node.initial.copy()
            continue
        pose = node.initial.copy()
        pose[:3, :3] = Rotation.from_rotvec(correction[3:]).as_matrix() @ node.initial[:3, :3]
        pose[:3, 3] = node.initial[:3, 3] + correction[:3]
        result[node.id] = pose
    return result


def _solve_fixed_weights(nodes: list[Node], train_edges: list[Edge], gauges: set[str],
                         weights: dict[str, float], x0: np.ndarray,
                         max_evaluations: int, translation_bound: float,
                         rotation_bound_degrees: float) -> tuple[np.ndarray, dict]:
    variable_ids = [node.id for node in nodes if node.id not in gauges]
    variable_lookup = {node_id: index for index, node_id in enumerate(variable_ids)}
    whitening = {edge.id: np.linalg.cholesky(edge.information).T for edge in train_edges}

    def residual(x):
        poses = _decode_corrections(x, nodes, variable_ids)
        blocks = []
        for edge in train_edges:
            error = pose_error(edge.measurement, poses[edge.source], poses[edge.target])
            blocks.append(math.sqrt(weights[edge.id]) * (whitening[edge.id] @ error))
        return np.concatenate(blocks) if blocks else np.zeros(0)

    rows, columns = len(train_edges) * 6, len(variable_ids) * 6
    sparsity = lil_matrix((rows, columns), dtype=np.int8)
    for edge_index, edge in enumerate(train_edges):
        for node_id in (edge.source, edge.target):
            if node_id in variable_lookup:
                column = variable_lookup[node_id] * 6
                sparsity[edge_index * 6:(edge_index + 1) * 6, column:column + 6] = 1
    if not variable_ids or not train_edges:
        return x0.copy(), {"success": True, "nfev": 0, "cost": 0., "message": "nothing to optimize"}
    translation_limits = [translation_bound] * 3
    rotation_limits = [math.radians(rotation_bound_degrees)] * 3
    bounds = np.tile(translation_limits + rotation_limits, len(variable_ids))
    fit = least_squares(
        residual, x0, jac_sparsity=sparsity.tocsr(), bounds=(-bounds, bounds),
        method="trf", tr_solver="lsmr", x_scale="jac", max_nfev=max_evaluations,
        ftol=1e-9, xtol=1e-9, gtol=1e-9,
        tr_options={"atol": 1e-10, "btol": 1e-10, "maxiter": 500},
    )
    return fit.x, {"success": bool(fit.success), "nfev": int(fit.nfev),
                   "cost": float(fit.cost), "message": str(fit.message),
                   "active_bounds": int(np.count_nonzero(fit.active_mask))}


def solve(nodes: list[Node], edges: list[Edge], method: str,
          *, huber_delta: float = 3., dcs_phi: float = 5.991,
          group_switch_prior: float = 10., outer_iterations: int = 12,
          max_evaluations: int = 150, translation_bound: float = 20.,
          rotation_bound_degrees: float = 179.) -> tuple[dict[str, np.ndarray], dict]:
    if method not in METHODS:
        raise ValueError(f"method must be one of {METHODS}")
    train_edges = [edge for edge in edges if edge.used_for_fit]
    components = connected_components([node.id for node in nodes], train_edges)
    gauge_by_component = select_gauges(nodes, components)
    gauges = set(gauge_by_component.values())
    variable_ids = [node.id for node in nodes if node.id not in gauges]
    x = np.zeros(len(variable_ids) * 6)
    poses = {node.id: node.initial.copy() for node in nodes}
    weights = {edge.id: 1. for edge in train_edges}
    group_report: dict[str, dict] = {}
    iterations = []
    for outer in range(outer_iterations):
        updated_weights, group_report = _robust_weights(
            method, train_edges, poses, huber_delta, dcs_phi, group_switch_prior)
        maximum_weight_change = max((abs(updated_weights[k] - weights.get(k, 1.))
                                     for k in updated_weights), default=0.)
        weights = updated_weights
        previous = x.copy()
        x, solver = _solve_fixed_weights(
            nodes, train_edges, gauges, weights, x, max_evaluations,
            translation_bound, rotation_bound_degrees)
        poses = _decode_corrections(x, nodes, variable_ids)
        step = float(np.max(np.abs(x - previous))) if len(x) else 0.
        iterations.append({"outer_iteration": outer + 1, "maximum_weight_change": float(maximum_weight_change),
                           "maximum_parameter_step": step, **solver})
        if outer >= 1 and maximum_weight_change < 1e-5 and step < 1e-7:
            break

    # Report weights at the final poses, rather than the weights from the
    # preceding outer iteration.  No extra fit is hidden after this report.
    weights, group_report = _robust_weights(
        method, train_edges, poses, huber_delta, dcs_phi, group_switch_prior)
    edge_report = []
    for edge in edges:
        error, chi2 = _edge_chi2(edge, poses)
        weight = weights.get(edge.id) if edge.used_for_fit else None
        edge_report.append({
            "id": edge.id,
            "from": edge.source,
            "to": edge.target,
            "type": edge.edge_type,
            "role": edge.role,
            "region_group": edge.region_group,
            "used_for_fit": bool(edge.used_for_fit),
            "switching_allowed": bool(edge.used_for_fit and not edge.is_odometry),
            "information_weight": None if weight is None else float(weight),
            "residual_scale": None if weight is None else float(math.sqrt(weight)),
            "translation_error_m": float(np.linalg.norm(error[:3])),
            "rotation_error_deg": float(np.degrees(np.linalg.norm(error[3:]))),
            "chi2_before_robust_weight": chi2,
            "information_diagnostic": edge.information_diagnostic,
        })
    report = {
        "method": method,
        "components": [{"node_ids": component,
                        "gauge_id": gauge_by_component[component[0]],
                        "gauge_pose_unchanged": bool(np.allclose(
                            poses[gauge_by_component[component[0]]],
                            next(node.initial for node in nodes if node.id == gauge_by_component[component[0]]),
                            atol=0., rtol=0.))}
                       for component in components],
        "component_count": len(components),
        "heldout_edge_count_in_objective": 0,
        "fit_edge_ids": [edge.id for edge in train_edges],
        "heldout_edge_ids": [edge.id for edge in edges if not edge.used_for_fit],
        "fit_edge_ids_complete": True,
        "fit_edge_policy": "all and only edges whose role is train",
        "odometry_switching_policy": "forbidden; every training odometry information_weight is exactly 1",
        "parameters": {"huber_delta": huber_delta, "dcs_phi": dcs_phi,
                       "group_switch_prior": group_switch_prior,
                       "outer_iterations_limit": outer_iterations,
                       "max_evaluations_per_outer_iteration": max_evaluations,
                       "translation_correction_bound_m": translation_bound,
                       "rotation_correction_bound_deg": rotation_bound_degrees},
        "solver_iterations": iterations,
        "edges": edge_report,
        "groups": group_report,
        "poses": {node_id: transform_record(pose) for node_id, pose in sorted(poses.items())},
    }
    return poses, report


def _odometry_path(edge: Edge, odometry_edges: list[Edge]) -> tuple[np.ndarray | None, list[str]]:
    adjacency: dict[str, list[tuple[str, np.ndarray, str]]] = {}
    for candidate in odometry_edges:
        adjacency.setdefault(candidate.source, []).append(
            (candidate.target, candidate.measurement, candidate.id))
        adjacency.setdefault(candidate.target, []).append(
            (candidate.source, inverse(candidate.measurement), candidate.id))
    queue = deque([(edge.source, np.eye(4), [])])
    visited = {edge.source}
    while queue:
        current, transform, path = queue.popleft()
        if current == edge.target:
            return transform, path
        for neighbor, measurement, edge_id in sorted(adjacency.get(current, []), key=lambda row: row[0]):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, transform @ measurement, path + [edge_id]))
    return None, []


def cycle_diagnostics(edges: list[Edge]) -> list[dict]:
    odometry = [edge for edge in edges if edge.used_for_fit and edge.is_odometry]
    result = []
    for edge in edges:
        if edge.is_odometry:
            continue
        path_transform, path_ids = _odometry_path(edge, odometry)
        row = {"edge_id": edge.id, "role": edge.role, "region_group": edge.region_group,
               "odometry_path_available": path_transform is not None,
               "odometry_path_edge_ids": path_ids}
        if path_transform is not None:
            error = inverse(edge.measurement) @ path_transform
            row.update({
                "cycle_translation_error_m": float(np.linalg.norm(error[:3, 3])),
                "cycle_rotation_error_deg": float(np.degrees(
                    Rotation.from_matrix(error[:3, :3]).magnitude())),
            })
        result.append(row)
    return result


def leave_one_group_diagnostics(nodes: list[Node], edges: list[Edge], method: str,
                                solve_options: dict) -> list[dict]:
    groups = sorted({edge.region_group for edge in edges
                     if edge.used_for_fit and not edge.is_odometry})
    full_components = connected_components([node.id for node in nodes],
                                            [edge for edge in edges if edge.used_for_fit])
    result = []
    for group in groups:
        removed = [edge for edge in edges
                   if edge.used_for_fit and not edge.is_odometry and edge.region_group == group]
        removed_ids = {edge.id for edge in removed}
        retained = [edge for edge in edges if edge.id not in removed_ids]
        poses, report = solve(nodes, retained, method, **solve_options)
        residuals = []
        for edge in removed:
            error, chi2 = _edge_chi2(edge, poses)
            residuals.append({"edge_id": edge.id,
                              "translation_error_m": float(np.linalg.norm(error[:3])),
                              "rotation_error_deg": float(np.degrees(np.linalg.norm(error[3:]))),
                              "chi2": chi2})
        result.append({
            "region_group": group,
            "removed_edge_ids": [edge.id for edge in removed],
            "removed_edge_count": len(removed),
            "components_before": len(full_components),
            "components_after": report["component_count"],
            "disconnects_graph": bool(report["component_count"] > len(full_components)),
            "removed_group_prediction": residuals,
            "removed_group_median_translation_error_m": float(np.median(
                [row["translation_error_m"] for row in residuals])) if residuals else None,
            "removed_group_median_rotation_error_deg": float(np.median(
                [row["rotation_error_deg"] for row in residuals])) if residuals else None,
            "fit_heldout_edge_count": report["heldout_edge_count_in_objective"],
        })
    return result


def optimize_manifest(manifest: dict, *, methods: Iterable[str] = METHODS,
                      information_floor: float = 1e-6,
                      information_ceiling: float = 1e8,
                      include_leave_one_group: bool = True,
                      **solve_options) -> dict:
    nodes, edges = parse_manifest(manifest, information_floor, information_ceiling)
    methods = tuple(methods)
    if not methods or any(method not in METHODS for method in methods):
        raise ValueError(f"methods must be a non-empty subset of {METHODS}")
    variants = {}
    for method in methods:
        _, report = solve(nodes, edges, method, **solve_options)
        report["cycle_diagnostics"] = cycle_diagnostics(edges)
        report["leave_one_group_diagnostics"] = (
            leave_one_group_diagnostics(nodes, edges, method, solve_options)
            if include_leave_one_group else [])
        variants[method] = report
    source_path = Path(__file__).resolve()
    canonical_input = json.dumps(manifest, sort_keys=True, separators=(",", ":"),
                                 ensure_ascii=False).encode()
    return {
        "schema_version": SCHEMA_VERSION,
        "backend": "generic robust submap pose graph",
        "created_utc": utc_now(),
        "accepted": False,
        "navigation": False,
        "status": "experimental_comparison_only",
        "limitations": [
            "solver convergence is not independent geometric validation",
            "held-out residuals are diagnostic observations, not surveyed ground truth",
            "a switched-off sole bridge correctly leaves components unsupported",
        ],
        "input_counts": {"submaps": len(nodes), "edges": len(edges),
                         "training_edges": sum(edge.used_for_fit for edge in edges),
                         "heldout_edges": sum(not edge.used_for_fit for edge in edges)},
        "information_eigenvalue_clip": {"minimum": information_floor,
                                        "maximum": information_ceiling},
        "provenance": {
            "canonical_manifest_sha256": hashlib.sha256(canonical_input).hexdigest(),
            "canonical_manifest_hash_definition": "SHA-256 of UTF-8 JSON sorted by key with compact separators",
            "source_path": str(source_path),
            "source_sha256": sha256_file(source_path),
        },
        "variants": variants,
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True,
                        help="New output directory; an existing path is refused")
    parser.add_argument("--methods", nargs="+", choices=METHODS, default=list(METHODS))
    parser.add_argument("--information-floor", type=float, default=1e-6)
    parser.add_argument("--information-ceiling", type=float, default=1e8)
    parser.add_argument("--huber-delta", type=float, default=3.)
    parser.add_argument("--dcs-phi", type=float, default=5.991)
    parser.add_argument("--group-switch-prior", type=float, default=10.)
    parser.add_argument("--outer-iterations", type=int, default=12)
    parser.add_argument("--max-evaluations", type=int, default=150)
    parser.add_argument("--translation-bound", type=float, default=20.)
    parser.add_argument("--rotation-bound-degrees", type=float, default=179.)
    parser.add_argument("--skip-leave-one-group", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    # Refuse before mkdir and before solving so a typo cannot partially mutate
    # an earlier experiment directory.
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {args.output}")
    if not args.manifest.is_file():
        raise FileNotFoundError(args.manifest)
    manifest = json.loads(args.manifest.read_text())
    solve_options = {
        "huber_delta": args.huber_delta,
        "dcs_phi": args.dcs_phi,
        "group_switch_prior": args.group_switch_prior,
        "outer_iterations": args.outer_iterations,
        "max_evaluations": args.max_evaluations,
        "translation_bound": args.translation_bound,
        "rotation_bound_degrees": args.rotation_bound_degrees,
    }
    report = optimize_manifest(
        manifest, methods=args.methods,
        information_floor=args.information_floor,
        information_ceiling=args.information_ceiling,
        include_leave_one_group=not args.skip_leave_one_group,
        **solve_options)
    source_path = Path(__file__).resolve()
    report["provenance"].update({
        "manifest_path": str(args.manifest.resolve()),
        "manifest_sha256": sha256_file(args.manifest),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "scipy": scipy_version,
        "command_options": vars(args) | {"manifest": str(args.manifest), "output": str(args.output)},
    })
    args.output.mkdir(parents=True)
    shutil.copyfile(source_path, args.output / "source_snapshot.py")
    (args.output / "report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"output": str(args.output), "accepted": False, "navigation": False,
                      "methods": list(report["variants"])}))


if __name__ == "__main__":
    main()
