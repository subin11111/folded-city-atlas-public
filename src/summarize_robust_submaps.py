#!/usr/bin/env python3
"""Summarize robust-submap backend and sensitivity runs without promoting them.

The canonical input is the graph manifest consumed by
``optimize_robust_submaps.py``.  One or more backend reports may be compared
against it, and optional sensitivity graph/report pairs may be added.  Every
figure is rendered from the supplied graph and report values; this tool does
not open a sensor database or invent missing observations.

Fail-closed rules are deliberate:

* a report must prove that all and only training edges entered its fit;
* its canonical manifest hash must match the graph paired with it;
* held-out observations in a physical region that also supplied training
  evidence are marked correlated and excluded from approval-style metrics;
* a structural transition that is a graph bridge is a continuity assumption,
  not independently testable evidence, and is excluded too; and
* outputs always state ``accepted=false`` and ``navigation=false``.

CLI examples::

    python src/summarize_robust_submaps.py \
      --manifest graph_input/manifest.json \
      --report default optimization/report.json \
      --sensitivity weak-boundary graph_weak/manifest.json opt_weak/report.json \
      --output evaluation/robust_summary
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import platform
import shutil
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
from scipy import __version__ as scipy_version
from scipy.spatial.transform import Rotation


SCHEMA_VERSION = 1
ACTIVE_INFORMATION_WEIGHT = 0.01
EDGE_TYPES = ("odometry", "loop", "cross_session", "structural")
TYPE_COLORS = {
    "odometry": "#7a7a7a",
    "loop": "#2878b5",
    "cross_session": "#e07a2d",
    "structural": "#b44e74",
}
SESSION_COLORS = ("#2878b5", "#e07a2d", "#3a923a", "#b44e74", "#7a5ab5",
                  "#8c6d31", "#4c9da6", "#d05f5f")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_manifest_sha256(payload: dict[str, Any]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=False).encode()
    return hashlib.sha256(encoded).hexdigest()


def as_transform(value: Any, field: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape == (7,):
        if not np.isfinite(array).all() or np.linalg.norm(array[3:]) < 1e-12:
            raise ValueError(f"{field} is not a finite pose")
        result = np.eye(4)
        result[:3, :3] = Rotation.from_quat(array[3:] / np.linalg.norm(array[3:])).as_matrix()
        result[:3, 3] = array[:3]
        return result
    if array.shape != (4, 4) or not np.isfinite(array).all():
        raise ValueError(f"{field} must be a finite length-7 pose or 4x4 matrix")
    if not np.allclose(array[3], [0., 0., 0., 1.], atol=1e-8):
        raise ValueError(f"{field} has an invalid homogeneous row")
    rotation = array[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-5)):
        raise ValueError(f"{field} rotation is not in SO(3)")
    return array.copy()


def _role(edge: dict[str, Any]) -> str:
    value = str(edge.get("role", "")).lower()
    if value in {"holdout", "held_out", "audit"}:
        value = "heldout"
    if value not in {"train", "heldout"}:
        raise ValueError(f"edge {edge.get('id')!r} role must be train or heldout")
    return value


def validate_graph(graph: dict[str, Any], label: str) -> dict[str, Any]:
    if graph.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"{label}: graph schema_version must equal {SCHEMA_VERSION}")
    submaps = graph.get("submaps")
    edges = graph.get("edges")
    if not isinstance(submaps, list) or not submaps:
        raise ValueError(f"{label}: graph has no submaps")
    if not isinstance(edges, list):
        raise ValueError(f"{label}: graph edges must be a list")
    nodes: dict[str, dict[str, Any]] = {}
    for index, row in enumerate(submaps):
        node_id = str(row.get("id", ""))
        if not node_id or node_id in nodes:
            raise ValueError(f"{label}: missing or duplicate submap id at {index}")
        session = str(row.get("session_id", ""))
        if not session:
            raise ValueError(f"{label}: submap {node_id} has no session_id")
        nodes[node_id] = {**row, "id": node_id, "session_id": session,
                          "transform": as_transform(row.get("pose"), f"{label}:{node_id}:pose")}
    normalized_edges: list[dict[str, Any]] = []
    edge_ids: set[str] = set()
    for index, row in enumerate(edges):
        edge_id = str(row.get("id", ""))
        if not edge_id or edge_id in edge_ids:
            raise ValueError(f"{label}: missing or duplicate edge id at {index}")
        edge_ids.add(edge_id)
        source, target = str(row.get("from", "")), str(row.get("to", ""))
        if source not in nodes or target not in nodes or source == target:
            raise ValueError(f"{label}: edge {edge_id} has invalid endpoints")
        edge_type = str(row.get("type", ""))
        if edge_type not in EDGE_TYPES:
            raise ValueError(f"{label}: edge {edge_id} has unsupported type {edge_type!r}")
        normalized_edges.append({**row, "id": edge_id, "from": source, "to": target,
                                 "type": edge_type, "role": _role(row),
                                 "region_group": str(row.get("region_group") or f"edge:{edge_id}")})
    return {"payload": graph, "nodes": nodes, "edges": normalized_edges,
            "hash": canonical_manifest_sha256(graph)}


def connected_components(node_ids: Iterable[str], edges: Iterable[dict[str, Any]]) -> list[list[str]]:
    adjacency = {str(node_id): set() for node_id in node_ids}
    for edge in edges:
        adjacency[edge["from"]].add(edge["to"])
        adjacency[edge["to"]].add(edge["from"])
    remaining = set(adjacency)
    result: list[list[str]] = []
    while remaining:
        root = min(remaining)
        stack, component = [root], []
        remaining.remove(root)
        while stack:
            current = stack.pop()
            component.append(current)
            for neighbor in sorted(adjacency[current]):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)
        result.append(sorted(component))
    return sorted(result, key=lambda values: values[0])


def structural_bridge_map(graph: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Independently determine which training structural edges are cut edges."""
    training = [edge for edge in graph["edges"] if edge["role"] == "train"]

    def alternate_path(source: str, target: str, excluded: str,
                       non_structural_only: bool = False) -> bool:
        adjacency: dict[str, set[str]] = {}
        for edge in training:
            if edge["id"] == excluded or (non_structural_only and edge["type"] == "structural"):
                continue
            adjacency.setdefault(edge["from"], set()).add(edge["to"])
            adjacency.setdefault(edge["to"], set()).add(edge["from"])
        stack, visited = [source], {source}
        while stack:
            current = stack.pop()
            if current == target:
                return True
            for neighbor in adjacency.get(current, ()):
                if neighbor not in visited:
                    visited.add(neighbor)
                    stack.append(neighbor)
        return False

    result = {}
    for edge in graph["edges"]:
        if edge["type"] != "structural":
            continue
        is_training = edge["role"] == "train"
        alternate = (alternate_path(edge["from"], edge["to"], edge["id"])
                     if is_training else False)
        non_structural = (alternate_path(edge["from"], edge["to"], edge["id"], True)
                          if is_training else False)
        declared = edge.get("provenance", {}).get("graph_bridge")
        result[edge["id"]] = {
            "computed_graph_bridge": bool(is_training and not alternate),
            "cycle_supported": bool(alternate),
            "non_structural_alternate_path": bool(non_structural),
            "declared_graph_bridge": declared,
            "declaration_matches": declared is None or bool(declared) == bool(is_training and not alternate),
        }
    return result


def mixed_region_map(graph: dict[str, Any]) -> dict[str, bool]:
    roles: dict[str, set[str]] = {}
    for edge in graph["edges"]:
        if edge["type"] != "odometry":
            roles.setdefault(edge["region_group"], set()).add(edge["role"])
    return {region: {"train", "heldout"} <= region_roles
            for region, region_roles in roles.items()}


def _finite_number(value: Any, field: str, allow_none: bool = False) -> float | None:
    if value is None and allow_none:
        return None
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{field} must be finite")
    return number


def _report_variants(report: dict[str, Any], label: str) -> dict[str, dict[str, Any]]:
    variants = report.get("variants")
    if isinstance(variants, dict) and variants:
        return variants
    # Supporting a single saved variant makes the summarizer useful for a
    # deliberately isolated sensitivity solve without weakening validation.
    if isinstance(report.get("poses"), dict) and isinstance(report.get("edges"), list):
        return {str(report.get("method") or label): report}
    raise ValueError(f"{label}: backend report has no variants")


def validate_report(report: dict[str, Any], graph: dict[str, Any], label: str) -> None:
    reported_hash = report.get("provenance", {}).get("canonical_manifest_sha256")
    if reported_hash != graph["hash"]:
        raise ValueError(
            f"{label}: backend canonical manifest hash does not match its graph")
    training_ids = {edge["id"] for edge in graph["edges"] if edge["role"] == "train"}
    heldout_ids = {edge["id"] for edge in graph["edges"] if edge["role"] == "heldout"}
    all_ids = training_ids | heldout_ids
    node_ids = set(graph["nodes"])
    for method, variant in _report_variants(report, label).items():
        prefix = f"{label}:{method}"
        if variant.get("heldout_edge_count_in_objective") != 0:
            raise ValueError(f"{prefix}: heldout edge entered objective")
        if variant.get("fit_edge_ids_complete") is not True:
            raise ValueError(f"{prefix}: fit edge inventory is not declared complete")
        if set(map(str, variant.get("fit_edge_ids", []))) != training_ids:
            raise ValueError(f"{prefix}: fit_edge_ids differ from graph training edges")
        if set(map(str, variant.get("heldout_edge_ids", []))) != heldout_ids:
            raise ValueError(f"{prefix}: heldout_edge_ids differ from graph")
        poses = variant.get("poses")
        if not isinstance(poses, dict) or set(map(str, poses)) != node_ids:
            raise ValueError(f"{prefix}: pose inventory differs from graph submaps")
        rows = variant.get("edges")
        if not isinstance(rows, list):
            raise ValueError(f"{prefix}: edge diagnostics are missing")
        by_id = {str(row.get("id")): row for row in rows}
        if len(by_id) != len(rows) or set(by_id) != all_ids:
            raise ValueError(f"{prefix}: diagnostic edge inventory differs from graph")
        for edge in graph["edges"]:
            row = by_id[edge["id"]]
            for field in ("from", "to", "type", "role", "region_group"):
                if str(row.get(field)) != str(edge[field]):
                    raise ValueError(f"{prefix}:{edge['id']}: {field} differs from graph")
            used = row.get("used_for_fit")
            if used is not (edge["role"] == "train"):
                raise ValueError(f"{prefix}:{edge['id']}: used_for_fit contradicts role")
            weight = row.get("information_weight")
            if edge["role"] == "heldout" and weight is not None:
                raise ValueError(f"{prefix}:{edge['id']}: heldout edge has a fit weight")
            if edge["role"] == "train" and edge["type"] == "odometry" and float(weight) != 1.:
                raise ValueError(f"{prefix}:{edge['id']}: odometry was downweighted")
            _finite_number(row.get("translation_error_m"), f"{prefix}:translation")
            _finite_number(row.get("rotation_error_deg"), f"{prefix}:rotation")
            _finite_number(row.get("chi2_before_robust_weight"), f"{prefix}:chi2")


def _stats(values: Iterable[float]) -> dict[str, float | int | None]:
    array = np.asarray(list(values), dtype=float)
    if not len(array):
        return {"count": 0, "median": None, "p90": None, "maximum": None}
    if not np.isfinite(array).all():
        raise ValueError("non-finite value in aggregate")
    return {"count": int(len(array)), "median": float(np.median(array)),
            "p90": float(np.percentile(array, 90)), "maximum": float(np.max(array))}


def _candidate_id(graph_label: str, report_label: str, method: str) -> str:
    if graph_label == "canonical":
        return f"{report_label}/{method}"
    return f"sensitivity:{graph_label}/{method}"


def summarize_candidate(graph_label: str, report_label: str, method: str,
                        graph: dict[str, Any], variant: dict[str, Any]) -> tuple[dict, list, list, list]:
    candidate_id = _candidate_id(graph_label, report_label, method)
    mixed = mixed_region_map(graph)
    bridges = structural_bridge_map(graph)
    diagnostics = {str(row["id"]): row for row in variant["edges"]}

    edge_rows = []
    for edge in graph["edges"]:
        diagnostic = diagnostics[edge["id"]]
        bridge = bridges.get(edge["id"], {})
        is_mixed = bool(mixed.get(edge["region_group"], False))
        is_graph_bridge = bool(bridge.get("computed_graph_bridge", False))
        reasons = []
        if edge["role"] != "heldout":
            reasons.append("not_heldout")
        if edge["type"] == "odometry":
            reasons.append("odometry_backbone")
        if is_mixed:
            reasons.append("mixed_train_heldout_region")
        if is_graph_bridge:
            reasons.append("structural_graph_bridge_transition")
        eligible = not reasons
        weight = diagnostic.get("information_weight")
        edge_rows.append({
            "candidate_id": candidate_id,
            "graph_label": graph_label,
            "report_label": report_label,
            "method": method,
            "edge_id": edge["id"],
            "from": edge["from"],
            "to": edge["to"],
            "type": edge["type"],
            "role": edge["role"],
            "region_group": edge["region_group"],
            "used_for_fit": bool(diagnostic["used_for_fit"]),
            "information_weight": None if weight is None else _finite_number(weight, "weight"),
            "translation_error_m": _finite_number(diagnostic["translation_error_m"], "translation"),
            "rotation_error_deg": _finite_number(diagnostic["rotation_error_deg"], "rotation"),
            "chi2_before_robust_weight": _finite_number(
                diagnostic["chi2_before_robust_weight"], "chi2"),
            "mixed_train_heldout_region": is_mixed,
            "structural_graph_bridge_transition": is_graph_bridge,
            "approval_metric_eligible": eligible,
            "approval_metric_exclusion_reason": "|".join(reasons),
        })

    correction_rows = []
    for node_id, node in sorted(graph["nodes"].items()):
        initial = node["transform"]
        candidate = as_transform(variant["poses"][node_id], f"{candidate_id}:{node_id}:pose")
        delta_translation = candidate[:3, 3] - initial[:3, 3]
        angle = Rotation.from_matrix(candidate[:3, :3] @ initial[:3, :3].T).magnitude()
        correction_rows.append({
            "candidate_id": candidate_id,
            "graph_label": graph_label,
            "report_label": report_label,
            "method": method,
            "submap_id": node_id,
            "session_id": node["session_id"],
            "dx_m": float(delta_translation[0]),
            "dy_m": float(delta_translation[1]),
            "dz_m": float(delta_translation[2]),
            "translation_correction_m": float(np.linalg.norm(delta_translation)),
            "rotation_correction_deg": float(np.degrees(angle)),
        })

    # Effective components are diagnostic only.  In particular, switching a
    # sole bridge off is reported as loss of support, never treated as proof
    # that either side was correctly placed.
    active_edges = []
    for edge, row in zip(graph["edges"], edge_rows):
        if edge["role"] != "train":
            continue
        weight = row["information_weight"]
        if weight is not None and weight >= ACTIVE_INFORMATION_WEIGHT:
            active_edges.append(edge)
    nominal_components = connected_components(
        graph["nodes"], [edge for edge in graph["edges"] if edge["role"] == "train"])
    active_components = connected_components(graph["nodes"], active_edges)

    heldout = [row for row in edge_rows if row["role"] == "heldout"]
    eligible = [row for row in heldout if row["approval_metric_eligible"]]
    correction_translation = [row["translation_correction_m"] for row in correction_rows]
    correction_rotation = [row["rotation_correction_deg"] for row in correction_rows]
    summary = {
        "candidate_id": candidate_id,
        "graph_label": graph_label,
        "report_label": report_label,
        "method": method,
        "accepted": False,
        "navigation": False,
        "status": "experimental_diagnostic_only",
        "nominal_component_count": len(nominal_components),
        "effective_component_count_at_weight_0_01": len(active_components),
        "nominal_components": nominal_components,
        "effective_components_at_weight_0_01": active_components,
        "active_information_weight_threshold": ACTIVE_INFORMATION_WEIGHT,
        "structural_graph_bridge_count": sum(
            row["structural_graph_bridge_transition"] for row in edge_rows),
        "structural_graph_bridges_below_active_weight": sum(
            row["structural_graph_bridge_transition"]
            and (row["information_weight"] or 0.) < ACTIVE_INFORMATION_WEIGHT
            for row in edge_rows),
        "mixed_train_heldout_region_count": len({
            row["region_group"] for row in edge_rows if row["mixed_train_heldout_region"]}),
        "heldout_observation_count": len(heldout),
        "approval_metric_eligible_heldout_count": len(eligible),
        "excluded_heldout_count": len(heldout) - len(eligible),
        "approval_metric_subset": {
            "status": "conditional_diagnostic_only_not_an_approval",
            "translation_error_m": _stats(row["translation_error_m"] for row in eligible),
            "rotation_error_deg": _stats(row["rotation_error_deg"] for row in eligible),
        },
        "all_heldout_diagnostic_only": {
            "translation_error_m": _stats(row["translation_error_m"] for row in heldout),
            "rotation_error_deg": _stats(row["rotation_error_deg"] for row in heldout),
        },
        "pose_correction": {
            "translation_m": _stats(correction_translation),
            "rotation_deg": _stats(correction_rotation),
        },
        "solver_iteration_count": len(variant.get("solver_iterations", [])),
        "solver_last_iteration": (variant.get("solver_iterations") or [None])[-1],
    }
    return summary, edge_rows, correction_rows, active_components


def summarize_regions(edge_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for row in edge_rows:
        groups.setdefault((row["candidate_id"], row["region_group"], row["type"]), []).append(row)
    output = []
    for (candidate, region, edge_type), rows in sorted(groups.items()):
        weights = [row["information_weight"] for row in rows
                   if row["information_weight"] is not None]
        chi2 = [row["chi2_before_robust_weight"] for row in rows]
        translation = [row["translation_error_m"] for row in rows]
        rotation = [row["rotation_error_deg"] for row in rows]
        output.append({
            "candidate_id": candidate,
            "region_group": region,
            "type": edge_type,
            "edge_count": len(rows),
            "roles": "|".join(sorted({row["role"] for row in rows})),
            "mixed_train_heldout_region": any(row["mixed_train_heldout_region"] for row in rows),
            "structural_graph_bridge_transition": any(
                row["structural_graph_bridge_transition"] for row in rows),
            "approval_metric_eligible_count": sum(row["approval_metric_eligible"] for row in rows),
            "information_weight_minimum": min(weights) if weights else None,
            "information_weight_median": float(np.median(weights)) if weights else None,
            "information_weight_maximum": max(weights) if weights else None,
            "chi2_median": float(np.median(chi2)),
            "chi2_p90": float(np.percentile(chi2, 90)),
            "chi2_maximum": max(chi2),
            "translation_error_median": float(np.median(translation)),
            "rotation_error_deg_median": float(np.median(rotation)),
        })
    return output


def graph_diagnostics(label: str, graph: dict[str, Any]) -> tuple[dict, list, list]:
    training = [edge for edge in graph["edges"] if edge["role"] == "train"]
    components = connected_components(graph["nodes"], training)
    component_rows = []
    for index, component in enumerate(components):
        sessions = sorted({graph["nodes"][node]["session_id"] for node in component})
        for node in component:
            component_rows.append({"graph_label": label, "component_index": index,
                                   "submap_id": node,
                                   "session_id": graph["nodes"][node]["session_id"],
                                   "component_sessions": "|".join(sessions),
                                   "component_node_count": len(component)})
    bridge_info = structural_bridge_map(graph)
    bridge_rows = []
    for edge in graph["edges"]:
        if edge["type"] != "structural":
            continue
        info = bridge_info[edge["id"]]
        bridge_rows.append({"graph_label": label, "edge_id": edge["id"],
                            "from": edge["from"], "to": edge["to"],
                            "role": edge["role"], "region_group": edge["region_group"], **info,
                            "approval_metric_eligible": False,
                            "approval_metric_exclusion_reason": (
                                "structural_graph_bridge_transition" if info["computed_graph_bridge"]
                                else "not_a_locked_heldout_observation")})
    mixed = sorted(region for region, value in mixed_region_map(graph).items() if value)
    summary = {
        "graph_label": label,
        "submap_count": len(graph["nodes"]),
        "edge_count": len(graph["edges"]),
        "training_edge_count": len(training),
        "heldout_edge_count": sum(edge["role"] == "heldout" for edge in graph["edges"]),
        "component_count": len(components),
        "components": components,
        "session_components": [sorted({graph["nodes"][node]["session_id"] for node in component})
                               for component in components],
        "mixed_train_heldout_regions": mixed,
        "structural_transition_count": len(bridge_rows),
        "structural_graph_bridge_count": sum(row["computed_graph_bridge"] for row in bridge_rows),
        "structural_declaration_mismatches": [row["edge_id"] for row in bridge_rows
                                              if not row["declaration_matches"]],
    }
    return summary, component_rows, bridge_rows


def summarize_fixed_patch_report(evaluation_label: str, payload: dict[str, Any],
                                 graph_file_sha256: str,
                                 candidates: list[dict[str, Any]]) -> tuple[dict, list, list]:
    """Normalize a fixed finite-patch evaluation and expose coverage loss.

    Patch values are conditional geometric diagnostics.  Missing candidate
    poses are kept as missing and reduce coverage; they are never silently
    dropped before medians are displayed.
    """
    if payload.get("accepted") is not False or payload.get("navigation") is not False:
        raise ValueError(f"{evaluation_label}: fixed-patch report is not explicitly unapproved")
    reported_graph_hash = payload.get("inputs", {}).get("graph_manifest_sha256")
    if reported_graph_hash != graph_file_sha256:
        raise ValueError(f"{evaluation_label}: fixed-patch graph file hash does not match paired graph")
    total = int(payload.get("fixed_patch_count", 0))
    comparisons = payload.get("comparisons")
    stages = [str(value) for value in payload.get("stages", [])]
    candidate_stages = [str(value) for value in payload.get("candidate_stages", [])]
    aggregates = payload.get("stage_aggregates", {})
    overall = aggregates.get("overall", {})
    if total <= 0 or not isinstance(comparisons, list) or len(comparisons) != total:
        raise ValueError(f"{evaluation_label}: invalid fixed-patch inventory")
    if not candidate_stages or any(stage not in stages or stage not in overall
                                   for stage in candidate_stages):
        raise ValueError(f"{evaluation_label}: invalid candidate stage inventory")
    baseline_stage = "previous_preview" if "previous_preview" in overall else stages[0]
    baseline_aggregate = overall[baseline_stage]
    severe_by_stage = payload.get("severe_regressions_by_candidate", {})

    candidate_rows = []
    comparison_rows = []
    headline_rows = []
    for stage in candidate_stages:
        aggregate = overall[stage]
        available = int(aggregate.get("available_count", 0))
        missing = int(aggregate.get("missing_count", total - available))
        coverage = _finite_number(aggregate.get("coverage_fraction"), "fixed patch coverage")
        if available + missing != total or not np.isclose(coverage, available / total):
            raise ValueError(f"{evaluation_label}:{stage}: inconsistent fixed-patch coverage")
        severe = severe_by_stage.get(stage, [])
        if not isinstance(severe, list):
            raise ValueError(f"{evaluation_label}:{stage}: severe regression inventory is not a list")
        matched = [candidate["candidate_id"] for candidate in candidates
                   if candidate["graph_label"] == evaluation_label and candidate["method"] == stage]
        partition = aggregates.get("by_graph_partition", {}).get(
            "graph_independent_historical", {}).get(stage, {})
        candidate_row = {
            "evaluation_label": evaluation_label,
            "candidate_stage": stage,
            "matched_candidate_ids": "|".join(matched),
            "total_fixed_patches": total,
            "available_count": available,
            "missing_count": missing,
            "coverage_fraction": coverage,
            "coverage_percent": 100. * coverage,
            "coverage_change_percentage_points_vs_previous": 100. * (
                coverage - float(baseline_aggregate["coverage_fraction"])),
            "median_m": _finite_number(aggregate.get("median_m"), "fixed patch median", True),
            "p90_m": _finite_number(aggregate.get("p90_m"), "fixed patch p90", True),
            "maximum_m": _finite_number(aggregate.get("maximum_m"), "fixed patch maximum", True),
            "severe_regression_count": len(severe),
            "severe_regression_threshold_m": float(payload.get("severe_regression_threshold_m", .05)),
            "independent_historical_total": partition.get("total_fixed_patches"),
            "independent_historical_available": partition.get("available_count"),
            "independent_historical_coverage_fraction": partition.get("coverage_fraction"),
            "accepted": False,
            "navigation": False,
        }
        candidate_rows.append(candidate_row)

        for patch in comparisons:
            values = patch.get("stage_plane_abs_median_m", {})
            candidate_delta = patch.get("candidate_deltas", {}).get(stage, {})
            value = values.get(stage)
            available_patch = bool(candidate_delta.get("available", value is not None))
            if available_patch != (value is not None):
                raise ValueError(
                    f"{evaluation_label}:{stage}:{patch.get('session')}:{patch.get('a')}:{patch.get('b')}: "
                    "availability contradicts value")
            row = {
                "evaluation_label": evaluation_label,
                "candidate_stage": stage,
                "session": str(patch.get("session")),
                "node_a": int(patch.get("a")),
                "node_b": int(patch.get("b")),
                "kind": str(patch.get("kind")),
                "plane_a": patch.get("plane_a"),
                "plane_b": patch.get("plane_b"),
                "graph_partition_status": str(patch.get("graph_partition_status")),
                "independent_ground_truth": bool(patch.get("independent_ground_truth", False)),
                "available": available_patch,
                "previous_preview_m": values.get("previous_preview"),
                "selected_initial_m": values.get("selected_initial"),
                "candidate_m": value,
                "best_baseline_m": patch.get("best_baseline_m"),
                "delta_vs_previous_m": candidate_delta.get("delta_vs_previous_m"),
                "delta_vs_initial_m": candidate_delta.get("delta_vs_initial_m"),
                "delta_vs_best_baseline_m": candidate_delta.get("delta_vs_best_baseline_m"),
                "severe_regression": bool(candidate_delta.get("severe_regression", False)),
                "correlation_reason": patch.get("graph_correlation_reason"),
            }
            comparison_rows.append(row)
            if row["severe_regression"] and row["delta_vs_best_baseline_m"] is not None:
                headline_rows.append(row)

    headline_rows.sort(key=lambda row: (-float(row["delta_vs_best_baseline_m"]),
                                        row["session"], row["node_a"], row["node_b"],
                                        row["kind"], row["candidate_stage"]))
    headline = None
    if headline_rows:
        row = headline_rows[0]
        headline = {
            "candidate_stage": row["candidate_stage"],
            "patch": f'{row["session"]}:{row["node_a"]}<->{row["node_b"]}:{row["kind"]}',
            "graph_partition_status": row["graph_partition_status"],
            "independent_ground_truth": row["independent_ground_truth"],
            "best_baseline_m": row["best_baseline_m"],
            "candidate_m": row["candidate_m"],
            "delta_m": row["delta_vs_best_baseline_m"],
            "best_baseline_cm": 100. * float(row["best_baseline_m"]),
            "candidate_cm": 100. * float(row["candidate_m"]),
            "delta_cm": 100. * float(row["delta_vs_best_baseline_m"]),
        }
    summary = {
        "evaluation_label": evaluation_label,
        "status": "conditional_fixed_patch_diagnostic_only",
        "accepted": False,
        "navigation": False,
        "fixed_patch_count": total,
        "baseline_stage": baseline_stage,
        "candidate_stages": candidate_stages,
        "candidate_summary": candidate_rows,
        "largest_severe_regression": headline,
        "severe_regression_threshold_m": float(payload.get("severe_regression_threshold_m", .05)),
        "coverage_policy": (
            "all fixed patch IDs remain in the denominator; unavailable candidate poses reduce coverage"),
        "metric_definition": (
            "per-patch symmetric absolute plane-distance median in metres; conditional historical evidence, "
            "not surveyed ground truth"),
        "validation_partition_definition": payload.get("validation_partition_definition"),
        "limitations": payload.get("limitations", []),
    }
    return summary, candidate_rows, comparison_rows


CSV_FIELDS = {
    "edge_diagnostics.csv": [
        "candidate_id", "graph_label", "report_label", "method", "edge_id", "from", "to",
        "type", "role", "region_group", "used_for_fit", "information_weight",
        "translation_error_m", "rotation_error_deg", "chi2_before_robust_weight",
        "mixed_train_heldout_region", "structural_graph_bridge_transition",
        "approval_metric_eligible", "approval_metric_exclusion_reason",
    ],
    "region_summary.csv": [
        "candidate_id", "region_group", "type", "edge_count", "roles",
        "mixed_train_heldout_region", "structural_graph_bridge_transition",
        "approval_metric_eligible_count", "information_weight_minimum",
        "information_weight_median", "information_weight_maximum", "chi2_median", "chi2_p90",
        "chi2_maximum", "translation_error_median", "rotation_error_deg_median",
    ],
    "corrections.csv": [
        "candidate_id", "graph_label", "report_label", "method", "submap_id", "session_id",
        "dx_m", "dy_m", "dz_m", "translation_correction_m", "rotation_correction_deg",
    ],
    "candidate_comparison.csv": [
        "candidate_id", "graph_label", "report_label", "method", "nominal_component_count",
        "effective_component_count_at_weight_0_01", "heldout_observation_count",
        "approval_metric_eligible_heldout_count", "excluded_heldout_count",
        "heldout_translation_median_m", "heldout_translation_p90_m", "heldout_translation_maximum_m",
        "heldout_rotation_median_deg", "heldout_rotation_p90_deg", "heldout_rotation_maximum_deg",
        "correction_translation_median_m", "correction_translation_p90_m",
        "correction_rotation_median_deg", "correction_rotation_p90_deg", "accepted", "navigation",
    ],
    "components.csv": [
        "graph_label", "component_index", "submap_id", "session_id", "component_sessions",
        "component_node_count",
    ],
    "structural_bridges.csv": [
        "graph_label", "edge_id", "from", "to", "role", "region_group",
        "computed_graph_bridge", "cycle_supported", "non_structural_alternate_path",
        "declared_graph_bridge", "declaration_matches", "approval_metric_eligible",
        "approval_metric_exclusion_reason",
    ],
    "fixed_patch_candidate_summary.csv": [
        "evaluation_label", "candidate_stage", "matched_candidate_ids", "total_fixed_patches",
        "available_count", "missing_count", "coverage_fraction", "coverage_percent",
        "coverage_change_percentage_points_vs_previous", "median_m", "p90_m", "maximum_m",
        "severe_regression_count", "severe_regression_threshold_m",
        "independent_historical_total", "independent_historical_available",
        "independent_historical_coverage_fraction", "accepted", "navigation",
    ],
    "fixed_patch_comparisons.csv": [
        "evaluation_label", "candidate_stage", "session", "node_a", "node_b", "kind",
        "plane_a", "plane_b", "graph_partition_status", "independent_ground_truth", "available",
        "previous_preview_m", "selected_initial_m", "candidate_m", "best_baseline_m",
        "delta_vs_previous_m", "delta_vs_initial_m", "delta_vs_best_baseline_m",
        "severe_regression", "correlation_reason",
    ],
}


def flatten_candidate(candidate: dict[str, Any]) -> dict[str, Any]:
    eligible = candidate["approval_metric_subset"]
    correction = candidate["pose_correction"]
    return {
        "candidate_id": candidate["candidate_id"], "graph_label": candidate["graph_label"],
        "report_label": candidate["report_label"], "method": candidate["method"],
        "nominal_component_count": candidate["nominal_component_count"],
        "effective_component_count_at_weight_0_01": candidate["effective_component_count_at_weight_0_01"],
        "heldout_observation_count": candidate["heldout_observation_count"],
        "approval_metric_eligible_heldout_count": candidate["approval_metric_eligible_heldout_count"],
        "excluded_heldout_count": candidate["excluded_heldout_count"],
        "heldout_translation_median_m": eligible["translation_error_m"]["median"],
        "heldout_translation_p90_m": eligible["translation_error_m"]["p90"],
        "heldout_translation_maximum_m": eligible["translation_error_m"]["maximum"],
        "heldout_rotation_median_deg": eligible["rotation_error_deg"]["median"],
        "heldout_rotation_p90_deg": eligible["rotation_error_deg"]["p90"],
        "heldout_rotation_maximum_deg": eligible["rotation_error_deg"]["maximum"],
        "correction_translation_median_m": correction["translation_m"]["median"],
        "correction_translation_p90_m": correction["translation_m"]["p90"],
        "correction_rotation_median_deg": correction["rotation_deg"]["median"],
        "correction_rotation_p90_deg": correction["rotation_deg"]["p90"],
        "accepted": False, "navigation": False,
    }


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _save_figure(fig: plt.Figure, path: Path) -> None:
    fig.savefig(path, dpi=170, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def _candidate_colors(candidate_ids: list[str]) -> dict[str, Any]:
    cmap = plt.get_cmap("tab10")
    return {name: cmap(index % 10) for index, name in enumerate(candidate_ids)}


def _display_candidate(value: str) -> str:
    """Wrap stable candidate ids for figures without altering CSV/JSON ids."""
    return value.replace("sensitivity:", "sens:").replace("/", "\n", 1)


def render_edge_type(edge_rows: list[dict[str, Any]], path: Path) -> dict[str, Any]:
    candidates = list(dict.fromkeys(row["candidate_id"] for row in edge_rows))
    candidate_colors = _candidate_colors(candidates)
    types = [kind for kind in EDGE_TYPES if any(row["type"] == kind for row in edge_rows)]
    fig, axes = plt.subplots(1, 2, figsize=(max(10, len(types) * 2.2), 5.2))
    offsets = np.linspace(-.32, .32, max(1, len(candidates)))
    for candidate_index, candidate in enumerate(candidates):
        for type_index, kind in enumerate(types):
            rows = [row for row in edge_rows if row["candidate_id"] == candidate and row["type"] == kind]
            weights = [row["information_weight"] for row in rows if row["information_weight"] is not None]
            chi2 = [row["chi2_before_robust_weight"] for row in rows]
            x = type_index + offsets[candidate_index]
            if weights:
                shown_weights = np.maximum(weights, 1e-12)
                axes[0].scatter([x] * len(weights), shown_weights, s=18, alpha=.45,
                                color=candidate_colors[candidate])
                axes[0].plot(x, max(float(np.median(weights)), 1e-12), marker="_", ms=14,
                             color=candidate_colors[candidate], lw=2)
            if chi2:
                axes[1].scatter([x] * len(chi2), np.maximum(chi2, 1e-12), s=18, alpha=.45,
                                color=candidate_colors[candidate], marker="o")
                axes[1].plot(x, max(float(np.median(chi2)), 1e-12), marker="_", ms=14,
                             color=candidate_colors[candidate], lw=2)
    for axis in axes:
        axis.set_xticks(range(len(types)), types, rotation=20, ha="right")
        axis.grid(axis="y", alpha=.25)
    axes[0].set(title="Final information weights by edge type", ylabel="information weight")
    axes[0].set_yscale("log")
    axes[1].set(title="Unweighted chi-square by edge type", ylabel="chi-square")
    axes[1].set_yscale("log")
    handles = [Line2D([0], [0], marker="o", color="none", markerfacecolor=candidate_colors[name],
                      label=name) for name in candidates]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(.5, 1.05), ncol=min(4, len(handles)))
    fig.suptitle("Robust backend diagnostics (points are actual supplied edges)", y=1.13)
    _save_figure(fig, path)
    return {"candidate_count": len(candidates), "edge_count": len(edge_rows), "edge_types": types}


def render_edge_regions(region_rows: list[dict[str, Any]], path: Path,
                        maximum_display_regions: int = 40) -> dict[str, Any]:
    candidates = list(dict.fromkeys(row["candidate_id"] for row in region_rows))
    # A physical region can legitimately contain more than one edge type.
    # Keep those categories distinct instead of silently letting the last row
    # win in a dictionary lookup.
    region_types = sorted({(row["region_group"], row["type"]) for row in region_rows})
    score = {key: max(row["chi2_p90"] for row in region_rows
                      if (row["region_group"], row["type"]) == key)
             for key in region_types}
    displayed = sorted(region_types, key=lambda key: (-score[key], key))[:maximum_display_regions]
    chi2 = np.full((len(displayed), len(candidates)), np.nan)
    weights = np.full_like(chi2, np.nan)
    lookup = {(row["region_group"], row["type"], row["candidate_id"]): row
              for row in region_rows}
    for y, (region, edge_type) in enumerate(displayed):
        for x, candidate in enumerate(candidates):
            row = lookup.get((region, edge_type, candidate))
            if row:
                chi2[y, x] = math.log10(max(row["chi2_median"], 1e-12))
                if row["information_weight_median"] is not None:
                    weights[y, x] = row["information_weight_median"]
    height = max(5., min(22., 1.3 + .38 * len(displayed)))
    fig, axes = plt.subplots(1, 2, figsize=(max(15., 4.5 + 1.8 * len(candidates)), height))
    for axis, matrix, title, cmap in (
        (axes[0], chi2, "log10 median chi-square", "magma"),
        (axes[1], weights, "median information weight", "viridis"),
    ):
        shown = np.ma.masked_invalid(matrix)
        image = axis.imshow(shown, aspect="auto", interpolation="nearest", cmap=cmap)
        axis.set_xticks(range(len(candidates)), [_display_candidate(value) for value in candidates],
                        rotation=25, ha="right", fontsize=8)
        axis.set_yticks(range(len(displayed)),
                        [f"{region} [{edge_type}]" for region, edge_type in displayed], fontsize=7)
        axis.set_title(title)
        fig.colorbar(image, ax=axis, fraction=.035, pad=.02)
    # Repeating long physical-region names on the right panel makes the two
    # heatmaps collide.  Rows are shared exactly, so label them once on left.
    axes[1].tick_params(axis="y", labelleft=False)
    fig.subplots_adjust(left=.36, right=.96, bottom=.12, top=.90, wspace=.24)
    fig.suptitle(
        f"Physical-region/type diagnostics — highest {len(displayed)} of {len(region_types)} rows by p90 chi-square\n"
        "Full untruncated values are in region_summary.csv", fontsize=10, y=.97)
    _save_figure(fig, path)
    return {"region_type_row_count": len(region_types), "displayed_region_type_row_count": len(displayed),
            "displayed_region_types": [
                {"region_group": region, "type": edge_type} for region, edge_type in displayed],
            "selection_rule": (
                f"highest {maximum_display_regions} region/type rows by maximum candidate p90 chi-square")}


def render_corrections(correction_rows: list[dict[str, Any]], path: Path) -> dict[str, Any]:
    candidates = list(dict.fromkeys(row["candidate_id"] for row in correction_rows))
    sessions = sorted({row["session_id"] for row in correction_rows})
    fig, axes = plt.subplots(2, 1, figsize=(max(10., len(sessions) * 2.2), 9), sharex=True)
    width = min(.7 / max(1, len(candidates)), .22)
    centers = np.arange(len(sessions))
    colors = _candidate_colors(candidates)
    for candidate_index, candidate in enumerate(candidates):
        offset = (candidate_index - (len(candidates) - 1) / 2.) * width
        for axis, field in zip(axes, ("translation_correction_m", "rotation_correction_deg")):
            values = [[row[field] for row in correction_rows
                       if row["candidate_id"] == candidate and row["session_id"] == session]
                      for session in sessions]
            nonempty = [(centers[index] + offset, item) for index, item in enumerate(values) if item]
            if nonempty:
                positions, data = zip(*nonempty)
                boxes = axis.boxplot(data, positions=positions, widths=width * .85,
                                     showfliers=True, patch_artist=True, manage_ticks=False)
                for box in boxes["boxes"]:
                    box.set(facecolor=colors[candidate], alpha=.45, edgecolor=colors[candidate])
                for median in boxes["medians"]:
                    median.set(color=colors[candidate], linewidth=1.5)
    axes[0].set(ylabel="translation correction (m)", title="Submap pose correction distributions by session")
    axes[1].set(ylabel="rotation correction (deg)")
    axes[1].set_xticks(centers, sessions, rotation=20, ha="right")
    for axis in axes:
        axis.grid(axis="y", alpha=.25)
    handles = [Line2D([0], [0], color=colors[name], lw=7, alpha=.5, label=name) for name in candidates]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(.5, 1.01), ncol=min(4, len(handles)))
    _save_figure(fig, path)
    return {"candidate_count": len(candidates), "session_count": len(sessions),
            "correction_count": len(correction_rows), "sessions": sessions}


def render_heldout(edge_rows: list[dict[str, Any]], path: Path) -> dict[str, Any]:
    rows = [row for row in edge_rows if row["role"] == "heldout"]
    candidates = list(dict.fromkeys(row["candidate_id"] for row in edge_rows))
    edge_ids = sorted({row["edge_id"] for row in rows})
    fig, axes = plt.subplots(2, 1, figsize=(max(10., 1.2 + .75 * len(edge_ids)), 8), sharex=True)
    colors = _candidate_colors(candidates)
    offsets = np.linspace(-.28, .28, max(1, len(candidates)))
    lookup = {(row["candidate_id"], row["edge_id"]): row for row in rows}
    for candidate_index, candidate in enumerate(candidates):
        for edge_index, edge_id in enumerate(edge_ids):
            row = lookup.get((candidate, edge_id))
            if row is None:
                continue
            marker = "o" if row["approval_metric_eligible"] else "x"
            x = edge_index + offsets[candidate_index]
            axes[0].scatter(x, row["translation_error_m"], color=colors[candidate], marker=marker, s=48)
            axes[1].scatter(x, row["rotation_error_deg"], color=colors[candidate], marker=marker, s=48)
    if not rows:
        for axis in axes:
            axis.text(.5, .5, "No held-out edges supplied", ha="center", va="center",
                      transform=axis.transAxes)
    axes[0].set(ylabel="translation residual (m)", title="Held-out residual comparison")
    axes[1].set(ylabel="rotation residual (deg)")
    axes[1].set_xticks(range(len(edge_ids)), edge_ids, rotation=35, ha="right", fontsize=8)
    for axis in axes:
        axis.grid(axis="y", alpha=.25)
    handles = [Line2D([0], [0], marker="o", color="none", markerfacecolor=colors[name],
                      label=name) for name in candidates]
    handles += [Line2D([0], [0], marker="o", color="black", lw=0, label="eligible diagnostic subset"),
                Line2D([0], [0], marker="x", color="black", lw=0,
                       label="excluded: train-overlap region / structural graph bridge")]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(.5, 1.03), ncol=min(4, len(handles)))
    _save_figure(fig, path)
    return {"heldout_row_count": len(rows), "heldout_edge_count": len(edge_ids),
            "eligible_row_count": sum(row["approval_metric_eligible"] for row in rows),
            "excluded_row_count": sum(not row["approval_metric_eligible"] for row in rows)}


def render_graphs(graphs: dict[str, dict[str, Any]], path: Path) -> dict[str, Any]:
    labels = list(graphs)
    columns = min(2, len(labels))
    rows_count = int(math.ceil(len(labels) / columns))
    fig, axes = plt.subplots(rows_count, columns, figsize=(8 * columns, 6 * rows_count), squeeze=False)
    details = {}
    for axis, label in zip(axes.flat, labels):
        graph = graphs[label]
        training = [edge for edge in graph["edges"] if edge["role"] == "train"]
        components = connected_components(graph["nodes"], training)
        component_index = {node: index for index, component in enumerate(components) for node in component}
        bridge_info = structural_bridge_map(graph)
        positions = {node: row["transform"][:2, 3] for node, row in graph["nodes"].items()}
        for edge in graph["edges"]:
            first, second = positions[edge["from"]], positions[edge["to"]]
            if edge["role"] == "heldout":
                axis.plot([first[0], second[0]], [first[1], second[1]], color="#9a9a9a",
                          lw=.7, ls=":", alpha=.7, zorder=1)
            elif edge["type"] == "structural":
                is_bridge = bridge_info[edge["id"]]["computed_graph_bridge"]
                axis.plot([first[0], second[0]], [first[1], second[1]],
                          color="#d62728" if is_bridge else "#8c56a8", lw=2.2,
                          ls="--", alpha=.9, zorder=2)
            else:
                axis.plot([first[0], second[0]], [first[1], second[1]],
                          color="#bcbcbc", lw=.6, alpha=.55, zorder=1)
        cmap = plt.get_cmap("tab10")
        for node, position in positions.items():
            axis.scatter(position[0], position[1], s=16, color=cmap(component_index[node] % 10), zorder=3)
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(alpha=.18)
        axis.set(title=f"{label}: {len(components)} component(s)", xlabel="world x (m)", ylabel="world y (m)")
        details[label] = {"component_count": len(components), "submap_count": len(positions),
                          "structural_graph_bridge_count": sum(
                              row["computed_graph_bridge"] for row in bridge_info.values())}
    for axis in axes.flat[len(labels):]:
        axis.axis("off")
    handles = [
        Line2D([0], [0], color="#bcbcbc", label="training edge"),
        Line2D([0], [0], color="#8c56a8", ls="--", lw=2, label="cycle-supported structural"),
        Line2D([0], [0], color="#d62728", ls="--", lw=2, label="structural graph bridge"),
        Line2D([0], [0], color="#9a9a9a", ls=":", label="held-out (not a connector)"),
    ]
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(.5, 1.01), ncol=4)
    fig.suptitle("Graph components and structural-transition support", y=1.04)
    _save_figure(fig, path)
    return details


def render_candidate_comparison(candidates: list[dict[str, Any]], path: Path) -> dict[str, Any]:
    labels = [row["candidate_id"] for row in candidates]
    fields = [
        ("approval_metric_subset", "translation_error_m", "median", "heldout translation median (m)"),
        ("approval_metric_subset", "translation_error_m", "p90", "heldout translation p90 (m)"),
        ("approval_metric_subset", "rotation_error_deg", "p90", "heldout rotation p90 (deg)"),
        ("pose_correction", "translation_m", "p90", "pose correction p90 (m)"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(max(11., 1.7 * len(labels)), 10.5))
    missing = {}
    colors = [_candidate_colors(labels)[label] for label in labels]
    for axis, (top, metric, statistic, title) in zip(axes.flat, fields):
        values = [candidate[top][metric][statistic] for candidate in candidates]
        shown = [0. if value is None else value for value in values]
        bars = axis.bar(range(len(labels)), shown, color=colors, alpha=.75)
        for index, (bar, value) in enumerate(zip(bars, values)):
            if value is None:
                bar.set_facecolor("none")
                bar.set_edgecolor("#888888")
                bar.set_hatch("//")
                axis.text(index, 0., "no eligible\nholdout", ha="center", va="bottom", fontsize=7)
                missing.setdefault(labels[index], []).append(title)
        axis.set_title(title)
        axis.set_xticks(range(len(labels)), [_display_candidate(value) for value in labels],
                        rotation=24, ha="right", fontsize=8)
        axis.grid(axis="y", alpha=.25)
    fig.subplots_adjust(hspace=.78, wspace=.20, bottom=.15, top=.91)
    fig.suptitle("Candidate comparison — conditional diagnostics only, never an approval", y=.97)
    _save_figure(fig, path)
    return {"candidate_count": len(candidates), "missing_metrics": missing,
            "policy": "only heldout edges outside mixed-role regions and structural graph bridges"}


def render_fixed_patch_coverage(rows: list[dict[str, Any]], path: Path) -> dict[str, Any]:
    labels = [f'{row["evaluation_label"]}\n{row["candidate_stage"]}' for row in rows]
    colors = [plt.get_cmap("tab10")(index % 10) for index in range(len(rows))]
    fig, axes = plt.subplots(1, 3, figsize=(max(13., 2.0 * len(rows)), 5.5))
    x = np.arange(len(rows))
    coverage = [100. * float(row["coverage_fraction"]) for row in rows]
    independent = [100. * float(row["independent_historical_coverage_fraction"])
                   if row["independent_historical_coverage_fraction"] is not None else 0.
                   for row in rows]
    severe = [int(row["severe_regression_count"]) for row in rows]
    for axis, values, title, ylabel in (
        (axes[0], coverage, "All fixed-patch coverage", "coverage (%)"),
        (axes[1], independent, "Graph-independent historical subset", "coverage (%)"),
        (axes[2], severe, "Severe regressions vs best baseline", "patch count"),
    ):
        bars = axis.bar(x, values, color=colors, alpha=.78)
        axis.set_xticks(x, labels, rotation=20, ha="right", fontsize=8)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=.25)
        for index, (bar, value) in enumerate(zip(bars, values)):
            if axis is axes[0]:
                text_value = f'{rows[index]["available_count"]}/{rows[index]["total_fixed_patches"]}\n{value:.1f}%'
            elif axis is axes[1]:
                text_value = (f'{rows[index]["independent_historical_available"]}/'
                              f'{rows[index]["independent_historical_total"]}\n{value:.1f}%')
            else:
                text_value = str(value)
            axis.text(bar.get_x() + bar.get_width() / 2., bar.get_height(), text_value,
                      ha="center", va="bottom", fontsize=8)
    axes[0].set_ylim(0., max(105., max(coverage, default=0.) * 1.16))
    axes[1].set_ylim(0., max(105., max(independent, default=0.) * 1.16))
    axes[2].set_ylim(0., max(1., max(severe, default=0.) * 1.2))
    fig.suptitle("Fixed finite-patch audit — missing poses stay in the denominator\n"
                 "Conditional historical evidence; never an approval", y=1.03)
    fig.subplots_adjust(bottom=.23, top=.82, wspace=.32)
    _save_figure(fig, path)
    return {"candidate_stage_count": len(rows),
            "minimum_coverage_fraction": min((float(row["coverage_fraction"]) for row in rows),
                                             default=None),
            "maximum_severe_regression_count": max(severe, default=0)}


def render_fixed_patch_regressions(rows: list[dict[str, Any]], path: Path,
                                   maximum_patches: int = 12) -> dict[str, Any]:
    severe = [row for row in rows if row["severe_regression"] and row["available"]]
    patch_keys = sorted({(row["evaluation_label"], row["session"], row["node_a"], row["node_b"],
                         row["kind"], row["plane_a"], row["plane_b"]) for row in severe})
    score = {key: max(float(row["delta_vs_best_baseline_m"]) for row in severe
                      if (row["evaluation_label"], row["session"], row["node_a"], row["node_b"],
                          row["kind"], row["plane_a"], row["plane_b"]) == key)
             for key in patch_keys}
    selected = sorted(patch_keys, key=lambda key: (-score[key], key))[:maximum_patches]
    stages = list(dict.fromkeys((row["evaluation_label"], row["candidate_stage"]) for row in severe))
    labels = [f"{key[1]} {key[2]}<->{key[3]} {key[4]}" for key in selected]
    fig, axis = plt.subplots(figsize=(max(12., 1.2 * len(selected)), 7.2))
    x = np.arange(len(selected), dtype=float)
    lookup = {
        ((row["evaluation_label"], row["session"], row["node_a"], row["node_b"], row["kind"],
          row["plane_a"], row["plane_b"]), (row["evaluation_label"], row["candidate_stage"])): row
        for row in severe
    }
    baseline = []
    for key in selected:
        matching = [row for row in severe if
                    (row["evaluation_label"], row["session"], row["node_a"], row["node_b"],
                     row["kind"], row["plane_a"], row["plane_b"]) == key]
        baseline.append(100. * float(matching[0]["best_baseline_m"]))
    axis.plot(x, baseline, color="#555555", marker="s", lw=2., label="best baseline")
    colors = plt.get_cmap("tab10")
    for index, stage in enumerate(stages):
        values = []
        for key in selected:
            row = lookup.get((key, stage))
            values.append(np.nan if row is None else 100. * float(row["candidate_m"]))
        axis.plot(x, values, marker="o", lw=1.6, color=colors(index % 10),
                  label=f"{stage[0]}/{stage[1]}")
    axis.set_xticks(x, labels, rotation=28, ha="right", fontsize=8)
    axis.set_ylabel("finite-patch plane residual (cm)")
    axis.set_title(f"Largest {len(selected)} fixed-patch regressions vs best baseline")
    axis.grid(axis="y", alpha=.25)
    axis.legend(loc="upper right", fontsize=8)
    headline = None
    if selected:
        first = selected[0]
        first_rows = [row for row in severe if
                      (row["evaluation_label"], row["session"], row["node_a"], row["node_b"],
                       row["kind"], row["plane_a"], row["plane_b"]) == first]
        worst = max(first_rows, key=lambda row: float(row["delta_vs_best_baseline_m"]))
        headline = {
            "patch": labels[0], "candidate_stage": worst["candidate_stage"],
            "best_baseline_cm": 100. * float(worst["best_baseline_m"]),
            "candidate_cm": 100. * float(worst["candidate_m"]),
            "delta_cm": 100. * float(worst["delta_vs_best_baseline_m"]),
        }
        axis.annotate(
            f'{headline["best_baseline_cm"]:.1f} -> {headline["candidate_cm"]:.1f} cm',
            xy=(0., headline["candidate_cm"]), xytext=(.6, headline["candidate_cm"] * .88),
            arrowprops={"arrowstyle": "->", "color": "#a61d24"}, color="#a61d24", fontsize=9)
    fig.text(.5, .01, "Per-patch symmetric absolute plane-distance median; conditional, not surveyed truth",
             ha="center", fontsize=8)
    fig.subplots_adjust(bottom=.28, top=.90)
    _save_figure(fig, path)
    return {"severe_comparison_row_count": len(severe),
            "unique_severe_patch_count": len(patch_keys),
            "displayed_patch_count": len(selected), "headline": headline,
            "selection_rule": f"highest {maximum_patches} unique patches by maximum delta vs best baseline"}


def build_summary(canonical_path: Path, report_specs: list[tuple[str, Path]],
                  sensitivity_specs: list[tuple[str, Path, Path]]) -> tuple[dict, dict[str, list], dict]:
    if not report_specs:
        raise ValueError("at least one backend report is required")
    graph_specs = [("canonical", canonical_path)] + [
        (label, graph_path) for label, graph_path, _ in sensitivity_specs]
    labels = [label for label, _ in graph_specs]
    if len(labels) != len(set(labels)):
        raise ValueError("graph/sensitivity labels must be unique")
    report_labels = [label for label, _ in report_specs]
    if len(report_labels) != len(set(report_labels)):
        raise ValueError("backend report labels must be unique")

    graphs: dict[str, dict[str, Any]] = {}
    input_records: list[dict[str, str]] = []
    for label, path in graph_specs:
        if not path.is_file():
            raise FileNotFoundError(path)
        payload = json.loads(path.read_text(encoding="utf-8"))
        graphs[label] = validate_graph(payload, label)
        input_records.append({"kind": "graph_manifest", "label": label,
                              "path": str(path.resolve()), "sha256": sha256_file(path),
                              "canonical_json_sha256": graphs[label]["hash"]})

    pairings: list[tuple[str, str, Path]] = [
        ("canonical", label, path) for label, path in report_specs]
    pairings += [(label, label, report_path) for label, _, report_path in sensitivity_specs]
    candidates, edge_rows, correction_rows = [], [], []
    for graph_label, report_label, path in pairings:
        if not path.is_file():
            raise FileNotFoundError(path)
        report = json.loads(path.read_text(encoding="utf-8"))
        graph = graphs[graph_label]
        validate_report(report, graph, report_label)
        input_records.append({"kind": "backend_report", "label": report_label,
                              "graph_label": graph_label, "path": str(path.resolve()),
                              "sha256": sha256_file(path)})
        for method, variant in _report_variants(report, report_label).items():
            candidate, edges, corrections, _ = summarize_candidate(
                graph_label, report_label, str(method), graph, variant)
            candidates.append(candidate)
            edge_rows.extend(edges)
            correction_rows.extend(corrections)
    candidate_ids = [row["candidate_id"] for row in candidates]
    if len(candidate_ids) != len(set(candidate_ids)):
        raise ValueError("candidate identifiers are not unique; change report/sensitivity labels")

    graph_summaries, component_rows, bridge_rows = {}, [], []
    for label, graph in graphs.items():
        graph_summary, components, bridges = graph_diagnostics(label, graph)
        graph_summaries[label] = graph_summary
        component_rows.extend(components)
        bridge_rows.extend(bridges)
    region_rows = summarize_regions(edge_rows)
    tables = {
        "edge_diagnostics.csv": edge_rows,
        "region_summary.csv": region_rows,
        "corrections.csv": correction_rows,
        "candidate_comparison.csv": [flatten_candidate(row) for row in candidates],
        "components.csv": component_rows,
        "structural_bridges.csv": bridge_rows,
    }
    source = Path(__file__).resolve()
    summary = {
        "schema_version": SCHEMA_VERSION,
        "created_utc": utc_now(),
        "status": "experimental_diagnostic_only_no_approval",
        "accepted": False,
        "navigation": False,
        "fail_closed_policy": {
            "heldout_edges_in_objective": "fatal input error",
            "incomplete_or_mismatched_fit_inventory": "fatal input error",
            "backend_graph_hash_mismatch": "fatal input error",
            "mixed_train_heldout_regions": "excluded from approval-metric subset",
            "structural_graph_bridge_transitions": "excluded from approval-metric subset",
            "graph_bridge_interpretation": (
                "a switched-off sole bridge means continuity is unsupported; its residual cannot validate placement"),
            "output_policy": "accepted=false and navigation=false unconditionally",
        },
        "approval_metric_definition": (
            "heldout residuals only, excluding any physical region represented in training and excluding "
            "structural transitions that are graph bridges; still diagnostic, not surveyed ground truth"),
        "active_component_weight_threshold": ACTIVE_INFORMATION_WEIGHT,
        "inputs": input_records,
        "source": {"path": str(source), "sha256": sha256_file(source)},
        "runtime": {"python": platform.python_version(), "numpy": np.__version__,
                    "scipy": scipy_version, "matplotlib": matplotlib.__version__},
        "graphs": graph_summaries,
        "candidates": candidates,
        "counts": {"graphs": len(graphs), "candidates": len(candidates),
                   "edge_diagnostic_rows": len(edge_rows), "region_rows": len(region_rows),
                   "correction_rows": len(correction_rows)},
        "limitations": [
            "solver convergence and a small residual are not independent geometric validation",
            "heldout graph constraints may still share capture, calibration, or scene biases",
            "no candidate is promoted to a navigation map by this reporting tool",
        ],
    }
    return summary, tables, graphs


def create_output(output: Path, summary: dict[str, Any], tables: dict[str, list],
                  graphs: dict[str, dict[str, Any]]) -> dict[str, Any]:
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {output}")
    output.mkdir(parents=True)
    source = Path(__file__).resolve()
    shutil.copyfile(source, output / "source_snapshot.py")
    for filename, rows in tables.items():
        write_csv(output / filename, rows, CSV_FIELDS[filename])

    figure_metadata = {
        "edge_weights_chi2_by_type.png": render_edge_type(
            tables["edge_diagnostics.csv"], output / "edge_weights_chi2_by_type.png"),
        "edge_weights_chi2_by_region.png": render_edge_regions(
            tables["region_summary.csv"], output / "edge_weights_chi2_by_region.png"),
        "correction_distributions_by_session.png": render_corrections(
            tables["corrections.csv"], output / "correction_distributions_by_session.png"),
        "heldout_residual_comparison.png": render_heldout(
            tables["edge_diagnostics.csv"], output / "heldout_residual_comparison.png"),
        "graph_components_structural_bridges.png": render_graphs(
            graphs, output / "graph_components_structural_bridges.png"),
        "candidate_comparison.png": render_candidate_comparison(
            summary["candidates"], output / "candidate_comparison.png"),
    }
    artifact_names = ["source_snapshot.py", *tables, *figure_metadata]
    artifacts = {name: {"path": str((output / name).resolve()),
                        "sha256": sha256_file(output / name)} for name in artifact_names}
    for name, metadata in figure_metadata.items():
        artifacts[name]["rendered_data"] = metadata
    summary["artifacts"] = artifacts
    summary_path = output / "summary.json"
    with summary_path.open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, sort_keys=True,
                  allow_nan=False)
        stream.write("\n")
    return summary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", "--graph-manifest", dest="manifest", type=Path, required=True,
                        help="Canonical robust-submap graph manifest")
    parser.add_argument("--report", "--backend-report", action="append", nargs=2,
                        metavar=("LABEL", "PATH"), required=True,
                        help="Label and backend report path; repeat to compare runs")
    parser.add_argument("--sensitivity", action="append", nargs=3, default=[],
                        metavar=("LABEL", "GRAPH", "REPORT"),
                        help="Label, graph manifest, and paired backend report for a sensitivity run")
    parser.add_argument("--output", type=Path, required=True,
                        help="New output directory; existing paths are refused")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite existing output: {args.output}")
    report_specs = [(str(label), Path(path)) for label, path in args.report]
    sensitivity_specs = [(str(label), Path(graph), Path(report))
                         for label, graph, report in args.sensitivity]
    summary, tables, graphs = build_summary(args.manifest, report_specs, sensitivity_specs)
    create_output(args.output, summary, tables, graphs)
    print(json.dumps({"output": str(args.output), "accepted": False, "navigation": False,
                      "candidates": [row["candidate_id"] for row in summary["candidates"]]}))


if __name__ == "__main__":
    main()
