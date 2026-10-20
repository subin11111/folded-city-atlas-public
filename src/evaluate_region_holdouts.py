#!/usr/bin/env python3
"""Fail-closed, physical-region validation for submap pose graphs.

This evaluator deliberately counts a physical place once.  Ten adjacent frame
matches assigned to one ``region_id`` are one observation, not ten independent
holdouts.  ``spatial_group`` is an additional guard: regions in the same
spatial group are collapsed for statistics and are all removed when computing
a leave-one-region-out (LORO) cycle.

Input schemas (version 1)
-------------------------
``regions.json``::

  {"regions": [{"region_id": "lobby-east", "connection_id": "A__B",
                 "spatial_group": "east", "locked_holdout": false,
                 "critical": false, "correlation_group_ids": ["cg-1"]}],
   "thresholds": {...}, "critical_pairs": ["edge-id"]}

``edges.json`` accepts the robust-backend schema::

  {"submaps": [{"id": "A-0", "session_id": "A", "pose": [x,y,z,qx,qy,qz,qw]}],
   "edges": [{"id": "e", "from": "A-0", "to": "B-0",
              "measurement": [x,y,z,qx,qy,qz,qw],
              "type": "cross_session", "role": "train",
              "region_group": "lobby-east"}]}

Matrices may replace seven-vectors.  The convention is
``measurement = inv(T_world_from) @ T_world_to``.  Legacy
``session_a/node_a/session_b/node_b/T_a_b`` rows are accepted as well.

``candidate.json`` may contain top-level ``poses``/``submaps`` or backend
``variants.<name>.poses``.  A variant must publish ``fit_edge_ids`` and
``fit_edge_ids_complete=true`` to prove that locked holdouts did not enter the
fit.  ``baseline.json`` uses the same pose formats.  The CLI never opens a
sensor database and refuses to overwrite an output directory.
"""
from __future__ import annotations

import argparse
import copy
import csv
from collections import defaultdict, deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys
from typing import Any, Iterable

import numpy as np
from scipy.spatial.transform import Rotation


DEFAULT_THRESHOLDS = {
    "minimum_regions_per_connection": 3,
    "minimum_spatial_groups_per_connection": 3,
    "minimum_training_spatial_groups": 2,
    "minimum_locked_holdout_spatial_groups": 1,
    "minimum_path_information_weight": .01,
    "candidate_translation_m": {"median": .05, "p90": .10, "maximum": .20},
    "candidate_rotation_deg": {"median": 1.5, "p90": 2., "maximum": 5.},
    "locked_translation_m": {"median": .05, "p90": .10, "maximum": .20},
    "locked_rotation_deg": {"median": 1.5, "p90": 2., "maximum": 5.},
    "loro_translation_m": {"median": .05, "p90": .10, "maximum": .20},
    "loro_rotation_deg": {"median": 1.5, "p90": 2., "maximum": 5.},
    "severe_regression_translation_delta_m": .05,
    "severe_regression_rotation_delta_deg": 2.,
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_sha256(payload: Any) -> str:
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"),
                                     ensure_ascii=False).encode("utf-8")).hexdigest()


def graph_factor_fingerprint(graph: dict[str, Any]) -> str:
    """Hash poses and objective factors while allowing region label aliases.

    A physical-region declaration may relabel a correlation group.  It may
    not silently change roles, endpoints, measurements, or uncertainty while
    claiming to validate the graph that generated the candidate poses.
    """
    node_fields = ("id", "session_id", "pose", "gauge")
    edge_fields = ("id", "from", "to", "measurement", "type", "role",
                   "information", "covariance")
    return canonical_sha256({
        "submaps": sorted(({key: row[key] for key in node_fields if key in row}
                            for row in graph.get("submaps", [])), key=lambda row: row["id"]),
        "edges": sorted(({key: row[key] for key in edge_fields if key in row}
                          for row in graph.get("edges", [])), key=lambda row: row["id"]),
    })


def validate_candidate_graph_identity(graph: dict[str, Any], candidate: dict[str, Any]) -> dict[str, Any]:
    view = graph.get("metadata", {}).get("region_validation_view", {})
    source_hash = view.get("source_canonical_manifest_sha256")
    candidate_hash = candidate.get("provenance", {}).get("canonical_manifest_sha256")
    fingerprint = graph_factor_fingerprint(graph)
    if source_hash is not None:
        if candidate_hash != source_hash:
            raise ValueError("Candidate source graph does not match the physical-region declaration")
        if fingerprint != view.get("source_factor_fingerprint_sha256"):
            raise ValueError("Region validation graph changed objective factors or initial poses")
        return {"verified": True, "source_canonical_manifest_sha256": source_hash,
                "factor_fingerprint_sha256": fingerprint,
                "policy": "same source graph and objective factors; region aliases alone may differ"}
    if candidate_hash is not None and not view:
        if canonical_sha256(graph) != candidate_hash:
            raise ValueError("Candidate source graph does not match the validation graph")
        return {"verified": True, "source_canonical_manifest_sha256": candidate_hash,
                "factor_fingerprint_sha256": fingerprint,
                "policy": "identical canonical graph manifest"}
    return {"verified": False,
            "policy": "legacy or synthetic payload lacks source graph identity metadata"}


def transform(value: Any) -> np.ndarray:
    """Parse a rigid transform from 4x4 or [x,y,z,qx,qy,qz,qw]."""
    array = np.asarray(value, dtype=float)
    if array.shape == (7,):
        result = np.eye(4)
        result[:3, :3] = Rotation.from_quat(array[3:]).as_matrix()
        result[:3, 3] = array[:3]
    elif array.shape == (4, 4):
        result = array.copy()
    else:
        raise ValueError(f"Transform must be 4x4 or length 7, got {array.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError("Transform contains a non-finite value")
    if not np.allclose(result[3], [0., 0., 0., 1.], atol=1e-8):
        raise ValueError("Transform bottom row is not homogeneous")
    rotation = result[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=2e-4) or not np.isclose(
            np.linalg.det(rotation), 1., atol=2e-4):
        raise ValueError("Transform rotation is not in SO(3)")
    return result


def inverse(value: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = value[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ value[:3, 3]
    return result


def innovation(measurement: np.ndarray, prediction: np.ndarray) -> dict[str, float]:
    delta = inverse(measurement) @ prediction
    return {
        "translation_m": float(np.linalg.norm(delta[:3, 3])),
        "rotation_deg": float(np.degrees(Rotation.from_matrix(delta[:3, :3]).magnitude())),
    }


def _rows(payload: Any, names: Iterable[str]) -> list[dict[str, Any]]:
    if isinstance(payload, list):
        return payload
    for name in names:
        if isinstance(payload, dict) and isinstance(payload.get(name), list):
            return payload[name]
    return []


def _endpoint(row: dict[str, Any], side: str) -> str:
    value = row.get(side)
    if isinstance(value, dict):
        if "id" in value:
            return str(value["id"])
        session = value.get("session", value.get("session_id"))
        node = value.get("node", value.get("node_id", value.get("submap_id")))
        if session is not None and node is not None:
            return f"{session}:{node}"
    if value is not None:
        return str(value)
    suffix = "a" if side == "from" else "b"
    session = row.get(f"session_{suffix}")
    node = row.get(f"node_{suffix}", row.get(f"submap_{suffix}"))
    if session is None or node is None:
        raise ValueError(f"Edge {row.get('id', row.get('edge_id'))!r} lacks endpoint {side}")
    return f"{session}:{node}"


def normalize_edges(payload: Any) -> tuple[list[dict[str, Any]], dict[str, str]]:
    """Normalize backend or legacy edges and return endpoint -> session map."""
    sessions: dict[str, str] = {}
    for row in _rows(payload, ("submaps",)):
        endpoint = str(row.get("id", row.get("submap_id")))
        if endpoint == "None":
            raise ValueError("Submap lacks id")
        session = row.get("session_id", row.get("session"))
        if session is not None:
            sessions[endpoint] = str(session)
    result = []
    seen = set()
    for index, row in enumerate(_rows(payload, ("edges", "constraints", "observations", "records"))):
        edge_id = str(row.get("id", row.get("edge_id", f"edge-{index}")))
        if edge_id in seen:
            raise ValueError(f"Duplicate edge id: {edge_id}")
        seen.add(edge_id)
        a, b = _endpoint(row, "from"), _endpoint(row, "to")
        if "session_a" in row:
            sessions.setdefault(a, str(row["session_a"]))
        if "session_b" in row:
            sessions.setdefault(b, str(row["session_b"]))
        raw_measurement = row.get("measurement", row.get("T_a_b", row.get("transform")))
        if raw_measurement is None:
            raise ValueError(f"Edge {edge_id} lacks measurement")
        held = bool(row.get("held_out", row.get("joint_holdout", False)))
        role = str(row.get("role", row.get("joint_role", "heldout" if held else "train"))).lower()
        if role in {"holdout", "held_out", "audit"}:
            role = "heldout"
        if held and role != "heldout":
            raise ValueError(f"Edge {edge_id} has inconsistent role and held_out")
        # Odometry is the backbone used to predict an omitted region; its
        # backend ``region_group`` is a correlation label, not an independent
        # validation place.
        region = None if str(row.get("type", "unknown")) == "odometry" else row.get(
            "region_group", row.get("region_id"))
        result.append({
            "edge_id": edge_id, "from": a, "to": b,
            "measurement": transform(raw_measurement), "type": str(row.get("type", "unknown")),
            "role": role, "region_id": None if region is None else str(region),
            "used_for_fit": row.get("used_for_fit"), "raw": row,
        })
    if not result:
        raise ValueError("No edges found")
    return result, sessions


def normalize_regions(payload: Any) -> tuple[dict[str, dict[str, Any]], dict[str, Any], set[str]]:
    rows = _rows(payload, ("regions", "physical_regions"))
    if not rows:
        raise ValueError("No physical regions found")
    regions = {}
    correlation_owner: dict[str, str] = {}
    for row in rows:
        region_id = str(row.get("region_id", row.get("id", row.get("region_group"))))
        if region_id == "None" or region_id in regions:
            raise ValueError(f"Missing or duplicate region id: {region_id}")
        if not isinstance(row.get("locked_holdout"), bool):
            raise ValueError(f"Region {region_id} must explicitly set locked_holdout")
        connection = row.get("connection_id", row.get("connection"))
        if connection is None:
            raise ValueError(f"Region {region_id} lacks connection_id")
        spatial = row.get("spatial_group")
        for group in row.get("correlation_group_ids", []):
            group = str(group)
            owner_key = f"{connection}\0{group}"
            if owner_key in correlation_owner and correlation_owner[owner_key] != region_id:
                raise ValueError(
                    f"Correlation group {group} split across {correlation_owner[owner_key]} and {region_id}")
            correlation_owner[owner_key] = region_id
        regions[region_id] = {
            "region_id": region_id, "connection_id": str(connection),
            "spatial_group": None if spatial is None else str(spatial),
            "locked_holdout": row["locked_holdout"], "critical": bool(row.get("critical", False)),
            "correlation_group_ids": [str(v) for v in row.get("correlation_group_ids", [])],
            "description": row.get("description"),
            "source_region_groups": row.get("source_region_groups", []),
            "declared_edge_ids": row.get("edge_ids", []),
            "independence_status": row.get("independence_status"),
            "connection_coverage_status": row.get("connection_coverage_status"),
        }
    # Copy nested defaults so a manifest may override only ``maximum`` without
    # accidentally deleting the required median and p90 gates.
    thresholds = json.loads(json.dumps(DEFAULT_THRESHOLDS))
    overrides = payload.get("thresholds", {}) if isinstance(payload, dict) else {}
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(thresholds.get(key), dict):
            thresholds[key].update(value)
        else:
            thresholds[key] = value
    critical = set()
    if isinstance(payload, dict):
        for item in payload.get("critical_pairs", []):
            critical.add(str(item.get("edge_id", item.get("id"))) if isinstance(item, dict) else str(item))
    return regions, thresholds, critical


def _canonical_connection(first: str, second: str) -> str:
    return "__".join(sorted((str(first), str(second))))


def load_correlation_provenance(quality_manifest_path: Path) -> tuple[dict[tuple[str, int], dict[str, Any]], list[Path]]:
    """Load only quality CSVs; sensor databases are never opened."""
    quality_manifest_path = quality_manifest_path.resolve()
    payload = json.loads(quality_manifest_path.read_text(encoding="utf-8"))
    lookup: dict[tuple[str, int], dict[str, Any]] = {}
    sources = [quality_manifest_path]
    for session, row in payload.get("sessions", {}).items():
        value = row.get("artifacts", {}).get("frames_csv", row.get("frames_csv"))
        if value is None:
            raise ValueError(f"Quality session {session} lacks frames_csv")
        path = Path(value)
        if not path.is_absolute():
            path = quality_manifest_path.parent / path
        path = path.resolve(); sources.append(path)
        with path.open(newline="", encoding="utf-8") as stream:
            for frame in csv.DictReader(stream):
                group = frame.get("correlation_group_id")
                if not group:
                    continue
                key = (str(session), int(frame["node_id"]))
                lookup[key] = {"correlation_group_id": str(group),
                               "evidence_weight": float(frame.get("evidence_weight") or 0.),
                               "primary_submap_id": frame.get("primary_submap_id")}
    return lookup, sources


def derive_region_declaration(graph: dict[str, Any], correlations: dict[tuple[str, int], dict[str, Any]],
                              required_connections: Iterable[str] = ()) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Build regions and a non-mutating validation view of a backend graph."""
    sessions = {str(row["id"]): str(row["session_id"]) for row in graph.get("submaps", [])}
    if not sessions:
        raise ValueError("Graph contains no submaps")
    records = []
    for edge in graph.get("edges", []):
        if edge.get("type") == "odometry":
            continue
        edge_id = str(edge.get("id"))
        source, target = str(edge.get("from")), str(edge.get("to"))
        if source not in sessions or target not in sessions:
            raise ValueError(f"Edge {edge_id} endpoint is absent from submaps")
        connection = _canonical_connection(sessions[source], sessions[target])
        raw_group = str(edge.get("region_group") or f"edge:{edge_id}")
        endpoint_groups, missing = [], []
        frame_endpoints = edge.get("provenance", {}).get("frame_endpoints", [])
        if len(frame_endpoints) == 4:
            for session, node in ((frame_endpoints[0], frame_endpoints[1]),
                                  (frame_endpoints[2], frame_endpoints[3])):
                evidence = correlations.get((str(session), int(node)))
                if evidence is None:
                    missing.append(f"{session}:{node}")
                else:
                    endpoint_groups.append(evidence["correlation_group_id"])
        elif edge.get("provenance", {}).get("native_path_edge_ids"):
            # Structural run-boundary factors are condensed native odometry.
            # Their evidence region is every quality correlation group touched
            # by the recorded native path, not two overlapping submaps counted
            # as independent observations.
            session = sessions[source]
            native_nodes = set()
            for native_id in edge["provenance"]["native_path_edge_ids"]:
                fields = str(native_id).rsplit(":", 2)
                if len(fields) == 3:
                    try:
                        native_nodes.update((int(fields[-2]), int(fields[-1])))
                    except ValueError:
                        missing.append(f"unparsed_native_edge:{native_id}")
            for node in sorted(native_nodes):
                evidence = correlations.get((session, node))
                if evidence is None:
                    missing.append(f"{session}:{node}")
                else:
                    endpoint_groups.append(evidence["correlation_group_id"])
        else:
            missing.append("frame_endpoints_not_recorded")
        records.append({"edge": edge, "edge_id": edge_id, "connection_id": connection,
                        "raw_region_group": raw_group,
                        "correlation_group_ids": sorted(set(endpoint_groups)),
                        "missing_correlation_provenance": missing})
    if not records:
        raise ValueError("Graph has no loop/cross/structural region observations")

    # Union physical cells that reuse a correlation group within a connection.
    keys = sorted({(row["connection_id"], row["raw_region_group"]) for row in records})
    parent = {key: key for key in keys}

    def find(key):
        while parent[key] != key:
            parent[key] = parent[parent[key]]
            key = parent[key]
        return key

    def union(first, second):
        first, second = find(first), find(second)
        if first != second:
            parent[max(first, second)] = min(first, second)

    owners: dict[tuple[str, str], tuple[str, str]] = {}
    for row in records:
        key = (row["connection_id"], row["raw_region_group"])
        for correlation in row["correlation_group_ids"]:
            owner_key = (row["connection_id"], correlation)
            if owner_key in owners:
                union(key, owners[owner_key])
            else:
                owners[owner_key] = key
    components: dict[tuple[str, str], set[tuple[str, str]]] = defaultdict(set)
    for key in keys:
        components[find(key)].add(key)
    key_to_region = {}
    for members in components.values():
        connection = min(members)[0]
        raw_groups = sorted(group for _, group in members)
        if len(raw_groups) == 1:
            region_id = raw_groups[0]
        else:
            token = hashlib.sha256("\n".join(raw_groups).encode()).hexdigest()[:12]
            region_id = f"{connection}:correlation_union_{token}"
        for key in members:
            key_to_region[key] = region_id

    validation_graph = copy.deepcopy(graph)
    raw_by_id = {row["edge_id"]: row for row in records}
    for edge in validation_graph["edges"]:
        if edge.get("type") != "odometry":
            row = raw_by_id[str(edge["id"])]
            edge.setdefault("provenance", {})["original_region_group"] = edge.get("region_group")
            edge["region_group"] = key_to_region[(row["connection_id"], row["raw_region_group"])]
    validation_graph.setdefault("metadata", {})["region_validation_view"] = {
        "accepted": False, "navigation": False,
        "source_canonical_manifest_sha256": canonical_sha256(graph),
        "source_factor_fingerprint_sha256": graph_factor_fingerprint(graph),
        "note": "Only region_group labels are correlation-unioned; measurements, roles and poses are unchanged."}

    aggregated: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in records:
        aggregated[key_to_region[(row["connection_id"], row["raw_region_group"])]].append(row)
    regions = []
    for region_id, members in sorted(aggregated.items()):
        roles = sorted({str(row["edge"].get("role")) for row in members})
        edge_ids = sorted(row["edge_id"] for row in members)
        correlation_ids = sorted({group for row in members for group in row["correlation_group_ids"]})
        missing = sorted({value for row in members for value in row["missing_correlation_provenance"]})
        mixed = len(roles) > 1
        status = ("mixed_train_heldout_same_physical_region" if mixed else
                  "locked_holdout" if roles == ["heldout"] else "training")
        regions.append({
            "region_id": region_id, "connection_id": members[0]["connection_id"],
            "spatial_group": region_id, "locked_holdout": "heldout" in roles,
            "critical": "heldout" in roles or any(row["edge"].get("type") == "cross_session" for row in members),
            "correlation_group_ids": correlation_ids,
            "source_region_groups": sorted({row["raw_region_group"] for row in members}),
            "edge_ids": edge_ids, "edge_roles": roles,
            "edge_types": sorted({str(row["edge"].get("type")) for row in members}),
            "missing_correlation_provenance": missing,
            "independence_status": status,
            "description": "One physical/correlation-unioned validation region; all member edges count once.",
        })

    canonical_required = {_canonical_connection(*value.split("__", 1)) for value in required_connections}
    existing_connections = {row["connection_id"] for row in regions}
    for connection in sorted(canonical_required - existing_connections):
        regions.append({"region_id": f"missing:{connection}", "connection_id": connection,
            "spatial_group": None, "locked_holdout": True, "critical": False,
            "correlation_group_ids": [], "source_region_groups": [], "edge_ids": [],
            "edge_roles": [], "edge_types": [], "missing_correlation_provenance": [],
            "independence_status": "required_connection_has_no_observation",
            "description": "Placeholder retained so missing required connection fails closed."})

    coverage = {}
    for connection in sorted({row["connection_id"] for row in regions}):
        scoped = [row for row in regions if row["connection_id"] == connection and row["edge_ids"]]
        reasons = []
        training = sum(not row["locked_holdout"] for row in scoped)
        locked = sum(row["locked_holdout"] for row in scoped)
        if len(scoped) < DEFAULT_THRESHOLDS["minimum_spatial_groups_per_connection"]:
            reasons.append("fewer_than_3_spatial_regions")
        if training < DEFAULT_THRESHOLDS["minimum_training_spatial_groups"]:
            reasons.append("fewer_than_2_training_regions")
        if locked < DEFAULT_THRESHOLDS["minimum_locked_holdout_spatial_groups"]:
            reasons.append("no_locked_holdout_region")
        if any(row["independence_status"].startswith("mixed_") for row in scoped):
            reasons.append("train_holdout_mixed_within_physical_region")
        if any(row["missing_correlation_provenance"] for row in scoped):
            reasons.append("missing_endpoint_correlation_provenance")
        coverage[connection] = {"regions": len(scoped), "training_regions": training,
                                "locked_holdout_regions": locked,
                                "failures": reasons, "sufficient": not reasons}
        for row in regions:
            if row["connection_id"] == connection:
                row["connection_coverage_status"] = "sufficient" if not reasons else "insufficient:" + ",".join(reasons)

    critical = sorted({row["edge_id"] for row in records
                       if row["edge"].get("role") == "heldout" or row["edge"].get("type") == "cross_session"})
    correlation_connections: dict[str, set[str]] = defaultdict(set)
    for row in regions:
        for group in row["correlation_group_ids"]:
            correlation_connections[group].add(row["connection_id"])
    cross_connection_reuse = {group: sorted(connections) for group, connections in correlation_connections.items()
                              if len(connections) > 1}
    declaration = {
        "schema_version": 1, "created_utc": utc_now(),
        "status": "PHYSICAL_REGION_DECLARATION_ONLY_NOT_VALIDATED_MAP",
        "accepted": False, "navigation_ready": False,
        "regions": sorted(regions, key=lambda row: (row["connection_id"], row["region_id"])),
        "critical_pairs": critical, "thresholds": copy.deepcopy(DEFAULT_THRESHOLDS),
        "required_connections": sorted(canonical_required), "connection_coverage": coverage,
        "independence_policy": [
            "2 m graph region cells are initial physical groups.",
            "Groups sharing a quality correlation_group_id within one connection are unioned.",
            "Every unioned region contributes one value regardless of member edge/frame count.",
            "Any region containing a heldout edge is locked; a training edge in that region is reported as leakage and cannot support validation."],
    }
    report = {"created_utc": utc_now(), "accepted": False, "navigation_ready": False,
              "raw_region_group_count": len(keys), "correlation_unioned_region_count": len(aggregated),
              "missing_required_connections": sorted(canonical_required - existing_connections),
              "connection_coverage": coverage,
              "mixed_role_regions": [row["region_id"] for row in regions if row["independence_status"].startswith("mixed_")],
              "regions_missing_correlation_provenance": [row["region_id"] for row in regions if row["missing_correlation_provenance"]],
              "cross_connection_correlation_reuse": cross_connection_reuse,
              "source_databases_accessed": []}
    return declaration, validation_graph, report


def write_region_declaration(output: Path, declaration: dict[str, Any], validation_graph: dict[str, Any],
                             report: dict[str, Any], input_paths: list[Path]) -> None:
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output}")
    report["source_sha256"] = sha256_file(Path(__file__))
    report["input_sha256"] = {str(path.resolve()): sha256_file(path.resolve()) for path in input_paths}
    output.mkdir(parents=True)
    shutil.copyfile(Path(__file__), output / "source_snapshot.py")
    (output / "regions.json").write_text(json.dumps(declaration, indent=2) + "\n", encoding="utf-8")
    (output / "edges.json").write_text(json.dumps(validation_graph, indent=2) + "\n", encoding="utf-8")
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def _variant(payload: dict[str, Any], name: str | None) -> dict[str, Any]:
    variants = payload.get("variants")
    if name is not None:
        if not isinstance(variants, dict) or name not in variants:
            raise ValueError(f"Variant not found: {name}")
        return variants[name]
    if isinstance(variants, dict):
        if len(variants) != 1:
            raise ValueError("Select a variant when a pose payload contains multiple variants")
        return next(iter(variants.values()))
    return payload


def normalize_poses(payload: dict[str, Any], variant_name: str | None = None) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    selected = _variant(payload, variant_name)
    poses: dict[str, np.ndarray] = {}
    raw_poses = selected.get("poses")
    if isinstance(raw_poses, dict):
        poses = {str(key): transform(value) for key, value in raw_poses.items()}
    else:
        rows = _rows(selected, ("poses", "submaps")) or _rows(payload, ("submaps",))
        for row in rows:
            key = row.get("id", row.get("submap_id"))
            value = row.get("pose", row.get("matrix", row.get("T_world_submap")))
            if key is None or value is None:
                raise ValueError("Pose row lacks id or pose")
            poses[str(key)] = transform(value)
    if not poses:
        raise ValueError("No poses found")
    fit_ids = {str(v) for v in selected.get("fit_edge_ids", payload.get("fit_edge_ids", []))}
    heldout_ids = {str(v) for v in selected.get("heldout_edge_ids", payload.get("heldout_edge_ids", []))}
    complete = selected.get("fit_edge_ids_complete", payload.get("fit_edge_ids_complete", False)) is True
    edge_status = selected.get("edges", [])
    if edge_status and all("id" in row and "used_for_fit" in row for row in edge_status):
        derived = {str(row["id"]) for row in edge_status if row["used_for_fit"]}
        if fit_ids and fit_ids != derived:
            raise ValueError("fit_edge_ids contradict variant edge used_for_fit flags")
        fit_ids = derived
    meta = {"fit_edge_ids": fit_ids, "heldout_edge_ids": heldout_ids,
            "fit_edge_ids_complete": complete, "fit_edge_policy": selected.get("fit_edge_policy"),
            "edge_status": {str(row["id"]): row for row in edge_status if "id" in row},
            "leave_one_group_diagnostics": {
                str(row["region_group"]): row for row in selected.get("leave_one_group_diagnostics", [])
                if "region_group" in row}}
    return poses, meta


def metric_summary(values: Iterable[float]) -> dict[str, Any]:
    array = np.asarray(list(values), dtype=float)
    if len(array) == 0:
        return {"count": 0, "median": None, "p90": None, "maximum": None}
    return {"count": int(len(array)), "median": float(np.median(array)),
            "p90": float(np.percentile(array, 90)), "maximum": float(np.max(array))}


def _pair_summary(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    available = [row[key] for row in rows if row.get(key) is not None]
    return {
        "translation_m": metric_summary(v["translation_m"] for v in available),
        "rotation_deg": metric_summary(v["rotation_deg"] for v in available),
    }


def _representative(rows: list[dict[str, Any]], key: str) -> dict[str, float] | None:
    available = [row[key] for row in rows if row.get(key) is not None]
    if not available:
        return None
    return {"translation_m": float(np.median([v["translation_m"] for v in available])),
            "rotation_deg": float(np.median([v["rotation_deg"] for v in available]))}


def shared_evidence_regions(regions: dict[str, dict[str, Any]]) -> dict[str, set[str]]:
    """Union reused sensor evidence globally, including across connections."""
    parent = {region_id: region_id for region_id in regions}

    def find(region_id):
        while parent[region_id] != region_id:
            parent[region_id] = parent[parent[region_id]]
            region_id = parent[region_id]
        return region_id

    owners = {}
    for region_id, region in regions.items():
        labels = [("correlation", group) for group in region["correlation_group_ids"]]
        if region["spatial_group"] is not None:
            labels.append(("spatial", region["connection_id"], region["spatial_group"]))
        for label in labels:
            if label in owners:
                first, second = find(region_id), find(owners[label])
                parent[max(first, second)] = min(first, second)
            else:
                owners[label] = region_id
    components = defaultdict(set)
    for region_id in regions:
        components[find(region_id)].add(region_id)
    return {region_id: components[find(region_id)] for region_id in regions}


def shortest_prediction(edges: list[dict[str, Any]], start: str, goal: str,
                        blocked_regions: set[str], allowed_edge_ids: set[str] | None) -> tuple[np.ndarray | None, list[str]]:
    """Compose a shortest measurement path after removing complete regions."""
    adjacency: dict[str, list[tuple[str, str, np.ndarray]]] = defaultdict(list)
    for edge in edges:
        if edge["role"] == "heldout" or edge["region_id"] in blocked_regions:
            continue
        if allowed_edge_ids is not None and edge["edge_id"] not in allowed_edge_ids:
            continue
        adjacency[edge["from"]].append((edge["edge_id"], edge["to"], edge["measurement"]))
        adjacency[edge["to"]].append((edge["edge_id"], edge["from"], inverse(edge["measurement"])))
    queue = deque([(start, np.eye(4), [])])
    visited = {start}
    while queue:
        node, pose, path = queue.popleft()
        if node == goal:
            return pose, path
        for edge_id, neighbour, step in sorted(adjacency[node], key=lambda item: (item[0], item[1])):
            if neighbour not in visited:
                visited.add(neighbour)
                queue.append((neighbour, pose @ step, path + [edge_id]))
    return None, []


def _threshold_failures(label: str, summary: dict[str, Any], translation: dict[str, float],
                        rotation: dict[str, float]) -> list[str]:
    failures = []
    for metric, limits in (("translation_m", translation), ("rotation_deg", rotation)):
        for statistic, limit in limits.items():
            value = summary[metric][statistic]
            if value is None:
                failures.append(f"{label}_{metric}_{statistic}_missing")
            elif value > limit:
                failures.append(f"{label}_{metric}_{statistic}_failed")
    return failures


def evaluate(regions_payload: dict[str, Any], edges_payload: dict[str, Any],
             candidate_payload: dict[str, Any], baseline_payload: dict[str, Any],
             candidate_variant: str | None = None, baseline_variant: str | None = None,
             selection_conditioned: bool = False) -> dict[str, Any]:
    graph_identity = validate_candidate_graph_identity(edges_payload, candidate_payload)
    regions, thresholds, explicit_critical = normalize_regions(regions_payload)
    edges, sessions = normalize_edges(edges_payload)
    candidate, fit = normalize_poses(candidate_payload, candidate_variant)
    baseline, _ = normalize_poses(baseline_payload, baseline_variant)
    edge_ids = {edge["edge_id"] for edge in edges}
    if fit["fit_edge_ids"] - edge_ids:
        raise ValueError("Candidate fit_edge_ids reference unknown edges")

    region_edges: dict[str, list[dict[str, Any]]] = defaultdict(list)
    global_failures = []
    if selection_conditioned:
        # A candidate selected using these same gate/holdout observations can
        # be diagnosed here, but those observations are no longer independent
        # evidence for admission.  Fail closed even when every numeric gate
        # passes; a separately locked test set is required for approval.
        global_failures.append("selection_conditioned_gate_is_not_independent_validation")
    role_conflicts = []
    for edge in edges:
        region_id = edge["region_id"]
        if region_id is None:
            continue
        if region_id not in regions:
            raise ValueError(f"Edge {edge['edge_id']} references undefined region {region_id}")
        region = regions[region_id]
        is_held = edge["role"] == "heldout"
        if is_held != region["locked_holdout"]:
            role_conflicts.append(edge["edge_id"])
        region_edges[region_id].append(edge)
    if role_conflicts:
        global_failures.append("locked_region_contains_training_edges:" + ",".join(sorted(role_conflicts)))
    empty = sorted(set(regions) - set(region_edges))
    if empty:
        global_failures.append("regions_without_observations:" + ",".join(empty))

    locked_edge_ids = {edge["edge_id"] for edge in edges
                       if edge["region_id"] in regions and regions[edge["region_id"]]["locked_holdout"]}
    leaked = locked_edge_ids & fit["fit_edge_ids"]
    if leaked:
        global_failures.append("locked_holdout_leakage_into_fit:" + ",".join(sorted(leaked)))
    if not fit["fit_edge_ids_complete"]:
        global_failures.append("fit_edge_membership_not_proven_complete")
    if fit["heldout_edge_ids"] and not locked_edge_ids <= fit["heldout_edge_ids"]:
        global_failures.append("candidate_heldout_edge_manifest_incomplete")
    for edge in edges:
        if edge["used_for_fit"] is not None and bool(edge["used_for_fit"]) != (edge["edge_id"] in fit["fit_edge_ids"]):
            raise ValueError(f"Edge {edge['edge_id']} used_for_fit contradicts candidate fit manifest")

    # A frame reused by a session-internal factor and a cross-session factor
    # remains the same sensor evidence.  Connection boundaries cannot turn it
    # into an independent holdout or an independent prediction path.
    evidence_groups = shared_evidence_regions(regions)
    locked_evidence_regions = set().union(*(evidence_groups[region_id]
        for region_id, region in regions.items() if region["locked_holdout"]))
    locked_shared_edge_ids = {edge["edge_id"] for edge in edges
                             if edge["region_id"] in locked_evidence_regions}
    shared_leaked = (locked_shared_edge_ids - locked_edge_ids) & fit["fit_edge_ids"]
    if shared_leaked:
        global_failures.append("locked_sensor_evidence_reused_in_fit:" + ",".join(sorted(shared_leaked)))

    # A constraint that the robust backend has practically switched off must
    # not become evidence in a held-out path.  Generic pose manifests without
    # per-edge weights retain all declared fit edges and disclose that fact.
    support_edge_ids = set()
    weight_filter_complete = True
    minimum_weight = float(thresholds["minimum_path_information_weight"])
    for edge_id in fit["fit_edge_ids"]:
        status = fit["edge_status"].get(edge_id)
        if status is None:
            weight_filter_complete = False
            support_edge_ids.add(edge_id)
            continue
        if "type" not in status or "information_weight" not in status:
            weight_filter_complete = False
            support_edge_ids.add(edge_id)
            continue
        weight = status.get("information_weight")
        if status.get("type") == "odometry" or weight is None or float(weight) >= minimum_weight:
            support_edge_ids.add(edge_id)
    # If any observation in a physical region was predeclared held out, none
    # of that region may support prediction of another region, even if an old
    # graph mislabeled a neighbouring observation as training.
    support_edge_ids -= locked_shared_edge_ids

    # Separate fitted components have independent gauges. Their stored display
    # coordinates cannot define a measurable heldout relative pose.
    adjacency = {node: set() for node in candidate}
    for edge in edges:
        if edge["edge_id"] in fit["fit_edge_ids"]:
            adjacency[edge["from"]].add(edge["to"])
            adjacency[edge["to"]].add(edge["from"])
    fit_components = {}
    for node in sorted(candidate):
        if node in fit_components:
            continue
        pending = [node]
        fit_components[node] = node
        while pending:
            current = pending.pop()
            for other in adjacency[current]:
                if other not in fit_components:
                    fit_components[other] = node
                    pending.append(other)

    edge_metrics = []
    for region_id, members in sorted(region_edges.items()):
        region = regions[region_id]
        blocked = evidence_groups[region_id]
        for edge in members:
            if edge["from"] not in candidate or edge["to"] not in candidate:
                raise ValueError(f"Candidate poses miss endpoint of {edge['edge_id']}")
            if edge["from"] not in baseline or edge["to"] not in baseline:
                raise ValueError(f"Baseline poses miss endpoint of {edge['edge_id']}")
            same_component = fit_components[edge["from"]] == fit_components[edge["to"]]
            direct = (innovation(edge["measurement"], inverse(candidate[edge["from"]]) @ candidate[edge["to"]])
                      if same_component else None)
            before = innovation(edge["measurement"], inverse(baseline[edge["from"]]) @ baseline[edge["to"]])
            # Prefer the backend's true leave-one-group reoptimization when
            # it removed exactly this spatial unit and kept the graph joined.
            # Locked holdouts are absent from that fit by construction and use
            # the disjoint measurement-path fallback.
            diagnostic = fit["leave_one_group_diagnostics"].get(region_id)
            diagnostic_rows = {} if diagnostic is None else {
                str(row["edge_id"]): row for row in diagnostic.get("removed_group_prediction", [])}
            backend_row = diagnostic_rows.get(edge["edge_id"])
            use_backend = (not region["locked_holdout"] and blocked == {region_id} and
                           backend_row is not None and
                           diagnostic.get("disconnects_graph") is False)
            if use_backend:
                prediction, path = None, []
                cycle = {"translation_m": float(backend_row["translation_error_m"]),
                         "rotation_deg": float(backend_row["rotation_error_deg"])}
                loro_method = "backend_leave_one_group_reoptimization"
            else:
                prediction, path = shortest_prediction(
                    edges, edge["from"], edge["to"], blocked,
                    support_edge_ids if fit["fit_edge_ids_complete"] else None)
                cycle = None if prediction is None else innovation(edge["measurement"], prediction)
                loro_method = "measurement_path_excluding_spatial_group"
            severe_t = direct is not None and direct["translation_m"] > before["translation_m"] + thresholds["severe_regression_translation_delta_m"]
            severe_r = direct is not None and direct["rotation_deg"] > before["rotation_deg"] + thresholds["severe_regression_rotation_delta_deg"]
            edge_metrics.append({
                "edge_id": edge["edge_id"], "from": edge["from"], "to": edge["to"],
                "region_id": region_id, "connection_id": region["connection_id"],
                "spatial_group": region["spatial_group"], "locked_holdout": region["locked_holdout"],
                "candidate": direct, "baseline": before, "loro_cycle": cycle,
                "same_fitted_component": same_component,
                "loro_method": loro_method,
                "loro_path_edge_ids": path, "loro_blocked_region_ids": sorted(blocked),
                "loro_backend_disconnects_graph": None if diagnostic is None else bool(diagnostic.get("disconnects_graph")),
                "severe_regression": bool(severe_t or severe_r),
                "severe_translation_regression": bool(severe_t),
                "severe_rotation_regression": bool(severe_r),
            })

    region_metrics = []
    for region_id, region in sorted(regions.items()):
        rows = [row for row in edge_metrics if row["region_id"] == region_id]
        region_metrics.append({**region, "edge_count": len(rows),
            "candidate": _pair_summary(rows, "candidate"), "candidate_representative": _representative(rows, "candidate"),
            "baseline": _pair_summary(rows, "baseline"), "baseline_representative": _representative(rows, "baseline"),
            "loro_cycle": _pair_summary(rows, "loro_cycle"), "loro_representative": _representative(rows, "loro_cycle"),
            "severe_regression": any(row["severe_regression"] for row in rows)})

    connection_reports = []
    for connection in sorted({region["connection_id"] for region in regions.values()}):
        scoped = [row for row in region_metrics if row["connection_id"] == connection]
        observed = [row for row in scoped if row["edge_count"] > 0]
        group_rows = []
        grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for row in observed:
            key = row["spatial_group"]
            if key is not None:
                grouped[key].append(row)
        for key, rows in sorted(grouped.items()):
            locked_values = {row["locked_holdout"] for row in rows}
            mixed_locked_status = len(locked_values) != 1
            candidates = [row["candidate_representative"] for row in rows if row["candidate_representative"]]
            baselines = [row["baseline_representative"] for row in rows if row["baseline_representative"]]
            loros = [row["loro_representative"] for row in rows if row["loro_representative"]]
            group_rows.append({"spatial_group": key, "region_ids": [row["region_id"] for row in rows],
                "locked_holdout": any(locked_values), "mixed_locked_status": mixed_locked_status,
                "candidate": None if not candidates else {k: float(np.median([v[k] for v in candidates])) for k in candidates[0]},
                "baseline": None if not baselines else {k: float(np.median([v[k] for v in baselines])) for k in baselines[0]},
                "loro_cycle": None if not loros else {k: float(np.median([v[k] for v in loros])) for k in loros[0]},
                "severe_regression": any(row["severe_regression"] for row in rows)})
        candidate_summary = _pair_summary(group_rows, "candidate")
        locked_summary = _pair_summary([row for row in group_rows if row["locked_holdout"]], "candidate")
        loro_summary = _pair_summary(group_rows, "loro_cycle")
        failures = []
        defined = len(observed); groups = len(group_rows)
        training_groups = sum(not row["locked_holdout"] for row in group_rows)
        locked_groups = sum(row["locked_holdout"] for row in group_rows)
        if any(row["spatial_group"] is None for row in observed):
            failures.append("spatial_group_labels_missing")
        if any(row["mixed_locked_status"] for row in group_rows):
            failures.append("spatial_group_train_holdout_mixture")
        for value, limit, label in [
            (defined, thresholds["minimum_regions_per_connection"], "physical_region_coverage"),
            (groups, thresholds["minimum_spatial_groups_per_connection"], "spatial_group_coverage"),
            (training_groups, thresholds["minimum_training_spatial_groups"], "training_spatial_group_coverage"),
            (locked_groups, thresholds["minimum_locked_holdout_spatial_groups"], "locked_holdout_spatial_group_coverage")]:
            if value < limit:
                failures.append(label + "_failed")
        failures += _threshold_failures("candidate", candidate_summary,
            thresholds["candidate_translation_m"], thresholds["candidate_rotation_deg"])
        failures += _threshold_failures("locked", locked_summary,
            thresholds["locked_translation_m"], thresholds["locked_rotation_deg"])
        failures += _threshold_failures("loro", loro_summary,
            thresholds["loro_translation_m"], thresholds["loro_rotation_deg"])
        if any(row["severe_regression"] for row in group_rows):
            failures.append("severe_regression_detected")
        connection_reports.append({"connection_id": connection,
            "coverage": {"physical_regions": defined, "spatial_groups": groups,
                "training_spatial_groups": training_groups, "locked_holdout_spatial_groups": locked_groups,
                "candidate_evaluable_spatial_groups": sum(row["candidate"] is not None for row in group_rows),
                "loro_evaluable_spatial_groups": sum(row["loro_cycle"] is not None for row in group_rows)},
            "spatial_group_metrics": group_rows, "candidate": candidate_summary,
            "locked_holdout_candidate": locked_summary, "loro_cycle": loro_summary,
            "failures": failures, "accepted": not failures})

    critical_ids = set(explicit_critical) | locked_edge_ids
    for region in regions.values():
        if region["critical"]:
            critical_ids |= {edge["edge_id"] for edge in region_edges.get(region["region_id"], [])}
    missing_critical = critical_ids - {row["edge_id"] for row in edge_metrics}
    critical_rows = []
    for edge_id in sorted(critical_ids - missing_critical):
        row = next(row for row in edge_metrics if row["edge_id"] == edge_id)
        failures = []
        if row["candidate"] is None:
            failures.append("candidate_relative_pose_unobservable_across_components")
        elif row["candidate"]["translation_m"] > thresholds["candidate_translation_m"]["maximum"]:
            failures.append("candidate_translation_failed")
        if row["candidate"] is not None and row["candidate"]["rotation_deg"] > thresholds["candidate_rotation_deg"]["maximum"]:
            failures.append("candidate_rotation_failed")
        if row["loro_cycle"] is None:
            failures.append("loro_prediction_missing")
        else:
            if row["loro_cycle"]["translation_m"] > thresholds["loro_translation_m"]["maximum"]:
                failures.append("loro_translation_failed")
            if row["loro_cycle"]["rotation_deg"] > thresholds["loro_rotation_deg"]["maximum"]:
                failures.append("loro_rotation_failed")
        if row["severe_regression"]:
            failures.append("severe_regression")
        critical_rows.append({**row, "failures": failures, "passed": not failures})
    if missing_critical:
        global_failures.append("critical_pairs_missing:" + ",".join(sorted(missing_critical)))
    if any(not row["passed"] for row in critical_rows):
        global_failures.append("critical_pair_failures")
    disconnected = sorted(row["edge_id"] for row in edge_metrics if not row["same_fitted_component"])
    if disconnected:
        global_failures.append("unobservable_relative_pose_across_components:" + ",".join(disconnected))
    if not connection_reports:
        global_failures.append("no_connection_reports")
    if any(not row["accepted"] for row in connection_reports):
        global_failures.append("one_or_more_connections_failed")

    return {
        "schema_version": 1, "created_utc": utc_now(),
        "method": "Physical-region aggregation; same-spatial-group LORO measurement-cycle prediction; locked holdouts never used for paths or fit; baseline non-regression guard.",
        "transform_convention": "T_from_to = inv(T_world_from) @ T_world_to; right residual inv(measurement) @ prediction",
        "thresholds": thresholds, "candidate_variant": candidate_variant, "baseline_variant": baseline_variant,
        "graph_identity": graph_identity,
        "selection_conditioned": bool(selection_conditioned),
        "fit_manifest": {"fit_edge_ids_complete": fit["fit_edge_ids_complete"],
            "fit_edge_ids": sorted(fit["fit_edge_ids"]), "heldout_edge_ids": sorted(fit["heldout_edge_ids"]),
            "fit_edge_policy": fit["fit_edge_policy"],
            "minimum_path_information_weight": minimum_weight,
            "path_support_edge_ids": sorted(support_edge_ids),
            "heldout_shared_evidence_edge_ids": sorted(locked_shared_edge_ids),
            "path_weight_filter_complete": weight_filter_complete},
        "shared_evidence_region_components": sorted(
            {tuple(sorted(group)) for group in evidence_groups.values()}),
        "endpoint_sessions": sessions, "regions": region_metrics, "edges": edge_metrics,
        "connections": connection_reports, "critical_pairs": critical_rows,
        "global_failures": global_failures, "accepted": not global_failures,
        "navigation_ready": False,
        "limitations": [
            "LORO fallback is a measurement-path cycle, not a full leave-one-region-out graph reoptimization.",
            "Region and spatial-group declarations must be made before examining candidate residuals.",
            "Paths exclude globally shared declared sensor correlation groups, including reuse across different session connections.",
            "A selection-conditioned candidate is diagnostic only: gate/holdout observations used to select it are excluded from independent admission evidence.",
            "Passing this geometric gate does not replace surveyed door, corridor, stair or floor dimensions."],
        "source_databases_accessed": [],
    }


def _flatten_metrics(row: dict[str, Any]) -> dict[str, Any]:
    output = {}
    for key, value in row.items():
        if isinstance(value, dict):
            for child, child_value in value.items():
                if isinstance(child_value, dict):
                    for grandchild, scalar in child_value.items():
                        output[f"{key}.{child}.{grandchild}"] = scalar
                elif isinstance(child_value, (str, int, float, bool)) or child_value is None:
                    output[f"{key}.{child}"] = child_value
        elif isinstance(value, list):
            output[key] = json.dumps(value, separators=(",", ":"))
        elif isinstance(value, (str, int, float, bool)) or value is None:
            output[key] = value
    return output


def write_output(output: Path, report: dict[str, Any], input_paths: list[Path]) -> None:
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output}")
    report["source_sha256"] = sha256_file(Path(__file__))
    report["input_sha256"] = {str(path.resolve()): sha256_file(path.resolve()) for path in input_paths}
    report["command"] = sys.argv
    output.mkdir(parents=True)
    shutil.copyfile(Path(__file__), output / "source_snapshot.py")
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    for name in ("edges", "regions", "critical_pairs"):
        rows = [_flatten_metrics(row) for row in report[name]]
        with (output / f"{name}.csv").open("w", newline="", encoding="utf-8") as stream:
            if rows:
                writer = csv.DictWriter(stream, fieldnames=sorted({key for row in rows for key in row}))
                writer.writeheader(); writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--derive-regions-from-graph", type=Path,
                        help="Create a correlation-aware region declaration from a backend graph")
    parser.add_argument("--quality-manifest", type=Path)
    parser.add_argument("--required-connection", action="append", default=[])
    parser.add_argument("--regions", type=Path)
    parser.add_argument("--edges", type=Path)
    parser.add_argument("--candidate", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--candidate-variant")
    parser.add_argument("--baseline-variant")
    parser.add_argument("--selection-conditioned", action="store_true",
                        help="Mark a candidate selected using this gate/holdout; never admit it as independent validation")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.derive_regions_from_graph is not None:
        if args.quality_manifest is None:
            parser.error("--quality-manifest is required with --derive-regions-from-graph")
        graph_path = args.derive_regions_from_graph.resolve()
        quality_path = args.quality_manifest.resolve()
        graph = json.loads(graph_path.read_text(encoding="utf-8"))
        correlations, quality_sources = load_correlation_provenance(quality_path)
        declaration, validation_graph, derivation = derive_region_declaration(
            graph, correlations, args.required_connection)
        write_region_declaration(args.output.resolve(), declaration, validation_graph,
                                 derivation, [graph_path, *quality_sources])
        print(json.dumps({"output": str(args.output.resolve()), "accepted": False,
                          "connection_coverage": derivation["connection_coverage"]}, indent=2))
        return
    missing = [name for name in ("regions", "edges", "candidate", "baseline")
               if getattr(args, name) is None]
    if missing:
        parser.error("validation mode requires " + ", ".join("--" + name for name in missing))
    paths = [args.regions, args.edges, args.candidate, args.baseline]
    payloads = [json.loads(path.read_text(encoding="utf-8")) for path in paths]
    report = evaluate(*payloads, candidate_variant=args.candidate_variant,
                      baseline_variant=args.baseline_variant,
                      selection_conditioned=args.selection_conditioned)
    write_output(args.output.resolve(), report, paths)
    print(json.dumps({"output": str(args.output.resolve()), "accepted": report["accepted"],
                      "failures": report["global_failures"]}, indent=2))


if __name__ == "__main__":
    main()
