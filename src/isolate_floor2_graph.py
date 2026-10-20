#!/usr/bin/env python3
"""Isolate one capture for a controlled cross-session-factor ablation.

This is not new registration, upper-floor segmentation, or local bundle
adjustment. The capture also contains lobby and stairs. Retained measurements,
covariances, train/heldout roles, region labels, and initial world poses are
copied without numerical modification. One initial-pose gauge is selected in
each *training* component; heldout edges never connect fitted components.
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import sys

from optimize_robust_submaps import (connected_components, parse_manifest,
                                     select_gauges, sha256_file)


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    ensure_ascii=False).encode("utf-8")).hexdigest()


def isolate(graph, session="floor2"):
    """Return an independent graph and an auditable exclusion record."""
    parse_manifest(graph)  # Reject malformed input before changing its support.
    selected = [copy.deepcopy(row) for row in graph["submaps"]
                if row["session_id"] == session]
    if not selected:
        raise ValueError(f"session has no submaps: {session}")
    ids = {row["id"] for row in selected}
    kept, excluded = [], []
    for row in graph["edges"]:
        touches = row["from"] in ids or row["to"] in ids
        both = row["from"] in ids and row["to"] in ids
        if both and row["type"] != "cross_session":
            kept.append(copy.deepcopy(row))
        else:
            excluded.append({"reason": "cross_session_factor" if touches else
                             "other_session_only", "touches_selected_session": touches,
                             "edge": copy.deepcopy(row)})
    result = {"schema_version": graph["schema_version"], "submaps": selected,
              "edges": kept, "metadata": {}}
    nodes, edges = parse_manifest(result)
    components = connected_components([node.id for node in nodes], edges)
    gauges = select_gauges(nodes, components)
    for row in selected:
        row["gauge"] = row["id"] in set(gauges.values())
    counts = Counter((row["type"], row["role"]) for row in kept)
    detail = {
        "session_id": session,
        "source_canonical_manifest_sha256": canonical_sha256(graph),
        "selected_node_count": len(selected), "retained_edge_count": len(kept),
        "retained_edge_counts": {f"{kind}:{role}": value
                                 for (kind, role), value in sorted(counts.items())},
        "excluded_edge_count": len(excluded),
        "removed_touching_cross_session_edge_ids": [r["edge"]["id"] for r in excluded
                                                    if r["touches_selected_session"]],
        "training_components": components, "gauge_by_component": gauges,
        "retained_edge_rows_sha256": canonical_sha256(kept),
        "initial_poses_unchanged": True,
        "retained_measurements_covariances_roles_unchanged": True,
        "new_registration_performed": False, "accepted": False, "navigation": False,
        "limitations": [
            ("This is the entire floor2 capture, including its lobby and stairs, not an upper-floor geometric mask."
             if session == "floor2" else
             f"This is the entire {session} capture, not a semantic or geometric region mask."),
            "Only cross-session/other-session factors are removed; no RGB-D rematching or per-frame refinement is performed.",
            "A separate initial-pose gauge is fixed per training component; relative poses between components are unobservable.",
            "Historical train/heldout region correlations are retained for a controlled ablation, not independent validation.",
        ],
    }
    result["metadata"] = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "accepted": False, "navigation": False,
        "status": f"{session.upper()}_CAPTURE_ONLY_CAUSAL_ABLATION_NOT_NEW_REGISTRATION",
        "isolation": detail,
    }
    # Keep only method assumptions; never copy stale full-graph diagnostics.
    original = graph.get("metadata", {})
    for key in ("initial_graph", "boundary_transition_policy", "constraints_input_contract"):
        if key in original:
            result["metadata"][key] = copy.deepcopy(original[key])
    if session in original.get("odometry_model_sigma_by_session", {}):
        result["metadata"]["odometry_model_sigma_by_session"] = {
            session: copy.deepcopy(original["odometry_model_sigma_by_session"][session])}
    return result, excluded


def quality_view(quality, quality_path, session):
    if session not in quality.get("sessions", {}):
        raise ValueError(f"quality manifest has no session {session}")
    result = {
        "status": "SINGLE_SESSION_VIEW_OF_EXISTING_QUALITY_AUDIT",
        "source_manifest": str(quality_path.resolve()),
        "source_manifest_sha256": sha256_file(quality_path),
        "selection": {"session_id": session, "new_quality_measurements": False},
        "sessions": {session: copy.deepcopy(quality["sessions"][session])},
    }
    for key in ("thresholds", "algorithms", "independence_statement"):
        if key in quality:
            result[key] = copy.deepcopy(quality[key])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--session", default="floor2",
                        help="Session id to retain (default: floor2 for backward compatibility)")
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError(f"refusing to overwrite {args.output}")
    graph = json.loads(args.manifest.read_text(encoding="utf-8"))
    result, excluded = isolate(graph, args.session)
    metadata = graph["metadata"]
    quality_path = Path(metadata["quality_manifest"])
    expected_hashes = metadata["source_sha256"]
    quality = json.loads(quality_path.read_text(encoding="utf-8"))
    view = quality_view(quality, quality_path, args.session)
    session = view["sessions"][args.session]
    used_paths = [quality_path, Path(session["artifacts"]["frames_csv"]),
                  Path(session["artifacts"]["submaps_json"]),
                  Path(metadata["initial_graph"]) / args.session / "initial.csv"]
    verified = {}
    for path in used_paths:
        value = sha256_file(path)
        if expected_hashes.get(str(path.resolve())) != value:
            raise ValueError(f"source artifact differs from original graph provenance: {path}")
        verified[str(path.resolve())] = value
    # Raw DB is not opened by this graph-only stage. Its historical hash is
    # retained in the unchanged quality-session record, not asserted anew.
    args.output.mkdir(parents=True)
    quality_dir = args.output / "quality_view"
    quality_dir.mkdir()
    view_path = quality_dir / "manifest.json"
    view_path.write_text(json.dumps(view, indent=2) + "\n", encoding="utf-8")
    verified[str(view_path.resolve())] = sha256_file(view_path)
    verified[str(args.manifest.resolve())] = sha256_file(args.manifest)
    result["metadata"].update({
        "quality_manifest": str(view_path.resolve()),
        "source_sha256": verified,
        "source_graph_manifest": str(args.manifest.resolve()),
        "source_graph_manifest_sha256": sha256_file(args.manifest),
    })
    manifest_path = args.output / "manifest.json"
    manifest_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    excluded_path = args.output / "excluded_edges.json"
    excluded_path.write_text(json.dumps(excluded, indent=2) + "\n", encoding="utf-8")
    source = Path(__file__).resolve()
    shutil.copyfile(source, args.output / "source_snapshot.py")
    report = {
        **result["metadata"]["isolation"],
        "argv": sys.argv if argv is None else [str(source), *argv],
        "source_sha256": sha256_file(source),
        "backend_contract_source_sha256": sha256_file(source.with_name("optimize_robust_submaps.py")),
        "verified_source_sha256": verified,
        "output_manifest_sha256": sha256_file(manifest_path),
        "output_canonical_manifest_sha256": canonical_sha256(result),
        "excluded_edges_sha256": sha256_file(excluded_path),
        "raw_databases_opened": False,
    }
    (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "nodes": len(result["submaps"]),
                      "edges": len(result["edges"]),
                      "components": len(report["training_components"]), "accepted": False}))


if __name__ == "__main__":
    main()
