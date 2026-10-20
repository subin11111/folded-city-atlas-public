import csv
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from optimize_robust_submaps import optimize_manifest  # noqa: E402
from summarize_robust_submaps import (  # noqa: E402
    build_summary,
    main,
    structural_bridge_map,
    validate_graph,
)


def pose(x, y=0., z=0., yaw=0.):
    quaternion = Rotation.from_euler("z", yaw).as_quat()
    return [x, y, z, *quaternion]


def edge(edge_id, source, target, dx, *, edge_type="odometry", role="train",
         group=None, dy=0.):
    return {
        "id": edge_id,
        "from": source,
        "to": target,
        "measurement": pose(dx, dy),
        "information": (np.eye(6) * 50.).tolist(),
        "type": edge_type,
        "role": role,
        "region_group": group or edge_id,
        "provenance": {},
    }


def graph_fixture():
    # A and B are joined by a verified cross-session edge.  The structural
    # transition to C has no alternate path and is therefore a graph bridge.
    return {
        "schema_version": 1,
        "submaps": [
            {"id": "a0", "session_id": "A", "pose": pose(0.), "gauge": True},
            {"id": "a1", "session_id": "A", "pose": pose(1.)},
            {"id": "b0", "session_id": "B", "pose": pose(2.)},
            {"id": "b1", "session_id": "B", "pose": pose(3.)},
            {"id": "c0", "session_id": "C", "pose": pose(4.)},
        ],
        "edges": [
            edge("odo_a", "a0", "a1", 1.),
            edge("odo_b", "b0", "b1", 1.),
            edge("door_train", "a1", "b0", 1., edge_type="cross_session", group="door"),
            edge("boundary", "b1", "c0", 1., edge_type="structural", group="boundary"),
            # Same physical region as training: correlated leakage, excluded.
            edge("door_holdout", "a0", "b1", 3.1, edge_type="cross_session",
                 role="heldout", group="door"),
            # A distinct reserved region is the only eligible diagnostic row.
            edge("audit_holdout", "a0", "b1", 3., edge_type="cross_session",
                 role="heldout", group="audit-west"),
        ],
        "metadata": {"accepted": False, "navigation": False},
    }


def write_pair(root: Path, graph=None, methods=("dcs",)):
    graph = graph or graph_fixture()
    graph_path = root / "graph.json"
    graph_path.write_text(json.dumps(graph))
    report = optimize_manifest(graph, methods=methods, include_leave_one_group=False,
                               outer_iterations=3, max_evaluations=40)
    report_path = root / "report.json"
    report_path.write_text(json.dumps(report))
    return graph, graph_path, report, report_path


def test_mixed_region_and_structural_graph_bridge_are_fail_closed(tmp_path):
    graph, graph_path, _, report_path = write_pair(tmp_path)
    summary, tables, _ = build_summary(graph_path, [("default", report_path)], [])

    assert not summary["accepted"] and not summary["navigation"]
    candidate = summary["candidates"][0]
    assert candidate["approval_metric_eligible_heldout_count"] == 1
    assert candidate["excluded_heldout_count"] == 1
    rows = {row["edge_id"]: row for row in tables["edge_diagnostics.csv"]}
    assert rows["audit_holdout"]["approval_metric_eligible"]
    assert not rows["door_holdout"]["approval_metric_eligible"]
    assert rows["door_holdout"]["mixed_train_heldout_region"]
    assert "mixed_train_heldout_region" in rows["door_holdout"]["approval_metric_exclusion_reason"]
    assert rows["boundary"]["structural_graph_bridge_transition"]
    assert not rows["boundary"]["approval_metric_eligible"]
    assert summary["graphs"]["canonical"]["structural_graph_bridge_count"] == 1

    normalized = validate_graph(graph, "test")
    bridge = structural_bridge_map(normalized)
    assert bridge["boundary"]["computed_graph_bridge"]


def test_full_cli_renders_hashed_actual_data_and_refuses_overwrite(tmp_path):
    _, graph_path, _, report_path = write_pair(tmp_path, methods=("dcs", "group_switch"))
    output = tmp_path / "summary"
    arguments = ["--manifest", str(graph_path), "--report", "main", str(report_path),
                 "--output", str(output)]
    main(arguments)

    saved = json.loads((output / "summary.json").read_text())
    assert saved["status"] == "experimental_diagnostic_only_no_approval"
    assert not saved["accepted"] and not saved["navigation"]
    expected = {
        "edge_weights_chi2_by_type.png",
        "edge_weights_chi2_by_region.png",
        "correction_distributions_by_session.png",
        "heldout_residual_comparison.png",
        "graph_components_structural_bridges.png",
        "candidate_comparison.png",
        "edge_diagnostics.csv",
        "region_summary.csv",
        "corrections.csv",
        "candidate_comparison.csv",
        "components.csv",
        "structural_bridges.csv",
        "source_snapshot.py",
    }
    assert expected <= set(saved["artifacts"])
    assert all(len(saved["artifacts"][name]["sha256"]) == 64 for name in expected)
    assert all((output / name).stat().st_size > 0 for name in expected)
    with (output / "candidate_comparison.csv").open(newline="") as stream:
        candidates = list(csv.DictReader(stream))
    assert {row["candidate_id"] for row in candidates} == {"main/dcs", "main/group_switch"}
    assert all(row["accepted"] == "False" and row["navigation"] == "False"
               for row in candidates)
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        main(arguments)


def test_report_graph_hash_and_fit_inventory_mismatch_are_fatal(tmp_path):
    graph, graph_path, report, report_path = write_pair(tmp_path)
    changed = json.loads(json.dumps(graph))
    changed["metadata"]["note"] = "changes canonical graph hash"
    changed_path = tmp_path / "changed_graph.json"
    changed_path.write_text(json.dumps(changed))
    with pytest.raises(ValueError, match="canonical manifest hash"):
        build_summary(changed_path, [("bad", report_path)], [])

    report["variants"]["dcs"]["fit_edge_ids"].append("door_holdout")
    broken_report = tmp_path / "broken_report.json"
    broken_report.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="fit_edge_ids differ"):
        build_summary(graph_path, [("bad", broken_report)], [])


def test_sensitivity_graph_and_report_are_paired_and_separately_hashed(tmp_path):
    primary = tmp_path / "primary"
    primary.mkdir()
    _, graph_path, _, report_path = write_pair(primary)

    sensitivity = tmp_path / "sensitivity"
    sensitivity.mkdir()
    graph = graph_fixture()
    graph["metadata"]["boundary_covariance_inflation"] = 100.
    _, sensitivity_graph, _, sensitivity_report = write_pair(sensitivity, graph)
    summary, _, graphs = build_summary(
        graph_path, [("default", report_path)],
        [("weak-boundary", sensitivity_graph, sensitivity_report)],
    )
    assert set(graphs) == {"canonical", "weak-boundary"}
    assert {row["candidate_id"] for row in summary["candidates"]} == {
        "default/dcs", "sensitivity:weak-boundary/dcs"}
    graph_inputs = [row for row in summary["inputs"] if row["kind"] == "graph_manifest"]
    assert len(graph_inputs) == 2
    assert all(len(row["sha256"]) == 64 and len(row["canonical_json_sha256"]) == 64
               for row in graph_inputs)
