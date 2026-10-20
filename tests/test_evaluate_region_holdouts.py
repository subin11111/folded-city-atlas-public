import copy
import json
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from evaluate_region_holdouts import (derive_region_declaration, evaluate, normalize_regions,
                                      shortest_prediction, validate_candidate_graph_identity,
                                      canonical_sha256, write_output)


def pose(x, y, z=0.):
    return [x, y, z, 0., 0., 0., 1.]


def synthetic():
    poses = {
        "A0": pose(0, 0), "A1": pose(1, 0), "A2": pose(2, 0),
        "B0": pose(0, 1), "B1": pose(1, 1), "B2": pose(2, 1),
    }
    submaps = [{"id": key, "session_id": key[0], "pose": value}
               for key, value in poses.items()]
    edges = [
        {"id": "oa01", "from": "A0", "to": "A1", "measurement": pose(1, 0),
         "type": "odometry", "role": "train"},
        {"id": "oa12", "from": "A1", "to": "A2", "measurement": pose(1, 0),
         "type": "odometry", "role": "train"},
        {"id": "ob01", "from": "B0", "to": "B1", "measurement": pose(1, 0),
         "type": "odometry", "role": "train"},
        {"id": "ob12", "from": "B1", "to": "B2", "measurement": pose(1, 0),
         "type": "odometry", "role": "train"},
        {"id": "cross0", "from": "A0", "to": "B0", "measurement": pose(0, 1),
         "type": "cross_session", "role": "train", "region_group": "r0"},
        {"id": "cross1", "from": "A1", "to": "B1", "measurement": pose(0, 1),
         "type": "cross_session", "role": "train", "region_group": "r1"},
        {"id": "cross2", "from": "A2", "to": "B2", "measurement": pose(0, 1),
         "type": "cross_session", "role": "heldout", "held_out": True,
         "region_group": "r2"},
    ]
    regions = {"regions": [
        {"region_id": "r0", "connection_id": "A__B", "spatial_group": "west",
         "locked_holdout": False, "correlation_group_ids": ["ca0", "cb0"]},
        {"region_id": "r1", "connection_id": "A__B", "spatial_group": "middle",
         "locked_holdout": False, "correlation_group_ids": ["ca1", "cb1"]},
        {"region_id": "r2", "connection_id": "A__B", "spatial_group": "east",
         "locked_holdout": True, "critical": True,
         "correlation_group_ids": ["ca2", "cb2"]},
    ]}
    fit = [row["id"] for row in edges if row["role"] == "train"]
    candidate = {"variants": {"dcs": {"poses": poses,
        "fit_edge_ids": fit, "heldout_edge_ids": ["cross2"],
        "fit_edge_ids_complete": True, "fit_edge_policy": "all and only role=train",
        "edges": [{"id": row["id"], "used_for_fit": row["id"] in fit} for row in edges]}}}
    baseline = {"submaps": submaps}
    return regions, {"submaps": submaps, "edges": edges}, candidate, baseline


def test_backend_schema_passes_three_spatial_region_gate_and_loro_is_disjoint():
    regions, edges, candidate, baseline = synthetic()
    report = evaluate(regions, edges, candidate, baseline, candidate_variant="dcs")
    assert report["accepted"]
    connection = report["connections"][0]
    assert connection["coverage"] == {
        "physical_regions": 3, "spatial_groups": 3,
        "training_spatial_groups": 2, "locked_holdout_spatial_groups": 1,
        "candidate_evaluable_spatial_groups": 3, "loro_evaluable_spatial_groups": 3,
    }
    heldout = next(row for row in report["edges"] if row["edge_id"] == "cross2")
    assert "cross2" not in heldout["loro_path_edge_ids"]
    assert set(heldout["loro_path_edge_ids"]) <= set(report["fit_manifest"]["fit_edge_ids"])
    assert heldout["loro_cycle"]["translation_m"] < 1e-12


def test_selection_conditioned_candidate_is_diagnostic_only_even_when_all_gates_pass():
    regions, edges, candidate, baseline = synthetic()
    report = evaluate(regions, edges, candidate, baseline, candidate_variant="dcs",
                      selection_conditioned=True)
    assert report["selection_conditioned"] is True
    assert not report["accepted"]
    assert report["connections"][0]["accepted"]
    assert report["global_failures"] == [
        "selection_conditioned_gate_is_not_independent_validation"]
    assert not report["navigation_ready"]


def test_disconnected_display_gauges_are_not_heldout_measurements():
    regions, payload, candidate, baseline = synthetic()
    for region in regions["regions"]:
        region["locked_holdout"] = True
    for edge in payload["edges"]:
        if edge["type"] == "cross_session":
            edge["role"] = "heldout"
    variant = candidate["variants"]["dcs"]
    variant["fit_edge_ids"] = [e["id"] for e in payload["edges"] if e["type"] == "odometry"]
    variant["heldout_edge_ids"] = ["cross0", "cross1", "cross2"]
    for row in variant["edges"]:
        row["used_for_fit"] = row["id"] in variant["fit_edge_ids"]
    report = evaluate(regions, payload, candidate, baseline, candidate_variant="dcs")
    assert all(row["candidate"] is None for row in report["edges"])
    assert any(reason.startswith("unobservable_relative_pose_across_components:")
               for reason in report["global_failures"])
    assert not report["accepted"]


def test_many_adjacent_edges_in_one_region_count_once():
    regions, payload, candidate, baseline = synthetic()
    duplicate = dict(payload["edges"][4], id="cross0-neighbour")
    payload["edges"].append(duplicate)
    variant = candidate["variants"]["dcs"]
    variant["fit_edge_ids"].append("cross0-neighbour")
    variant["edges"].append({"id": "cross0-neighbour", "used_for_fit": True})
    report = evaluate(regions, payload, candidate, baseline, candidate_variant="dcs")
    assert report["accepted"]
    assert report["connections"][0]["coverage"]["physical_regions"] == 3
    assert next(row for row in report["regions"] if row["region_id"] == "r0")["edge_count"] == 2


def test_backend_leave_one_group_reoptimization_is_preferred_to_path_fallback():
    regions, edges, candidate, baseline = synthetic()
    candidate["variants"]["dcs"]["leave_one_group_diagnostics"] = [{
        "region_group": "r0", "disconnects_graph": False,
        "removed_group_prediction": [{"edge_id": "cross0",
            "translation_error_m": .03, "rotation_error_deg": .4}],
    }]
    report = evaluate(regions, edges, candidate, baseline, candidate_variant="dcs")
    row = next(row for row in report["edges"] if row["edge_id"] == "cross0")
    assert row["loro_method"] == "backend_leave_one_group_reoptimization"
    assert row["loro_cycle"] == {"translation_m": .03, "rotation_deg": .4}
    assert row["loro_path_edge_ids"] == []


def test_locked_holdout_fit_leakage_is_reported_fail_closed():
    regions, edges, candidate, baseline = synthetic()
    candidate["variants"]["dcs"]["fit_edge_ids"].append("cross2")
    candidate["variants"]["dcs"]["edges"][-1]["used_for_fit"] = True
    report = evaluate(regions, edges, candidate, baseline, candidate_variant="dcs")
    assert not report["accepted"]
    assert any(reason.startswith("locked_holdout_leakage_into_fit:cross2")
               for reason in report["global_failures"])


def test_severe_regression_fails_even_when_absolute_max_is_below_gate():
    regions, edges, candidate, baseline = synthetic()
    candidate["variants"]["dcs"]["poses"]["B2"] = pose(2.12, 1)
    report = evaluate(regions, edges, candidate, baseline, candidate_variant="dcs")
    assert not report["accepted"]
    assert "severe_regression_detected" in report["connections"][0]["failures"]
    critical = next(row for row in report["critical_pairs"] if row["edge_id"] == "cross2")
    assert critical["candidate"]["translation_m"] == pytest.approx(.12)
    assert critical["loro_cycle"]["translation_m"] < 1e-12
    assert "severe_regression" in critical["failures"]


def test_spatially_correlated_regions_do_not_satisfy_coverage():
    regions, edges, candidate, baseline = synthetic()
    regions["regions"][1]["spatial_group"] = "west"
    report = evaluate(regions, edges, candidate, baseline, candidate_variant="dcs")
    assert not report["accepted"]
    connection = report["connections"][0]
    assert connection["coverage"]["physical_regions"] == 3
    assert connection["coverage"]["spatial_groups"] == 2
    assert "spatial_group_coverage_failed" in connection["failures"]


def test_correlation_group_cannot_be_split_across_physical_regions():
    regions, _, _, _ = synthetic()
    regions["regions"][1]["correlation_group_ids"].append("ca0")
    with pytest.raises(ValueError, match="Correlation group ca0 split"):
        normalize_regions(regions)


def test_refuses_to_overwrite_output(tmp_path):
    output = tmp_path / "result"
    output.mkdir()
    with pytest.raises(FileExistsError, match="Refusing to overwrite"):
        write_output(output, {"edges": [], "regions": [], "critical_pairs": []}, [])


def test_fresh_output_contains_exact_evaluator_source_snapshot(tmp_path):
    output = tmp_path / "result"
    write_output(output, {"edges": [], "regions": [], "critical_pairs": []}, [])
    source = Path(__file__).resolve().parents[1] / "src" / "evaluate_region_holdouts.py"
    assert (output / "source_snapshot.py").read_bytes() == source.read_bytes()
    written = json.loads((output / "report.json").read_text(encoding="utf-8"))
    assert written["source_sha256"]
    assert written["input_sha256"] == {}


def test_shortest_prediction_blocks_every_edge_in_requested_region():
    identity = np.eye(4)
    edges = [
        {"edge_id": "bad-direct", "from": "a", "to": "b", "measurement": identity,
         "role": "train", "region_id": "same-place"},
        {"edge_id": "alt1", "from": "a", "to": "c", "measurement": identity,
         "role": "train", "region_id": None},
        {"edge_id": "alt2", "from": "c", "to": "b", "measurement": identity,
         "role": "train", "region_id": "other-place"},
    ]
    prediction, path = shortest_prediction(edges, "a", "b", {"same-place"}, None)
    np.testing.assert_allclose(prediction, identity)
    assert path == ["alt1", "alt2"]


def test_region_derivation_unions_shared_correlation_and_retains_mixed_role_failure():
    graph = {"schema_version": 1, "submaps": [
        {"id": "A0", "session_id": "A", "pose": pose(0, 0)},
        {"id": "B0", "session_id": "B", "pose": pose(0, 1)},
        {"id": "A1", "session_id": "A", "pose": pose(1, 0)},
        {"id": "B1", "session_id": "B", "pose": pose(1, 1)},
    ], "edges": [
        {"id": "e0", "from": "A0", "to": "B0", "measurement": pose(0, 1),
         "covariance": np.eye(6).tolist(), "type": "cross_session", "role": "train",
         "region_group": "cell0", "provenance": {"frame_endpoints": ["A", 1, "B", 1]}},
        {"id": "e1", "from": "A1", "to": "B1", "measurement": pose(0, 1),
         "covariance": np.eye(6).tolist(), "type": "cross_session", "role": "heldout",
         "region_group": "cell1", "provenance": {"frame_endpoints": ["A", 1, "B", 2]}},
    ]}
    correlations = {
        ("A", 1): {"correlation_group_id": "A:g1"},
        ("B", 1): {"correlation_group_id": "B:g1"},
        ("B", 2): {"correlation_group_id": "B:g2"},
    }
    declaration, validation_graph, report = derive_region_declaration(
        graph, correlations, ["A__B", "C__D"])
    observed = [row for row in declaration["regions"] if row["edge_ids"]]
    assert len(observed) == 1
    assert observed[0]["locked_holdout"]
    assert observed[0]["independence_status"] == "mixed_train_heldout_same_physical_region"
    assert observed[0]["source_region_groups"] == ["cell0", "cell1"]
    remapped = {row["region_group"] for row in validation_graph["edges"]}
    assert remapped == {observed[0]["region_id"]}
    assert report["mixed_role_regions"] == [observed[0]["region_id"]]
    assert report["missing_required_connections"] == ["C__D"]
    assert not declaration["accepted"] and not declaration["navigation_ready"]


def test_region_aliases_preserve_candidate_graph_identity_but_role_changes_do_not():
    _, graph, candidate, _ = synthetic()
    candidate["provenance"] = {"canonical_manifest_sha256": canonical_sha256(graph)}
    _, aliased, _ = derive_region_declaration(graph, {}, [])
    assert validate_candidate_graph_identity(aliased, candidate)["verified"]
    tampered = copy.deepcopy(aliased)
    tampered["edges"][4]["role"] = "heldout"
    with pytest.raises(ValueError, match="changed objective factors"):
        validate_candidate_graph_identity(tampered, candidate)
    tampered = copy.deepcopy(aliased)
    tampered["edges"][4]["measurement"][0] += .1
    with pytest.raises(ValueError, match="changed objective factors"):
        validate_candidate_graph_identity(tampered, candidate)


def test_candidate_from_another_graph_cannot_use_stale_region_declarations():
    _, graph, candidate, _ = synthetic()
    _, aliased, _ = derive_region_declaration(graph, {}, [])
    changed = copy.deepcopy(graph)
    changed["edges"] = changed["edges"][:-1]
    candidate["provenance"] = {"canonical_manifest_sha256": canonical_sha256(changed)}
    with pytest.raises(ValueError, match="does not match the physical-region declaration"):
        validate_candidate_graph_identity(aliased, candidate)


def test_locked_sensor_evidence_cannot_reenter_fit_through_another_connection():
    regions, graph, candidate, baseline = synthetic()
    regions["regions"].append({
        "region_id": "r3", "connection_id": "A__C", "spatial_group": "third-session",
        "locked_holdout": False, "correlation_group_ids": ["ca2", "cc0"],
    })
    graph["submaps"].append({"id": "C0", "session_id": "C", "pose": pose(2, 2)})
    graph["edges"].append({"id": "cross3", "from": "A2", "to": "C0",
        "measurement": pose(0, 2), "type": "cross_session", "role": "train", "region_group": "r3"})
    variant = candidate["variants"]["dcs"]
    variant["poses"]["C0"] = pose(2, 2)
    variant["fit_edge_ids"].append("cross3")
    variant["edges"].append({"id": "cross3", "used_for_fit": True})
    baseline["submaps"] = graph["submaps"]
    report = evaluate(regions, graph, candidate, baseline, candidate_variant="dcs")
    assert "locked_sensor_evidence_reused_in_fit:cross3" in report["global_failures"]
    assert "cross3" not in report["fit_manifest"]["path_support_edge_ids"]
    heldout = next(row for row in report["edges"] if row["edge_id"] == "cross2")
    assert heldout["loro_blocked_region_ids"] == ["r2", "r3"]
    assert not report["accepted"]
