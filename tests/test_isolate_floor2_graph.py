import copy
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from isolate_floor2_graph import isolate, quality_view


def fixture_graph():
    nodes = [{"id": key, "session_id": session,
              "pose": [i, 2, 3, 0, 0, 0, 1], "anchor_node_id": i}
             for i, (key, session) in enumerate(
                 [("f1", "floor2"), ("f2", "floor2"), ("f3", "floor2"), ("l1", "lobby")])]
    def edge(key, a, b, kind, role):
        return {"id": key, "from": a, "to": b, "type": kind, "role": role,
                "measurement": [1, 0, 0, 0, 0, 0, 1], "covariance": np.eye(6).tolist(),
                "region_group": key, "provenance": {"evidence": "original"}}
    return {"schema_version": 1, "submaps": nodes, "edges": [
        edge("odom", "f1", "f2", "odometry", "train"),
        edge("heldout", "f2", "f3", "loop", "heldout"),
        edge("cross", "f3", "l1", "cross_session", "train"),
        edge("cross_heldout", "f1", "l1", "cross_session", "heldout")],
        "metadata": {"diagnostics": {"stale_full_graph_count": 999}}}


def test_retains_exact_rows_and_world_poses_without_mutating_source():
    source = fixture_graph()
    before = copy.deepcopy(source)
    result, excluded = isolate(source)
    assert source == before
    assert result["edges"] == source["edges"][:2]
    assert [row["pose"] for row in result["submaps"]] == [row["pose"] for row in source["submaps"][:3]]
    assert {row["edge"]["id"] for row in excluded} == {"cross", "cross_heldout"}
    assert "diagnostics" not in result["metadata"]
    assert not result["metadata"]["isolation"]["new_registration_performed"]


def test_heldout_does_not_connect_gauges():
    result, _ = isolate(fixture_graph())
    assert result["metadata"]["isolation"]["training_components"] == [["f1", "f2"], ["f3"]]
    assert {r["id"] for r in result["submaps"] if r["gauge"]} == {"f1", "f3"}
    assert {r["session_id"] for r in result["submaps"]} == {"floor2"}


def test_missing_session_and_invalid_endpoints_rejected():
    with pytest.raises(ValueError, match="no submaps"):
        isolate(fixture_graph(), "missing")
    bad = fixture_graph()
    bad["edges"][0]["to"] = "missing"
    with pytest.raises(ValueError, match="invalid endpoints"):
        isolate(bad)


def test_quality_view_preserves_only_session_records(tmp_path):
    source = {"sessions": {"floor2": {"artifacts": {"frames_csv": "/data/f.csv"}},
                           "lobby": {"count": 5}}, "thresholds": {"min": 1},
              "all_databases_unchanged": True}
    path = tmp_path / "manifest.json"
    path.write_text(json.dumps(source))
    view = quality_view(source, path, "floor2")
    assert view["sessions"] == {"floor2": source["sessions"]["floor2"]}
    assert view["thresholds"] == source["thresholds"]
    assert "all_databases_unchanged" not in view
    assert not view["selection"]["new_quality_measurements"]
