import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from evaluate_submap_candidates import (aggregate_metrics, ensure_fresh_output,
                                        make_comparison, pair_graph_provenance,
                                        severe_regressions)


def comparison(value, *, session="lobby", status="graph_training"):
    return make_comparison(
        {
            "session": session,
            "a": 1,
            "b": 2,
            "kind": "wall",
            "plane_a": 0,
            "plane_b": 0,
            "graph_partition_status": status,
        },
        {"previous_preview": .10, "selected_initial": .12, "dcs": value},
        ["dcs"],
    )


def test_aggregate_reports_distribution_and_missing_coverage():
    rows = [
        comparison(.08),
        make_comparison(
            {"session": "outside", "a": 3, "b": 4, "kind": "horizontal",
             "plane_a": 0, "plane_b": 0,
             "graph_partition_status": "graph_independent_historical"},
            {"previous_preview": .20, "selected_initial": .18, "dcs": None},
            ["dcs"],
        ),
    ]
    aggregate = aggregate_metrics(rows, ["selected_initial", "dcs"])
    assert aggregate["selected_initial"]["available_count"] == 2
    assert np.isclose(aggregate["selected_initial"]["median_m"], .15)
    assert aggregate["dcs"]["available_count"] == 1
    assert aggregate["dcs"]["missing_count"] == 1
    assert aggregate["dcs"]["coverage_fraction"] == .5
    assert aggregate["dcs"]["p90_m"] == .08


def test_severe_regression_is_against_better_frozen_baseline():
    good = comparison(.145)  # +4.5 cm versus the better .10 m baseline.
    bad = comparison(.151)   # +5.1 cm: severe even though close to initial.
    missing = make_comparison(
        {"session": "floor2", "a": 8, "b": 9, "kind": "wall",
         "plane_a": 0, "plane_b": 0,
         "graph_partition_status": "correlated_heldout"},
        {"previous_preview": .10, "selected_initial": .11, "dcs": None},
        ["dcs"],
    )
    severe = severe_regressions([good, bad, missing], ["dcs"])
    assert severe["dcs"] == [bad]
    assert bad["candidate_deltas"]["dcs"]["severe_regression"] is True
    assert missing["candidate_deltas"]["dcs"]["severe_regression"] is None


def test_existing_output_is_refused(tmp_path):
    output = tmp_path / "evaluation"
    output.mkdir()
    marker = output / "keep.txt"
    marker.write_text("unchanged")
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        ensure_fresh_output(output)
    assert marker.read_text() == "unchanged"


def test_graph_provenance_distinguishes_training_heldout_and_absent():
    graph = {
        "edges": [
            {"id": "train", "role": "train", "region_group": "door",
             "provenance": {"frame_endpoints": ["lobby", 1, "lobby", 2]}},
            {"id": "held", "role": "heldout", "region_group": "door",
             "provenance": {"frame_endpoints": ["lobby", 3, "lobby", 4]}},
        ]
    }
    assert pair_graph_provenance(graph, "lobby", 2, 1)["graph_partition_status"] == "graph_training"
    held = pair_graph_provenance(graph, "lobby", 3, 4)
    assert held["graph_partition_status"] == "correlated_heldout"
    assert "physical-region" in held["graph_correlation_reason"]
    absent = pair_graph_provenance(graph, "lobby", 5, 6)
    assert absent["graph_partition_status"] == "graph_independent_historical"
    assert absent["independent_ground_truth"] is False
