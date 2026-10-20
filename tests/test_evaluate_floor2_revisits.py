import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from evaluate_floor2_revisits import (support_metrics, local_normals, read_selection,
                                    graph_binding, frame_window_metrics, window_observability,
                                    check_ba_selection)
from optimize_session import apply, sha256_file


def planar_grid():
    x, y = np.meshgrid(np.linspace(-.2, .2, 9), np.linspace(-.2, .2, 9))
    return np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))


def test_coplanar_but_disjoint_support_is_not_a_success():
    a = planar_grid()
    b = a + [2., 0., 0.]
    assert np.max(np.abs(b[:, 2])) == 0.  # A plane-only score would be perfect.
    score = support_metrics(a, b)
    assert score["minimum_bidirectional_support_fraction"] == 0.
    assert score["symmetric_clipped_rmse_m"] == pytest.approx(.3)
    assert score["a_nn_median_m"] > 1.


def test_support_uses_both_directions_and_does_not_discard_failed_points():
    a = planar_grid()
    b = np.vstack((a, a + [4, 0, 0]))
    score = support_metrics(a, b)
    assert score["a_within_radius_fraction"] == 1.
    assert score["b_within_radius_fraction"] == .5
    assert score["minimum_bidirectional_support_fraction"] == .5
    assert score["symmetric_clipped_rmse_m"] > .1


def test_joint_gauge_preserves_support_and_normal_scores():
    a = planar_grid()
    b = a + [0, 0, .04]
    normals = np.tile([0., 0., 1.], (len(a), 1))
    gauge = np.eye(4)
    gauge[:3, :3] = Rotation.from_euler("xyz", [.3, .2, -.1]).as_matrix()
    gauge[:3, 3] = [10, -3, 8]
    before = support_metrics(a, b, normals_a=normals, normals_b=normals)
    moved = normals @ gauge[:3, :3].T
    after = support_metrics(apply(a, gauge), apply(b, gauge), normals_a=moved, normals_b=moved)
    assert after["symmetric_clipped_rmse_m"] == pytest.approx(before["symmetric_clipped_rmse_m"])
    assert after["minimum_bidirectional_support_fraction"] == before["minimum_bidirectional_support_fraction"]
    assert after["normal_difference_median_degrees"] == pytest.approx(0., abs=2e-6)


def test_normal_statistic_cannot_claim_agreement_without_overlap():
    points = planar_grid()
    normals = local_normals(points)
    result = support_metrics(points, points + [5, 0, 0], normals_a=normals, normals_b=normals)
    assert result["normal_correspondence_count_by_direction"] == [0, 0]
    assert result["normal_difference_median_degrees"] is None


def test_training_validation_overlap_rejected(tmp_path):
    file = tmp_path / "selection.json"
    file.write_text(json.dumps({"training_node_ids": [1, 3], "validation_windows": [
        {"name": "bad", "visit_a": [1], "visit_b": [2]}]}))
    with pytest.raises(ValueError, match="overlap"):
        read_selection(file)


def test_empty_validation_window_rejected(tmp_path):
    file = tmp_path / "selection.json"
    file.write_text(json.dumps({"training_node_ids": [1], "validation_windows": [
        {"name": "bad", "visit_a": [], "visit_b": [2]}]}))
    with pytest.raises(ValueError, match="empty"):
        read_selection(file)


def test_missing_whole_frame_does_not_reduce_scored_inventory():
    class Clouds:
        def get(self, node):
            return planar_grid(), np.empty((0, 3))
    poses = {"complete": {n: np.eye(4) for n in [1, 2, 3, 4]},
             "missing": {n: np.eye(4) for n in [1, 2, 3]}}
    selection = {"validation_windows": [{"name": "window", "visit_a": [1, 2], "visit_b": [3, 4]}]}
    rows, _, _ = frame_window_metrics(Clouds(), poses, selection)
    assert rows[0]["available"] is True
    assert rows[1]["available"] is False
    assert rows[1]["missing_node_ids"] == [4]
    assert "symmetric_clipped_rmse_m" not in rows[1]


def test_binding_checks_both_files_and_catches_stale_trajectory(tmp_path):
    poses, graph, report = [tmp_path / name for name in ("trajectory.csv", "manifest.json", "report.json")]
    poses.write_text("poses")
    graph.write_text("{}")
    report.write_text(json.dumps({"trajectory_sha256": sha256_file(poses), "graph_manifest_sha256": sha256_file(graph)}))
    assert graph_binding(poses, graph, report)["verified"] is True
    poses.write_text("different poses")
    with pytest.raises(ValueError, match="trajectory hash"):
        graph_binding(poses, graph, report)


def test_unbound_graph_is_not_certified(tmp_path):
    poses, graph, report = [tmp_path / name for name in ("trajectory.csv", "manifest.json", "report.json")]
    poses.write_text("poses")
    graph.write_text("{}")
    report.write_text("{}")
    assert graph_binding(poses, graph, report)["verified"] is False


def test_empty_cloud_is_unavailable_not_zero_error():
    score = support_metrics(np.empty((0, 3)), planar_grid())
    assert score["available"] is False
    assert "symmetric_clipped_rmse_m" not in score


def test_candidate_improvement_cannot_clear_baseline_window_observability_warning():
    rows = [{"window": "w", "stage": "previous_preview", "available": True,
             "minimum_bidirectional_support_fraction": .03},
            {"window": "w", "stage": "candidate", "available": True,
             "minimum_bidirectional_support_fraction": 1.}]
    result = window_observability(rows)["w"]
    assert result["status"] == "low_overlap_inconclusive"
    assert result["historical_baseline_maximum_minimum_bidirectional_support"] == .03


def test_incomplete_ba_report_and_wrong_selection_rejected(tmp_path):
    report, selection = tmp_path / "report.json", tmp_path / "selection.json"
    selection.write_text("{}")
    report.write_text(json.dumps({"complete": False, "selection_sha256": sha256_file(selection)}))
    with pytest.raises(ValueError, match="incomplete"):
        check_ba_selection(report, selection)
    report.write_text(json.dumps({"complete": True, "selection_sha256": "wrong"}))
    with pytest.raises(ValueError, match="selection hash"):
        check_ba_selection(report, selection)
