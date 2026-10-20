import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from evaluate_floor2_local import (matched_support, display_align, provenance,
                                  candidate_paths, common_sample_nodes, parse_args,
                                  checked_trajectory, all_stage_common_support)


def row(index, candidate, previous=.04, initial=.05, integrated=.06):
    return {"a": index, "b": index + 10, "kind": "wall", "plane_a": 0, "plane_b": 0,
            "values": {"previous_preview": previous, "selected_initial": initial,
                       "integrated_dcs": integrated, "candidate": candidate}}


def test_matched_support_never_credits_missing_bad_patch():
    result = matched_support([row(1, .08), row(2, None, previous=10.)], "candidate")
    assert result["candidate_missing_count"] == 1
    assert result["per_baseline"]["previous_preview"]["baseline"]["median_m"] == .04
    assert result["common_all_baselines"]["matched_count"] == 1
    assert result["candidate_coverage_fraction"] == .5


def test_each_baseline_and_best_have_own_matched_support():
    result = matched_support([row(1, .1, integrated=None), row(2, .13)], "candidate")
    assert result["per_baseline"]["integrated_dcs"]["matched_count"] == 1
    assert result["per_baseline"]["previous_preview"]["severe_regression_count"] == 2
    assert result["per_baseline"]["selected_initial"]["severe_regression_count"] == 1
    best = result["per_patch_best_available_baseline"]
    assert best["severe_regression_count"] == 2
    assert sorted(best["baseline_available_count_by_patch"].values()) == [2, 3]


def test_cross_candidate_summary_has_one_identical_patch_inventory():
    rows = [row(1, .08), row(2, .10), row(3, .12, integrated=None)]
    for index, item in enumerate(rows):
        item["values"]["other"] = .09 if index != 1 else None
    result = all_stage_common_support(rows, ["candidate", "other"])
    assert result["matched_count"] == 1
    assert result["metrics"]["candidate"]["median_m"] == .08
    assert result["metrics"]["other"]["median_m"] == .09
    assert result["metrics"]["previous_preview"]["median_m"] == .04
    assert len(result["excluded_patch_ids"]) == 2


def test_display_alignment_is_one_rigid_transform_only():
    a, b = np.eye(4), np.eye(4)
    b[0, 3] = 2
    gauge = np.eye(4)
    gauge[1, 3] = 4
    poses = {"selected_initial": {1: a, 3: b}, "new": {1: gauge @ a, 3: gauge @ b}}
    aligned, metadata = display_align(poses)
    np.testing.assert_allclose(aligned["new"][3], b)
    assert metadata["used_for_metrics"] is False
    assert 2 not in aligned["new"]


def test_missing_graph_does_not_claim_independence():
    assert provenance(None, 1, 2)["graph_partition_status"] == "not_provided"
    assert provenance(None, 1, 2)["independent_ground_truth"] is False


def test_duplicate_baseline_names_rejected_before_file_loading():
    with pytest.raises(ValueError, match="reserved"):
        candidate_paths([("selected_initial", "irrelevant")])


def test_xy_sampling_uses_common_frames_but_records_missing_support():
    poses = {"selected_initial": {n: np.eye(4) for n in range(1, 8)},
             "new": {n: np.eye(4) for n in [1, 5, 7]}}
    nodes, metadata = common_sample_nodes(poses, 2)
    assert nodes == [1, 5, 7]
    assert metadata["reference_sampled_nodes"] == [1, 3, 5, 7]
    assert metadata["excluded_by_missing_stage"]["new"] == [3]


def test_bad_stride_rejected():
    with pytest.raises(SystemExit):
        parse_args(["--trajectory", "candidate", "unused", "--output", "unused", "--frame-stride", "0"])


def test_duplicate_pose_ids_are_rejected(tmp_path):
    path = tmp_path / "poses.csv"
    path.write_text("id,x,y,z,qx,qy,qz,qw\n1,0,0,0,0,0,0,1\n1,1,0,0,0,0,0,1\n")
    with pytest.raises(ValueError, match="duplicate"):
        checked_trajectory(path)


def test_nonfinite_poses_are_rejected(tmp_path):
    path = tmp_path / "poses.csv"
    path.write_text("id,x,y,z,qx,qy,qz,qw\n1,nan,0,0,0,0,0,1\n")
    with pytest.raises(ValueError, match="nonfinite"):
        checked_trajectory(path)
