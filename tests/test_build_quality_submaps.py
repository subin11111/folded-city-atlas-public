import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from build_quality_submaps import (
    assign_correlation_groups,
    assign_submap_memberships,
    build_submaps,
    depth_quality_metrics,
    score_quality,
)


def pose(x=0.0, yaw_degrees=0.0):
    angle = math.radians(yaw_degrees)
    result = np.eye(4)
    result[:2, :2] = [[math.cos(angle), -math.sin(angle)],
                      [math.sin(angle), math.cos(angle)]]
    result[0, 3] = x
    return result


def good_metrics(**updates):
    metrics = {
        "usable_depth_fraction": .70,
        "confidence_100_fraction": .50,
        "depth_neighbor_fraction": .85,
        "blur_laplacian_variance_640px": 30.0,
        "clipped_exposure_fraction": .10,
        "intrinsic_jump_depth_px": .10,
        "linear_step_m": .10,
        "angular_step_deg": 2.0,
        "timestamp_step_s": 1.0,
    }
    metrics.update(updates)
    return metrics


def test_quality_grade_has_explicit_critical_veto():
    score, grade, reasons, critical = score_quality(good_metrics())
    assert score == 100 and grade == "A"
    assert reasons == [] and critical == []
    score, grade, reasons, critical = score_quality(
        good_metrics(intrinsic_jump_depth_px=4.0)
    )
    assert grade == "C"
    assert "critical_intrinsic_jump" in reasons
    assert critical == ["critical_intrinsic_jump"]


def test_missing_rgb_is_neutral_but_recorded():
    score, grade, reasons, critical = score_quality(good_metrics(
        blur_laplacian_variance_640px=None,
        clipped_exposure_fraction=None,
    ))
    assert grade == "A"
    assert "rgb_unavailable_neutral_score" in reasons
    assert critical == []
    assert score == 94


def test_depth_metrics_report_confidence_categories_and_edges():
    depth = np.ones((4, 4), np.float32)
    depth[:, 2:] = 2.0
    confidence = np.array([
        [100, 100, 50, 50],
        [100, 100, 50, 50],
        [100, 100, 0, 0],
        [100, 100, 0, 0],
    ], np.uint8)
    result = depth_quality_metrics(depth, confidence)
    assert result["confidence_100_fraction"] == .5
    assert result["confidence_50_fraction"] == .25
    assert result["confidence_0_fraction"] == .25
    assert result["usable_depth_fraction"] == .75
    assert 0.0 < result["depth_edge_fraction"] < 1.0
    assert result["depth_neighbor_fraction"] > .5


def synthetic_frame(index, x, usable=True, hard=False):
    return {
        "frame_index": index,
        "node_id": index + 1,
        "stamp": float(index),
        "_pose": pose(x),
        "session_path_m": x,
        "linear_step_m": 0.0 if index == 0 else .25,
        "quality_grade": "A" if usable else "C",
        "usable_for_submap": usable,
        "hard_boundary": hard,
    }


def test_correlated_frames_sum_to_one_evidence_unit():
    frames = [
        synthetic_frame(0, 0.00), synthetic_frame(1, 0.05),
        synthetic_frame(2, 0.10), synthetic_frame(3, 0.45),
    ]
    assign_correlation_groups(frames, "demo")
    assert [f["correlation_group_id"] for f in frames[:3]] == ["demo:g0001"] * 3
    assert frames[3]["correlation_group_id"] == "demo:g0002"
    assert math.isclose(sum(f["evidence_weight"] for f in frames[:3]), 1.0)
    assert frames[3]["evidence_weight"] == 1.0


def test_submaps_overlap_and_never_cross_quality_boundary():
    frames = [synthetic_frame(i, i * .25) for i in range(41)]
    # A C frame splits two physical runs. It must be in no submap.
    frames[20]["usable_for_submap"] = False
    frames[20]["quality_grade"] = "C"
    frames[20]["hard_boundary"] = True
    assign_correlation_groups(frames, "demo")
    submaps = build_submaps(frames, "demo")
    assign_submap_memberships(frames, submaps)
    assert submaps
    assert all(21 not in submap["node_ids"] for submap in submaps)
    assert all(submap["path_length_m"] <= 5.0 + 1e-12 for submap in submaps)
    assert all(len(set(submap["correlation_group_ids"])) ==
               submap["effective_evidence_groups"] for submap in submaps)
    for previous, current in zip(submaps, submaps[1:]):
        if previous["run_id"] == current["run_id"]:
            assert set(previous["node_ids"]) & set(current["node_ids"])
    assert all(frame["primary_submap_id"] for frame in frames if frame["usable_for_submap"])
    assert all(
        frame["primary_submap_id"] in frame["membership_submap_ids"]
        for frame in frames if frame["usable_for_submap"]
    )
    primary_groups = [group for submap in submaps for group in submap["primary_correlation_group_ids"]]
    assert len(primary_groups) == len(set(primary_groups))


def test_c_frame_gets_zero_weight_and_resets_correlation():
    frames = [
        synthetic_frame(0, 0.0), synthetic_frame(1, 0.05, usable=False, hard=True),
        synthetic_frame(2, 0.06),
    ]
    assign_correlation_groups(frames, "demo")
    assert frames[1]["correlation_group_id"] == ""
    assert frames[1]["evidence_weight"] == 0.0
    assert frames[0]["correlation_group_id"] != frames[2]["correlation_group_id"]


def test_sensor_only_c_frame_is_skipped_without_fragmenting_submap_run():
    frames = [synthetic_frame(i, i * .5) for i in range(9)]
    frames[4]["usable_for_submap"] = False
    frames[4]["quality_grade"] = "C"
    frames[4]["hard_boundary"] = False
    assign_correlation_groups(frames, "demo")
    submaps = build_submaps(frames, "demo")
    assert len({submap["run_id"] for submap in submaps}) == 1
    assert all(5 not in submap["node_ids"] for submap in submaps)
    assert max(submap["path_length_m"] for submap in submaps) >= 3.5
