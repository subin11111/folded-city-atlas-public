import sys
import json
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from export_submap_candidate_clouds import (cloud_bounds, crop_structured_cloud,
                                            ensure_fresh_output,
                                            parse_args,
                                            region_bounds_from_poses,
                                            sample_cloud,
                                            validate_upstream)
from optimize_session import sha256_file


DTYPE = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
         ("r", "u1"), ("g", "u1"), ("b", "u1")]


def cloud(rows):
    return np.asarray(rows, dtype=DTYPE)


def test_cloud_sampling_and_bounds_are_deterministic():
    first = cloud([(0, 0, 0, 1, 2, 3), (1, 2, 3, 1, 2, 3),
                   (2, 4, 6, 1, 2, 3), (3, 6, 9, 1, 2, 3)])
    sampled = sample_cloud(first, 2)
    assert sampled["x"].tolist() == [0., 3.]
    bounds = cloud_bounds([first], 0, 100)
    assert bounds == {"x": [0., 3.], "y": [0., 6.], "z": [0., 9.]}


def test_crop_and_pose_region_use_fixed_xyz_box():
    first = cloud([(0, 0, 0, 1, 2, 3), (2, 2, 2, 1, 2, 3),
                   (5, 5, 5, 1, 2, 3)])
    cropped = crop_structured_cloud(first, np.array([-1, -1, -1]), np.array([3, 3, 3]))
    assert cropped["x"].tolist() == [0., 2.]
    poses = [{1: np.array([0., 2., 4.]), 2: np.array([2., 4., 6.])},
             {1: np.array([-1., 3., 5.])}]
    lower, upper = region_bounds_from_poses(poses, [1, 2], (1., 2., 3.))
    assert np.allclose(lower, [-2., 0., 1.])
    assert np.allclose(upper, [3., 6., 9.])


def test_existing_cloud_output_is_never_replaced(tmp_path):
    output = tmp_path / "clouds"
    output.mkdir()
    marker = output / "accepted-map.ply"
    marker.write_bytes(b"keep")
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        ensure_fresh_output(output)
    assert marker.read_bytes() == b"keep"


def test_single_verified_candidate_is_supported():
    args = parse_args(["--expanded-root", "expanded", "--output", "clouds",
                       "--variants", "dcs"])
    assert args.variants == ["dcs"]
    assert not args.rejected


def upstream_fixture(tmp_path):
    quality = tmp_path / "quality.json"
    quality.write_text(json.dumps({"sessions": {
        "lobby": {"database_sha256_after": "original-db-sha256"}}}))
    graph = tmp_path / "graph.json"
    graph.write_text("{}")
    report = tmp_path / "report.json"
    report.write_text("{}")
    return {"inputs": {
        "quality_manifest": str(quality),
        "quality_manifest_sha256": sha256_file(quality),
        "graph_manifest": str(graph),
        "graph_manifest_sha256": sha256_file(graph),
        "backend_reports": [{"path": str(report), "sha256": sha256_file(report)}],
    }}


def test_upstream_change_fails_before_reconstruction(tmp_path):
    expanded = upstream_fixture(tmp_path)
    verified = validate_upstream(expanded, {"lobby": "original-db-sha256"})
    assert len(verified) == 3
    Path(expanded["inputs"]["graph_manifest"]).write_text('{"changed": true}')
    with pytest.raises(ValueError, match="declared hash mismatch"):
        validate_upstream(expanded, {"lobby": "original-db-sha256"})


def test_database_must_match_original_quality_audit(tmp_path):
    expanded = upstream_fixture(tmp_path)
    with pytest.raises(ValueError, match="database differs from quality audit"):
        validate_upstream(expanded, {"lobby": "changed-db-sha256"})
