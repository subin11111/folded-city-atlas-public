import csv
import hashlib
import json
import sys
from pathlib import Path

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from render_submap_quality import (  # noqa: E402
    create_output,
    load_quality_inputs,
    parse_bool,
    recorded_diagnostic_thresholds,
    render,
    verify_inventory,
)


FRAME_FIELDS = [
    "node_id",
    "pose_x_m",
    "pose_y_m",
    "pose_z_m",
    "session_path_m",
    "linear_step_m",
    "angular_step_deg",
    "intrinsic_jump_depth_px",
    "quality_score",
    "quality_grade",
    "hard_boundary",
    "correlation_group_id",
    "primary_submap_id",
]


def write_fixture(root: Path) -> tuple[Path, Path]:
    session_dir = root / "audit" / "test_session"
    session_dir.mkdir(parents=True)
    frames = [
        {
            "node_id": 1,
            "pose_x_m": 0.0,
            "pose_y_m": 0.0,
            "pose_z_m": 0.1,
            "session_path_m": 0.0,
            "linear_step_m": "",
            "angular_step_deg": "",
            "intrinsic_jump_depth_px": "",
            "quality_score": 91,
            "quality_grade": "A",
            "hard_boundary": "False",
            "correlation_group_id": "test:g0001",
            "primary_submap_id": "test:s0001",
        },
        {
            "node_id": 2,
            "pose_x_m": 1.0,
            "pose_y_m": 0.5,
            "pose_z_m": 0.2,
            "session_path_m": 1.2,
            "linear_step_m": 1.2,
            "angular_step_deg": 80,
            "intrinsic_jump_depth_px": 3.2,
            "quality_score": 40,
            "quality_grade": "C",
            "hard_boundary": "True",
            "correlation_group_id": "",
            "primary_submap_id": "",
        },
        {
            "node_id": 3,
            "pose_x_m": 1.2,
            "pose_y_m": 0.6,
            "pose_z_m": 0.25,
            "session_path_m": 1.5,
            "linear_step_m": 0.3,
            "angular_step_deg": 9,
            "intrinsic_jump_depth_px": 0.2,
            "quality_score": 68,
            "quality_grade": "B",
            "hard_boundary": "False",
            "correlation_group_id": "test:g0002",
            "primary_submap_id": "test:s0002",
        },
    ]
    frames_path = session_dir / "frames.csv"
    with frames_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FRAME_FIELDS)
        writer.writeheader()
        writer.writerows(frames)
    submaps = {
        "session": "test_session",
        "submaps": [
            {
                "submap_id": "test:s0001",
                "anchor_node_id": 1,
                "within_nominal_2_to_5m": True,
                "primary_evidence_groups": 1,
            },
            {
                "submap_id": "test:s0002",
                "anchor_node_id": 3,
                "within_nominal_2_to_5m": False,
                "primary_evidence_groups": 1,
            },
        ],
    }
    submaps_path = session_dir / "submaps.json"
    submaps_path.write_text(json.dumps(submaps))
    db_path = root / "test.db"
    db_path.write_bytes(b"immutable sensor database fixture")
    db_hash = hashlib.sha256(db_path.read_bytes()).hexdigest()
    manifest = {
        "status": "TEST_QUALITY_AUDIT",
        "sessions": {
            "test_session": {
                "source_database": str(db_path),
                "database_sha256_before": db_hash,
                "frame_count": 3,
                "usable_frame_count": 2,
                "correlation_group_count": 2,
                "submap_count": 2,
                "artifacts": {
                    "frames_csv": str(frames_path),
                    "submaps_json": str(submaps_path),
                },
            }
        },
    }
    manifest_path = root / "audit" / "manifest.json"
    manifest_path.write_text(json.dumps(manifest))
    return manifest_path, db_path


def test_boolean_parser_rejects_ambiguous_values():
    assert parse_bool("True")
    assert not parse_bool("0")
    with pytest.raises(ValueError, match="Unsupported boolean"):
        parse_bool("maybe")


def test_diagnostic_thresholds_are_read_from_audit_manifest():
    thresholds = recorded_diagnostic_thresholds(
        {
            "grade_score": {"A_min": 81, "B_min": 51},
            "intrinsic_jump_depth_pixels": {"A_max": 0.4, "B_max": 1.8},
            "hard_boundary": {
                "intrinsic_jump_depth_pixels": 2.8,
                "linear_pose_increment_m": 1.1,
                "angular_pose_increment_deg": 70,
            },
        }
    )
    assert thresholds["score_a_min"] == 81
    assert thresholds["score_b_min"] == 51
    assert thresholds["intrinsic_a_max"] == 0.4
    assert thresholds["intrinsic_hard"] == 2.8
    assert thresholds["linear_hard"] == 1.1
    assert thresholds["angular_hard"] == 70
    assert thresholds["angular_c_gate"] == 35  # Legacy/default value if absent.


def test_loader_types_exact_rows_and_preserves_declared_anchors(tmp_path):
    manifest_path, _ = write_fixture(tmp_path)
    _, sessions, files = load_quality_inputs(manifest_path)
    data = sessions["test_session"]
    assert [row["node_id"] for row in data["frames"]] == [1, 2, 3]
    assert [row["quality_grade"] for row in data["frames"]] == ["A", "C", "B"]
    assert data["frames"][1]["hard_boundary"]
    assert data["submaps"][1]["anchor_node_id"] == 3
    assert set(files) == {
        "quality_manifest",
        "test_session:frames_csv",
        "test_session:submaps_json",
    }


def test_full_render_has_hashed_inventory_and_never_claims_acceptance(tmp_path):
    manifest_path, db_path = write_fixture(tmp_path)
    output = tmp_path / "figures"
    result = render(manifest_path, output)
    expected = {
        "01_quality_group_submap_summary.png",
        "02_test_session_trajectory_quality_submaps.png",
        "03_test_session_path_diagnostics.png",
    }
    assert set(result["figures"]) == expected
    assert set(result["figure_hashes"]) == expected
    assert all((output / name).read_bytes().startswith(b"\x89PNG") for name in expected)
    assert result["actual_data_only"]
    assert not result["synthetic_or_ai_generated_imagery"]
    assert result["accepted"] is False
    assert result["navigation"] is False
    assert result["source_databases_unchanged"]
    assert hashlib.sha256(db_path.read_bytes()).hexdigest() == result["source_database_hashes_after"]["test_session"]
    trajectory = result["figures"]["02_test_session_trajectory_quality_submaps.png"]
    assert trajectory["hard_boundary_count_plotted"] == 1
    assert trajectory["nominal_anchor_count_plotted"] == 1
    assert trajectory["short_anchor_count_plotted"] == 1
    written = json.loads((output / "manifest.json").read_text())
    assert written["figure_hashes"] == result["figure_hashes"]
    with pytest.raises(FileExistsError):
        render(manifest_path, output)


def test_output_and_figure_inventory_refuse_silent_overwrite_or_omission(tmp_path):
    output = tmp_path / "evidence"
    create_output(output)
    with pytest.raises(FileExistsError):
        create_output(output)
    (output / "unindexed.png").write_bytes(b"not a real plot")
    with pytest.raises(ValueError, match="inventory differs"):
        verify_inventory(output, {})
