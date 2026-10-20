import copy
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from optimize_robust_submaps import main, optimize_manifest


def pose(x, y=0., z=0., yaw=0.):
    quaternion = Rotation.from_euler("z", yaw).as_quat()
    return [x, y, z, *quaternion]


def edge(edge_id, source, target, dx, *, edge_type="odometry", role="train",
         group=None, information=None):
    return {
        "id": edge_id,
        "from": source,
        "to": target,
        "measurement": pose(dx),
        "information": (np.eye(6) * 100. if information is None else information).tolist(),
        "type": edge_type,
        "role": role,
        "region_group": group or edge_id,
    }


def chain_manifest(include_bad=True, include_heldout=False):
    edges = [
        edge("odo01", "n0", "n1", 1.),
        edge("odo12", "n1", "n2", 1.),
        edge("odo23", "n2", "n3", 1.),
        edge("good_loop", "n0", "n3", 3., edge_type="loop", group="far_wall"),
    ]
    if include_bad:
        edges.append(edge("bad_loop", "n0", "n3", 13., edge_type="loop", group="glass_door"))
    if include_heldout:
        edges.append(edge("audit_impossible", "n0", "n3", -50., edge_type="loop",
                          role="heldout", group="audit_only",
                          information=np.eye(6) * 1e7))
    return {
        "schema_version": 1,
        "submaps": [{"id": f"n{i}", "session_id": "s", "pose": pose(float(i)),
                     "gauge": i == 0} for i in range(4)],
        "edges": edges,
    }


def rows(report, method):
    return {row["id"]: row for row in report["variants"][method]["edges"]}


def test_dcs_downweights_bad_loop_but_never_switches_odometry():
    report = optimize_manifest(chain_manifest(), methods=["dcs"],
                               include_leave_one_group=False,
                               outer_iterations=8, max_evaluations=80)
    result = rows(report, "dcs")
    assert result["bad_loop"]["information_weight"] < 1e-4
    assert result["good_loop"]["information_weight"] > .99
    assert all(result[name]["information_weight"] == 1.
               for name in ("odo01", "odo12", "odo23"))
    assert all(not result[name]["switching_allowed"]
               for name in ("odo01", "odo12", "odo23"))
    assert report["variants"]["dcs"]["heldout_edge_count_in_objective"] == 0
    assert not report["accepted"] and not report["navigation"]


def test_heldout_edge_is_never_fit_and_cannot_change_solution():
    base = optimize_manifest(chain_manifest(include_bad=False), methods=["dcs"],
                             include_leave_one_group=False,
                             outer_iterations=5, max_evaluations=60)
    audited = optimize_manifest(chain_manifest(include_bad=False, include_heldout=True),
                                methods=["dcs"], include_leave_one_group=False,
                                outer_iterations=5, max_evaluations=60)
    assert audited["variants"]["dcs"]["poses"] == base["variants"]["dcs"]["poses"]
    audit = rows(audited, "dcs")["audit_impossible"]
    assert not audit["used_for_fit"]
    assert audit["information_weight"] is None
    assert audit["translation_error_m"] > 50.
    assert audited["variants"]["dcs"]["heldout_edge_count_in_objective"] == 0


def test_disconnected_components_have_independent_unchanged_gauges():
    manifest = {
        "schema_version": 1,
        "submaps": [
            {"id": "a0", "session_id": "a", "pose": pose(0.), "gauge": True},
            {"id": "a1", "session_id": "a", "pose": pose(1.2)},
            {"id": "b0", "session_id": "b", "pose": pose(100.), "gauge": True},
            {"id": "b1", "session_id": "b", "pose": pose(101.2)},
            {"id": "isolated", "session_id": "c", "pose": pose(-20.)},
        ],
        "edges": [
            edge("a_odo", "a0", "a1", 1.),
            edge("b_odo", "b0", "b1", 1.),
            # Even a cross-component audit must not join components.
            edge("audit_cross", "a1", "b1", 999., edge_type="cross_session",
                 role="heldout", group="audit_cross"),
        ],
    }
    report = optimize_manifest(manifest, methods=["baseline_huber"],
                               include_leave_one_group=False,
                               outer_iterations=4, max_evaluations=60)
    variant = report["variants"]["baseline_huber"]
    assert variant["component_count"] == 3
    assert variant["fit_edge_ids_complete"]
    assert variant["fit_edge_ids"] == ["a_odo", "b_odo"]
    assert variant["heldout_edge_ids"] == ["audit_cross"]
    assert [row["gauge_id"] for row in variant["components"]] == ["a0", "b0", "isolated"]
    assert all(row["gauge_pose_unchanged"] for row in variant["components"])
    assert variant["poses"]["a0"] == pose(0.)
    assert variant["poses"]["b0"] == pose(100.)
    assert variant["poses"]["isolated"] == pose(-20.)


def test_group_switch_is_shared_and_leave_one_group_and_cycles_are_explicit():
    manifest = chain_manifest()
    # A correlated second observation from the same physical glass doorway
    # must share one switch rather than count as a second independent region.
    manifest["edges"].append(
        edge("bad_loop_duplicate", "n0", "n3", 12.5,
             edge_type="cross_session", group="glass_door"))
    report = optimize_manifest(manifest, methods=["group_switch"],
                               outer_iterations=8, max_evaluations=80)
    variant = report["variants"]["group_switch"]
    result = rows(report, "group_switch")
    assert result["bad_loop"]["information_weight"] == result["bad_loop_duplicate"]["information_weight"]
    assert result["bad_loop"]["information_weight"] < 1e-4
    assert variant["groups"]["glass_door"]["aggregate"].startswith("median")
    assert {row["edge_id"] for row in variant["cycle_diagnostics"]} == {
        "good_loop", "bad_loop", "bad_loop_duplicate"}
    assert all(row["odometry_path_available"] for row in variant["cycle_diagnostics"])
    leave_out = {row["region_group"]: row for row in variant["leave_one_group_diagnostics"]}
    assert set(leave_out) == {"far_wall", "glass_door"}
    assert leave_out["glass_door"]["removed_edge_count"] == 2
    assert not leave_out["glass_door"]["disconnects_graph"]


def test_information_clipping_provenance_and_refuse_overwrite(tmp_path):
    manifest = chain_manifest(include_bad=False)
    anisotropic = np.diag([1e-12, 2., 3., 4., 5., 1e12])
    manifest["edges"][0]["information"] = anisotropic.tolist()
    manifest_path = tmp_path / "manifest.json"
    manifest_path.write_text(json.dumps(manifest))
    output = tmp_path / "result"
    arguments = ["--manifest", str(manifest_path), "--output", str(output),
                 "--methods", "dcs", "--skip-leave-one-group",
                 "--outer-iterations", "3", "--max-evaluations", "50"]
    main(arguments)
    saved = json.loads((output / "report.json").read_text())
    diagnostic = rows(saved, "dcs")["odo01"]["information_diagnostic"]
    assert diagnostic["was_clipped"]
    assert diagnostic["used_eigenvalues"][0] == pytest.approx(1e-6)
    assert diagnostic["used_eigenvalues"][-1] == pytest.approx(1e8)
    assert len(saved["provenance"]["manifest_sha256"]) == 64
    assert len(saved["provenance"]["source_sha256"]) == 64
    assert (output / "source_snapshot.py").is_file()
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        main(arguments)


@pytest.mark.parametrize("method", ["baseline_huber", "dcs", "group_switch"])
def test_repeated_region_measurements_have_single_measurement_influence(method):
    manifest = {
        "schema_version": 1,
        "submaps": [
            {"id": "n0", "session_id": "s", "pose": pose(0.), "gauge": True},
            {"id": "n1", "session_id": "s", "pose": pose(1.)},
        ],
        "edges": [
            # Sharing a label must not let normalization weaken odometry.
            edge("odo", "n0", "n1", 1., group="wall"),
            edge("wall0", "n0", "n1", 1.1, edge_type="loop", group="wall"),
        ],
    }
    repeated = copy.deepcopy(manifest)
    for index in range(1, 5):
        duplicate = copy.deepcopy(manifest["edges"][1])
        duplicate["id"] = f"wall{index}"
        repeated["edges"].append(duplicate)
    repeated["edges"].append(edge(
        "heldout_same_region", "n0", "n1", -100., edge_type="loop",
        role="heldout", group="wall", information=np.eye(6) * 1e8))
    options = {"methods": [method], "include_leave_one_group": False,
               "outer_iterations": 12, "max_evaluations": 80}
    single = optimize_manifest(manifest, **options)
    duplicate = optimize_manifest(repeated, **options)
    single_variant, duplicate_variant = single["variants"][method], duplicate["variants"][method]
    for node_id in ("n0", "n1"):
        np.testing.assert_allclose(duplicate_variant["poses"][node_id],
                                   single_variant["poses"][node_id], atol=1e-7, rtol=0.)
    # Verify a real conflicting measurement influenced the pose, rather than
    # comparing two cases where every non-odometry constraint was rejected.
    assert 1.03 < single_variant["poses"]["n1"][0] < 1.06
    single_rows, duplicate_rows = rows(single, method), rows(duplicate, method)
    assert sum(duplicate_rows[f"wall{i}"]["information_weight"] for i in range(5)) == pytest.approx(
        single_rows["wall0"]["information_weight"], abs=1e-7)
    assert duplicate_rows["odo"]["information_weight"] == 1.
    assert duplicate_rows["heldout_same_region"]["information_weight"] is None
    assert duplicate_variant["groups"]["wall"]["correlation_member_count"] == 5
    assert duplicate_variant["groups"]["wall"]["correlation_information_normalization"] == .2
