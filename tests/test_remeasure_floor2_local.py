import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from remeasure_floor2_local import (SETTINGS, acceptance_reasons, choose_pairs,
                                    holdout_exclusions, partition_masks)


def test_depth_partition_has_three_pixel_guard():
    train, test = partition_masks((192, 256))
    assert train.any() and test.any()
    assert not np.any(train & binary_dilation(test, structure=np.ones((7, 7))))
    assert np.array_equal(partition_masks((192, 256))[0], train)


def test_whole_pair_holdout_endpoints_excluded_inclusive():
    pairs, excluded = holdout_exclusions([{"a": 280, "b": 350, "held_out": True},
                                         {"a": 400, "b": 500, "held_out": False}])
    assert pairs == [(280, 350)]
    assert 270 in excluded and 290 in excluded and 340 in excluded and 360 in excluded
    assert 269 not in excluded and 400 not in excluded


def test_pair_selection_does_not_cross_quality_run_or_bad_intermediate():
    frames = {n: {"quality_grade": "A", "stamp": str(n)} for n in range(250, 265)}
    frames[255]["quality_grade"] = "C"
    poses = {n: np.eye(4) for n in frames}
    submaps = [{"run_id": "a", "node_ids": list(range(250, 259))},
               {"run_id": "b", "node_ids": list(range(259, 265))}]
    chosen, _ = choose_pairs(frames, submaps, poses, {262}, 100)
    assert chosen
    for row in chosen:
        assert 255 not in range(row["a"], row["b"]+1)
        assert 262 not in range(row["a"], row["b"]+1)
        assert not row["a"] < 259 <= row["b"]


def test_acceptance_requires_data_rank_and_strict_change_bounds():
    before = {"clipped_rmse_m": .1}
    after = {"overlap": .8, "rmse_m": .04, "clipped_rmse_m": .08}
    fit = {"rank": 6, "minimum_eigenvalue_ratio": .01}
    delta = {"translation_m": .05, "rotation_deg": .5}
    cycle = {"translation_m": .01, "rotation_deg": .2}
    assert acceptance_reasons(before, after, fit, delta, cycle) == []
    assert "point_plane_rank_below_6" in acceptance_reasons(before, after, dict(fit, rank=5), delta, cycle)
    assert "translation_correction_above_15cm" in acceptance_reasons(before, after, fit, dict(delta, translation_m=.16), cycle)


def test_no_test_data_parameter_in_acceptance_function():
    import inspect
    assert list(inspect.signature(acceptance_reasons).parameters) == ["before", "after", "fit", "change", "cycle"]


def test_covariance_is_explicit_model_not_hessian():
    from revalidate_loop_constraints import covariance_model
    model = covariance_model(SETTINGS["model_sigma_translation_m"], SETTINGS["model_sigma_rotation_deg"])
    assert np.allclose(np.diag(model["covariance_6x6"])[:3], .01)
    assert "not fitted" in model["source"]
