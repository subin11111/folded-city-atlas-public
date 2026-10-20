import pytest

from summarize_geometry_review import patch_summary


def test_two_patches_on_one_pair_are_not_two_regions():
    rows = [{'session': 'floor2', 'a': 1, 'b': 9, 'symmetric_plane_abs_median_m': x}
            for x in (.02, .08)]
    result = patch_summary(rows)
    assert result['patches'] == 2
    assert result['frame_pairs'] == 1
    assert result['median_m'] == pytest.approx(.05)


def test_no_observations_is_not_zero_error():
    assert patch_summary([])['median_m'] is None
    with pytest.raises(ValueError):
        patch_summary([{'session': 'x', 'a': 1, 'b': 2, 'symmetric_plane_abs_median_m': float('nan')}])
