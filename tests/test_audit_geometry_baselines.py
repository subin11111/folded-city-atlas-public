import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from audit_geometry_baselines import select_baseline


def metrics(rmse=.1, overlap=.5, wall=.12, count=4):
    return {'graph_reserve_geometry':{'count':count,'clipped_rmse_median_m':rmse,'overlap_median':overlap},
            'wall_plane_abs_m':{'count':5,'p90':wall}}


def test_no_reserve_cannot_promote_numerically_attractive_candidate():
    assert select_baseline({'app':metrics(count=0),'independent':metrics(.01,count=0)})[0]=='app'


def test_marginal_gain_is_not_new_baseline():
    assert select_baseline({'app':metrics(),'independent':metrics(.095)})[0]=='app'


def test_clear_gain_with_nondegrading_walls_is_selected():
    assert select_baseline({'app':metrics(),'independent':metrics(.08,.55,.1)})[0]=='independent'


def test_nearest_neighbor_gain_cannot_hide_wall_regression():
    assert select_baseline({'app':metrics(),'independent':metrics(.07,.55,.2)})[0]=='app'


def test_reduced_coverage_is_not_improvement():
    assert select_baseline({'app':metrics(),'independent':metrics(.07,.4,.1)})[0]=='app'
