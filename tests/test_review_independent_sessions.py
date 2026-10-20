from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from review_independent_sessions import LABELS, choose_candidate, normalized_trajectory
from export_rtabmap_db import load_pose_csv
from optimize_session import inverse, trajectory


def fixture():
    stages=['incumbent_preview',*LABELS]
    stats={m:dict(count=10,median_m=.1,p90_m=.2,maximum_m=.3,severe_regression_count=0) for m in stages}
    geom={m:{'adjacent':dict(count=5,clipped_rmse_median_m=.05,overlap_median=.7),
             'heldout':dict(count=2,clipped_rmse_median_m=.08,overlap_median=.6)} for m in stages}
    solver={m:dict(last_inner_success=True,active_bounds=0,strict_outer_converged=True) for m in LABELS}
    return stats,geom,solver


def test_improvement_is_compared_with_incumbent_not_a_worse_initialization():
    stats,geom,solver=fixture()
    stats['selected_initial']=dict(count=10,median_m=1,p90_m=2,maximum_m=3,severe_regression_count=0)
    chosen,decisions=choose_candidate('outside',stats,geom,solver)
    assert chosen=='incumbent_preview'
    assert 'no_material_5mm_distribution_improvement' in decisions['dcs']['reasons']


def test_aggregate_improvement_does_not_hide_a_bad_wall_or_holdout():
    stats,geom,solver=fixture()
    stats['dcs'].update(median_m=.05,p90_m=.1,severe_regression_count=1)
    assert choose_candidate('outside',stats,geom,solver)[0]=='incumbent_preview'
    stats['dcs']['severe_regression_count']=0
    geom['dcs']['heldout']['overlap_median']=.3
    assert choose_candidate('outside',stats,geom,solver)[0]=='incumbent_preview'
    geom['dcs']['heldout']['overlap_median']=.6
    assert choose_candidate('outside',stats,geom,solver)[0]=='dcs'


def test_bridge_with_no_validation_cannot_earn_promotion_from_solver_success():
    stats,geom,solver=fixture()
    for value in stats.values():value.update(count=0,median_m=None,p90_m=None,maximum_m=None)
    assert choose_candidate('bridge_out1f',stats,geom,solver)[0]=='incumbent_preview'


def test_display_gauge_cannot_change_internal_relative_poses(tmp_path):
    first=np.eye(4);second=np.eye(4);second[:3,3]=[1,2,3]
    gauge=np.eye(4);gauge[:3,3]=[20,-3,5]
    source=tmp_path/'source.csv';reference=tmp_path/'reference.csv';out=tmp_path/'normalized.csv'
    trajectory(source,{1:first,2:second},{1:0,2:1})
    trajectory(reference,{1:gauge@first,2:gauge@second},{1:0,2:1})
    normalized_trajectory(source,reference,out,[1,2])
    result=load_pose_csv(out)
    assert np.allclose(result[1],gauge)
    assert np.allclose(inverse(result[1])@result[2],inverse(first)@second)
