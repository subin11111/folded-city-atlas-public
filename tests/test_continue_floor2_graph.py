import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from continue_floor2_graph import encode_warm_start, strict_convergence, continue_graph
from optimize_robust_submaps import Node, Edge, _decode_corrections, transform_record


def pose(x=0.,rotation=(0.,0.,0.)):
    t=np.eye(4);t[0,3]=x;t[:3,:3]=Rotation.from_rotvec(rotation).as_matrix();return t


def test_roundtrip_left_rotation_and_world_translation_warm_start():
    nodes=[Node('a','s',pose(2,(.2,.1,-.3))),Node('b','s',pose(3,(-.2,.3,.4)))]
    x=np.array([.25,-.3,.4,.15,-.1,.07])
    poses=_decode_corrections(x,nodes,['b'])
    encoded=encode_warm_start(nodes,{'a'},poses,5.,20.)
    np.testing.assert_allclose(encoded,x,atol=1e-14,rtol=0)


def test_warm_start_cannot_shift_gauge_or_recenter_original_bounds():
    nodes=[Node('a','s',pose()),Node('b','s',pose(3))]
    with pytest.raises(ValueError,match='gauge'):
        encode_warm_start(nodes,{'a'},{'a':pose(.01),'b':pose(3)},5.,20.)
    with pytest.raises(ValueError,match='bounds'):
        encode_warm_start(nodes,{'a'},{'a':pose(),'b':pose(8.01)},5.,20.)
    with pytest.raises(ValueError,match='inventories'):
        encode_warm_start(nodes,{'a'},{'a':pose()},5.,20.)


def test_success_flag_required_and_original_strict_stopping_thresholds():
    assert strict_convergence({'success':True},0.,1e-8,2)
    assert not strict_convergence({'success':False},0.,0.,2)
    assert not strict_convergence({'success':True},1e-5,0.,2)
    assert not strict_convergence({'success':True},0.,1e-7,2)
    assert not strict_convergence({'success':True},0.,0.,1)


@pytest.mark.parametrize('scheme',['2-point','3-point'])
def test_continuation_reduces_training_error_without_using_heldout(scheme):
    nodes=[Node('a','s',pose(),True),Node('b','s',pose(1.1)),Node('c','s',pose(2.2))]
    edges=[Edge('ab','a','b',pose(1),np.eye(6)*10,'odometry','train','ab',{}),
           Edge('bc','b','c',pose(1),np.eye(6)*10,'odometry','train','bc',{}),
           Edge('ac_bad','a','c',pose(4),np.eye(6)*1e6,'loop','heldout','bad',{})]
    warm=dict(method='baseline_huber',parameters=dict(huber_delta=3.,dcs_phi=5.991,
        group_switch_prior=10.,translation_correction_bound_m=5.,rotation_correction_bound_deg=20.),
        poses={n.id:transform_record(n.initial) for n in nodes})
    checkpoints=[]
    poses,report=continue_graph(nodes,edges,warm,outer_iterations=5,max_evaluations=100,
        jacobian_scheme=scheme,
        checkpoint=lambda row,p,x:checkpoints.append(row))
    assert report['strict_converged']
    assert report['final_weighted_cost']<1e-15
    assert report['heldout_edge_ids']==['ac_bad']
    assert report['fit_edge_ids']==['ab','bc']
    np.testing.assert_allclose(poses['c'],pose(2),atol=1e-7)
    np.testing.assert_array_equal(poses['a'],nodes[0].initial)
    assert len(checkpoints)>=2
    assert report['edges'][-1]['information_weight'] is None
