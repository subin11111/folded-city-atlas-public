import json
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from floor2_vectorized_solver import build_residual, solve_fixed_weights
from optimize_robust_submaps import (Node, Edge, _decode_corrections, pose_error,
                                     _solve_fixed_weights, parse_manifest,
                                     connected_components, select_gauges, _robust_weights)


def pose(xyz, rotation):
    value=np.eye(4)
    value[:3,3]=xyz
    value[:3,:3]=Rotation.from_rotvec(rotation).as_matrix()
    return value


def reference_residual(x,nodes,edges,gauges,weights):
    variable=[node.id for node in nodes if node.id not in gauges]
    poses=_decode_corrections(x,nodes,variable)
    return np.concatenate([np.sqrt(weights[e.id]) * (np.linalg.cholesky(e.information).T @
        pose_error(e.measurement,poses[e.source],poses[e.target])) for e in edges]) if edges else np.zeros(0)


@pytest.mark.parametrize('angle',[0.,1e-13,1e-7,.7,np.pi-1e-8,np.pi,np.pi+1e-8])
def test_residual_random_transforms_weights_covariances_and_so3_limits(angle):
    rng=np.random.default_rng(123)
    nodes=[Node(str(i),'floor2',pose(rng.normal(size=3)*30,rng.normal(size=3))) for i in range(12)]
    gauges={'0','7'}
    edges=[]
    for i in range(25):
        a,b=rng.choice(12,size=2,replace=False)
        axis=rng.normal(size=3);axis/=np.linalg.norm(axis)
        random=rng.normal(size=(6,6));information=(random@random.T+np.eye(6))*10
        measurement=pose(rng.normal(size=3),axis*angle)
        edges.append(Edge(str(i),str(a),str(b),measurement,information,'registration','train','g',{}))
    weights={e.id:float(rng.uniform(.001,1)) for e in edges}
    problem=build_residual(nodes,edges,gauges,weights)
    for x in [np.zeros(60),rng.normal(size=60)*.05]:
        expected=reference_residual(x,nodes,edges,gauges,weights)
        np.testing.assert_allclose(problem(x),expected,rtol=0,atol=1e-10)
        decoded=_decode_corrections(x,nodes,problem.variable_ids)
        np.testing.assert_allclose(problem.poses(x),np.stack([decoded[n.id] for n in nodes]),rtol=0,atol=1e-12)


@pytest.mark.parametrize('angle',[1e-13,np.pi-1e-10,np.pi,np.pi+1e-10])
def test_actual_error_rotation_near_zero_or_pi(angle):
    nodes=[Node('a','s',np.eye(4)),Node('b','s',pose([1.,2.,3.],[angle,0,0]))]
    edge=Edge('ab','a','b',np.eye(4),np.diag([2.,3.,4.,5.,6.,7.]),'loop','train','g',{})
    problem=build_residual(nodes,[edge],{'a'},{'ab':.3})
    np.testing.assert_allclose(problem(np.zeros(6)),reference_residual(np.zeros(6),nodes,[edge],{'a'},{'ab':.3}),
                               atol=1e-10,rtol=0)


def test_jacobian_sparsity_has_both_edge_endpoints_only():
    nodes=[Node(str(i),'s',np.eye(4)) for i in range(4)]
    edges=[Edge('ab','0','2',np.eye(4),np.eye(6),'loop','train','a',{}),
           Edge('bc','1','3',np.eye(4),np.eye(6),'loop','train','b',{})]
    problem=build_residual(nodes,edges,{'0'},{e.id:1. for e in edges})
    expected=np.zeros((12,18),dtype=np.int8)
    expected[:6,6:12]=1
    expected[6:,:6]=1;expected[6:,12:]=1
    np.testing.assert_array_equal(problem.jac_sparsity.toarray(),expected)


def test_small_synthetic_solve_matches_reference_cost_and_poses():
    nodes=[Node(str(i),'s',pose([i+.1*i,0,0],[0,0,.03*i])) for i in range(5)]
    edges=[Edge(str(i),str(i),str(i+1),pose([1,0,0],[0,0,0]),np.eye(6)*100,
                'odometry','train',str(i),{}) for i in range(4)]
    edges.append(Edge('loop','0','4',pose([4.01,.01,0],[0,0,.003]),np.eye(6)*30,'loop','train','loop',{}))
    weights={e.id:1. for e in edges}
    weights['loop']=.8
    args=(nodes,edges,{'0'},weights,np.zeros(24),100,5.,20.)
    xr,rr=_solve_fixed_weights(*args)
    xv,rv=solve_fixed_weights(*args)
    np.testing.assert_allclose(xv,xr,atol=2e-7,rtol=0)
    assert abs(rv['cost']-rr['cost'])<1e-10
    assert rv['success'] and rr['success']


@pytest.mark.parametrize('no_variables',[False,True])
def test_empty_objective_or_all_gauge_matches_reference(no_variables):
    nodes=[Node('a','s',np.eye(4))]
    gauges={'a'} if no_variables else set()
    x=np.zeros(0 if no_variables else 6)
    a=solve_fixed_weights(nodes,[],gauges,{},x,1,5.,20.)
    b=_solve_fixed_weights(nodes,[],gauges,{},x,1,5.,20.)
    np.testing.assert_array_equal(a[0],b[0]);assert a[1]==b[1]


def test_real_1215_node_residual_equivalence_when_experiment_present():
    path=Path(__file__).resolve().parents[1]/'outputs/student_union_20261017_floor2_local_v1/frame_baseline/manifest.json'
    if not path.exists(): pytest.skip('Local experiment outputs intentionally untracked')
    nodes,edges=parse_manifest(json.loads(path.read_text()))
    train=[e for e in edges if e.used_for_fit]
    gauges=set(select_gauges(nodes,connected_components([n.id for n in nodes],train)).values())
    weights,_=_robust_weights('baseline_huber',train,{n.id:n.initial for n in nodes},3.,5.991,10.)
    problem=build_residual(nodes,train,gauges,weights)
    x=np.random.default_rng(927).normal(size=6*len(problem.variable_ids))*.02
    assert len(nodes)==1215
    np.testing.assert_allclose(problem(x),reference_residual(x,nodes,train,gauges,weights),rtol=0,atol=1e-10)
