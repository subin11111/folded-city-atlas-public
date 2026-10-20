"""Synthetic negative and positive controls for conservative session matching."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from check_session_overlap import (apply_transform, geometry_support, independent_consensus,
                                   rigid_fit, rigid_ransac,Session)


def test_rigid_fit_uses_source_to_target_convention():
    rng=np.random.default_rng(10)
    source=rng.normal(size=(30,3))
    expected=np.eye(4)
    expected[:3,:3]=Rotation.from_euler('xyz',[.2,-.4,.5]).as_matrix()
    expected[:3,3]=[2,-1,.5]
    np.testing.assert_allclose(rigid_fit(source,apply_transform(expected,source)),expected,atol=1e-12)


def test_seeded_ransac_rejects_majority_outliers():
    rng=np.random.default_rng(11)
    source=rng.normal(size=(80,3))
    expected=np.eye(4)
    expected[:3,:3]=Rotation.from_euler('z',.6).as_matrix()
    expected[:3,3]=[2,3,-.4]
    target=apply_transform(expected,source)+rng.normal(scale=.003,size=source.shape)
    target[30:]=rng.uniform(-5,5,size=(50,3))
    recovered,inliers,trials=rigid_ransac(source,target,42,iterations=1000)
    assert inliers.sum()==30
    assert trials>0
    np.testing.assert_allclose(recovered,expected,atol=.003)


def test_single_textured_plane_not_enough():
    rng=np.random.default_rng(12)
    xy=rng.uniform(-1,1,size=(50,2))
    points=np.column_stack((xy,np.ones(50)))
    pixels=(xy+1)*[200,300]+[100,100]
    accepted,metrics=geometry_support(points,pixels,(960,720,3))
    assert not accepted
    assert metrics['std_m'][2]<1e-10


def test_same_place_repeated_frames_not_independent_evidence():
    session=SimpleNamespace(stamps={1:0,2:1,3:2})
    poses={i:np.eye(4) for i in (1,2,3)}
    pairs=[{'accepted':True,'outside_id':i,'lobby_id':i,
            'lobby_node_to_outside_node':np.eye(4).tolist()} for i in (1,2,3)]
    best,_=independent_consensus(pairs,session,session,poses,poses)
    assert len(best)==1


def test_consensus_respects_pose_composition_and_spatial_separation():
    session=SimpleNamespace(stamps={1:0,2:10,3:20})
    poses_a={i:np.eye(4) for i in (1,2,3)}
    poses_b={i:np.eye(4) for i in (1,2,3)}
    expected=np.eye(4)
    expected[:3,:3]=Rotation.from_euler('z',.5).as_matrix()
    expected[:3,3]=[4,2,0]
    pairs=[]
    for i in (1,2,3):
        poses_a[i][:3,3]=[i,0,0]
        poses_b[i][:3,3]=[0,i,0]
        pair=np.linalg.inv(poses_a[i]) @ expected @ poses_b[i]
        pairs.append({'accepted':True,'outside_id':i,'lobby_id':i,'lobby_node_to_outside_node':pair.tolist()})
    best,_=independent_consensus(pairs,session,session,poses_a,poses_b)
    assert len(best)==3
    np.testing.assert_allclose(best[0]['lobby_map_to_outside_map'],expected,atol=1e-12)


def test_unprojection_uses_calibration_scale_and_optical_local_transform():
    session=object.__new__(Session)
    depth=np.full((10,10),2.,dtype=np.float32)
    confidence=np.full((10,10),100,dtype=np.uint8)
    intrinsic=np.array([[40,0,20],[0,40,20],[0,0,1]],dtype=float)
    local=np.eye(4)
    local[:3,:3]=Rotation.from_euler('z',np.pi/2).as_matrix()
    local[:3,3]=[.1,.2,.3]
    session.depth=lambda node:(depth,confidence,40,40,intrinsic,local)
    session.image=lambda node:np.zeros((80,80,3),np.uint8)
    points=session.unproject(1,np.array([[40,40],[48,40]]))
    np.testing.assert_allclose(points,[[.1,.2,2.3],[.1,.4,2.3]],atol=1e-8)
    confidence[:]=0
    assert np.isnan(session.unproject(1,np.array([[40,40]]))).all()


def test_consensus_cannot_chain_inconsistent_transforms():
    session=SimpleNamespace(stamps={1:0,2:10,3:20})
    poses={i:np.eye(4) for i in (1,2,3)}
    pairs=[]
    for i,offset in zip((1,2,3),(0.,.3,-.3)):
        poses[i][0,3]=i
        pair=np.eye(4)
        pair[1,3]=offset
        pairs.append({'accepted':True,'outside_id':i,'lobby_id':i,'lobby_node_to_outside_node':pair.tolist()})
    best,_=independent_consensus(pairs,session,session,poses,poses)
    assert len(best)==2
