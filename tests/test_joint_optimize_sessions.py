import sys
from pathlib import Path
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from joint_optimize_sessions import eligible_cross, session_components, initialize_maps, constraint_edge, records


def link(a='a',b='b',held=False):
    return dict(session_a=a,session_b=b,node_a=1,node_b=1,T_a_b=np.eye(4).tolist(),
        eligible_for_joint=True,local_geometry_verified=True,independent_support_count=3,held_out=held)


def test_reject_missing_support_and_intra():
    r=link()
    assert len(eligible_cross([r]))==1
    assert not eligible_cross([{**r,'independent_support_count':2}])
    assert not eligible_cross([{**r,'local_geometry_verified':False}])
    assert not eligible_cross([link('a','a')])


def test_heldout_cannot_connect_or_initialize():
    assert session_components(['a','b','c'],[link(),link('b','c',True)])==[['a','b'],['c']]
    local={'a':{1:np.eye(4)},'b':{1:np.eye(4)}}
    with pytest.raises(ValueError): initialize_maps(local,[link(held=True)],'a')
    with pytest.raises(ValueError): initialize_maps(local,[{**link(),'initialization_allowed':False}],'a')


def test_initialization_direction_and_gravity_preservation():
    truth=np.eye(4)
    truth[:3,:3]=Rotation.from_euler('z',.6).as_matrix()
    truth[:3,3]=[4,5,2]
    local={'a':{1:np.eye(4)},'b':{1:np.eye(4)}}
    r={**link(),'T_a_b':truth.tolist()}
    initialized=initialize_maps(local,[r],'a')
    np.testing.assert_allclose(initialized['b'],truth,atol=1e-12)
    inverse=initialize_maps(local,[r],'b')
    np.testing.assert_allclose(inverse['a'],np.linalg.inv(truth),atol=1e-12)


def test_covariance_floor_and_invalid_transform():
    index={('a',1):1,('b',1):2}
    r={**link(),'sigma_translation_m':.001,'sigma_rotation_deg':.01}
    edge=constraint_edge(r,index)
    np.testing.assert_allclose(np.diag(edge.information)[:3],[100]*3)
    z=np.eye(4); z[0,0]=2
    with pytest.raises(ValueError): constraint_edge({**r,'T_a_b':z.tolist()},index)


def test_normalize_cross_holdout_without_leakage(tmp_path):
    import json
    path=tmp_path/'edges.json'
    r=link(); del r['held_out']
    path.write_text(json.dumps([{**r,'joint_holdout':True,'initialization_allowed':False}]))
    assert records(path)[0]['held_out'] is True
    path.write_text(json.dumps([{**r,'joint_holdout':True,'held_out':False}]))
    with pytest.raises(ValueError,match='Conflicting'): records(path)
