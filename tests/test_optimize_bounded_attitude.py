import sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from optimize_bounded_attitude import optimize_attitude
from optimize_session import Edge, inverse, relative_error


def test_attitude_correction_keeps_anchor_and_real_height():
    truth={i:np.eye(4) for i in (1,2,3,9)}
    for i in truth: truth[i][:3,3]=[i,0,i*.4]
    raw={i:p.copy() for i,p in truth.items()}
    raw[3][:3,:3]=Rotation.from_euler('x',2,degrees=True).as_matrix()
    info=np.eye(6)*1e5
    edges=[Edge(a,b,0,inverse(truth[a])@truth[b],info,info) for a,b in ((1,2),(2,3))]
    poses,report=optimize_attitude(raw,edges,[],[[1,2,3],[9]],gravity_sigma_degrees=5,max_evaluations=80)
    assert report['success']
    np.testing.assert_allclose(poses[1],raw[1],atol=1e-12)
    np.testing.assert_allclose(poses[9],raw[9],atol=1e-12)
    assert abs(poses[3][2,3]-truth[3][2,3])<.003
    assert np.linalg.norm(relative_error(edges[-1].measurement,poses[2],poses[3])[3:]) < .001
    assert report['maximum_tilt_change_degrees'] < 5


def test_no_rotation_or_height_change_for_consistent_graph():
    a=np.eye(4);b=np.eye(4);b[:3,3]=[2,1,4]
    b[:3,:3]=Rotation.from_euler('xyz',[.1,-.2,.5]).as_matrix()
    edge=Edge(1,2,0,inverse(a)@b,np.eye(6),np.eye(6))
    result,report=optimize_attitude({1:a,2:b},[edge],[],[[1,2]],max_evaluations=10)
    np.testing.assert_allclose(result[2],b,atol=1e-10)
    assert report['success']
