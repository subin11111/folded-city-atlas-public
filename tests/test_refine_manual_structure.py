from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from refine_manual_structure import SETTINGS, fit_bundle, project
from optimize_session import apply, inverse


def test_asymmetric_depth_can_fit_a_pose_without_discarding_rgb_only_observations(monkeypatch):
    monkeypatch.setitem(SETTINGS,'finite_surface_factors',[])
    k=np.array([[500.,0,320],[0,500,240],[0,0,1]])
    points=np.array([[-.6,-.4,3], [.7,-.5,3.2], [.6,.7,3.5],[-.5,.6,3.1], [.1,.1,2.5],[-.2,-.1,4]])
    truth=np.eye(4);truth[:3,:3]=Rotation.from_rotvec([.02,-.04,.01]).as_matrix();truth[:3,3]=[.2,-.1,.15]
    observations=[]
    for side in ['a','b']:
        local=points if side=='a' else apply(points,inverse(truth))
        pixels,z=project(local,np.eye(4),k)
        for n in range(len(points)):
            observations.append(dict(side=side,landmark=n,pixel=pixels[n],intrinsic=k,
                                     center_from_optical=np.eye(4),depth_z=float(z[n]) if side=='a' else None))
    initial=truth.copy();initial[:3,3]+=[.05,-.03,.02]
    fitted,landmarks,report=fit_bundle(observations,initial,points+.01)
    assert report['success'] and report['active_bounds']==0
    assert report['pose_data_rank_after_landmark_elimination']==6
    assert np.allclose(fitted,truth,atol=1e-6)
    assert np.allclose(landmarks,points,atol=1e-6)


def test_project_uses_optical_extrinsics_and_reports_cheirality():
    k=np.eye(3);pose=np.eye(4);pose[:3,3]=[2,0,0]
    pixels,z=project(np.array([[4,2,2],[2,0,-1]]),pose,k)
    assert np.allclose(pixels[0],[1,1])
    assert z[1]<0
