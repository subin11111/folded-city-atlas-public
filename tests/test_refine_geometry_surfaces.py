import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from refine_geometry_surfaces import observability, reduce_voxels, refine
from optimize_session import apply, inverse


def test_plane_does_not_fake_six_dof_observability():
    grid=np.mgrid[-2:2:20j,-2:2:20j].reshape(2,-1).T
    points=np.column_stack([grid,np.zeros(len(grid))])
    normal=np.tile([0.,0.,1.],(len(points),1))
    result=observability(points,normal)
    assert result['rank']==3
    assert result['minimum_eigenvalue_ratio']<1e-10


def test_corner_registration_recovers_known_transform():
    grid=np.mgrid[-1:1:23j,-1:1:23j].reshape(2,-1).T
    target=reduce_voxels(np.vstack([np.column_stack([grid,np.ones(len(grid))]),
                           np.column_stack([np.ones(len(grid)),grid]),
                           np.column_stack([grid[:,0],np.ones(len(grid)),grid[:,1]])]))
    truth=np.eye(4)
    truth[:3,:3]=Rotation.from_euler('xyz',[1.,-1.,2.],degrees=True).as_matrix()
    truth[:3,3]=[.05,-.04,.03]
    source=apply(target,inverse(truth))
    candidate,info=refine(target,source,np.eye(4))
    error=candidate@inverse(truth)
    assert info['rank']==6
    assert np.linalg.norm(error[:3,3])<.008
    assert Rotation.from_matrix(error[:3,:3]).magnitude()<np.radians(.3)
