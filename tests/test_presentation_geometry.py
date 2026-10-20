import numpy as np
from presentation_geometry import LevelPresentation


def model():
    value=LevelPresentation.__new__(LevelPresentation);value.lower=-1.55;value.upper=2.60
    value.models={}
    for k,z in [('lower',-1.7),('upper',2.8),('sub_lower',-1.3)]:
        value.models[k]={'low':np.array([-10.,-10.]),'spacing':20.,'heights':np.full((2,2),z)}
    return value


def test_level_warp_has_exact_xy_and_shared_floor_heights():
    p=np.array([[0,0,-1.7],[1,2,2.8],[2,3,.5]],np.float32);q=model().warp(p,'floor2')
    assert np.array_equal(q[:,:2],p[:,:2])
    assert np.allclose(q[:2,2],[-1.55,2.60],atol=1e-6)


def test_outside_not_flattened_and_sub_lower_aligns_to_same_floor():
    p=np.array([[0,0,-1.3],[1,2,.3]],np.float32);m=model()
    assert np.array_equal(m.warp(p,'outside'),p)
    assert abs(m.warp(p,'bridge_12_sub')[0,2]+1.55)<1e-6


def test_vertical_order_preserved_through_stair_range():
    z=np.linspace(-2,4,100);p=np.c_[np.zeros(100),np.zeros(100),z].astype(np.float32)
    assert (np.diff(model().warp(p,'floor2')[:,2])>0).all()
