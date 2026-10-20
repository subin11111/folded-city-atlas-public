import numpy as np
from compose_outside_bridge_preview import map_transform, yaw_transform, fit_gravity_preserving


def test_map_transform_column_point_convention():
    a=yaw_transform([1,2,3,.5]); b=yaw_transform([-2,1,0,-.3]); measured=yaw_transform([.1,.2,-.4,.1])
    p=np.array([3.,4.,1.,1.])
    assert np.allclose(map_transform(a,measured,b)@b@p,a@measured@p)


def test_gravity_fit_recovers_translation_and_yaw_without_flattening():
    rng=np.random.default_rng(3)
    source=rng.normal(size=(40,3));truth=yaw_transform([4,-2,1.3,.4])
    target=source@truth[:3,:3].T+truth[:3,3]
    fitted,report=fit_gravity_preserving(source,target,np.repeat([0,1],20),yaw_transform([3,-1,0,.3]))
    assert report['success']
    assert np.allclose(fitted,truth,atol=1e-7)
    assert np.allclose((source@fitted[:3,:3].T+fitted[:3,3])[:,2]-source[:,2],1.3)
