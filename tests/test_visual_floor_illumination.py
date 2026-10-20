import numpy as np
from visually_coherent_floor_textures import local_floor_plane,illumination_fields


def test_measured_local_floor_plane_rejects_sparse_points_and_outliers():
    assert local_floor_plane(np.zeros((40,3))) is None
    rng=np.random.default_rng(4);xy=rng.uniform(-2,2,(600,2));z=.02*xy[:,0]+rng.normal(0,.008,600);points=np.c_[xy,z];points[:100,2]+=.4
    plane=local_floor_plane(points)
    assert plane is not None and abs(plane[1][2])>.99
    assert np.median(abs((points[100:]-plane[0])@plane[1]))<.02


def test_same_position_illumination_consensus_preserves_spatial_materials():
    y,x=np.indices((60,80));material=np.where(x<40,.1,.3);grain=1+.12*(x%4<2);base=material*grain;first=np.dstack([base*.8,base,base*.7]);second=first*np.linspace(.5,1.5,80)[None,:,None];weights=np.ones((2,60,80),np.float32)
    cfg={'illumination_radius_m':.4,'spatial_gain_bounds':[.35,2.8]}
    fields,_,target=illumination_fields([first,second],weights,np.ones(2),.05,cfg)
    fixed=first*fields[0,:,:,None]
    assert fixed[:,55:70,1].mean()>fixed[:,10:25,1].mean()*2
    assert np.ptp(fixed[30,10:25,1])>.005
    assert np.ptp(target[30])>.05
