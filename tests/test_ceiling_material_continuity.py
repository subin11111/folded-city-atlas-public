import numpy as np

from match_ceiling_extension_materials import appearance_streams, harmonic_fill, plaster_detail
from surface_model_utils import write_glb


def test_real_baked_vertex_colour_is_not_replaced_by_grey(tmp_path):
    path=tmp_path/'vertex_colours.glb'
    xyz=np.array([[0,0,0],[1,0,0],[0,1,0]],np.float32)
    linear=np.array([[.02,.04,.01],[.03,.05,.02],[.04,.06,.03]],np.float32)
    write_glb(path,xyz,np.array([[0,1,2]]),np.tile([0,0,1],(3,1)),linear)
    _,colours,images,_,_=appearance_streams(path,np.array([True]))
    assert images.tolist()==[-1]
    assert np.allclose(colours[0],linear)
    assert not np.allclose(colours,.72)


def test_harmonic_colour_fill_preserves_dark_local_colour():
    rgb=np.zeros((40,40,3),np.float32)
    support=np.zeros((40,40),bool)
    support[:,:8]=True
    rgb[support]=[.02,.035,.018]
    result=harmonic_fill(rgb,support,np.ones_like(support),.025,.1)
    assert np.max(abs(result-[.02,.035,.018]))<1e-6


def test_no_fake_microtexture_without_a_valid_original_sample():
    rgb=np.ones((40,40,3),np.float32)*.2
    detail,donor=plaster_detail(rgb,np.ones((40,40),bool),.025)
    assert donor is None and not detail.any()
