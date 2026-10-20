import numpy as np
from blend_photographic_floors import multiband_pair,scalar_photo_gains
from rebuild_prime_hall import inside_polygon
from audit_photographic_finish import glyph_components


def test_multiband_keeps_photographic_detail_and_distant_background():
    y,x=np.indices((128,256));fg=np.dstack([.2+.2*(x%8<4),.1+.1*(x%8<4),.3+.1*(x%8<4)]).astype(np.float32);bg=np.dstack([.15+.05*(y%8<4)]*3).astype(np.float32);alpha=np.zeros((128,256),np.float32);alpha[:,:80]=1
    result=multiband_pair(fg,bg,alpha,4)
    assert np.ptp(result[40,10:50,0])>.15
    assert np.array_equal(result[:,160:],bg[:,160:])
    assert np.ptp(result[:,200,0])>.04


def test_multiband_identical_images_are_unchanged():
    rng=np.random.default_rng(7);rgb=rng.uniform(.05,.8,(64,80,3)).astype(np.float32);alpha=np.zeros((64,80),np.float32);alpha[:,:40]=1
    assert np.allclose(multiband_pair(rgb,rgb,alpha),rgb,atol=2e-7)


def test_overlap_correction_is_bounded_scalar_not_uniform_paint():
    y,x=np.indices((30,30));image=np.dstack([.1+.1*(x%5<2),.2+.1*(x%5<2),.3+.1*(x%5<2)]);weights=np.ones((2,30,30));gains,edges=scalar_photo_gains([image,image*1.2],weights,[.75,1.33])
    assert edges and np.all((gains>=.75)&(gains<=1.33))
    assert abs(gains[0]-gains[1]*1.2)<.01
    assert np.ptp((image*gains[0])[:,:,0])>.07


def test_sign_cleanup_is_finite_and_includes_polygon_boundary():
    poly=[[0,0],[2,0],[2,1],[0,1]]
    assert inside_polygon([[1,.5],[0,0],[3,.5],[-1,.5]],poly).tolist()==[True,True,False,False]


def test_dim_original_letter_components_do_not_require_white_paint():
    rgb=np.full((40,180,3),8,np.uint8)
    for x in range(9):rgb[10:30,5+x*19:13+x*19]=85
    _,ids,_=glyph_components(rgb,[[0,0],[179,0],[179,39],[0,39]])
    assert len(ids)==9


def test_multiband_softens_exposure_step_without_mean_replacement():
    fg=np.full((64,192,3),.4,np.float32);bg=np.full_like(fg,.1);alpha=np.zeros((64,192),np.float32);alpha[:,:96]=1
    result=multiband_pair(fg,bg,alpha,5)
    assert abs(np.diff(result[32,:,0])).max()<.03
    assert result[32,5,0]>.35 and result[32,-5,0]<.15
