import sys
from pathlib import Path

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from prepare_roadview_pack import fit_se2,keyframes,enu_to_lonlat
from osm_local_overlay import slippy_pixel,pixel_lonlat,placement_pixel_offset
from repair_visual_appearance import linear_to_srgb,solve_log_gains,photometric_error
from refine_surface_display import srgb_to_linear
from audit_roadview_coverage import directional_union


def test_fixed_scale_gnss_fit():
    rng=np.random.default_rng(9);x=rng.normal(size=(100,2))*10
    r=np.array([[0.,-1.],[1.,0.]]);t=np.array([3.,7.]);y=x@r.T+t
    rr,tt,residual=fit_se2(x,y,np.ones(100)*14)
    np.testing.assert_allclose(rr,r,atol=1e-12);np.testing.assert_allclose(tt,t,atol=1e-12)
    assert residual.max()<1e-10 and np.linalg.det(rr)>0


def test_keyframes_preserve_endpoints_and_selected_subset():
    poses={i:np.eye(4) for i in range(5)}
    for i in poses:poses[i][0,3]=i*.4
    nodes=keyframes(poses,{i:i*.1 for i in poses})
    assert nodes==[0,3,4]


def test_slippy_roundtrip():
    lat,lon=37.542,127.078
    x,y=slippy_pixel(lat,lon,19);ll=pixel_lonlat(x,y,19)
    np.testing.assert_allclose(ll,[lon,lat],atol=1e-10)
    enu_lon,enu_lat=enu_to_lonlat(0,0,(lat,lon))
    assert enu_lon==lon and enu_lat==lat


def test_manual_east_south_offset_is_lower_right_and_reversible():
    shift=placement_pixel_offset(2.,-2.,37.542,19)
    assert shift[0]>0 and shift[1]>0
    np.testing.assert_allclose(shift+placement_pixel_offset(-2.,2.,37.542,19),[0,0])
    np.testing.assert_array_equal(placement_pixel_offset(0,0,37.542,19),[0,0])


def test_linear_colour_roundtrip():
    values=np.linspace(0,1,100)
    np.testing.assert_allclose(linear_to_srgb(srgb_to_linear(values)),values,atol=1e-7)


def test_gains_reduce_reserved_world_sample_error():
    rng=np.random.default_rng(23);base=rng.uniform(.06,.3,(200,3))
    observations=np.stack([base*.8,base,base*1.2]);training=np.arange(200)%5!=0
    gains,report=solve_log_gains(observations,training)
    before=photometric_error(observations,np.ones_like(gains),~training)
    after=photometric_error(observations,gains,~training)
    assert report['edges']==3 and after['median_absolute_log_rgb']<before['median_absolute_log_rgb']*.01
    assert gains.min()>=.65 and gains.max()<=1.55


def test_no_correspondences_do_not_identify_gains():
    gains,report=solve_log_gains(np.full((3,50,3),np.nan),np.ones(50,bool))
    np.testing.assert_array_equal(gains,np.ones((3,3)))
    assert not report['gains_identifiable']


def test_one_perspective_photo_does_not_mean_360_coverage():
    node={'heading_in_map_deg':0.,'horizontal_fov_deg':90.}
    assert directional_union([node])==.25


def test_osm_current_viewport_cache_avoids_repeat_requests(tmp_path,monkeypatch):
    import io
    from PIL import Image
    import osm_local_overlay as osm
    blob=io.BytesIO();Image.new('RGB',(256,256),'white').save(blob,format='PNG');requests=[]
    class Response:
        headers={'Cache-Control':'max-age=60','ETag':'example'}
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def read(self):return blob.getvalue()
    def get(request,timeout):
        requests.append(request);return Response()
    monkeypatch.setattr(osm,'urlopen',get)
    first,meta=osm.current_viewport(37.542,127.078,tmp_path)
    second,_=osm.current_viewport(37.542,127.078,tmp_path)
    assert len(requests)==9 and first.size==second.size==(768,768)
    assert len(meta['tiles'])==9 and meta['attribution']=='© OpenStreetMap contributors'
    assert 'FoldedCityAtlas' in requests[0].get_header('User-agent')
