import numpy as np
from regularize_indoor_horizontals import bounded_holes,photo_projection,roof_mask
from extract_observed_roof_regions import bounded_plane_polygon,roof_depth_mask


def test_bounded_holes_fill_internal_tears_without_outer_growth():
    old=np.zeros((12,12),bool);old[2:10,2:10]=True;old[5:7,5:7]=False
    after=bounded_holes(old,np.ones_like(old),.1,.1,.3)
    assert after[5:7,5:7].all()
    assert not after[:2].any() and not after[:,:2].any()
    allowed=np.ones_like(old);allowed[5,5]=False
    assert np.array_equal(bounded_holes(old,allowed,.1,.1,.3),old)


def test_large_or_distant_unobserved_hole_stays_open():
    old=np.ones((20,20),bool);old[4:16,4:16]=False
    assert np.array_equal(bounded_holes(old,np.ones_like(old),.1,.5,1.),old)
    assert np.array_equal(bounded_holes(old,np.ones_like(old),.1,3.,.2),old)


def test_roof_mask_comes_from_photo_boundaries_not_floor_footprint():
    record={'boundaries_xy':[[[0,0],[1,0],[1,1],[0,1]]]}
    cfg={'maximum_ceiling_hole_area_m2':1.,'maximum_ceiling_hole_distance_m':.2,'minimum_ceiling_component_area_m2':.05}
    mask,observed,support,low=roof_mask([record],.1,cfg)
    assert not (mask&~observed).any()
    assert np.all(support[mask]==1)
    assert mask.sum()*.1**2<1.3


def test_roof_projection_uses_its_measured_plane_and_photo_polygon():
    record={'source_plane_centre':[0,0,5.],'source_plane_normal':[0,0,1.],
            'world_optical':np.eye(4).tolist(),'K':[[100.,0,360.],[0,100.,480.],[0,0,1.]],
            'polygons_rgb':[[[340,460],[380,460],[380,500],[340,500]]]}
    px,score=photo_projection(record,np.array([[0.,0.],[10.,0.]]))
    assert np.allclose(px[0],[360,480]) and score[0]>0 and score[1]==0


def test_depth_normals_accept_horizontal_roof_but_not_invalid_depth():
    depth=np.full((10,10),5.,np.float32);frame={'depth':depth,'valid':np.ones_like(depth,bool),'k_depth':np.array([[100,0,4.5],[0,100,4.5],[0,0,1.]]),'world_optical':np.eye(4)}
    world,mask=roof_depth_mask(frame,[4.65,6.5]);assert mask.sum()==64
    frame['valid'][:]=False;assert not roof_depth_mask(frame,[4.65,6.5])[1].any()


def test_finite_plane_polygon_never_extends_past_far_plane():
    frame={'k_rgb':np.array([[100.,0,360],[0,100.,480],[0,0,1]]),'world_optical':np.eye(4)}
    polygon=[[0,0],[719,0],[719,959],[0,959]]
    clipped=bounded_plane_polygon(polygon,frame,np.array([0.,0.,5.]),np.array([0.,0.,1.]),far=4.)
    assert len(clipped)==0
