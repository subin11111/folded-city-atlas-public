import numpy as np
from smooth_shared_contours import apply_column_field, boundary_loops, contour_field, signed_area


def test_outer_and_hole_rings_are_not_confused():
    mask=np.ones((7,7),bool);mask[2:5,2:5]=False
    rings=boundary_loops(mask)
    assert len(rings)==2
    assert sorted(np.sign(signed_area(r)) for r in rings)==[-1,1]


def test_shared_column_cannot_split_floor_underside_and_fascia():
    v=np.array([[.1,.1,1.48],[.1,.1,1.98],[.1,.1,2.6]],np.float32)
    field=np.zeros((4,4,2),np.float32);field[1,1]=[.025,-.025]
    result=apply_column_field(v,v,np.zeros(2),.1,field)
    assert np.allclose(result[:,:2],[.125,.075])
    assert np.array_equal(result[:,2],v[:,2])


def test_stair_step_is_bounded_and_major_corners_are_retained():
    mask=np.zeros((25,25),bool)
    for y in range(2,23):mask[y,2:10+y//2]=True
    cfg={'contour_simplification_m':.12,'maximum_shared_XY_motion_m':.055}
    field,pinned,_=contour_field([mask],np.zeros(2),.1,cfg)
    assert np.linalg.norm(field,axis=2).max()<=.055001
    assert (np.linalg.norm(field,axis=2)>1e-6).any()
    assert not field[pinned].any()


def test_real_interior_holes_remain_pinned():
    mask=np.ones((9,9),bool);mask[3:6,3:6]=False
    cfg={'contour_simplification_m':.12,'maximum_shared_XY_motion_m':.055}
    field,pinned,_=contour_field([mask],np.zeros(2),.1,cfg)
    for ring in boundary_loops(mask):
        if signed_area(ring)<0:
            x,y=ring.T
            assert pinned[y,x].all() and not field[y,x].any()
