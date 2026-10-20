import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from complete_verified_upper_walls import inside_polygon,robust_plane,make_range_profile,profile_lift,measured_joint_line,pixels_on_joint
from photo_semantic_surfaces import lift_pixels


def test_empty_photo_region_is_a_boolean_mask_not_a_float_index():
    result=inside_polygon(np.empty((0,2)),[[0,0],[2,0],[2,2],[0,2]])
    assert result.dtype==bool and len(result)==0


def test_wall_and_ceiling_planes_are_not_interchangeable():
    y,x=np.mgrid[-1:1.01:.08,-1:1.01:.08];wall=np.c_[np.full(x.size,3.),x.ravel(),y.ravel()+4.]
    plane,report=robust_plane(wall)
    assert report['accepted'] and abs(plane[1][2])<.01
    plane,report=robust_plane(wall,horizontal=True)
    assert plane is None and not report['accepted']


def test_ceiling_is_measured_at_its_actual_height_not_a_floor_height_prior():
    y,x=np.mgrid[-1:1.01:.08,-1:1.01:.08];points=np.c_[x.ravel(),y.ravel(),np.full(x.size,5.553)]
    plane,report=robust_plane(points,horizontal=True)
    assert report['accepted'] and abs(plane[0][2]-5.553)<1e-8


def test_curved_wall_profile_retains_the_arc_instead_of_one_flat_plane():
    theta=np.linspace(-.4,.4,120);z=np.linspace(1.,3.,8);t,zz=np.meshgrid(theta,z)
    # The observed circular wall is seen from its centre in this test.
    points=np.c_[3*np.cos(t.ravel()),3*np.sin(t.ravel()),zz.ravel()]
    profile=make_range_profile(points,np.zeros(3),1.)
    pose=np.eye(4);pose[:3,:3]=np.array([[0,0,1],[1,0,0],[0,1,0.]])
    f={'world_optical':pose,'k_rgb':np.array([[100.,0,50],[0,100.,50],[0,0,1.]])}
    xyz,_,valid=profile_lift(np.array([[25.,50.],[50.,50.],[75.,50.]]),f,profile)
    assert valid.all();np.testing.assert_allclose(np.linalg.norm(xyz[:,:2],axis=1),3.,atol=.01)
    assert np.ptp(xyz[:,0])>.05


def test_curved_wall_does_not_extrapolate_to_unseen_angular_regions():
    theta=np.linspace(-.1,.1,200);points=np.c_[3*np.cos(theta),3*np.sin(theta),np.ones(200)]
    profile=make_range_profile(points,np.zeros(3),1.);pose=np.eye(4);pose[:3,:3]=np.array([[0,0,1],[1,0,0],[0,1,0.]])
    f={'world_optical':pose,'k_rgb':np.array([[100.,0,50],[0,100.,50],[0,0,1.]])}
    _,_,valid=profile_lift(np.array([[50.,50.],[100.,50.]]),f,profile)
    assert valid.tolist()==[True,False]


def test_curved_wall_does_not_bridge_a_large_internal_unsupported_gap():
    profile={'origin':0.,'angles':np.array([-.3,-.25,.25,.3]),'ranges':np.full(4,3.)}
    pose=np.eye(4);pose[:3,:3]=np.array([[0,0,1],[1,0,0],[0,1,0.]])
    f={'world_optical':pose,'k_rgb':np.array([[100.,0,50],[0,100.,50],[0,0,1.]])}
    _,_,valid=profile_lift(np.array([[50.,50.]]),f,profile)
    assert not valid[0]


def test_measured_joint_is_the_intersection_of_wall_and_actual_ceiling():
    wall=(np.array([0.,0.,3.]),np.array([0.,0.,1.]))
    ceiling=(np.array([0.,1.,3.]),np.array([0.,1.,0.]))
    f={'world_optical':np.eye(4),'k_rgb':np.array([[100.,0,50],[0,100.,50],[0,0,1.]])}
    line=measured_joint_line(wall,ceiling,f);pixels=pixels_on_joint(line,np.array([[25.,0.],[50.,0.],[75.,0.]]))
    xyz,_=lift_pixels(pixels,f['k_rgb'],f['world_optical'],*wall)
    np.testing.assert_allclose(xyz[:,1],1.,atol=1e-8)
