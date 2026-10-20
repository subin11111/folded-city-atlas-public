import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from flatten_horizontal_surfaces import strong_mask,build_sheet,ray_evidence,weighted_level,protected_points,stair_guard,spaced_nodes,vertical_structure_faces,view_samples


def test_large_bounded_floor_hole_is_filled_but_a_measured_void_is_not():
    observed=np.zeros((100,100),bool);observed[15:85,15:85]=True;observed[30:65,30:65]=False
    forbidden=np.zeros_like(observed);forbidden[40:50,40:50]=True
    mask=strong_mask(observed,forbidden,maximum_distance_m=4.)
    assert mask[35,35] and not mask[45,45] and not mask[0,0]
    assert not (mask&forbidden).any()


def test_floor_and_ceiling_sheets_are_exactly_flat_at_distinct_heights():
    import open3d as o3d
    mask=np.ones((5,7),bool)
    for kind,height,direction in [('floor',-1.6,1),('ceiling',1.7,-1),('floor',2.5,1)]:
        mesh,uv=build_sheet(o3d,mask,np.array([2.,3.]),height,.1,kind)
        assert np.ptp(np.asarray(mesh.vertices)[:,2])==0
        assert np.all(np.asarray(mesh.vertices)[:,2]==height)
        mesh.compute_triangle_normals();assert np.all(np.asarray(mesh.triangle_normals)[:,2]*direction>.999)
        assert np.all((uv>=0)&(uv<=1))


def test_ray_crossing_an_open_lower_floor_is_not_plane_support():
    k=np.array([[100.,0,2],[0,100.,2],[0,0,1.]])
    pose=np.eye(4);pose[:3,:3]=np.diag([1.,-1.,-1.]);pose[2,3]=2.
    frame={'depth':np.full((5,5),4.,np.float32),'k_depth':k,'world_optical':pose}
    support,free=ray_evidence({1:frame,2:frame},1.,np.array([-1.,-1.]),(20,20),.1,.65)
    assert free.max()==2 and support.max()==0


def test_nearby_depth_drift_does_not_become_an_open_atrium():
    k=np.array([[100.,0,2],[0,100.,2],[0,0,1.]])
    pose=np.eye(4);pose[:3,:3]=np.diag([1.,-1.,-1.]);pose[2,3]=2.
    frame={'depth':np.full((5,5),1.4,np.float32),'k_depth':k,'world_optical':pose}
    _,free=ray_evidence({1:frame,2:frame},1.,np.array([-1.,-1.]),(20,20),.1,.65)
    assert not free.any()


def test_stair_guard_is_spatial_not_a_global_height_exclusion():
    points=np.array([[0.,0.,0.],[0.,0.,3.],[4.,0.,0.]])
    assert protected_points(points,np.array([[0.,0.,0.]]),np.array([1.,1.,1.])).tolist()==[True,False,False]


def test_derived_stair_guard_does_not_classify_a_level_walk_as_stairs():
    poses={i:np.eye(4) for i in range(20)}
    for i,p in poses.items():p[:3,3]=[i*.25,0,1.5]
    centres,_,_=stair_guard(poses,{},derive=True);assert len(centres)==0
    for i,p in poses.items():p[2,3]=1.5+i*.15
    centres,_,_=stair_guard(poses,{},derive=True);assert len(centres)>10


def test_height_prior_does_not_average_different_floor_levels():
    z=np.r_[np.full(20,-1.6),np.full(20,2.5)];area=np.ones(40)
    assert weighted_level(z,area,-1.6)==-1.6 and weighted_level(z,area,2.5)==2.5


def test_spaced_views_do_not_count_same_place_as_independent_support():
    poses={i:np.eye(4) for i in range(30)}
    assert len(spaced_nodes(poses))==1


def test_vertical_wall_is_protected_but_short_steep_floor_shard_is_not():
    import open3d as o3d
    mesh=o3d.geometry.TriangleMesh();mesh.vertices=o3d.utility.Vector3dVector(np.array([[0,0,0],[1,0,0],[0,0,2],[1,0,2],[2,2,0],[2.05,2,.04],[2.1,2,0]],float))
    mesh.triangles=o3d.utility.Vector3iVector([[0,1,2],[1,3,2],[4,5,6]]);mesh.compute_triangle_normals()
    centres=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].mean(1)
    keep=vertical_structure_faces(o3d,mesh,centres,np.asarray(mesh.triangle_normals),{})
    assert keep.tolist()==[True,True,False]


def test_photo_sampler_handles_more_than_opencv_remap_row_limit():
    pose=np.diag([1.,-1.,-1.,1.]);pose[2,3]=2.
    frame={'rgb':np.full((60,80,3),128,np.uint8),'depth':np.full((60,80),2.,np.float32),
        'k_rgb':np.array([[100.,0,40],[0,100.,30],[0,0,1.]]),'world_optical':pose}
    rgb,score=view_samples(frame,np.zeros((40000,3)),0.,'floor')
    assert rgb.shape==(40000,3) and np.all(score>0)
    np.testing.assert_allclose(rgb,128/255.)
