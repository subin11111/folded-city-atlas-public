import sys
from pathlib import Path
import numpy as np
from PIL import Image

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from photo_semantic_surfaces import lift_pixels,fitted_plane,photo_mesh,existing_horizontal_plane,existing_display_wall_plane
from texture_boundary_leveling import level_boundaries,replace_glb_images
from surface_model_utils import write_glb,project
from audit_structural_delivery import glb_triangle_positions


def test_plane_lifts_rgb_corners_without_corner_depth():
    k=np.array([[100.,0,40],[0,100,30],[0,0,1.]])
    pixels=np.array([[10.,10.],[70,10],[70,50],[10,50]])
    xyz,t=lift_pixels(pixels,k,np.eye(4),np.array([0,0,2.]),np.array([0,0,1.]))
    np.testing.assert_allclose(xyz[:,2],2.)
    projected,z=project(xyz,k,np.eye(4))
    np.testing.assert_allclose(projected,pixels,atol=1e-9)


def test_semantic_plane_uses_surrounding_mapped_points_when_corners_have_no_depth():
    depth=np.ones((60,80),np.float32)*2;depth[10:13,10:13]=0;depth[48:51,68:71]=0
    frame={'depth':depth,'rgb':np.ones((60,80,3),np.uint8)*100,'k_depth':np.array([[100.,0,40],[0,100,30],[0,0,1.]]),'world_optical':np.eye(4)}
    obj={'polygon':[[10,10],[70,10],[70,50],[10,50]],'class':'sign'}
    plane,report=fitted_plane(frame,obj,{'centre':np.array([0,0,2.]),'basis':np.eye(3)})
    assert plane is not None and report['support_points']>100
    assert report['p90_plane_distance_m']<1e-6


def test_unrelated_background_is_not_used_as_the_claimed_panel_plane():
    frame={'depth':np.ones((60,80),np.float32)*4,'rgb':np.ones((60,80,3),np.uint8)*100,'k_depth':np.array([[100.,0,40],[0,100,30],[0,0,1.]]),'world_optical':np.eye(4)}
    plane,report=fitted_plane(frame,{'polygon':[[10,10],[70,10],[70,50],[10,50]],'class':'window'}, {'centre':np.array([0,0,2.]),'basis':np.eye(3)})
    assert plane is None and not report['accepted']


def test_boundary_leveling_reduces_measured_jump_and_preserves_unknown_texels():
    rgb=np.full((80,100,3),85,np.uint8);rgb[:,50:]=115
    labels=np.zeros((80,100),np.uint8);labels[:,50:]=1
    valid=np.ones((80,100),bool);valid[:5]=False
    result,report=level_boundaries(rgb,labels,valid)
    assert report['boundaries']
    assert report['median_absolute_boundary_jump_after']<report['median_absolute_boundary_jump_before']
    np.testing.assert_array_equal(result[:5],rgb[:5])


def test_replacing_embedded_photographs_leaves_triangle_positions_exact(tmp_path):
    v=np.array([[0,0,2],[1,0,2],[0,1,2]],float);t=np.array([[0,1,2]])
    old=tmp_path/'old.png';new=tmp_path/'new.png'
    Image.new('RGB',(8,8),'red').save(old);Image.new('RGB',(8,8),'green').save(new)
    source=tmp_path/'source.glb';target=tmp_path/'target.glb'
    write_glb(source,v,t,np.ones_like(v),np.ones_like(v),face_pages=np.array([0]),face_uv=np.array([[[0,0],[1,0],[0,1]]]),atlas_paths=[old])
    import hashlib
    assert replace_glb_images(source,target,{hashlib.sha256(old.read_bytes()).hexdigest():new})==1
    np.testing.assert_array_equal(glb_triangle_positions(source),glb_triangle_positions(target))


def test_small_sign_polygon_is_visible_from_its_original_camera(tmp_path):
    import open3d as o3d
    frame={'rgb':np.ones((960,720,3),np.uint8)*150,'k_rgb':np.array([[500.,0,360],[0,500,480],[0,0,1.]]),'world_optical':np.eye(4)}
    annotation={'id':'small_sign','class':'sign','polygon':[[350,400],[373,396],[377,440],[353,444]]}
    built,report=photo_mesh(o3d,frame,annotation,(np.array([0.,0.,2.]),np.array([0.,0.,1.])),tmp_path)
    assert built is not None and report['triangle_front_faces_reference_camera']
    mesh,uv,texture=built
    xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)]
    normal=np.cross(xyz[:,1]-xyz[:,0],xyz[:,2]-xyz[:,0])
    assert np.all(np.sum(normal*(-xyz.mean(1)),axis=1)>0)
    assert np.all((uv>=-1e-9)&(uv<=1+1e-9))


def test_reviewed_local_panel_can_use_existing_map_points_instead_of_a_wrong_plane_hint():
    from scipy.spatial import cKDTree
    from surface_model_utils import unproject
    from optimize_session import apply
    pose=np.eye(4);pose[:3,:3]=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
    k=np.array([[100.,0,40],[0,100,30],[0,0,1.]])
    frame={'depth':np.ones((60,80),np.float32)*2,'rgb':np.ones((60,80,3),np.uint8)*100,'k_depth':k,'world_optical':pose}
    points=apply(unproject(frame['depth'],k).reshape(-1,3),pose)
    plane,report=fitted_plane(frame,{'polygon':[[10,10],[70,10],[70,50],[10,50]],'class':'sign'},
        {'centre':np.array([4.,0,0]),'basis':np.array([[0,0,1],[1,0,0],[0,1,0.]])},mapped_tree=cKDTree(points))
    assert plane is not None and 'local_RGBD_RANSAC' in report['support_source']
    assert abs(plane[0][0]-2)<1e-6 and report['mapped_point_nn_median_m']<1e-6


def test_reviewed_floor_uses_its_measured_height_and_preserves_slope():
    from scipy.spatial import cKDTree
    from surface_model_utils import unproject
    from optimize_session import apply
    k=np.array([[100.,0,40],[0,100,30],[0,0,1.]])
    yy,xx=np.indices((60,80));depth=1.6/(1+.01*(xx-40)/100)
    pose=np.eye(4);pose[:3,:3]=np.diag([1.,-1.,-1.]);pose[2,3]=2.
    frame={'depth':depth.astype(np.float32),'rgb':np.ones((60,80,3),np.uint8)*100,'k_depth':k,'world_optical':pose}
    points=apply(unproject(frame['depth'],k).reshape(-1,3),pose)
    plane,report=fitted_plane(frame,{'polygon':[[10,10],[70,10],[70,50],[10,50]],'class':'floor'},
        {'centre':np.array([3.,0,0]),'basis':np.array([[0,0,1],[1,0,0],[0,1,0.]])},mapped_tree=cKDTree(points))
    assert plane is not None and report['local_ransac_inlier_fraction']>.95
    centre,normal=plane
    assert abs(centre[2]-.4-.01*centre[0])<1e-5
    assert abs(-normal[0]/normal[2]-.01)<1e-5


def test_photo_without_depth_can_use_the_existing_local_map_floor():
    k=np.array([[500.,0,360],[0,500,480],[0,0,1.]])
    pose=np.diag([1.,-1.,-1.,1.]);pose[2,3]=2.
    xx,yy=np.meshgrid(np.linspace(-.5,.5,30),np.linspace(-1.1,-.2,30))
    centres=np.c_[xx.ravel(),yy.ravel(),np.full(xx.size,.4)]
    normals=np.tile([0.,0.,1.],(len(centres),1))
    frame={'depth':np.zeros((256,192),np.float32),'k_rgb':k,'world_optical':pose}
    annotation={'class':'floor','polygon':[[180,510],[540,510],[540,880],[180,880]]}
    plane,report=existing_horizontal_plane(frame,annotation,centres,normals)
    assert plane is not None and report['support_source']=='existing_local_map_horizontal_faces'
    assert abs(plane[0][2]-.4)<1e-9 and report['p90_plane_distance_m']<1e-9
    assert len(report['map_support_face_indices'])>=30


def test_wall_sign_can_be_reprojected_without_deleting_the_protected_display_wall():
    k=np.array([[500.,0,360],[0,500,480],[0,0,1.]])
    pose=np.eye(4);pose[:3,:3]=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    yy,zz=np.meshgrid(np.linspace(-.5,.5,30),np.linspace(-.5,.5,30))
    centres=np.c_[np.full(yy.size,2.),yy.ravel(),zz.ravel()];original=centres.copy()
    normals=np.tile([1.,0.,0.],(len(centres),1))
    frame={'rgb':np.zeros((960,720,3),np.uint8),'k_rgb':k,'world_optical':pose}
    annotation={'class':'sign','polygon':[[250,370],[470,370],[470,590],[250,590]]}
    plane,report=existing_display_wall_plane(frame,annotation,(np.array([2.25,0.,0.]),np.array([1.,0.,0.])),centres,normals)
    assert plane is not None and abs(plane[0][0]-2.)<1e-9
    assert abs(report['display_plane_offset_from_same_frame_RGBD_m']-.25)<1e-9
    assert report['support_source']=='existing_local_map_vertical_faces'
    np.testing.assert_array_equal(centres,original)
