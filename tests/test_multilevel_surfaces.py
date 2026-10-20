import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from reconcile_multilevel_surfaces import anchor_lattice,limit_lattice_slopes,sample_lattice,build_anchored_sheet,remap_mask,variable_height_stair_guard


def test_shared_source_plane_preserves_local_height_without_independent_session_modes():
    y,x=np.mgrid[-3:3.1:.2,-3:3.1:.2];points=np.c_[x.ravel(),y.ravel(),(2.6+.012*x-.008*y).ravel()]
    model=anchor_lattice(points);q=np.array([[0.,0.],[1.,1.],[-1.,-1.]])
    np.testing.assert_allclose(sample_lattice(model,q),2.6+.012*q[:,0]-.008*q[:,1],atol=.002)
    assert np.ptp(sample_lattice(model,q))>0
    # Different session masks query the exact same group, never two constants.
    np.testing.assert_array_equal(sample_lattice(model,q),sample_lattice(model,q.copy()))


def test_affine_patches_are_continuous_at_coarse_tile_and_diagonal_boundaries():
    m={'low':np.array([0.,0.]),'spacing':2.,'heights':np.array([[2.4,2.5,2.4],[2.5,2.55,2.5],[2.6,2.6,2.6]])}
    q=np.array([[2.,.7],[.7,1.3]])
    for offset in [[1e-9,0],[0,1e-9]]:
        assert np.max(abs(sample_lattice(m,q+offset)-sample_lattice(m,q-offset)))<1e-8


def test_actual_triangle_gradients_are_limited_not_only_the_individual_fits():
    z=limit_lattice_slopes(np.array([[0.,.4,.2],[.5,-.2,.1],[.1,.2,.5]]),2.,4.)
    gx=z[:-1,1:]-z[:-1,:-1];gy=z[1:,:-1]-z[:-1,:-1]
    g2x=z[1:,1:]-z[1:,:-1];g2y=z[1:,1:]-z[:-1,1:]
    assert max(np.hypot(gx,gy).max(),np.hypot(g2x,g2y).max())/2<=np.tan(np.deg2rad(4.))+1e-7


def test_masked_atrium_does_not_create_floor_or_low_ceiling_triangles():
    import open3d as o3d
    mask=np.ones((10,10),bool);mask[3:7,3:7]=False
    lattice={'low':np.array([0.,0.]),'spacing':2.,'heights':np.array([[2.,2.05],[2.05,2.1]])}
    for kind,direction in [('floor',1),('ceiling',-1)]:
        mesh,_=build_anchored_sheet(o3d,mask,np.array([0.,0.]),.1,kind,lattice)
        centres=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].mean(1);ij=np.floor(centres[:,:2]/.1).astype(int)
        assert mask[ij[:,1],ij[:,0]].all()
        assert np.all(np.asarray(mesh.triangle_normals)[:,2]*direction>0)


def test_stair_guard_uses_new_local_height_instead_of_old_global_z():
    lattice={'low':np.array([0.,0.]),'spacing':2.,'heights':np.full((2,2),.4)}
    guard={'centres':[[.5,.5,.4]],'half':[.15,.15,.1]}
    mask=variable_height_stair_guard((10,10),np.array([0.,0.]),.1,lattice,guard)
    assert mask[5,5] and not mask[0,0]


def test_atrium_mask_remaps_in_world_xy_not_image_pixel_indices():
    mask=np.zeros((10,10),bool);mask[4:6,4:6]=True
    result=remap_mask(mask,np.array([-1.,-1.]),.2,np.array([-.2,-.2]),.1,(4,4))
    assert result.all()


def test_high_roof_is_not_included_in_reviewed_atrium_exclusion():
    import json
    cfg=json.loads((Path(__file__).resolve().parents[1]/'config/multilevel_surface_topology.json').read_text())
    assert 'lobby_ceiling_1f' in cfg['void_excluded_layers']
    assert 'floor2_ceiling_2f' not in cfg['void_excluded_layers']
    assert cfg['layer_groups']['floor2_floor_2f']==cfg['layer_groups']['sub_floor_2f']
