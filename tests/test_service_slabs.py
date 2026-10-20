import numpy as np
from complete_service_slabs import closed_slab_arrays,fill_bounded_holes,mask_lookup,remove_diagonal_contacts,prune_small_components


def test_closed_slab_has_flat_faces_and_each_edge_twice():
    mask=np.ones((3,4),bool);mask[0,0]=False
    vertices,parts=closed_slab_arrays(mask,np.array([2.,3.]),.1,1.5,2.6)
    assert np.all(vertices[parts[0],2]==1.5)
    assert np.all(vertices[parts[1],2]==2.6)
    faces=np.concatenate(parts);edges=np.sort(np.r_[faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]],axis=1)
    _,counts=np.unique(edges,axis=0,return_counts=True)
    assert np.all(counts==2)
    assert vertices[:,0].min()>=2 and vertices[:,0].max()<=2.4
    assert vertices[:,1].min()>=3 and vertices[:,1].max()<=3.3


def test_hole_fill_does_not_expand_boundary_or_cover_forbidden_void():
    mask=np.zeros((8,8),bool);mask[1:7,1:7]=True;mask[3,3]=False
    allowed=np.ones_like(mask);filled=fill_bounded_holes(mask,allowed,2)
    assert filled[3,3]
    assert not filled[0].any() and not filled[:,0].any()
    allowed[3,3]=False;filled=fill_bounded_holes(mask,allowed,2)
    assert not filled[3,3]


def test_outside_grid_never_counts_as_floor_support():
    q=mask_lookup(np.ones((2,2),bool),np.array([0.,0.]),.1,np.array([[-.01,0],[.01,.01],[.2,.01],[0,.2]]))
    assert q.tolist()==[False,True,False,False]


def test_diagonal_contact_cleanup_only_shrinks_and_yields_closed_edges():
    old=np.array([[1,0,0],[0,1,1],[0,1,1]],bool);mask=remove_diagonal_contacts(old)
    assert np.all(~mask|old)
    v,parts=closed_slab_arrays(mask,np.zeros(2),.1,1.,2.)
    faces=np.concatenate(parts);edges=np.sort(np.r_[faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]],axis=1);_,counts=np.unique(edges,axis=0,return_counts=True)
    assert np.all(counts==2)


def test_small_islands_are_removed_without_expanding_main_slab():
    old=np.zeros((10,10),bool);old[2:7,2:7]=True;old[0,0]=True
    mask=prune_small_components(old,10)
    assert mask.sum()==25
    assert not mask[0,0] and np.all(~mask|old)
