import numpy as np
from extend_closed_ceiling_bodies import complete_floor_cells
from audit_closed_ceiling_extension import welded_edge_counts
from complete_service_slabs import closed_slab_arrays


def test_complete_cell_requires_two_different_real_half_triangles():
    xyz=np.array([[[0,0,2.6],[.1,0,2.6],[0,.1,2.6]],
                  [[0,0,2.6],[.1,0,2.6],[0,.1,2.6]],
                  [[.1,0,2.6],[.1,.1,2.6],[0,.1,2.6]]],np.float32)
    one,_=complete_floor_cells(xyz[:2],np.zeros(2),(2,2),.1,2.6)
    both,_=complete_floor_cells(xyz,np.zeros(2),(2,2),.1,2.6)
    assert one[0,0]==1 and both[0,0]==3


def test_no_top_coverage_invented_from_big_triangle_or_wrong_height():
    xyz=np.array([[[0,0,2.6],[1,0,2.6],[0,1,2.6]],
                  [[0,0,1.8],[.1,0,1.8],[0,.1,1.8]]],np.float32)
    coverage,ids=complete_floor_cells(xyz,np.zeros(2),(12,12),.1,2.6)
    assert not coverage.any() and len(ids)==0


def test_cavity_is_closed_without_merging_floor_and_ceiling():
    mask=np.ones((3,3),bool)
    mask[1,1]=False  # A real void must also have inner perimeter skins.
    vertices,parts=closed_slab_arrays(mask,np.zeros(2),.1,1.98,2.60)
    faces=np.concatenate(parts)
    assert welded_edge_counts(vertices[faces].astype(np.float32))==(0,0)
    assert np.all(vertices[parts[0],2]==1.98)
    assert np.all(vertices[parts[1],2]==2.60)


def test_missing_skirt_cannot_be_reported_as_a_closed_cavity():
    vertices,parts=closed_slab_arrays(np.ones((1,1),bool),np.zeros(2),.1,1.98,2.60)
    boundary,_=welded_edge_counts(vertices[np.concatenate(parts[:2])])
    assert boundary==8
