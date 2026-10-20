import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from repair_surface_fragmentation import free_space_reject,support_mask


def test_two_consistent_free_space_observations_required():
    actual=free_space_reject([0,1,2,2,3],[0,1,2,4,5])
    np.testing.assert_array_equal(actual,[False,False,True,False,True])


def test_missing_observation_does_not_invent_a_conflict():
    np.testing.assert_array_equal(free_space_reject([0,0],[0,20]),[False,False])


def test_vertex_support_alone_must_not_bridge_a_doorway():
    v=np.array([[0.,0,0],[1,0,0],[0,1,0]])
    keep=support_mask(v,np.array([[0,1,2]]),cKDTree(v),distance=.1,maximum_edge=2.)
    assert not keep[0]  # All corners observed, but face centre is not supported.


def test_dense_local_triangle_supported_and_long_edges_rejected():
    v=np.array([[0.,0,0],[.05,0,0],[0,.05,0]])
    assert support_mask(v,np.array([[0,1,2]]),cKDTree(v))[0]
    assert not support_mask(v,np.array([[0,1,2]]),cKDTree(v),maximum_edge=.01)[0]
