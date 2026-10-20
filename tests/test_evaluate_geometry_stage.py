import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from evaluate_geometry_stage import graph_role
from evaluate_map_surfaces import pair_metrics


def test_explicit_reserved_edge_overrides_native_training_role():
    native={'verified_loops':[{'from_id':548,'to_id':981,'held_out':False}]}
    extra=[{'session_a':'floor2','session_b':'floor2','node_a':548,'node_b':981,'held_out':True}]
    assert graph_role('floor2',548,981,native,extra)[0]=='reserved_graph_holdout'


def test_unused_new_surface_fit_is_not_mislabeled_global_holdout():
    native={'verified_loops':[{'from_id':548,'to_id':981,'held_out':False}]}
    assert graph_role('floor2',548,981,native,[])[0]=='native_graph_training'


def test_fixed_patch_separation_is_global_gauge_invariant():
    uv=np.mgrid[-1:1:9j,-1:1:9j].reshape(2,-1).T
    cloud=np.column_stack([uv,np.zeros(len(uv))])
    patch={'testing':cloud,'training':cloud,'center':np.zeros(3),'normal':np.array([0.,0.,1.])}
    a=np.eye(4);b=np.eye(4);b[2,3]=.27
    gauge=np.eye(4);gauge[:3,:3]=Rotation.from_euler('xyz',[.1,-.2,.8]).as_matrix();gauge[:3,3]=[10.,-2.,1.]
    before=pair_metrics(patch,patch,a,b)
    after=pair_metrics(patch,patch,gauge@a,gauge@b)
    assert abs(before['symmetric_plane_abs_median_m']-.27)<1e-12
    assert abs(before['symmetric_plane_abs_median_m']-after['symmetric_plane_abs_median_m'])<1e-12
