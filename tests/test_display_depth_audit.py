import sys
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from audit_display_depth import compare_depths
from build_fidelity_display import visibility_masks
from build_geometry_preserving_display import component_keep


def test_new_hits_do_not_change_common_pixel_comparison():
    d=np.ones(4);before=np.array([1.,1.,np.inf,np.inf]);after=np.array([1.,1.,3.,3.])
    s=compare_depths(before,after,d)
    assert s['old_all_hits']['median_m']==0
    assert s['new_all_hits']['median_m']==1
    assert s['same_pixels_after']['median_m']==0
    assert s['newly_hit']['pixels']==2


def test_removed_foreground_exposes_background_without_moving_remaining_vertices():
    s=compare_depths(np.array([1.,2.]),np.array([3.,2.]),np.array([1.,2.]))
    assert s['display_depth_change_on_same_pixels']['p90_m']>1


def test_visibility_guard_separates_foreground_from_missing_reference():
    front,missing=visibility_masks(np.array([1.,1.,np.inf,1.]),np.array([.8,1.2,2.,np.inf]))
    np.testing.assert_array_equal(front,[True,False,True,False])
    np.testing.assert_array_equal(missing,[False,True,False,True])


def test_micro_cleanup_does_not_delete_large_components():
    np.testing.assert_array_equal(component_keep([.001,.01,1.],[0,1,2,1]),[False,True,True,True])
