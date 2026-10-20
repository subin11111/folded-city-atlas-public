import numpy as np
from clear_lower_ceiling_signs import slab_bottom_vertices
from view_local_roadview import photo_view_intrinsics


def test_only_existing_closed_slab_bottom_grid_vertices_selected():
    pack = {'mask': np.array([[True, False], [False, False]]), 'cell': .1,
            'low': np.array([0., 0.]), 'bottom': 1.75}
    points = np.array([[0, 0, 1.75], [.1, .1, 1.75], [.2, .2, 1.75],
                       [0, 0, 2.6], [.02, .02, 1.75], [.1, 0, 1.88]])
    assert slab_bottom_vertices(points, pack).tolist() == [True, True, False, False, False, False]


def test_native_photo_view_contains_original_field_on_wide_and_tall_panels():
    k=np.array([[670.,0,360],[0,670.,480],[0,0,1.]])
    for width,height in [(1290,1000),(800,800),(600,1200)]:
        changed=photo_view_intrinsics(k,width,height)
        scale=min(width/720,height/960)
        assert changed[0,0]==changed[1,1]==670*scale
        assert np.allclose(changed[:2,2],[width/2,height/2])
        assert width/changed[0,0]>=720/670-1e-10
        assert height/changed[1,1]>=960/670-1e-10
