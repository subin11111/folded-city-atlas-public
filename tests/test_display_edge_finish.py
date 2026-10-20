import numpy as np
from refine_display_edges import soften_texture_edges, trim_boundary_teeth
from soften_display_photo_boundaries import feather_photo_boundaries


CFG = {'iterations': 4, 'boundary_step': .35, 'maximum_total_motion_m': .05,
       'maximum_boundary_leg_m': .30, 'minimum_normal_coherence': .95,
       'texture_sigma_px': .75, 'texture_edge_blend': .35,
       'maximum_texture_linear_change': .035}


def test_convex_tooth_is_trimmed_but_protected_and_interior_vertices_stay():
    v = np.array([[-.1, 0, 2.6], [0, -.1, 2.6], [.1, 0, 2.6],
                  [.1, .2, 2.6], [-.1, .2, 2.6], [0, .08, 2.6]], np.float32)
    t = np.array([[0, 1, 5], [1, 2, 5], [2, 3, 5], [3, 4, 5], [4, 0, 5]])
    protected = np.array([True, False, True, True, True, False])
    result, report = trim_boundary_teeth(v, t, protected, CFG)
    assert result[1, 1] > v[1, 1]
    assert np.array_equal(result[protected], v[protected])
    assert np.array_equal(result[5], v[5])
    assert np.array_equal(result[:, 2], v[:, 2])
    assert report['maximum_actual_motion_m'] <= .050001
    assert report['new_faces'] == report['removed_faces'] == 0


def test_texture_flat_regions_and_colour_remain_exact_outside_small_edge_band():
    rgb = np.full((80, 80, 3), [150, 140, 120], np.uint8)
    rgb[30:50, 30:50] = [20, 22, 21]
    result, report = soften_texture_edges(rgb, CFG)
    assert np.array_equal(result[:20], rgb[:20])
    assert np.array_equal(result[36:44, 36:44], rgb[36:44, 36:44])
    assert report['outside_edge_band_exact']
    assert 0 < report['changed_pixels'] < 600
    assert np.max(abs(result.astype(int)-rgb.astype(int))) < 30


def test_known_photo_seam_is_feathered_without_changing_photo_interiors():
    rgb = np.full((100, 140, 3), 130, np.uint8)
    rgb[:, 70:] = 175
    labels = np.zeros((100, 140), np.uint8)
    labels[:, 70:] = 1
    cfg = {'band_width_m': .16, 'low_frequency_sigma_m': .10,
           'detail_sigma_px': 1.2, 'maximum_srgb_change': 24}
    result, band = feather_photo_boundaries(rgb, labels, np.ones(labels.shape, bool), .02, cfg)
    assert np.array_equal(result[:, :50], rgb[:, :50])
    assert np.array_equal(result[:, 90:], rgb[:, 90:])
    assert result[50, 68, 0] > rgb[50, 68, 0]
    assert result[50, 71, 0] < rgb[50, 71, 0]
    assert np.array_equal(result[band == 0], rgb[band == 0])
