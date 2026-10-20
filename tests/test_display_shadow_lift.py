import numpy as np
from lift_display_shadows import shadow_lift, lift_glb

CFG = {'lift': .18, 'knee_linear_luminance': .018, 'highlight_power': 3., 'maximum_linear_gain': 8.}


def test_black_and_white_fixed_midtones_and_shadows_lift_without_clipping():
    values = np.linspace(0, 1, 4096, dtype=np.float32)
    rgb = np.repeat(values[:, None], 3, axis=1)
    after = shadow_lift(rgb, CFG)
    np.testing.assert_array_equal(after[0], 0)
    np.testing.assert_array_equal(after[-1], 1)
    assert (np.diff(after[:, 0]) >= -1e-7).all()
    assert (after >= rgb-1e-7).all()
    assert after.max() <= 1
    assert shadow_lift(np.array([[.005]*3]), CFG).mean() > .03
    assert shadow_lift(np.array([[.8]*3]), CFG).mean() < .803


def test_absolute_chroma_and_spatial_details_not_blended():
    rgb = np.array([[.002, .004, .001], [.02, .04, .01], [.001, .002, .0005]], np.float32)
    result = shadow_lift(rgb, CFG)
    np.testing.assert_allclose(result[:, 0]-result[:, 1], rgb[:, 0]-rgb[:, 1], atol=1e-7)
    np.testing.assert_allclose(result[:, 2]-result[:, 1], rgb[:, 2]-rgb[:, 1], atol=1e-7)
    assert np.all(result[0] != result[1])


def test_gamut_limit_is_common_to_all_channels():
    rgb = np.array([[.05, .01, 1.]], np.float32)
    result = shadow_lift(rgb, CFG)
    assert result.max() <= 1
    np.testing.assert_array_equal(result, rgb)


def test_actual_glb_retains_original_geometry_bytes_and_appends_colour(tmp_path):
    from surface_model_utils import write_glb
    from refine_display_edges import unpack, accessor
    source, target = tmp_path/'source.glb', tmp_path/'bright.glb'
    xyz = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
    rgb = np.array([[.001, .003, .002], [.01, .02, .01], [0, 0, 0]], np.float32)
    write_glb(source, xyz, np.array([[0, 1, 2]]), np.tile([0, 0, 1], (3, 1)), rgb)
    old, before = unpack(source)
    lift_glb(source, target, CFG)
    new, after = unpack(target)
    assert after[:len(before)] == before
    a, b = old['meshes'][0]['primitives'][0], new['meshes'][0]['primitives'][0]
    assert a['attributes']['POSITION'] == b['attributes']['POSITION']
    np.testing.assert_array_equal(accessor(new, after, b['attributes']['POSITION']), xyz)
    np.testing.assert_allclose(accessor(new, after, b['attributes']['COLOR_0']), shadow_lift(rgb, CFG))
