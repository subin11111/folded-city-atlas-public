import json
from pathlib import Path

import numpy as np

from refine_shadow_colour import colour_shadow_lift, correct_shadow_texture, denoise_shadow_rgb

CFG = json.loads((Path(__file__).resolve().parents[1]/'config/display_shadow_colour.json').read_text())


def test_tone_retains_colour_ratios_not_additive_grey():
    rgb = np.array([[.002, .008, .003], [.03, .01, .02], [0, 0, 0], [.8, .7, .6]], np.float32)
    mapped = colour_shadow_lift(rgb, CFG)
    np.testing.assert_array_equal(mapped[2:], rgb[2:])
    np.testing.assert_allclose(mapped[:2, 0]*rgb[:2, 1], mapped[:2, 1]*rgb[:2, 0], atol=1e-8)
    assert (mapped >= rgb).all() and (mapped <= rgb*2.2+1e-7).all()


def test_curve_monotone_and_deep_sensor_floor_not_strongly_lifted():
    y = np.linspace(0, 1, 65536, dtype=np.float32)
    after = colour_shadow_lift(np.repeat(y[:, None], 3, axis=1), CFG)
    assert (np.diff(after[:, 0]) >= -1e-7).all()
    assert colour_shadow_lift(np.array([[1e-6]*3]), CFG).max() < 1.001e-6


def test_flat_shadow_noise_is_attenuated_without_erasing_colour():
    rng = np.random.default_rng(2)
    base = np.array([24, 38, 28])
    sample = np.uint8(np.clip(base+rng.normal(0, 2, (64, 64, 3)), 0, 255))
    clean = denoise_shadow_rgb(sample, np.full((64, 64), 255, np.uint8), CFG)
    assert clean[8:-8, 8:-8].var(axis=(0, 1)).mean() < sample[8:-8, 8:-8].var(axis=(0, 1)).mean()
    np.testing.assert_allclose(clean.mean((0, 1)), sample.mean((0, 1)), atol=.5)
    assert np.abs(clean-sample).max() <= CFG['maximum_denoise_srgb_steps']+1e-4


def test_alpha_black_and_highlights_exact_and_no_bright_colour_bleed():
    sample = np.full((24, 24, 3), [120, 180, 225], np.uint8)
    sample[:, 10:12] = 0
    sample[12:, :10] = [20, 30, 25]
    alpha = np.full((24, 24), 255, np.uint8)
    alpha[:4] = 0
    after = correct_shadow_texture(sample, alpha, CFG)
    np.testing.assert_array_equal(after[:, 10:12], 0)
    np.testing.assert_array_equal(after[:4], sample[:4])
    np.testing.assert_array_equal(after[4:12, :10], sample[4:12, :10])


def test_strong_sign_edge_is_not_spatially_smoothed():
    sample = np.full((32, 32, 3), 10, np.uint8)
    sample[:, 16:] = [60, 30, 15]
    clean = denoise_shadow_rgb(sample, np.full((32, 32), 255, np.uint8), CFG)
    np.testing.assert_allclose(clean[:, 15:17], sample[:, 15:17], atol=.01)


def test_real_glb_texture_roundtrip_and_row_halo(tmp_path):
    import io
    from PIL import Image
    from lift_display_shadows import append_blob, lift_glb
    from refine_display_edges import unpack, write_glb
    from surface_model_utils import write_glb as make_glb
    source, target = tmp_path/'texture.glb', tmp_path/'corrected.glb'
    starter = tmp_path/'starter.glb'
    make_glb(starter, np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32),
             np.array([[0, 1, 2]]), np.tile([0, 0, 1], (3, 1)), np.ones((3, 3), np.float32))
    doc, payload = unpack(starter)
    rng = np.random.default_rng(6)
    rgba = np.uint8(np.clip(rng.normal([25, 40, 30, 255], [2, 2, 2, 0], (280, 24, 4)), 0, 255))
    rgba[10:15, :, :3] = 0
    rgba[30:32, :, 3] = 0
    blob = io.BytesIO()
    Image.fromarray(rgba).save(blob, format='PNG')
    doc['images'] = [{'bufferView': append_blob(doc, payload, blob.getvalue()), 'mimeType': 'image/png'}]
    doc['textures'] = [{'source': 0}]
    doc['materials'][0]['pbrMetallicRoughness'] = {'baseColorTexture': {'index': 0}}
    del doc['meshes'][0]['primitives'][0]['attributes']['COLOR_0']
    write_glb(source, doc, payload)
    _, before = unpack(source)
    lift_glb(source, target, CFG)
    result, after = unpack(target)
    assert after[:len(before)] == before
    view = result['bufferViews'][result['images'][0]['bufferView']]
    output = np.asarray(Image.open(io.BytesIO(after[view.get('byteOffset', 0):view.get('byteOffset', 0)+view['byteLength']])).convert('RGBA'))
    np.testing.assert_array_equal(output[:, :, 3], rgba[:, :, 3])
    np.testing.assert_array_equal(output[:, :, :3], correct_shadow_texture(rgba[:, :, :3], rgba[:, :, 3], CFG))
