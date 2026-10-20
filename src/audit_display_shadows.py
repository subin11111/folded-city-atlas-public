#!/usr/bin/env python3
"""Check real shadow-lift exports against immutable geometry and source colours."""
import argparse
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from optimize_session import sha256_file, utc_now
from refine_display_edges import accessor, unpack
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

ROOT = Path(__file__).resolve().parents[1]


def pixels(doc, payload, descriptor):
    view = doc['bufferViews'][descriptor['bufferView']]
    offset = view.get('byteOffset', 0)
    return np.asarray(Image.open(io.BytesIO(payload[offset:offset+view['byteLength']])).convert('RGBA'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run = json.loads((args.bundle/'manifest.json').read_text())
    cfg = run['shadow_lift']['config']
    colour_method = cfg.get('method') == 'colour_preserving_denoised'
    base = Path(run['shadow_source_manifest']).parent
    sessions = {}
    for name in run['sessions']:
        old, before = unpack(base/name/'clean_textured.glb')
        new, after = unpack(args.bundle/name/'clean_textured.glb')
        assert after[:len(before)] == before, 'Original geometry/image bytes overwritten'
        assert sha256_file(base/name/'clean_structural.ply') == sha256_file(args.bundle/name/'clean_structural.ply')
        assert old['materials'] == new['materials'] and old['textures'] == new['textures']
        assert old.get('nodes') == new.get('nodes') and old.get('scenes') == new.get('scenes')
        assert len(old['meshes']) == len(new['meshes'])
        active, colour_count, faces = set(), 0, 0
        for old_mesh, new_mesh in zip(old['meshes'], new['meshes']):
            assert len(old_mesh['primitives']) == len(new_mesh['primitives'])
            for a, b in zip(old_mesh['primitives'], new_mesh['primitives']):
                assert {k: v for k, v in a.items() if k != 'attributes'} == {k: v for k, v in b.items() if k != 'attributes'}
                assert {k: v for k, v in a['attributes'].items() if k != 'COLOR_0'} == {k: v for k, v in b['attributes'].items() if k != 'COLOR_0'}
                faces += old['accessors'][a.get('indices', a['attributes']['POSITION'])]['count']//3
                if 'COLOR_0' in a['attributes']:
                    source = accessor(old, before, a['attributes']['COLOR_0'])
                    target = accessor(new, after, b['attributes']['COLOR_0'])
                    assert source.shape == target.shape and np.isfinite(target).all()
                    assert (target >= source-1e-7).all() and target.max() <= 1
                    black = (source == 0).all(1)
                    assert np.array_equal(target[black], source[black])
                    if colour_method:
                        assert (target <= source*cfg['maximum_linear_gain']+2e-7).all()
                        np.testing.assert_allclose(target[:, 0]*source[:, 1], target[:, 1]*source[:, 0], atol=2e-7)
                        np.testing.assert_allclose(target[:, 2]*source[:, 1], target[:, 1]*source[:, 2], atol=2e-7)
                        high = source @ np.array([.2126, .7152, .0722]) >= cfg['shadow_end_linear']
                        np.testing.assert_array_equal(target[high], source[high])
                    else:
                        np.testing.assert_allclose(target[:, 0]-target[:, 1], source[:, 0]-source[:, 1], atol=2e-7)
                        np.testing.assert_allclose(target[:, 2]-target[:, 1], source[:, 2]-source[:, 1], atol=2e-7)
                    colour_count += 1
                tex = old['materials'][a['material']].get('pbrMetallicRoughness', {}).get('baseColorTexture', {}).get('index')
                if tex is not None:
                    active.add(old['textures'][tex]['source'])
        for image_id in active:
            source = pixels(old, before, old['images'][image_id])
            target = pixels(new, after, new['images'][image_id])
            assert source.shape == target.shape
            assert np.array_equal(source[:, :, 3], target[:, :, 3])
            if colour_method:
                source_rgb = source[:, :, :3]
                # Bounds are independent of the generator's denoising implementation.
                steps = cfg['maximum_denoise_srgb_steps']
                upper = linear_to_srgb(np.minimum(srgb_to_linear(np.minimum(source_rgb.astype(float)+steps, 255)/255.)*cfg['maximum_linear_gain'], 1))*255
                assert (target[:, :, :3] <= upper+1).all()
                assert (target[:, :, :3].astype(float) >= source_rgb.astype(float)-steps-1).all()
                high = srgb_to_linear(source_rgb/255.) @ np.array([.2126, .7152, .0722]) >= cfg['shadow_end_linear']
                np.testing.assert_array_equal(target[:, :, :3][high], source_rgb[high])
                transparent = source[:, :, 3] == 0
                np.testing.assert_array_equal(target[transparent], source[transparent])
            else:
                assert (target[:, :, :3] >= source[:, :, :3]).all(), 'Unexpected darkening/blur'
            black = (source[:, :, :3] == 0).all(2)
            assert np.array_equal(target[:, :, :3][black], source[:, :, :3][black])
        sessions[name] = {'PLY_sha256_exact': True, 'original_binary_prefix_exact': True,
                          'position_normal_UV_indices_material_assignments_exact': True,
                          'triangle_count': faces, 'active_images_checked': len(active),
                          'baked_colour_primitives_checked': colour_count, 'image_size_and_alpha_exact': True,
                          'black_zero_is_preserved': True,
                          'colour_method': 'bounded_shadow_denoise_then_weak_ratio_gain' if colour_method else 'neutral_additive_lift',
                          'highlight_colours_exact': colour_method,
                          'spatial_filter_changes_allowed': colour_method}
        print('Verified unchanged geometry and actual shadow colours: '+name, flush=True)
    pointer = json.loads((ROOT/'config/current_map.json').read_text())
    assert sha256_file(ROOT/pointer['manifest']) == run['selected_manifest_sha256']
    road = ROOT/'outputs/student_union_roadview_v1/manifest.json'
    result = {'created_utc': utc_now(), 'bundle_manifest_sha256': sha256_file(args.bundle/'manifest.json'),
              'audit_source_sha256': sha256_file(Path(__file__)), 'sessions': sessions,
              'geometry_changes': 0, 'pose_changes': 0, 'UV_changes': 0,
              'original_photo_pack_manifest_sha256': sha256_file(road),
              'original_photo_pack_rewritten': False, 'selected_map_unchanged': True,
              'active_display_colour_changes': True, 'source_sign_and_floor_photos_retained_in_binary_prefix': True,
              'navigation_exported': False,
              'method': cfg.get('method', 'neutral_additive_lift'),
              'scope': 'Display colour derivative only. Shadow textures can be filtered in colour-preserving mode; fine-detail/noise tradeoff requires visual review. Not recovered illumination, clipped detail, missing surfaces or navigation approval.'}
    args.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
