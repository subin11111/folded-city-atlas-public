#!/usr/bin/env python3
"""Verify material-only changes and exact original floor/sign streams."""
import argparse
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from audit_structural_delivery import glb_triangle_positions
from optimize_session import sha256_file, utc_now
from refine_display_edges import unpack, accessor


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    run = json.loads((a.bundle/'manifest.json').read_text())
    base = Path(run['ceiling_material_continuity']['config']['source_bundle'])
    root = Path(__file__).resolve().parents[1]
    if not base.is_absolute():
        base = root/base
    sessions = {}
    for name, spec in run['sessions'].items():
        before_doc, before = unpack(base/name/'clean_textured.glb')
        after_doc, after = unpack(a.bundle/name/'clean_textured.glb')
        assert after[:len(before)] == before, 'Original binary, UV or photographs overwritten'
        assert sha256_file(base/name/'clean_structural.ply') == sha256_file(a.bundle/name/'clean_structural.ply')
        assert np.array_equal(glb_triangle_positions(base/name/'clean_textured.glb'), glb_triangle_positions(a.bundle/name/'clean_textured.glb'))
        original = before_doc['meshes'][0]['primitives']
        changed = after_doc['meshes'][0]['primitives']
        assert len(original) == len(changed)
        extra_count = len(spec['closure_appended'])
        allowed = set(range(len(original)-extra_count, len(original)))
        floor_primitives = []
        source_fascias = []
        for index, (old, new) in enumerate(zip(original, changed)):
            pos = accessor(before_doc, before, old['attributes']['POSITION'])
            material = before_doc['materials'][old['material']]
            texture_id = material.get('pbrMetallicRoughness', {}).get('baseColorTexture', {}).get('index')
            if texture_id is not None:
                image_id = before_doc['textures'][texture_id]['source']
                view = before_doc['bufferViews'][before_doc['images'][image_id]['bufferView']]
                if view['byteLength'] < 150:
                    start = view.get('byteOffset', 0)
                    rgb = np.asarray(Image.open(io.BytesIO(before[start:start+view['byteLength']])).convert('RGB'))
                    if rgb.shape in [(8, 8, 3), (16, 16, 3)] and np.all(rgb == [64, 69, 77]):
                        allowed.add(index); source_fascias.append(index)
            for key in ['POSITION', 'NORMAL', 'TEXCOORD_0']:
                assert old['attributes'].get(key) == new['attributes'].get(key)
            assert old.get('indices') == new.get('indices')
            if index not in allowed:
                assert old == new, 'Unexpected material or colour change outside ceiling/fascia'
            floor = any(np.max(abs(pos[:, 2]-height)) < 2e-5 for height in [-1.55, 2.6])
            if floor:
                assert index not in allowed, 'Floor texture classified as a ceiling/fascia'
                floor_primitives.append(index)
        # Every pre-existing image except the newly added closure atlases still
        # has its original descriptor. This includes every floor/sign photo.
        allowed_images = set()
        for index in range(len(original)-extra_count, len(original)):
            material = before_doc['materials'][original[index]['material']]
            tex = material['pbrMetallicRoughness']['baseColorTexture']['index']
            allowed_images.add(before_doc['textures'][tex]['source'])
        for index, descriptor in enumerate(before_doc['images']):
            if index not in allowed_images:
                assert descriptor == after_doc['images'][index]
        sessions[name] = {'all_triangle_XYZ_and_order_exact': True, 'PLY_file_exact': True,
                          'original_binary_prefix_exact': True, 'original_position_normal_UV_streams_exact': True,
                          'all_existing_floor_primitive_materials_exact': floor_primitives,
                          'original_sign_and_floor_image_descriptors_exact': True,
                          'only_declared_new_ceiling_materials_and_plain_fascias_changed': True,
                          'matched_original_uniform_fascia_primitives': source_fascias}
        print('Verified material-only delivery: '+name, flush=True)
    result = {'created_utc': utc_now(), 'bundle_manifest_sha256': sha256_file(a.bundle/'manifest.json'),
              'audit_source_sha256': sha256_file(Path(__file__)), 'sessions': sessions,
              'floor_texture_changes': 0, 'sign_photo_changes': 0, 'geometry_changes': 0,
              'scope': 'Material stream preservation, not a numerical substitute for looking at before/after renders.'}
    a.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
