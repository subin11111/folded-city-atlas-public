#!/usr/bin/env python3
"""Actual edge-finish artifacts: preserved horizontal/semantic faces and UVs."""
import argparse
import copy
import hashlib
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from audit_structural_delivery import glb_triangle_positions
from optimize_session import sha256_file, utc_now
from refine_display_edges import accessor, unpack

ROOT = Path(__file__).resolve().parents[1]


def image_bytes(doc, payload, index):
    image = doc['images'][index]
    view = doc['bufferViews'][image['bufferView']]
    start = view.get('byteOffset', 0)
    return bytes(payload[start:start+view['byteLength']])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run = json.loads((args.bundle/'manifest.json').read_text())
    base = Path(run['edge_finish_source_manifest']).parent
    cfg = run['edge_finish_config']
    results = {}
    import open3d as o3d
    for name in run['sessions']:
        before_path, after_path = base/name/'clean_textured.glb', args.bundle/name/'clean_textured.glb'
        old = glb_triangle_positions(before_path).astype(np.float32)
        new = glb_triangle_positions(after_path).astype(np.float32)
        assert new.shape == old.shape
        horizontal = np.ptp(old[:, :, 2], axis=1) < 1e-5
        assert np.array_equal(new[horizontal], old[horizontal]), 'A horizontal face/contour moved'
        original, ids = np.unique(old.reshape(-1, 3), axis=0, return_inverse=True)
        pack = np.load(args.bundle/name/'boundary_displacements.npz')
        assert np.array_equal(original, pack['source'])
        target = pack['target']
        assert np.array_equal(new, target[ids.reshape(-1, 3)])
        assert np.array_equal(target[pack['protected']], original[pack['protected']])
        motion = np.linalg.norm(target-original, axis=1)
        assert motion.max(initial=0) <= cfg['maximum_total_motion_m']+2e-6
        triangles = ids.reshape(-1, 3)
        edges, counts = np.unique(np.sort(triangles[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1), axis=0, return_counts=True)
        boundary = np.zeros(len(original), bool)
        boundary[edges[counts == 1].ravel()] = True
        assert np.array_equal(target[~boundary], original[~boundary]), 'Interior surface moved'
        normal = np.cross(old[:, 1]-old[:, 0], old[:, 2]-old[:, 0])
        changed = np.cross(new[:, 1]-new[:, 0], new[:, 2]-new[:, 0])
        assert (np.sum(normal*changed, axis=1) >= np.sum(normal*normal, axis=1)*.2499).all()
        ply = o3d.io.read_triangle_mesh(str(args.bundle/name/'clean_structural.ply'))
        assert np.array_equal(np.asarray(ply.vertices)[np.asarray(ply.triangles)].astype(np.float32), new)
        a, ab = unpack(before_path)
        b, bb = unpack(after_path)
        assert a['materials'] == b['materials'][:len(a['materials'])]
        assert a['textures'] == b['textures'][:len(a['textures'])]
        assert a['nodes'] == b['nodes'] and len(a['meshes']) == len(b['meshes'])
        image_map = {i: i for i in range(len(a['images']))}
        for old_mesh, new_mesh in zip(a['meshes'], b['meshes']):
            assert len(old_mesh['primitives']) == len(new_mesh['primitives'])
            for old_primitive, new_primitive in zip(old_mesh['primitives'], new_mesh['primitives']):
                aa, ba = copy.deepcopy(old_primitive), copy.deepcopy(new_primitive)
                old_material, new_material = aa.pop('material'), ba.pop('material')
                assert aa == ba, 'Primitive attributes/indices changed'
                am, bm = copy.deepcopy(a['materials'][old_material]), copy.deepcopy(b['materials'][new_material])
                at = am.get('pbrMetallicRoughness', {}).get('baseColorTexture', {})
                bt = bm.get('pbrMetallicRoughness', {}).get('baseColorTexture', {})
                if 'index' in at:
                    ai, bi = a['textures'][at.pop('index')]['source'], b['textures'][bt.pop('index')]['source']
                    image_map[ai] = bi
                assert am == bm, 'Non-texture material settings changed'
        positions = {p['attributes']['POSITION'] for m in a['meshes'] for p in m['primitives']}
        for index in range(len(a['accessors'])):
            if index not in positions:
                assert np.array_equal(accessor(a, ab, index), accessor(b, bb, index)), 'UV/normal/index accessor changed'
        protected = {sha256_file(p) for p in (base/name).glob('*.png') if p.name != 'continuous_2f_roof.png'}
        image_changes, protected_images = 0, 0
        for i in range(len(a['images'])):
            old_image, new_image = image_bytes(a, ab, i), image_bytes(b, bb, image_map[i])
            if hashlib.sha256(old_image).hexdigest() in protected:
                assert new_image == old_image, 'Protected original sign/door/glazing photo changed'
                protected_images += 1
            if old_image == new_image:
                continue
            x = np.asarray(Image.open(io.BytesIO(old_image)).convert('RGB')).astype(np.int16)
            y = np.asarray(Image.open(io.BytesIO(new_image)).convert('RGB')).astype(np.int16)
            limit = cfg['maximum_texture_srgb_change']
            photo_finish = run.get('photo_boundary_finish', {})
            if hashlib.sha256(old_image).hexdigest() == photo_finish.get('source_roof_sha256'):
                limit += photo_finish['config']['maximum_srgb_change']
                edge_source = Path(photo_finish['edge_source_manifest']).parent
                edge_doc, edge_bytes = unpack(edge_source/name/'clean_textured.glb')
                previous = np.asarray(Image.open(io.BytesIO(image_bytes(edge_doc, edge_bytes, i))).convert('RGB')).astype(np.int16)
                band = np.asarray(Image.open(args.bundle/'ceiling_photo_edge_band.png')) > 0
                assert np.array_equal(previous[~band], y[~band]), 'Changed ceiling outside photo handover band'
            assert x.shape == y.shape and abs(x-y).max() <= limit
            image_changes += 1
        results[name] = {'triangle_count_unchanged': len(old), 'GLB_PLY_equal': True,
                         'horizontal_faces_and_XY_contours_exact': True,
                         'protected_semantic_and_stair_vertices_exact': True,
                         'interior_vertices_exact': True, 'UV_normal_indices_exact': True,
                         'new_or_removed_faces': 0, 'moved_boundary_vertices': int((motion > 1e-7).sum()),
                         'maximum_actual_motion_m': float(motion.max(initial=0)),
                         'protected_images_exact': protected_images, 'edge_softened_images': image_changes,
                         'GLB_sha256': sha256_file(after_path)}
        print('Actual edge-finish preservation verified: '+name, flush=True)
    assert sha256_file(base/'semantic_objects.json') == sha256_file(args.bundle/'semantic_objects.json')
    pointer = json.loads((ROOT/'config/current_map.json').read_text())
    assert sha256_file(ROOT/pointer['manifest']) == pointer['manifest_sha256'] == run['selected_manifest_sha256']
    selected = json.loads((ROOT/pointer['manifest']).read_text())
    for item in selected['database_integrity'].values():
        assert sha256_file(Path(item['database'])) == item['sha256']
    for path, digest in run['inputs_sha256'].items():
        assert sha256_file(Path(path)) == digest, 'Input provenance mismatch: '+path
    output = {'created_utc': utc_now(), 'bundle_manifest_sha256': sha256_file(args.bundle/'manifest.json'),
              'sessions': results, 'selected_map_and_source_databases_unchanged': True,
              'all_input_bindings_valid': True, 'audit_source_sha256': sha256_file(Path(__file__)),
              'scope': 'Artifact preservation only, not visual acceptance. Inspect actual before/after render images separately.'}
    args.output.write_text(json.dumps(output, indent=2)+'\n')


if __name__ == '__main__':
    main()
