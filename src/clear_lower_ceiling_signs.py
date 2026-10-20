#!/usr/bin/env python3
"""Lift one existing closed slab underside above its photographed sign top."""
import argparse
import json
from pathlib import Path
import shutil

import numpy as np

from audit_structural_delivery import glb_triangle_positions
from optimize_session import sha256_file, utc_now
from presentation_geometry import warp_glb
from refine_display_edges import unpack, accessor

ROOT = Path(__file__).resolve().parents[1]


def slab_bottom_vertices(points, pack):
    mask = pack['mask']
    corners = np.zeros((mask.shape[0]+1, mask.shape[1]+1), bool)
    corners[:-1, :-1] |= mask
    corners[1:, :-1] |= mask
    corners[:-1, 1:] |= mask
    corners[1:, 1:] |= mask
    grid = (points[:, :2]-pack['low'])/pack['cell']
    ij = np.rint(grid).astype(int)
    x, y = ij.T
    valid = (abs(points[:, 2]-pack['bottom']) < 1e-5) & (abs(grid-ij).max(1) < 5e-4)
    valid &= (x >= 0) & (x < corners.shape[1]) & (y >= 0) & (y < corners.shape[0])
    ids = np.flatnonzero(valid)
    valid[ids] &= corners[y[ids], x[ids]]
    return valid


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'config/lower_ceiling_sign_clearance.json')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    cfg = json.loads(a.config.read_text())
    source = ROOT/cfg['source_bundle']
    run = json.loads((source/'manifest.json').read_text())
    semantic_path = source/'semantic_objects.json'
    objects = json.loads(semantic_path.read_text())['objects']
    sign = next(o for o in objects if o['id'] == cfg['sign_id'])
    sign_top = float(np.asarray(sign['geometry']['world_boundary'])[:, 2].max())
    session = cfg['slab_session']
    slab = next(r for r in run['sessions'][session]['service_slabs'] if r['id'] == cfg['slab_id'])
    pack_path = ROOT/cfg['slab_mask']
    pack = dict(np.load(pack_path))
    before = float(pack['bottom'])
    target = float(np.ceil((sign_top+cfg['minimum_display_clearance_m'])/cfg['round_height_step_m'])*cfg['round_height_step_m'])
    assert before < target < float(pack['top'])-.2
    assert target-before <= cfg['maximum_raise_m']
    inputs = {str(p.resolve()): sha256_file(p) for p in [Path(__file__), a.config, source/'manifest.json', semantic_path, pack_path, ROOT/'src/presentation_geometry.py']}
    a.output.mkdir(parents=True, exist_ok=False)
    sessions, cleanup, audit = {}, {}, {}
    import open3d as o3d
    for name, spec in run['sessions'].items():
        folder = a.output/name
        folder.mkdir()
        old = source/name/'clean_textured.glb'
        old_ply = source/name/'clean_structural.ply'
        if name == session:
            def warp(points):
                result = points.copy()
                result[slab_bottom_vertices(points, pack), 2] = target
                return result
            warp_glb(old, folder/'clean_textured.glb', warp)
            mesh = o3d.io.read_triangle_mesh(str(old_ply))
            mesh.vertices = o3d.utility.Vector3dVector(warp(np.asarray(mesh.vertices)))
            mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh(str(folder/'clean_structural.ply'), mesh)
        else:
            shutil.copyfile(old, folder/'clean_textured.glb')
            shutil.copyfile(old_ply, folder/'clean_structural.ply')
        for path in (source/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply', 'clean_textured.glb']:
                shutil.copyfile(path, folder/path.name)
        x = glb_triangle_positions(old).astype(np.float32)
        y = glb_triangle_positions(folder/'clean_textured.glb').astype(np.float32)
        selected = slab_bottom_vertices(x.reshape(-1, 3), pack).reshape(-1, 3) if name == session else np.zeros(x.shape[:2], bool)
        assert np.array_equal(x[:, :, :2], y[:, :, :2])
        assert np.array_equal(x[~selected], y[~selected])
        if selected.any():
            assert np.all(y[:, :, 2][selected] == np.float32(target))
        actual = o3d.io.read_triangle_mesh(str(folder/'clean_structural.ply'))
        assert np.array_equal(np.asarray(actual.vertices)[np.asarray(actual.triangles)].astype(np.float32), y)
        ad, ab = unpack(old)
        bd, bb = unpack(folder/'clean_textured.glb')
        assert ad['meshes'] == bd['meshes'] and ad['materials'] == bd['materials'] and ad['images'] == bd['images']
        positions = {p['attributes']['POSITION'] for m in ad['meshes'] for p in m['primitives']}
        for index in range(len(ad['accessors'])):
            if index not in positions:
                assert np.array_equal(accessor(ad, ab, index), accessor(bd, bb, index))
        # All embedded photo bytes, including currently active appended images.
        for image in ad['images']:
            view = ad['bufferViews'][image['bufferView']]
            start, size = view.get('byteOffset', 0), view['byteLength']
            assert ab[start:start+size] == bb[start:start+size]
        for obj in objects:
            if obj['session'] == name:
                boundary = np.asarray(obj['geometry'].get('world_boundary', []))
                if len(boundary):
                    assert not slab_bottom_vertices(boundary, pack).any(), 'Clearance would move semantic geometry'
        audit[name] = {'XY_exact': True, 'non_target_XYZ_exact': True, 'images_UV_materials_indices_exact': True,
                       'GLB_PLY_equal': True, 'triangle_count_unchanged': len(x),
                       'changed_triangle_corners': int(selected.sum()),
                       'GLB_sha256': sha256_file(folder/'clean_textured.glb')}
        sessions[name] = {**spec, 'lower_ceiling_clearance': audit[name]}
        cleanup[name] = {'before_faces': len(x), 'after_faces': len(y), 'removed_faces': 0,
                         'glb_sha256': sha256_file(folder/'clean_textured.glb'),
                         'ply_sha256': sha256_file(folder/'clean_structural.ply')}
        for p in [old, old_ply]:
            inputs[str(p.resolve())] = sha256_file(p)
        print('Local ceiling clearance; preserved: '+name, flush=True)
    for path in source.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json']:
            shutil.copyfile(path, a.output/path.name)
    record = {'config': cfg, 'old_underside_m': before, 'new_underside_m': target,
              'sign_top_m': sign_top, 'upper_floor_unchanged_m': float(pack['top']),
              'source_manifest': str((source/'manifest.json').resolve())}
    result = {**run, 'created_utc': utc_now(), 'sessions': sessions,
              'inputs_sha256': {**run['inputs_sha256'], **inputs},
              'lower_ceiling_sign_clearance': record,
              'previous_material_manifest': str((source/'manifest.json').resolve()),
              'floor_material_source_manifest': str((source/'manifest.json').resolve()), 'scope': cfg['scope']}
    (a.output/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    digest = sha256_file(a.output/'manifest.json')
    (a.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': digest, 'sessions': cleanup}, indent=2)+'\n')
    (a.output/'geometry_audit.json').write_text(json.dumps({'bundle_manifest_sha256': digest,
        'sessions': audit, 'clearance': record, 'source_map_pose_changes': 0,
        'source_sha256': sha256_file(Path(__file__)), 'scope': 'Actual output preservation check, not a surveyed ceiling height or visual acceptance.'}, indent=2)+'\n')
    for path, digest in inputs.items():
        assert sha256_file(Path(path)) == digest


if __name__ == '__main__':
    main()
