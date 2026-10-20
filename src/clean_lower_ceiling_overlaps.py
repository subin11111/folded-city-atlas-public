#!/usr/bin/env python3
"""Remove photographed local low-ceiling remnants beneath corrected bank slab."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from clean_structural_fragments import filter_glb
from complete_service_slabs import mask_lookup
from audit_structural_delivery import glb_triangle_positions
from optimize_session import sha256_file, utc_now
from refine_display_edges import unpack, accessor
from presentation_geometry import LevelPresentation

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'config/lower_ceiling_overlap_cleanup.json')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    cfg = json.loads(a.config.read_text())
    source = ROOT/cfg['source_bundle']
    run = json.loads((source/'manifest.json').read_text())
    mask_path = ROOT/cfg['source_mask']
    pack = np.load(mask_path)
    ceiling = run['lower_ceiling_sign_clearance']
    a.output.mkdir(parents=True, exist_ok=False)
    inputs = {str(p.resolve()): sha256_file(p) for p in [Path(__file__), a.config, mask_path, source/'manifest.json', ROOT/'src/clean_structural_fragments.py']}
    sessions, cleanup, audit = {}, {}, {}
    import open3d as o3d
    for name, spec in run['sessions'].items():
        folder = a.output/name
        folder.mkdir()
        old = source/name/'clean_textured.glb'
        mesh = o3d.io.read_triangle_mesh(str(source/name/'clean_structural.ply'))
        mesh.compute_triangle_normals()
        xyz = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32)
        remove = np.zeros(len(xyz), bool)
        if name == cfg['session']:
            inside = mask_lookup(pack['photographed'], pack['low'], float(pack['cell']), xyz.reshape(-1, 3)[:, :2]).reshape(-1, 3).any(1)
            normal_z = abs(np.asarray(mesh.triangle_normals)[:, 2])
            below = xyz[:, :, 2].max(1) < ceiling['new_underside_m']-.02
            near = abs(xyz[:, :, 2].mean(1)-ceiling['old_underside_m']) < cfg['old_height_band_m']
            remove = inside & below & near & (normal_z > cfg['minimum_abs_normal_z'])
            if cfg.get('sign_ray_cleanup'):
                sem = json.loads((source/'semantic_objects.json').read_text())['objects']
                for p in [source/'semantic_objects.json', ROOT/'outputs/student_union_roadview_v1/roadview.json', ROOT/'src/presentation_geometry.py']:
                    inputs[str(p.resolve())] = sha256_file(p)
                sign = next(o for o in sem if o['id'] == cfg['sign_ray_cleanup']['id'])
                boundary = np.asarray(sign['geometry']['world_boundary'])
                centre = boundary.mean(0)
                _, _, vt = np.linalg.svd(boundary-centre, full_matrices=False)
                plane_distance = abs((xyz.mean(1)-centre)@vt[-1])
                eligible = plane_distance < cfg['sign_ray_cleanup']['maximum_plane_distance_m']
                eligible &= (xyz.mean(1)[:, 2] > boundary[:, 2].min()-.05) & (xyz.mean(1)[:, 2] < boundary[:, 2].max()+.10)
                doc, payload = unpack(old)
                protected = np.zeros(len(xyz), bool)
                photo_hashes = {sha256_file(p) for p in (source/name).glob('*.png')}
                cursor = 0
                for primitive in doc['meshes'][0]['primitives']:
                    count = doc['accessors'][primitive.get('indices', primitive['attributes']['POSITION'])]['count']//3
                    mat = doc['materials'][primitive['material']]
                    texture = mat.get('pbrMetallicRoughness', {}).get('baseColorTexture', {}).get('index')
                    if texture is not None:
                        image = doc['images'][doc['textures'][texture]['source']]
                        view = doc['bufferViews'][image['bufferView']]
                        start = view.get('byteOffset', 0)
                        if hashlib.sha256(payload[start:start+view['byteLength']]).hexdigest() in photo_hashes:
                            protected[cursor:cursor+count] = True
                    cursor += count
                assert cursor == len(xyz)
                eligible &= ~protected
                y, x = np.indices((48, 160))
                u, v = (x.ravel()+.5)/160, (y.ravel()+.5)/48
                target = ((1-u)*(1-v))[:, None]*boundary[0]+(u*(1-v))[:, None]*boundary[1]+(u*v)[:, None]*boundary[2]+((1-u)*v)[:, None]*boundary[3]
                road = json.loads((ROOT/'outputs/student_union_roadview_v1/roadview.json').read_text())['nodes']
                level = LevelPresentation(ROOT, run['presentation_config'])
                scene = o3d.t.geometry.RaycastingScene()
                scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
                for node in cfg['sign_ray_cleanup']['views']:
                    camera = next(o for o in road if o['session'] == name and o['node'] == node)
                    eye = level.warp(np.asarray(camera['world_optical'])[:3, 3][None], name)[0]
                    rays = np.c_[np.broadcast_to(eye, target.shape), target-eye].astype(np.float32)
                    hits = scene.list_intersections(o3d.core.Tensor(rays))
                    t = hits['t_hit'].numpy()
                    ids = hits['primitive_ids'].numpy()[(t > 0) & (t < 1.005)].astype(int)
                    remove[ids[eligible[ids]]] = True
                remove &= ~protected
        keep = ~remove
        filter_glb(old, folder/'clean_textured.glb', keep)
        mesh.remove_triangles_by_mask(remove)
        mesh.remove_unreferenced_vertices()
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(folder/'clean_structural.ply'), mesh)
        np.save(folder/'lower_ceiling_kept_faces.npy', keep)
        current = glb_triangle_positions(folder/'clean_textured.glb').astype(np.float32)
        assert np.array_equal(current, xyz[keep])
        assert np.array_equal(current, np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32))
        for height in [-1.55, 2.6]:
            assert not (remove & (abs(xyz[:, :, 2]-height).max(1) < 2e-5)).any()
        if remove.any():
            if cfg.get('sign_ray_cleanup'):
                assert not (remove & protected).any()
                assert ((normal_z[remove] > cfg['minimum_abs_normal_z']) | eligible[remove]).all()
            else:
                assert (normal_z[remove] > cfg['minimum_abs_normal_z']).all()
        ad, ab = unpack(old)
        bd, bb = unpack(folder/'clean_textured.glb')
        assert bb[:len(ab)] == ab, 'Filtering changed original vertices/UVs/photos'
        assert ad['images'] == bd['images'] and ad['materials'] == bd['materials']
        for path in (source/name).iterdir():
            if path.is_file() and path.name not in ['clean_textured.glb', 'clean_structural.ply']:
                shutil.copyfile(path, folder/path.name)
        audit[name] = {'retained_XYZ_exact': True, 'original_binary_UV_photos_materials_exact': True,
                       'floor_and_2F_levels_preserved': True, 'GLB_PLY_equal': True,
                       'removed_local_old_ceiling_or_sign_prism_faces': int(remove.sum()),
                       'retained_faces': int(keep.sum()), 'GLB_sha256': sha256_file(folder/'clean_textured.glb')}
        sessions[name] = {**spec, 'lower_ceiling_overlap_cleanup': audit[name]}
        cleanup[name] = {'before_faces': len(keep), 'after_faces': len(current), 'removed_faces': int(remove.sum()),
                         'glb_sha256': sha256_file(folder/'clean_textured.glb'), 'ply_sha256': sha256_file(folder/'clean_structural.ply')}
        for p in [old, source/name/'clean_structural.ply']:
            inputs[str(p.resolve())] = sha256_file(p)
        print('Local low-ceiling remnants: '+name+' / removed '+str(int(remove.sum())), flush=True)
    for path in source.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json']:
            shutil.copyfile(path, a.output/path.name)
    result = {**run, 'created_utc': utc_now(), 'sessions': sessions,
              'inputs_sha256': {**run['inputs_sha256'], **inputs},
              'lower_ceiling_overlap_cleanup': cfg,
              'previous_material_manifest': ceiling['source_manifest'],
              'floor_material_source_manifest': ceiling['source_manifest']}
    (a.output/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    digest = sha256_file(a.output/'manifest.json')
    (a.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': digest, 'sessions': cleanup}, indent=2)+'\n')
    (a.output/'geometry_audit.json').write_text(json.dumps({'bundle_manifest_sha256': digest, 'sessions': audit,
        'prior_clearance_audit': json.loads((source/'geometry_audit.json').read_text()), 'source_map_pose_changes': 0,
        'source_sha256': sha256_file(Path(__file__)), 'scope': 'Local old-ceiling overlap filtering after verified Z-only underside correction; visual acceptance separate.'}, indent=2)+'\n')
    for path, digest in inputs.items():
        assert sha256_file(Path(path)) == digest


if __name__ == '__main__':
    main()
