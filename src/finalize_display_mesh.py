"""Conservative whole-map display cleanup with retained-face and seam provenance."""
import argparse
import copy
import gc
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial import cKDTree

from audit_structural_delivery import glb_triangle_positions
from complete_service_slabs import mask_lookup
from extend_closed_ceiling_bodies import complete_floor_cells
from lift_display_shadows import append_blob
from optimize_session import sha256_file, utc_now
from refine_display_edges import accessor, protection, unpack, write_glb, KEY

ROOT = Path(__file__).resolve().parents[1]


def face_geometry(xyz):
    normals = np.cross(xyz[:, 1]-xyz[:, 0], xyz[:, 2]-xyz[:, 0])
    area = np.linalg.norm(normals, axis=1)
    return normals/np.maximum(area[:, None], 1e-20), area/2


def components(vertices, faces):
    import open3d as o3d
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
    labels, counts, areas = mesh.cluster_connected_triangles()
    return np.asarray(labels), np.asarray(counts), np.asarray(areas)


def prune_fragments(vertices, faces, protected, cfg):
    labels, counts, areas = components(vertices, faces)
    low = np.full((len(areas), 3), np.inf)
    high = np.full((len(areas), 3), -np.inf)
    xyz = vertices[faces]
    np.minimum.at(low, labels, xyz.min(1))
    np.maximum.at(high, labels, xyz.max(1))
    guarded = np.zeros(len(areas), bool)
    np.logical_or.at(guarded, labels, protected)
    small = (areas < cfg['minimum_fragment_area_m2']) & (np.linalg.norm(high-low, axis=1) < cfg['maximum_fragment_span_m']) & ~guarded
    return small[labels], {'components_before': len(areas), 'small_components_removed': int(small.sum()),
        'fragment_triangles_removed': int(counts[small].sum()), 'fragment_area_removed_m2': float(areas[small].sum()),
        'protected_components_retained': int(guarded.sum())}


def duplicate_keep(face_ids, priority):
    """Retain the highest-authority face for the same unoriented triangle."""
    key = np.sort(np.asarray(face_ids), axis=1)
    order = np.argsort(-np.asarray(priority), kind='stable')
    _, first = np.unique(key[order], axis=0, return_index=True)
    keep = np.zeros(len(key), bool)
    keep[order[first]] = True
    return keep


def samples_on_faces(xyz):
    return np.concatenate([xyz, xyz.mean(1)[:, None],
        (xyz[:, [0, 1, 2]]+xyz[:, [1, 2, 0]])/2], axis=1)


def covered_horizontal_remnants(xyz, protected, coverage, low, cell, height, band, cfg):
    normals, _ = face_geometry(xyz)
    possible = (~protected) & (np.max(abs(xyz[:, :, 2]-height), axis=1) < band)
    possible &= abs(normals[:, 2]) >= cfg['minimum_horizontal_abs_normal_z']
    ids = np.flatnonzero(possible)
    result = np.zeros(len(xyz), bool)
    if len(ids):
        sample = samples_on_faces(xyz[ids])
        hit = mask_lookup(coverage, low, cell, sample[:, :, :2].reshape(-1, 2)).reshape(len(ids), -1)
        result[ids] = hit.all(1)
    return result


def join_tiny_seams(vertices, faces, protected, cfg):
    """Mutual-nearest boundary pairs only; all accepted pairs move together."""
    initial = np.asarray(vertices, np.float32)
    result = initial.copy()
    if not len(faces) or not cfg['enable_seam_join']:
        return result, {'joined_boundary_pairs': 0}
    edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    edge, count = np.unique(edges, axis=0, return_counts=True)
    boundary = np.unique(edge[count == 1])
    normal, area = face_geometry(initial[faces])
    summed = np.zeros_like(initial)
    weights = np.zeros(len(initial))
    for column in range(3):
        np.add.at(summed, faces[:, column], normal*area[:, None])
        np.add.at(weights, faces[:, column], area)
    coherence = np.linalg.norm(summed, axis=1)/np.maximum(weights, 1e-20)
    summed /= np.maximum(np.linalg.norm(summed, axis=1, keepdims=True), 1e-20)
    guarded = np.zeros(len(initial), bool)
    guarded[np.unique(faces[protected])] = True
    candidates = boundary[(~guarded[boundary]) & (coherence[boundary] > .98)]
    if len(candidates) < 2:
        return result, {'joined_boundary_pairs': 0}
    distance, near = cKDTree(initial[candidates]).query(initial[candidates], k=2)
    other = near[:, 1]
    mutual = (other[other] == np.arange(len(other))) & (np.arange(len(other)) < other)
    a, b = candidates[mutual], candidates[other[mutual]]
    gap = initial[b]-initial[a]
    agree = abs(np.einsum('ij,ij->i', summed[a], summed[b])) > cfg['seam_normal_agreement']
    coplanar = abs(np.einsum('ij,ij->i', gap, summed[a])) < cfg['seam_plane_distance_m']
    close = (np.linalg.norm(gap, axis=1) <= cfg['seam_join_distance_m']) & (np.linalg.norm(gap, axis=1) > 2e-5)
    a, b = a[agree & coplanar & close], b[agree & coplanar & close]
    if not len(a):
        return result, {'joined_boundary_pairs': 0}
    # Do not weld an edge of the same connected component onto itself.
    labels, _, _ = components(initial, faces)
    vertex_component = np.full(len(initial), -1, int)
    for col in range(3):
        vertex_component[faces[:, col]] = labels
    separate = vertex_component[a] != vertex_component[b]
    a, b = a[separate], b[separate]
    middle = (initial[a].astype(float)+initial[b])/2
    result[a] = result[b] = middle
    group = np.full(len(initial), -1, int)
    group[a] = group[b] = np.arange(len(a))
    before = np.cross(initial[faces[:, 1]]-initial[faces[:, 0]], initial[faces[:, 2]]-initial[faces[:, 0]])
    accepted = np.ones(len(a), bool)
    for _ in range(4):
        after = np.cross(result[faces[:, 1]]-result[faces[:, 0]], result[faces[:, 2]]-result[faces[:, 0]])
        invalid = np.einsum('ij,ij->i', before, after) < np.einsum('ij,ij->i', before, before)*cfg['minimum_triangle_area_fraction']
        bad = np.unique(group[faces[invalid]])
        bad = bad[bad >= 0]
        if not len(bad):
            break
        accepted[bad] = False
        result[a[bad]], result[b[bad]] = initial[a[bad]], initial[b[bad]]
    assert np.array_equal(result[guarded], initial[guarded])
    return result, {'joined_boundary_pairs': int(accepted.sum()), 'rejected_pair_groups': int((~accepted).sum()),
        'maximum_vertex_motion_m': float(np.linalg.norm(result-initial, axis=1).max(initial=0))}


def primitive_faces(doc, payload):
    cursor = 0
    for mesh in doc['meshes']:
        for primitive in mesh['primitives']:
            position = accessor(doc, payload, primitive['attributes']['POSITION'])
            indices = accessor(doc, payload, primitive['indices']).ravel().astype(np.uint32) if 'indices' in primitive else np.arange(len(position), dtype=np.uint32)
            faces = indices.reshape(-1, 3)
            yield primitive, position, faces, cursor
            cursor += len(faces)


def write_clean_glb(source, target, keep, vertices, moved):
    doc, payload = unpack(source)
    key = np.ascontiguousarray(vertices).view(KEY).ravel()
    all_primitives = [(p, positions.copy(), faces, cursor)
                      for p, positions, faces, cursor in primitive_faces(doc, payload)]
    replacements = {}
    for primitive, positions, faces, start in all_primitives:
        index = primitive['attributes']['POSITION']
        if index in replacements:
            continue
        ids = np.searchsorted(key, np.ascontiguousarray(positions).view(KEY).ravel())
        valid = ids < len(vertices)
        valid[valid] &= np.all(vertices[ids[valid]] == positions[valid], axis=1)
        # Legacy accessors retain unused vertices after index filtering. Their
        # bytes are not active geometry and must not be invented or indexed past.
        changed = positions.copy().astype('<f4')
        changed[valid] = moved[ids[valid]]
        if np.array_equal(changed, positions):
            replacements[index] = index
            continue
        raw = changed.tobytes()
        updated = copy.deepcopy(doc['accessors'][index])
        updated.update(bufferView=append_blob(doc, payload, raw), byteOffset=0,
                       min=changed.min(0).tolist(), max=changed.max(0).tolist())
        doc['accessors'].append(updated)
        replacements[index] = len(doc['accessors'])-1
    for mesh in doc['meshes']:
        mesh['primitives'] = []
    for primitive, positions, faces, start in all_primitives:
        selected = keep[start:start+len(faces)]
        if not selected.any():
            continue
        primitive['attributes']['POSITION'] = replacements[primitive['attributes']['POSITION']]
        if not selected.all():
            values = np.ascontiguousarray(faces[selected].ravel(), dtype='<u4')
            doc['accessors'].append({'bufferView': append_blob(doc, payload, values.tobytes()),
                'componentType': 5125, 'count': len(values), 'type': 'SCALAR'})
            primitive['indices'] = len(doc['accessors'])-1
        doc['meshes'][0]['primitives'].append(primitive)
    # Original image/colour/UV buffers and materials are never altered.
    write_glb(target, doc, payload)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/final_mesh_finish.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    base = ROOT/cfg['source_bundle']
    run = json.loads((base/'manifest.json').read_text())
    objects = json.loads((base/'semantic_objects.json').read_text())['objects']
    q = np.load(base/'closure_plan.npz')
    data, wall_targets = {}, []
    import open3d as o3d
    args.output.mkdir(parents=True, exist_ok=False)
    for name, spec in run['sessions'].items():
        xyz = glb_triangle_positions(base/name/'clean_textured.glb').copy()
        vertices, inverse = np.unique(xyz.reshape(-1, 3), axis=0, return_inverse=True)
        faces = inverse.reshape(-1, 3)
        normals, area = face_geometry(xyz)
        semantic_v = protection(vertices, name, {**run, 'edge_cfg': cfg}, objects)
        semantic = semantic_v[faces].any(1)
        fixed = np.zeros(len(xyz), bool)
        for height in [-1.55, 2.60]:
            fixed |= np.max(abs(xyz[:, :, 2]-height), axis=1) < 2e-5
        fixed[spec['closure_retained_triangles']:] = True
        protected = semantic | fixed
        raw = np.zeros(len(xyz), bool)
        doc, payload = unpack(base/name/'clean_textured.glb')
        for primitive, positions, part, start in primitive_faces(doc, payload):
            end = start+len(part)
            material = doc['materials'][primitive['material']].get('pbrMetallicRoughness', {})
            textured = 'baseColorTexture' in material
            raw[start:end] = not textured
            if not textured or len(part) < cfg['minimum_wall_chart_faces'] or area[start:end].sum() < cfg['minimum_wall_chart_area_m2']:
                continue
            sample = xyz[start:end]
            n = normals[start:end]
            ref = n[np.argmax(area[start:end])]
            if abs(ref[2]) > .2 or np.quantile(abs(n@ref), .05) < .995:
                continue
            centre = sample.reshape(-1, 3).mean(0)
            if np.quantile(abs((sample.reshape(-1, 3)-centre)@ref), .9) > cfg['wall_plane_p90_m']:
                continue
            wall_targets.append(sample.copy())
        del payload, doc
        fragment, report = prune_fragments(vertices, faces, protected, cfg)
        priority = np.full(len(faces), {'floor2': 50, 'lobby': 40, 'outside': 30, 'bridge_12_sub': 20, 'bridge_out1f': 10}[name], float)
        priority += fixed*200+semantic*500+(~raw)*100
        if name == 'lobby':
            priority += (np.max(abs(xyz[:, :, 2]+1.55), axis=1) < 2e-5)*100
        data[name] = dict(xyz=xyz, vertices=vertices, faces=faces, semantic=semantic,
                          fixed=fixed, protected=protected, raw=raw, keep=~fragment, priority=priority, report=report)
        print('Loaded cleanup geometry: '+name+' / '+str(report), flush=True)
        gc.collect()
    # Coverage of existing fixed horizontal surfaces; no new footprints are filled.
    floor_masks = {}
    for height in [-1.55, 2.60]:
        mask = np.zeros(q['body'].shape, np.uint8)
        for d in data.values():
            bits, _ = complete_floor_cells(d['xyz'], q['low'], mask.shape, float(q['cell']), height)
            mask |= bits
        floor_masks[height] = mask == 3
    wall_scene = o3d.t.geometry.RaycastingScene(nthreads=2)
    if wall_targets:
        target = np.concatenate(wall_targets)
        wall_scene.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(target.reshape(-1, 3)),
            o3d.core.Tensor(np.arange(target.size//3).reshape(-1, 3).astype(np.uint32))))
    for name, d in data.items():
        xyz = d['xyz']
        redundant = np.zeros(len(xyz), bool)
        if name != 'outside':
            for height, mask in floor_masks.items():
                redundant |= covered_horizontal_remnants(xyz, d['protected'], mask, q['low'], float(q['cell']), height, cfg['floor_remnant_band_m'], cfg)
            for height, mask, band in [(float(q['roof_height']), q['roof'], cfg['roof_remnant_band_m']),
                                       (run['ceiling_expansion_and_closure']['config']['lower_extension_height_m'], q['body'], cfg['floor_remnant_band_m'])]:
                redundant |= covered_horizontal_remnants(xyz, d['protected'], mask, q['low'], float(q['cell']), height, band, cfg)
        normal, _ = face_geometry(xyz)
        candidates = np.flatnonzero(d['raw'] & ~d['protected'] & d['keep'] & (abs(normal[:, 2]) < .4))
        covered_wall = np.zeros(len(xyz), bool)
        if wall_targets:
            for first in range(0, len(candidates), 20000):
                ids = candidates[first:first+20000]
                samples = samples_on_faces(xyz[ids])
                closest = wall_scene.compute_closest_points(o3d.core.Tensor(samples.reshape(-1, 3)))
                distance = np.linalg.norm(closest['points'].numpy().reshape(samples.shape)-samples, axis=2)
                target_normals = closest['primitive_normals'].numpy().reshape(samples.shape)
                agree = abs(np.einsum('ijk,ik->ij', target_normals, normal[ids]))
                covered_wall[ids] = ((distance < cfg['wall_replacement_distance_m']) & (agree > cfg['wall_normal_agreement'])).all(1)
        redundant |= covered_wall
        d['keep'] &= ~redundant
        d['report']['covered_old_surface_faces_removed'] = int(redundant.sum())
        d['report']['covered_old_wall_faces_removed'] = int(covered_wall.sum())
        print('Verified covered remnants: '+name+' / '+str(int(redundant.sum())), flush=True)
    # Quantized keys are analysis only: no vertex is globally snapped to a grid.
    names = list(data)
    sizes = [len(data[n]['vertices']) for n in names]
    _, global_ids = np.unique(np.rint(np.concatenate([data[n]['vertices'] for n in names])/cfg['duplicate_weld_m']).astype(np.int64), axis=0, return_inverse=True)
    mapping = np.split(global_ids, np.cumsum(sizes)[:-1])
    faces = np.concatenate([mapping[i][data[n]['faces']] for i, n in enumerate(names)])
    priority = np.concatenate([data[n]['priority'] for n in names])
    keep = np.concatenate([data[n]['keep'] for n in names])
    duplicate = np.ones(len(faces), bool)
    duplicate[keep] = duplicate_keep(faces[keep], priority[keep])
    duplicate_parts = np.split(duplicate, np.cumsum([len(data[n]['faces']) for n in names])[:-1])
    sessions, cleanup = {}, {}
    for name, not_duplicate in zip(names, duplicate_parts):
        d = data[name]
        d['report']['duplicate_triangles_removed'] = int((d['keep'] & ~not_duplicate).sum())
        d['keep'] &= not_duplicate
        retained_ids = np.flatnonzero(d['keep'])
        post_remove, post_report = prune_fragments(d['vertices'], d['faces'][d['keep']], d['protected'][d['keep']], cfg)
        d['keep'][retained_ids[post_remove]] = False
        d['report']['post_filter_fragment_cleanup'] = post_report
        retained = d['faces'][d['keep']]
        moved, seam_report = join_tiny_seams(d['vertices'], retained, d['protected'][d['keep']], cfg)
        out = args.output/name
        out.mkdir()
        write_clean_glb(base/name/'clean_textured.glb', out/'clean_textured.glb', d['keep'], d['vertices'], moved)
        mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(moved), o3d.utility.Vector3iVector(retained))
        mesh.remove_unreferenced_vertices()
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(out/'clean_structural.ply'), mesh)
        np.savez_compressed(out/'finish_provenance.npz', retained_source_faces=np.flatnonzero(d['keep']), source_vertices=d['vertices'], moved_vertices=moved,
                            semantic_source_faces=d['semantic'], fixed_source_faces=d['fixed'])
        welded, vertex_ids = np.unique(moved, axis=0, return_inverse=True)
        labels, counts, areas = components(welded, vertex_ids[retained])
        report = {**d['report'], **seam_report, 'triangles_before': len(d['xyz']), 'triangles_after': len(retained),
                  'components_after': len(areas), 'original_texture_colour_changes': 0,
                  'new_large_surfaces': 0, 'source_pose_changes': 0}
        for path in (base/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply', 'clean_textured.glb']:
                shutil.copyfile(path, out/path.name)
        sessions[name] = {**run['sessions'][name], 'final_mesh_finish_report': report}
        cleanup[name] = {'glb_sha256': sha256_file(out/'clean_textured.glb'), 'ply_sha256': sha256_file(out/'clean_structural.ply')}
        print('Final mesh candidate: '+name+' / '+str(report), flush=True)
    for path in base.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json', 'shadow_audit.json']:
            shutil.copyfile(path, args.output/path.name)
    result = {**run, 'created_utc': utc_now(), 'sessions': sessions, 'final_mesh_finish': cfg,
              'mesh_finish_source_manifest': str((base/'manifest.json').resolve()),
              'previous_material_manifest': str((base/'manifest.json').resolve()),
              'inputs_sha256': {**run['inputs_sha256'], **{str(p.resolve()): sha256_file(p) for p in
                 [base/'manifest.json', args.config, Path(__file__)]}}, 'scope': cfg['scope']}
    (args.output/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    (args.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': sha256_file(args.output/'manifest.json'), 'sessions': cleanup}, indent=2)+'\n')


if __name__ == '__main__':
    main()
