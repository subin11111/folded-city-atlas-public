#!/usr/bin/env python3
"""Joint outer-contour finish: move matching floor/ceiling/fascia columns together."""
import argparse
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
from scipy import ndimage

from audit_structural_delivery import glb_triangle_positions
from complete_service_slabs import mask_lookup
from extend_closed_ceiling_bodies import complete_floor_cells
from optimize_session import sha256_file, utc_now
from refine_display_edges import KEY, accessor, protection, trim_boundary_teeth, unpack, write_glb

ROOT = Path(__file__).resolve().parents[1]


def boundary_loops(mask):
    """Directed cell edges: positive outer rings, negative hole rings."""
    mask = np.pad(np.asarray(mask, bool), 1)
    edges = []
    for y, x in zip(*np.where(mask[1:-1, 1:-1])):
        j, i = y+1, x+1
        for exposed, a, b in [(not mask[j-1, i], (x, y), (x+1, y)),
                              (not mask[j, i+1], (x+1, y), (x+1, y+1)),
                              (not mask[j+1, i], (x+1, y+1), (x, y+1)),
                              (not mask[j, i-1], (x, y+1), (x, y))]:
            if exposed:
                edges.append((a, b))
    outgoing, incoming = {}, {}
    for a, b in edges:
        outgoing.setdefault(a, []).append(b)
        incoming[b] = incoming.get(b, 0)+1
    remaining = set(outgoing)
    result = []
    while remaining:
        start = min(remaining)
        ring, point = [], start
        while point in remaining and len(outgoing[point]) == 1 and incoming[point] == 1:
            remaining.remove(point); ring.append(point)
            point = outgoing[point][0]
        if point == start and len(ring) >= 4:
            result.append(np.asarray(ring, int))
        else:
            remaining.discard(start)
    return result


def signed_area(points):
    return .5*np.sum(points[:, 0]*np.roll(points[:, 1], -1)-points[:, 1]*np.roll(points[:, 0], -1))


def contour_field(masks, low, cell, cfg):
    shape = np.array(masks[0].shape)+1
    total = np.zeros((*shape, 2), np.float64)
    count = np.zeros(shape, np.int32)
    pinned = np.zeros(shape, bool)
    reports = []
    for mask in masks:
        for ring in boundary_loops(mask):
            x, y = ring.T
            if signed_area(ring) < 0:
                pinned[y, x] = True  # Never round away a genuine hole.
                continue
            points = low+ring*cell
            coarse = cv2.approxPolyDP(points.astype(np.float32), cfg['contour_simplification_m'], True).reshape(-1, 2)
            if len(coarse) < 3:
                pinned[y, x] = True
                continue
            nearest = np.full(len(points), np.inf)
            target = points.copy()
            corners = np.zeros(len(points), bool)
            for a, b in zip(coarse, np.roll(coarse, -1, axis=0)):
                segment = b-a
                fraction = np.clip((points-a)@segment/max(segment@segment, 1e-12), 0, 1)
                projection = a+fraction[:, None]*segment
                distance = np.linalg.norm(projection-points, axis=1)
                use = distance < nearest
                target[use], nearest[use] = projection[use], distance[use]
                corners |= np.linalg.norm(points-a, axis=1) < 2e-5
            delta = target-points
            length = np.linalg.norm(delta, axis=1)
            delta *= np.minimum(1., cfg['maximum_shared_XY_motion_m']/np.maximum(length, 1e-12))[:, None]
            total[y, x] += delta
            count[y, x] += 1
            pinned[y[corners], x[corners]] = True
            reports.append({'ring_vertices': len(ring), 'retained_major_corners': len(coarse)})
    total /= np.maximum(count[:, :, None], 1)
    total[pinned] = 0
    return total.astype(np.float32), pinned, reports


def grid_indices(vertices, low, cell, shape):
    q = (vertices[:, :2]-low)/cell
    grid = np.rint(q).astype(int)
    valid = np.max(abs(q-grid), axis=1) < 4e-4
    x, y = grid.T
    valid &= (x >= 0) & (x < shape[1]) & (y >= 0) & (y < shape[0])
    return grid, valid


def apply_column_field(vertices, raw_target, low, cell, field):
    result = raw_target.copy()
    grid, valid = grid_indices(vertices, low, cell, field.shape[:2])
    x, y = grid[valid].T
    result[valid, :2] += field[y, x]
    return result


def write_position_only_glb(source, target, vertices, moved):
    doc, payload = unpack(source)
    keys = np.ascontiguousarray(vertices).view(KEY).ravel()
    done = set()
    for mesh in doc['meshes']:
        for primitive in mesh['primitives']:
            index = primitive['attributes']['POSITION']
            if index in done:
                continue
            values = accessor(doc, payload, index)
            ids = np.searchsorted(keys, np.ascontiguousarray(values).view(KEY).ravel())
            valid = ids < len(vertices)
            valid[valid] &= np.all(vertices[ids[valid]] == values[valid], axis=1)
            values[valid] = moved[ids[valid]]
            doc['accessors'][index]['min'] = values.min(0).tolist()
            doc['accessors'][index]['max'] = values.max(0).tolist()
            done.add(index)
    write_glb(target, doc, payload)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'config/shared_contour_finish.json')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    cfg = json.loads(a.config.read_text())
    source = ROOT/cfg['source_bundle']
    run = json.loads((source/'manifest.json').read_text())
    objects = json.loads((source/'semantic_objects.json').read_text())['objects']
    q = np.load(source/'closure_plan.npz')
    low, cell = q['low'], float(q['cell'])
    xyzs = {name: glb_triangle_positions(source/name/'clean_textured.glb') for name in run['sessions']}
    lower_coverage = np.zeros(q['body'].shape, np.uint8)
    for name in ['lobby', 'floor2', 'bridge_12_sub', 'bridge_out1f']:
        bits, _ = complete_floor_cells(xyzs[name], low, q['body'].shape, cell, -1.55)
        lower_coverage |= bits
    field, pinned, rings = contour_field([q['actual_top'], q['body'], q['roof'], lower_coverage == 3], low, cell, cfg)
    # Exclude both sides of openings, rather than only the missing cells.
    forbidden = q['void'] | q['stair'] | q['sign_guard']
    forbidden = ndimage.binary_dilation(forbidden, iterations=int(np.ceil(cfg['opening_guard_m']/cell)))
    yy, xx = np.indices(field.shape[:2])
    xy = low+np.c_[xx.ravel(), yy.ravel()]*cell
    pinned |= mask_lookup(forbidden, low, cell, xy).reshape(pinned.shape)
    data = {}
    for name, xyz in xyzs.items():
        vertices, ids = np.unique(xyz.reshape(-1, 3), axis=0, return_inverse=True)
        triangles = ids.reshape(-1, 3)
        semantic = protection(vertices, name, {**run, 'edge_cfg': cfg}, objects)
        grid, valid = grid_indices(vertices, low, cell, field.shape[:2])
        x, y = grid[valid & semantic].T
        pinned[y, x] = True
        data[name] = {'source': vertices, 'triangles': triangles, 'semantic': semantic}
    field[pinned] = 0
    for name, d in data.items():
        v, tri = d['source'], d['triangles']
        horizontal = np.ptp(v[tri, 2], axis=1) < 1e-5
        protect = d['semantic'].copy()
        protect[np.unique(tri[horizontal])] = True
        grid, valid = grid_indices(v, low, cell, field.shape[:2])
        active = np.zeros(len(v), bool)
        x, y = grid[valid].T
        active[valid] = np.linalg.norm(field[y, x], axis=1) > 1e-7
        protect |= active
        d['raw_target'], d['raw_report'] = trim_boundary_teeth(v, tri, protect, cfg)
        d['horizontal'] = horizontal
    # A rollback is a whole XY column across every session/elevation. A local
    # per-triangle rollback would reopen the very cavity this stage protects.
    for _ in range(12):
        rejected = 0
        for d in data.values():
            v, tri = d['source'], d['triangles']
            target = apply_column_field(v, d['raw_target'], low, cell, field)
            old = v[tri]; new = target[tri]
            normal = np.cross(old[:, 1]-old[:, 0], old[:, 2]-old[:, 0])
            changed = np.cross(new[:, 1]-new[:, 0], new[:, 2]-new[:, 0])
            invalid = np.sum(normal*changed, axis=1) < np.sum(normal*normal, axis=1)*cfg['minimum_triangle_area_fraction']
            if not invalid.any():
                continue
            bad = np.unique(tri[invalid])
            grid, valid = grid_indices(v[bad], low, cell, field.shape[:2])
            x, y = grid[valid].T
            rejected += int((np.linalg.norm(field[y, x], axis=1) > 0).sum())
            field[y, x] = 0
            d['raw_target'][bad] = v[bad]
        if not rejected:
            break
    a.output.mkdir(parents=True, exist_ok=False)
    np.savez_compressed(a.output/'shared_column_field.npz', low=low, cell=cell, displacement=field, pinned=pinned)
    sessions, cleanup = {}, {}
    import open3d as o3d
    for name, spec in run['sessions'].items():
        folder = a.output/name
        folder.mkdir()
        d = data[name]; v, tri = d['source'], d['triangles']
        target = apply_column_field(v, d['raw_target'], low, cell, field)
        assert np.array_equal(target[d['semantic']], v[d['semantic']])
        assert np.array_equal(target[tri[d['horizontal']], 2], v[tri[d['horizontal']], 2])
        xyz, changed = v[tri], target[tri]
        old_normal = np.cross(xyz[:, 1]-xyz[:, 0], xyz[:, 2]-xyz[:, 0])
        new_normal = np.cross(changed[:, 1]-changed[:, 0], changed[:, 2]-changed[:, 0])
        assert (np.sum(old_normal*new_normal, axis=1) >= np.sum(old_normal*old_normal, axis=1)*(cfg['minimum_triangle_area_fraction']-1e-5)).all()
        write_position_only_glb(source/name/'clean_textured.glb', folder/'clean_textured.glb', v, target)
        mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(target), o3d.utility.Vector3iVector(tri))
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(folder/'clean_structural.ply'), mesh)
        assert np.array_equal(changed, glb_triangle_positions(folder/'clean_textured.glb'))
        np.savez_compressed(folder/'contour_displacements.npz', source=v, target=target, semantic=d['semantic'])
        for path in (source/name).iterdir():
            if path.is_file() and path.name not in ['clean_textured.glb', 'clean_structural.ply']:
                shutil.copyfile(path, folder/path.name)
        motion = np.linalg.norm(target-v, axis=1)
        report = {'moved_vertices': int((motion > 1e-7).sum()), 'maximum_motion_m': float(motion.max(initial=0)),
                  'raw_boundary_trim': d['raw_report'], 'material_changes': 0, 'added_or_removed_faces': 0}
        sessions[name] = {**spec, 'shared_contour_report': report,
                          'bounds_min_m': mesh.get_min_bound().tolist(), 'bounds_max_m': mesh.get_max_bound().tolist()}
        cleanup[name] = {'ply_sha256': sha256_file(folder/'clean_structural.ply'), 'glb_sha256': sha256_file(folder/'clean_textured.glb')}
        print(json.dumps({'session': name, **report}), flush=True)
    for path in source.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json']:
            shutil.copyfile(path, a.output/path.name)
    inputs = {str(path.resolve()): sha256_file(path) for path in [Path(__file__), a.config, source/'manifest.json',
              ROOT/'src/refine_display_edges.py', source/'closure_plan.npz', source/'semantic_objects.json']}
    result = {**run, 'created_utc': utc_now(), 'sessions': sessions,
              'inputs_sha256': {**run['inputs_sha256'], **inputs},
              'shared_contour_finish': {'config': cfg, 'outer_rings': rings,
                                        'moved_shared_XY_columns': int((np.linalg.norm(field, axis=2) > 1e-7).sum())},
              'contour_source_manifest': str((source/'manifest.json').resolve()),
              'previous_material_manifest': str((source/'manifest.json').resolve()),
              'floor_material_source_manifest': str((source/'manifest.json').resolve()), 'scope': cfg['scope']}
    (a.output/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    (a.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': sha256_file(a.output/'manifest.json'), 'sessions': cleanup}, indent=2)+'\n')


if __name__ == '__main__':
    main()
