#!/usr/bin/env python3
"""Bounded convex boundary-tooth trimming and narrow texture antialiasing."""
import argparse
import copy
import hashlib
import io
import json
from pathlib import Path
import shutil
import struct

import cv2
import numpy as np
from PIL import Image

from audit_structural_delivery import glb_triangle_positions
from optimize_session import sha256_file, utc_now
from presentation_geometry import LevelPresentation
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

ROOT = Path(__file__).resolve().parents[1]
TYPES = {5126: '<f4', 5125: '<u4', 5123: '<u2', 5121: 'u1'}
WIDTHS = {'VEC3': 3, 'VEC2': 2, 'SCALAR': 1}
KEY = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])


def unpack(path):
    raw = Path(path).read_bytes()
    size = struct.unpack_from('<I', raw, 12)[0]
    return json.loads(raw[20:20+size]), bytearray(raw[28+size:])


def accessor(doc, payload, index):
    row = doc['accessors'][index]
    view = doc['bufferViews'][row['bufferView']]
    dtype = np.dtype(TYPES[row['componentType']])
    width = WIDTHS[row['type']]
    return np.ndarray((row['count'], width), dtype=dtype, buffer=payload,
                      offset=view.get('byteOffset', 0)+row.get('byteOffset', 0),
                      strides=(view.get('byteStride', dtype.itemsize*width), dtype.itemsize))


def write_glb(path, doc, payload):
    payload.extend(b'\0'*(-len(payload) % 4))
    doc['buffers'][0]['byteLength'] = len(payload)
    encoded = json.dumps(doc, separators=(',', ':')).encode()
    encoded += b' '*(-len(encoded) % 4)
    with Path(path).open('xb') as stream:
        stream.write(struct.pack('<4sII', b'glTF', 2, 28+len(encoded)+len(payload)))
        stream.write(struct.pack('<I4s', len(encoded), b'JSON'))
        stream.write(encoded)
        stream.write(struct.pack('<I4s', len(payload), b'BIN\0'))
        stream.write(payload)


def trim_boundary_teeth(vertices, triangles, protected, cfg):
    """Move only inward convex contour vertices on a locally planar surface.

    Material/UV duplicate vertices must be welded by exact XYZ before calling.
    Strong 3-D corners and protected semantic/stair vertices never move.
    """
    initial = np.asarray(vertices, np.float32)
    result = initial.copy()
    edges = np.sort(triangles[:, [0, 1, 1, 2, 2, 0]].reshape(-1, 2), axis=1)
    edges, counts = np.unique(edges, axis=0, return_counts=True)
    boundary = edges[counts == 1]
    directed = np.concatenate([boundary, boundary[:, ::-1]])
    degree = np.bincount(directed[:, 0], minlength=len(result))
    directed = directed[np.argsort(directed[:, 0], kind='stable')]
    ends = np.r_[0, np.cumsum(degree)]
    candidates = np.flatnonzero((degree == 2) & ~protected)
    if not len(candidates):
        return result, {'boundary_vertices': int(np.sum(degree > 0)), 'moved_vertices': 0}
    neighbors = np.c_[directed[ends[candidates], 1], directed[ends[candidates]+1, 1]]
    xyz = initial[triangles]
    normals = np.cross(xyz[:, 1]-xyz[:, 0], xyz[:, 2]-xyz[:, 0])
    area = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(area[:, None], 1e-12)
    normal_sum = np.zeros_like(result)
    incident_count = np.zeros(len(result), np.int32)
    centroid_sum = np.zeros_like(result)
    centres = xyz.mean(1)
    for column in range(3):
        np.add.at(normal_sum, triangles[:, column], normals)
        np.add.at(incident_count, triangles[:, column], 1)
        np.add.at(centroid_sum, triangles[:, column], centres)
    coherence = np.linalg.norm(normal_sum, axis=1)/np.maximum(incident_count, 1)
    planar = coherence[candidates] >= cfg['minimum_normal_coherence']
    centre = centroid_sum/np.maximum(incident_count[:, None], 1)
    for _ in range(cfg['iterations']):
        a, b = result[neighbors[:, 0]], result[neighbors[:, 1]]
        p = result[candidates]
        da, db = a-p, b-p
        la, lb = np.linalg.norm(da, axis=1), np.linalg.norm(db, axis=1)
        cosine = np.sum(da*db, axis=1)/np.maximum(la*lb, 1e-12)
        step = ((a+b)*.5-p)*cfg['boundary_step']
        # Requiring motion toward the incident surface avoids filling concave
        # notches or growing a roof beyond its observed footprint.
        inward = np.sum(step*(centre[candidates]-p), axis=1) > 1e-8
        eligible = (planar & inward & (np.maximum(la, lb) <= cfg['maximum_boundary_leg_m'])
                    & (np.maximum(la, lb) < 4*np.maximum(np.minimum(la, lb), 1e-7))
                    & (cosine > np.cos(np.deg2rad(150))) & (cosine < np.cos(np.deg2rad(15))))
        proposed = p+step
        total = proposed-initial[candidates]
        distance = np.linalg.norm(total, axis=1)
        total *= np.minimum(1., cfg['maximum_total_motion_m']/np.maximum(distance, 1e-12))[:, None]
        result[candidates[eligible]] = (initial[candidates]+total)[eligible]
    # Reject any local flip/degeneracy rather than deleting a triangle.
    for _ in range(4):
        changed = result[triangles]
        new_normal = np.cross(changed[:, 1]-changed[:, 0], changed[:, 2]-changed[:, 0])
        invalid = np.sum(new_normal*normals, axis=1) < area*.25
        if not invalid.any():
            break
        result[np.unique(triangles[invalid])] = initial[np.unique(triangles[invalid])]
    displacement = np.linalg.norm(result-initial, axis=1)
    assert np.array_equal(result[protected], initial[protected])
    assert displacement.max(initial=0) <= cfg['maximum_total_motion_m']+2e-6
    return result, {'boundary_vertices': int(np.sum(degree > 0)),
                    'protected_vertices': int(protected.sum()),
                    'moved_vertices': int(np.sum(displacement > 1e-7)),
                    'maximum_actual_motion_m': float(displacement.max(initial=0)),
                    'new_faces': 0, 'removed_faces': 0}


def soften_texture_edges(rgb, cfg):
    """Antialias a narrow gradient band, without exposure/colour normalization."""
    linear = srgb_to_linear(rgb/255.).astype(np.float32)
    luminance = linear @ np.array([.2126, .7152, .0722], np.float32)
    gx = cv2.Sobel(luminance, cv2.CV_32F, 1, 0, ksize=3)/8
    gy = cv2.Sobel(luminance, cv2.CV_32F, 0, 1, ksize=3)/8
    gradient = np.hypot(gx, gy)
    weight = np.clip((gradient-.025)/.06, 0, 1)*cfg['texture_edge_blend']
    blurred = cv2.GaussianBlur(linear, (0, 0), cfg['texture_sigma_px'])
    delta = np.clip((blurred-linear)*weight[:, :, None],
                    -cfg['maximum_texture_linear_change'], cfg['maximum_texture_linear_change'])
    result = np.uint8(linear_to_srgb(np.clip(linear+delta, 0, 1))*255+.5)
    limit = cfg.get('maximum_texture_srgb_change', 16)
    result = np.clip(result.astype(np.int16), rgb.astype(np.int16)-limit,
                     rgb.astype(np.int16)+limit).clip(0, 255).astype(np.uint8)
    # Exact byte preservation outside the selected edge band.
    result[weight == 0] = rgb[weight == 0]
    return result, {'changed_pixels': int(np.any(result != rgb, axis=2).sum()),
                    'outside_edge_band_exact': bool(np.array_equal(result[weight == 0], rgb[weight == 0]))}


def protection(vertices, name, run, objects):
    protected = np.zeros(len(vertices), bool)
    margin = run['edge_cfg']['semantic_guard_m']
    for obj in objects:
        if obj['session'] != name or not obj.get('geometry', {}).get('accepted'):
            continue
        boundary = np.asarray(obj['geometry']['world_boundary'])
        low, high = boundary.min(0)-margin, boundary.max(0)+margin
        protected |= ((vertices >= low) & (vertices <= high)).all(1)
    # Existing photographed stair guard boxes are in source coordinates.
    prior = json.loads((ROOT/'outputs/student_union_verified_wall_delivery_v2/manifest.json').read_text())
    guard = prior['sessions'][name].get('flat_stair_guard', {})
    if guard.get('centres'):
        c = LevelPresentation(ROOT, run['presentation_config']).warp(np.asarray(guard['centres']), name)
        half = np.asarray(guard['half'])+[.12, .12, .5]
        for point in c:
            protected |= (abs(vertices-point) <= half).all(1)
    return protected


def finish_glb(source, target, original, changed, protected_hashes, cfg, image_dir):
    doc, payload = unpack(source)
    keys = np.ascontiguousarray(original).view(KEY).ravel()
    done = set()
    for mesh in doc['meshes']:
        for primitive in mesh['primitives']:
            index = primitive['attributes']['POSITION']
            if index in done:
                continue
            values = accessor(doc, payload, index)
            ids = np.searchsorted(keys, np.ascontiguousarray(values).view(KEY).ravel())
            valid = ids < len(original)
            valid[valid] &= np.all(original[ids[valid]] == values[valid], axis=1)
            values[valid] = changed[ids[valid]]
            doc['accessors'][index]['min'] = values.min(0).tolist()
            doc['accessors'][index]['max'] = values.max(0).tolist()
            done.add(index)
    del values  # Release the writable bytearray view before appending PNGs.
    active = set()
    for mesh in doc['meshes']:
        for primitive in mesh['primitives']:
            material = doc['materials'][primitive['material']]
            texture = material.get('pbrMetallicRoughness', {}).get('baseColorTexture', {}).get('index')
            if texture is not None:
                active.add(doc['textures'][texture]['source'])
    reports = []
    for index in sorted(active):
        image = doc['images'][index]
        view = doc['bufferViews'][image['bufferView']]
        start = view.get('byteOffset', 0)
        blob = bytes(payload[start:start+view['byteLength']])
        digest = hashlib.sha256(blob).hexdigest()
        if digest in protected_hashes:
            reports.append({'image': index, 'source_sha256': digest, 'protected_photo_exact': True})
            continue
        rgb = np.asarray(Image.open(io.BytesIO(blob)).convert('RGB'))
        if min(rgb.shape[:2]) < 32:
            continue
        result, report = soften_texture_edges(rgb, cfg)
        if not report['changed_pixels']:
            continue
        path = image_dir/f'image_{index:04d}.png'
        Image.fromarray(result).save(path)
        data = path.read_bytes()
        payload.extend(b'\0'*(-len(payload) % 4))
        offset = len(payload)
        payload.extend(data)
        doc['bufferViews'].append({'buffer': 0, 'byteOffset': offset, 'byteLength': len(data)})
        image['bufferView'] = len(doc['bufferViews'])-1
        image['mimeType'] = 'image/png'
        reports.append({'image': index, 'source_sha256': digest, 'output_sha256': sha256_file(path), **report})
    write_glb(target, doc, payload)
    return reports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/display_edge_finish.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    source = ROOT/cfg['source_bundle']
    run = json.loads((source/'manifest.json').read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    objects = json.loads((source/'semantic_objects.json').read_text())['objects']
    inputs = {str(p.resolve()): sha256_file(p) for p in [Path(__file__), args.config, source/'manifest.json', source/'semantic_objects.json', ROOT/'outputs/student_union_verified_wall_delivery_v2/manifest.json']}
    protected_hashes = {sha256_file(p) for n in run['sessions'] for p in (source/n).glob('*.png') if p.name != 'continuous_2f_roof.png'}
    sessions, cleanup = {}, {}
    import open3d as o3d
    for name, spec in run['sessions'].items():
        print('Edge-only finish: '+name, flush=True)
        folder = args.output/name
        folder.mkdir()
        image_dir = folder/'edge_textures'
        image_dir.mkdir()
        old = source/name/'clean_textured.glb'
        xyz = glb_triangle_positions(old).astype(np.float32)
        vertices, ids = np.unique(xyz.reshape(-1, 3), axis=0, return_inverse=True)
        triangles = ids.reshape(-1, 3)
        protected = protection(vertices, name, {**run, 'edge_cfg': cfg}, objects)
        # Separately tiled/overlapping horizontal sheets may not share contour
        # tessellation. Pulling their border back opens triangular cracks even
        # when each individual sheet remains manifold. Freeze those whole
        # horizontal faces, not merely their Z coordinates (rejected v16).
        horizontal = np.ptp(xyz[:, :, 2], axis=1) < 1e-5
        protected[np.unique(triangles[horizontal])] = True
        changed, report = trim_boundary_teeth(vertices, triangles, protected, cfg)
        np.savez_compressed(folder/'boundary_displacements.npz', source=vertices, target=changed, protected=protected)
        images = finish_glb(old, folder/'clean_textured.glb', vertices, changed, protected_hashes, cfg, image_dir)
        mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(changed), o3d.utility.Vector3iVector(triangles))
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(folder/'clean_structural.ply'), mesh)
        assert np.array_equal(changed[triangles], glb_triangle_positions(folder/'clean_textured.glb'))
        for path in (source/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply', 'clean_textured.glb']:
                shutil.copyfile(path, folder/path.name)
        sessions[name] = {**spec, 'edge_finish_report': report, 'texture_edge_reports': images,
                          'bounds_min_m': changed.min(0).tolist(), 'bounds_max_m': changed.max(0).tolist()}
        cleanup[name] = {'before_faces': len(triangles), 'after_faces': len(triangles), 'removed_faces': 0,
                         'ply_sha256': sha256_file(folder/'clean_structural.ply'),
                         'glb_sha256': sha256_file(folder/'clean_textured.glb')}
        for p in [old, source/name/'clean_structural.ply']:
            inputs[str(p.resolve())] = sha256_file(p)
        print(json.dumps({'session': name, **report}), flush=True)
    for path in source.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json']:
            shutil.copyfile(path, args.output/path.name)
    result = {**run, 'created_utc': utc_now(), 'sessions': sessions,
              'inputs_sha256': {**run['inputs_sha256'], **inputs},
              'edge_finish': True, 'edge_finish_config': cfg,
              'edge_finish_source_manifest': str((source/'manifest.json').resolve()),
              'previous_material_manifest': str((source/'manifest.json').resolve()),
              'floor_material_source_manifest': str((source/'manifest.json').resolve()),
              'scope': cfg['scope']}
    (args.output/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    (args.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': sha256_file(args.output/'manifest.json'), 'sessions': cleanup}, indent=2)+'\n')
    for p, digest in result['inputs_sha256'].items():
        assert sha256_file(Path(p)) == digest, 'Input changed: '+p


if __name__ == '__main__':
    main()
