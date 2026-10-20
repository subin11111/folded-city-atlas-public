#!/usr/bin/env python3
"""Match new display ceilings to real texture AND baked vertex-colour support."""
import argparse
import gc
import io
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage, sparse
from scipy.sparse.linalg import spsolve
from scipy.spatial import cKDTree

from audit_structural_delivery import glb_triangle_positions
from extend_closed_ceiling_bodies import bilinear_image
from optimize_session import sha256_file, utc_now
from refine_display_edges import unpack, accessor, write_glb
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

ROOT = Path(__file__).resolve().parents[1]


def appearance_streams(path, selected):
    doc, payload = unpack(path)
    uvs, colours, images = [], [], []
    cursor = 0
    for primitive in doc['meshes'][0]['primitives']:
        attrs = primitive['attributes']
        pos = accessor(doc, payload, attrs['POSITION'])
        ids = accessor(doc, payload, primitive['indices']).ravel().astype(int) if 'indices' in primitive else np.arange(len(pos))
        faces = ids.reshape(-1, 3)
        keep = selected[cursor:cursor+len(faces)]
        cursor += len(faces)
        if not keep.any():
            continue
        faces = faces[keep]
        mat = doc['materials'][primitive['material']].get('pbrMetallicRoughness', {})
        factor = np.asarray(mat.get('baseColorFactor', [1, 1, 1, 1]))[:3]
        if 'COLOR_0' in attrs:
            values = accessor(doc, payload, attrs['COLOR_0'])
            colour = values[faces, :3].astype(np.float32)
            if np.issubdtype(values.dtype, np.integer):
                colour /= np.iinfo(values.dtype).max
            colour *= factor
        else:
            colour = np.broadcast_to(factor, (len(faces), 3, 3)).copy()
        colours.append(colour)
        uv = accessor(doc, payload, attrs['TEXCOORD_0'])[faces].copy() if 'TEXCOORD_0' in attrs else np.zeros((len(faces), 3, 2))
        uvs.append(uv)
        tex = mat.get('baseColorTexture', {}).get('index')
        image = doc['textures'][tex]['source'] if tex is not None else -1
        images.append(np.full(len(faces), image, np.int32))
    assert cursor == len(selected)
    return np.concatenate(uvs), np.concatenate(colours), np.concatenate(images), doc, payload


def harmonic_fill(colour, support, domain, pixel_m, grid_m):
    """Local Dirichlet colour continuation, with no global grey/white prior."""
    h, w = domain.shape
    scale = min(1., pixel_m/grid_m)
    size = (max(4, int(w*scale)), max(4, int(h*scale)))
    weight = cv2.resize(support.astype(np.float32), size, interpolation=cv2.INTER_AREA)
    sums = cv2.resize(colour*support[:, :, None], size, interpolation=cv2.INTER_AREA)
    coarse = sums/np.maximum(weight[:, :, None], 1e-8)
    active = cv2.resize(domain.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST) > 0
    known = (weight > .22) & active
    if known.sum() < 10:
        raise ValueError('No real ceiling colour support')
    nearest = ndimage.distance_transform_edt(~known, return_distances=False, return_indices=True)
    coarse[~known] = coarse[nearest[0][~known], nearest[1][~known]]
    unknown = active & ~known
    labels = np.full(active.shape, -1, int)
    labels[unknown] = np.arange(int(unknown.sum()))
    y, x = np.where(unknown)
    rows, columns, values = [], [], []
    degree = np.zeros(len(y))
    rhs = np.zeros((len(y), 3))
    for dy, dx in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        yy, xx = y+dy, x+dx
        inside = (yy >= 0) & (yy < active.shape[0]) & (xx >= 0) & (xx < active.shape[1])
        ids = np.flatnonzero(inside)
        ids = ids[active[yy[ids], xx[ids]]]
        degree[ids] += 1
        other = labels[yy[ids], xx[ids]]
        linked = other >= 0
        rows.extend(ids[linked]); columns.extend(other[linked]); values.extend(np.full(linked.sum(), -1.))
        fixed = ids[~linked]
        rhs[fixed] += np.log(np.maximum(coarse[yy[fixed], xx[fixed]], .0005))
    # A minute nearest-colour prior anchors isolated components with no photos.
    rows.extend(np.arange(len(y))); columns.extend(np.arange(len(y))); values.extend(degree+.0001)
    rhs += .0001*np.log(np.maximum(coarse[y, x], .0005))
    if len(y):
        matrix = sparse.csr_matrix((values, (rows, columns)), shape=(len(y), len(y)))
        coarse[y, x] = np.exp(spsolve(matrix, rhs))
    result = cv2.resize(coarse, (w, h), interpolation=cv2.INTER_LINEAR)
    return np.clip(result, 0, 1)


def plaster_detail(colour, support, pixel_m):
    """A repeatable, fixture-free original high-frequency plaster sample."""
    blurred = cv2.GaussianBlur(colour, (0, 0), max(1., .06/pixel_m))
    residual = (colour-blurred).mean(2)
    size = 24
    valid = ndimage.minimum_filter(support.astype(np.uint8), size=size) > 0
    energy = cv2.boxFilter(residual**2, -1, (size, size))
    # Exclude lights, dark object boundaries and high contrast photo seams.
    luma = colour@np.array([.2126, .7152, .0722])
    valid &= (luma > .025) & (luma < .65) & (energy > 2e-6) & (energy < .00025)
    score = np.where(valid, energy, -1.)
    y, x = np.unravel_index(np.argmax(score), score.shape)
    if score[y, x] < 0:
        return np.zeros(support.shape, np.float32), None
    half = size//2
    tile = residual[y-half:y+half, x-half:x+half].copy()
    if tile.shape != (size, size):
        return np.zeros(support.shape, np.float32), None
    tile -= tile.mean()
    tile = np.clip(tile, -.018, .018)
    # Reuse the donor's short-range spectrum, not its repeating visible motif.
    # Random phase prevents a 24px lamp/noise shape repeating over metres.
    spectrum = abs(np.fft.fft2(tile))**2
    spectrum = cv2.GaussianBlur(np.fft.fftshift(spectrum), (0, 0), 1.)
    spectrum = cv2.resize(spectrum, (support.shape[1], support.shape[0]), interpolation=cv2.INTER_LINEAR)
    noise = np.random.default_rng(20261019).normal(size=support.shape)
    filtered = np.fft.ifft2(np.fft.fft2(noise)*np.sqrt(np.fft.ifftshift(np.maximum(spectrum, 0)))).real
    filtered -= cv2.GaussianBlur(filtered, (0, 0), 2.)
    filtered *= min(float(tile.std()), .004)/max(float(filtered.std()), 1e-8)
    return np.clip(filtered, -.012, .012), [int(x), int(y), size]


def sample_ceiling_hits(path, selected, triangles, rays):
    import open3d as o3d
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(triangles.reshape(-1, 3)),
                        o3d.core.Tensor(np.arange(triangles.size//3).reshape(-1, 3).astype(np.uint32))))
    hit = scene.cast_rays(o3d.core.Tensor(rays))
    distance = hit['t_hit'].numpy()
    ids = np.flatnonzero(np.isfinite(distance))
    uv, colours, images, doc, payload = appearance_streams(path, selected)
    tri = hit['primitive_ids'].numpy()[ids].astype(int)
    bary = hit['primitive_uvs'].numpy()[ids]
    weights = np.c_[1-bary.sum(1), bary]
    pixels = np.einsum('ij,ijk->ik', weights, uv[tri])
    rgb = np.einsum('ij,ijk->ik', weights, colours[tri])
    vertex_colour = images[tri] < 0
    for index in np.unique(images[tri]):
        if index < 0:
            continue
        which = images[tri] == index
        view = doc['bufferViews'][doc['images'][int(index)]['bufferView']]
        start = view.get('byteOffset', 0)
        image = np.asarray(Image.open(io.BytesIO(payload[start:start+view['byteLength']])).convert('RGB'))
        rgb[which] *= srgb_to_linear(bilinear_image(image, pixels[which]))
    return ids, distance[ids], rgb, vertex_colour


def ceiling_atlas(reference, xyzs, domain, low, cell, band, dest, cfg):
    h, w = domain.shape
    scale = min(1/cfg['texture_metres_per_pixel'], cfg['texture_maximum_side_px']/max(w*cell, h*cell))
    tw, th = max(8, int(w*cell*scale)), max(8, int(h*cell*scale))
    yy, xx = np.indices((th, tw))
    xy = low+np.c_[(xx.ravel()+.5)*w*cell/tw, (yy.ravel()+.5)*h*cell/th]
    rays = np.c_[xy, np.full(len(xy), band[0]-.05), np.zeros((len(xy), 2)), np.ones(len(xy))].astype(np.float32)
    colour = np.zeros((len(xy), 3), np.float32)
    nearest = np.full(len(xy), np.inf, np.float32)
    priority = np.full(len(xy), -1, np.int8)
    support = np.zeros(len(xy), bool)
    vertex_samples = 0
    for name in ['lobby', 'floor2', 'bridge_12_sub']:
        xyz = xyzs[name]
        normal = np.cross(xyz[:, 1]-xyz[:, 0], xyz[:, 2]-xyz[:, 0])
        nz = abs(normal[:, 2])/np.maximum(np.linalg.norm(normal, axis=1), 1e-9)
        selected = (nz > .86) & (xyz[:, :, 2].min(1) > band[0]) & (xyz[:, :, 2].max(1) < band[1])
        if not selected.any():
            continue
        flat = np.zeros(len(xyz), bool)
        for height in [1.98, 1.4804538488388062, 5.571264266967773]:
            flat |= np.max(abs(xyz[:, :, 2]-height), axis=1) < 2e-5
        # Prefer the accepted photo-ceiling sheets over old lower shards which
        # can carry a sign edge / dark point colour into the new ceiling atlas.
        for rank, choose in [(0, selected & ~flat), (1, selected & flat)]:
            if not choose.any():
                continue
            ids, distance, rgb, from_vertices = sample_ceiling_hits(reference/name/'clean_textured.glb', choose, xyz[choose], rays)
            use = (rank > priority[ids]) | ((rank == priority[ids]) & (distance < nearest[ids]))
            ids, distance, rgb = ids[use], distance[use], rgb[use]
            vertex_samples += int(from_vertices[use].sum())
            colour[ids] = rgb
            support[ids] = True
            nearest[ids] = distance
            priority[ids] = rank
            gc.collect()
    surface = cv2.resize(domain.astype(np.uint8), (tw, th), interpolation=cv2.INTER_NEAREST) > 0
    support = support.reshape(th, tw) & surface
    colour = colour.reshape(th, tw, 3)
    pixel = max(w*cell/tw, h*cell/th)
    # All real vertex colours count. Do not discard dark corridors by a global
    # brightness percentile, and never treat a missing PNG as uniform grey.
    luma = colour@np.array([.2126, .7152, .0722])
    paint = support & (luma < .80) & (luma > .001)
    fill = harmonic_fill(colour, paint, surface, pixel, cfg['fill_grid_m'])
    detail, donor = plaster_detail(colour, paint, pixel)
    outside_distance = ndimage.distance_transform_edt(~support)*pixel
    amplitude = np.clip(outside_distance/cfg['detail_fade_m'], 0, 1)*.65
    amplitude *= np.clip((fill@np.array([.2126, .7152, .0722]))/.20, 0, 1)
    textured_fill = np.clip(fill+detail[:, :, None]*amplitude[:, :, None], 0, 1)
    blend = np.clip(ndimage.distance_transform_edt(support)*pixel/cfg['boundary_blend_m'], 0, 1)
    result = textured_fill*(1-blend[:, :, None])+colour*blend[:, :, None]
    Image.fromarray(np.uint8(linear_to_srgb(result)*255+.5)).save(dest)
    Image.fromarray(np.uint8(linear_to_srgb(fill)*255+.5)).save(dest.with_name(dest.stem+'_paint.png'))
    Image.fromarray(np.uint8(np.where(surface, np.where(support, 255, 80), 0))).save(dest.with_name(dest.stem+'_support.png'))
    return {'real_vertex_colour_samples_not_grey': vertex_samples, 'source_support_fraction': float(support[surface].mean()),
            'microtexture_donor_atlas_pixel': donor, 'texture_size': [tw, th],
            'unknown_material_is_inferred': True, 'colour_extension': 'Local log-linear RGB harmonic interpolation, no global grey/white prior.'}


def append_image(doc, payload, path):
    payload.extend(b'\0'*(-len(payload)%4))
    offset = len(payload)
    encoded = path.read_bytes()
    payload.extend(encoded)
    doc['bufferViews'].append({'buffer': 0, 'byteOffset': offset, 'byteLength': len(encoded)})
    return {'bufferView': len(doc['bufferViews'])-1, 'mimeType': 'image/png'}


def wall_paint_reference(reference, xyzs):
    """Infer plain cavity fascia paint from nearby real upper-wall appearance."""
    points, values = [], []
    for name in ['lobby', 'floor2', 'bridge_12_sub']:
        xyz = xyzs[name]
        normal = np.cross(xyz[:, 1]-xyz[:, 0], xyz[:, 2]-xyz[:, 0])
        nz = abs(normal[:, 2])/np.maximum(np.linalg.norm(normal, axis=1), 1e-9)
        centre = xyz.mean(1)
        selected = (nz < .30) & (centre[:, 2] > 2.65) & (centre[:, 2] < 3.40)
        if not selected.any():
            continue
        uv, colours, images, doc, payload = appearance_streams(reference/name/'clean_textured.glb', selected)
        rgb = colours.mean(1)
        for index in np.unique(images):
            if index < 0:
                continue
            ids = np.flatnonzero(images == index)
            view = doc['bufferViews'][doc['images'][int(index)]['bufferView']]
            start = view.get('byteOffset', 0)
            image = np.asarray(Image.open(io.BytesIO(payload[start:start+view['byteLength']])).convert('RGB'))
            rgb[ids] *= srgb_to_linear(bilinear_image(image, uv[ids].mean(1)))
        luma = rgb@np.array([.2126, .7152, .0722])
        paint = (luma > .10) & (luma < .70) & (np.ptp(rgb, axis=1) < .23)
        points.append(centre[selected][paint]); values.append(rgb[paint])
        del doc, payload, uv, colours
    if not points or sum(map(len, points)) < 30:
        raise ValueError('Insufficient real upper-wall paint support')
    points, values = np.concatenate(points), np.concatenate(values)
    weighted = points*np.array([1, 1, .35])
    tree = cKDTree(weighted)

    def paint(query):
        target = np.asarray(query).copy()
        target[:, 2] = 2.95
        distances, ids = tree.query(target*np.array([1, 1, .35]), k=min(32, len(points)), workers=2)
        # Discard the darkest local detail (door edges, text, rails) rather
        # than extending it into an unobserved vertical structural strip.
        colours = values[ids]
        light = colours@np.array([.2126, .7152, .0722])
        good = light >= np.quantile(light, .4, axis=1)[:, None]
        weight = good/np.maximum(distances+.10, .10)**2
        result = (colours*weight[:, :, None]).sum(1)/weight.sum(1)[:, None]
        return np.clip(result, 0, 1)

    return paint, {'real_wall_colour_samples': len(points), 'sample_band_m': [2.65, 3.40],
                   'scope': 'Inferred plain fascia paint from neighbouring upper walls; not photographed pipe/beam detail.'}


def attach_vertex_paint(doc, payload, primitive, paint):
    pos = accessor(doc, payload, primitive['attributes']['POSITION'])
    values = np.asarray(paint(pos), '<f4')
    payload.extend(b'\0'*(-len(payload)%4)); offset = len(payload)
    payload.extend(values.tobytes())
    doc['bufferViews'].append({'buffer': 0, 'byteOffset': offset, 'byteLength': values.nbytes, 'target': 34962})
    doc['accessors'].append({'bufferView': len(doc['bufferViews'])-1, 'componentType': 5126, 'count': len(values), 'type': 'VEC3'})
    primitive['attributes']['COLOR_0'] = len(doc['accessors'])-1
    doc['materials'].append({'doubleSided': True, 'extensions': {'KHR_materials_unlit': {}},
                             'pbrMetallicRoughness': {'metallicFactor': 0., 'roughnessFactor': 1.}})
    primitive['material'] = len(doc['materials'])-1


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'config/ceiling_material_continuity.json')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    cfg = json.loads(a.config.read_text())
    source, reference = ROOT/cfg['source_bundle'], ROOT/cfg['appearance_reference']
    run = json.loads((source/'manifest.json').read_text())
    a.output.mkdir(parents=True, exist_ok=False)
    q = np.load(source/'closure_plan.npz')
    xyzs = {name: glb_triangle_positions(reference/name/'clean_textured.glb') for name in run['sessions']}
    reports = {}
    for label, domain, band in [('lower', q['body'], [.8, 2.36]), ('upper', q['roof'], [4.85, 6.3])]:
        print('Real-colour ceiling atlas: '+label, flush=True)
        reports[label] = ceiling_atlas(reference, xyzs, domain, q['low'], float(q['cell']), band, a.output/(label+'_matched_ceiling.png'), cfg)
    wall_paint, reports['fascia'] = wall_paint_reference(reference, xyzs)
    clean, sessions = {}, {}
    for name, spec in run['sessions'].items():
        folder = a.output/name
        shutil.copytree(source/name, folder)
        doc, payload = unpack(source/name/'clean_textured.glb')
        before_positions = glb_triangle_positions(source/name/'clean_textured.glb')
        primitives = doc['meshes'][0]['primitives']
        changed = []
        # Only the newly appended closure materials are replaced. Original
        # floors and signs keep their original image, UV and colour streams.
        count = len(spec['closure_appended'])
        for primitive, part in zip(primitives[-count:] if count else [], spec['closure_appended']):
            material = doc['materials'][primitive['material']]
            texture = doc['textures'][material['pbrMetallicRoughness']['baseColorTexture']['index']]
            label = 'upper' if part['part'] == 'expanded_upper_roof' else 'lower'
            suffix = '_paint.png' if part['part'] == 'perimeter_and_void_sides' else '.png'
            path = a.output/(label+'_matched_ceiling'+suffix)
            doc['images'][texture['source']] = append_image(doc, payload, path)
            if part['part'] == 'perimeter_and_void_sides':
                attach_vertex_paint(doc, payload, primitive, wall_paint)
            changed.append(part['part'])
        # The old two explicitly inferred service-slab skirts used tiny solid
        # grey images. Match their paint to the same local lower ceiling atlas.
        old_skirts = 0
        for primitive in primitives[:-count] if count else primitives:
            material = doc['materials'][primitive['material']]
            texture_id = material.get('pbrMetallicRoughness', {}).get('baseColorTexture', {}).get('index')
            if texture_id is None or 'TEXCOORD_0' not in primitive['attributes']:
                continue
            image_id = doc['textures'][texture_id]['source']
            view = doc['bufferViews'][doc['images'][image_id]['bufferView']]
            start = view.get('byteOffset', 0)
            if view['byteLength'] > 150:
                continue
            rgb = np.asarray(Image.open(io.BytesIO(payload[start:start+view['byteLength']])).convert('RGB'))
            if rgb.shape not in [(8, 8, 3), (16, 16, 3)] or not np.all(rgb == [64, 69, 77]):
                continue
            attach_vertex_paint(doc, payload, primitive, wall_paint)
            old_skirts += 1
        # write_glb deliberately never overwrites a file: retain the copied
        # source name as an explicit before artifact, then write the derivative.
        (folder/'clean_textured.glb').rename(folder/'before_material.glb')
        write_glb(folder/'clean_textured.glb', doc, payload)
        assert np.array_equal(before_positions, glb_triangle_positions(folder/'clean_textured.glb'))
        clean[name] = {'ply_sha256': sha256_file(folder/'clean_structural.ply'), 'glb_sha256': sha256_file(folder/'clean_textured.glb')}
        sessions[name] = {**spec, 'ceiling_material_finish': {'changed_new_parts': changed, 'old_grey_skirt_primitives_matched': old_skirts,
                                                         'all_triangle_XYZ_and_order_exact': True, 'floors_and_signs_unchanged': True}}
        print('Material only, geometry fixed: '+name, flush=True)
    for path in source.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json']:
            shutil.copyfile(path, a.output/path.name)
    inputs = {str(path.resolve()): sha256_file(path) for path in [Path(__file__), a.config, source/'manifest.json', reference/'manifest.json']}
    manifest = {**run, 'created_utc': utc_now(), 'sessions': sessions, 'inputs_sha256': {**run['inputs_sha256'], **inputs},
                'ceiling_material_continuity': {'config': cfg, 'textures': reports},
                'previous_material_manifest': str((source/'manifest.json').resolve()),
                'floor_material_source_manifest': str((source/'manifest.json').resolve())}
    (a.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (a.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': sha256_file(a.output/'manifest.json'), 'sessions': clean}, indent=2)+'\n')


if __name__ == '__main__':
    main()
