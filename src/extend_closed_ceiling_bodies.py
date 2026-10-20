#!/usr/bin/env python3
"""Wall/floor-bounded ceiling extensions with opaque inter-floor side skins."""
import argparse
import gc
import hashlib
import io
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
from PIL import Image
from scipy import ndimage

from append_plane_glb import append_planes
from audit_structural_delivery import glb_triangle_positions
from clean_structural_fragments import filter_glb
from complete_service_slabs import closed_slab_arrays, mask_lookup, remove_diagonal_contacts
from flatten_horizontal_surfaces import build_sheet, disk
from optimize_session import sha256_file, utc_now
from presentation_geometry import LevelPresentation
from reconcile_multilevel_surfaces import remap_mask
from refine_display_edges import unpack, accessor
from refine_surface_display import srgb_to_linear
from repair_visual_appearance import linear_to_srgb

ROOT = Path(__file__).resolve().parents[1]


def complete_floor_cells(xyz, low, shape, cell, height):
    """Require both actual canonical half-cell triangles, not two centroid hits."""
    result = np.zeros(shape, np.uint8)
    flat = np.max(abs(xyz[:, :, 2]-height), axis=1) < 2e-5
    ids = np.flatnonzero(flat)
    q = (xyz[ids, :, :2]-low)/cell
    grid = np.rint(q).astype(np.int32)
    aligned = np.max(abs(q-grid), axis=(1, 2)) < 4e-4
    width = np.ptp(grid[:, :, 0], axis=1)
    depth = np.ptp(grid[:, :, 1], axis=1)
    good = aligned & (width == 1) & (depth == 1)
    ids, grid = ids[good], grid[good]
    minimum = grid.min(1)
    local = grid-minimum[:, None]
    code = np.sort(local[:, :, 0]+2*local[:, :, 1], axis=1)
    first = np.all(code == [0, 1, 2], axis=1)
    second = np.all(code == [1, 2, 3], axis=1)
    x, y = minimum.T
    valid = (x >= 0) & (x < shape[1]) & (y >= 0) & (y < shape[0])
    np.bitwise_or.at(result, (y[valid & first], x[valid & first]), 1)
    np.bitwise_or.at(result, (y[valid & second], x[valid & second]), 2)
    accepted = ids[valid & (first | second)]
    return result, accepted


def semantic_roof_guard(objects, nodes, model, low, shape, cell, height, radius):
    """Do not newly block an existing photographed sign with a roof extension."""
    forbidden = np.zeros(shape, bool)
    for obj in objects:
        if obj.get('class') != 'sign' or not obj.get('geometry', {}).get('accepted'):
            continue
        boundary = np.asarray(obj['geometry']['world_boundary'])
        if len(boundary) != 4 or boundary[:, 2].max() < height-.02:
            continue
        node = next((n for n in nodes if n['session'] == obj['session'] and n['node'] == obj['node']), None)
        if node is None:
            continue
        eye = model.warp(np.asarray(node['world_optical'])[:3, 3][None], obj['session'])[0]
        yy, xx = np.indices((48, 180))
        u, v = (xx.ravel()+.5)/180, (yy.ravel()+.5)/48
        target = ((1-u)*(1-v))[:, None]*boundary[0]+(u*(1-v))[:, None]*boundary[1]+(u*v)[:, None]*boundary[2]+((1-u)*v)[:, None]*boundary[3]
        dz = target[:, 2]-eye[2]
        t = (height-eye[2])/np.where(abs(dz) > 1e-8, dz, np.nan)
        valid = (t > 0) & (t < 1.02)
        points = eye[:2]+(target[valid, :2]-eye[:2])*t[valid, None]
        ij = np.floor((points-low)/cell).astype(int)
        x, y = ij.T
        good = (x >= 0) & (x < shape[1]) & (y >= 0) & (y < shape[0])
        forbidden[y[good], x[good]] = True
    return ndimage.binary_dilation(forbidden, structure=disk(max(1, int(np.ceil(radius/cell)))))


def texture_streams(path, select):
    """Read only selected live triangle UV/image references; retain source photos."""
    doc, payload = unpack(path)
    uv_chunks, image_chunks = [], []
    cursor = 0
    for primitive in doc['meshes'][0]['primitives']:
        pos = accessor(doc, payload, primitive['attributes']['POSITION'])
        ids = accessor(doc, payload, primitive['indices']).ravel().astype(int) if 'indices' in primitive else np.arange(len(pos))
        faces = ids.reshape(-1, 3)
        keep = select[cursor:cursor+len(faces)]
        cursor += len(faces)
        if not keep.any():
            continue
        uv_index = primitive['attributes'].get('TEXCOORD_0')
        uv = accessor(doc, payload, uv_index)[faces[keep]].copy() if uv_index is not None else np.zeros((int(keep.sum()), 3, 2), np.float32)
        mat = doc['materials'][primitive['material']]
        tex = mat.get('pbrMetallicRoughness', {}).get('baseColorTexture', {}).get('index')
        image = doc['textures'][tex]['source'] if tex is not None else -1
        uv_chunks.append(uv)
        image_chunks.append(np.full(len(uv), image, np.int32))
    assert cursor == len(select)
    return np.concatenate(uv_chunks), np.concatenate(image_chunks), doc, payload


def bilinear_image(rgb, uv):
    h, w = rgb.shape[:2]
    x = np.clip(uv[:, 0]*w, 0, w-1)
    y = np.clip(uv[:, 1]*h, 0, h-1)
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    x1, y1 = np.minimum(x0+1, w-1), np.minimum(y0+1, h-1)
    a, b = (x-x0)[:, None], (y-y0)[:, None]
    return ((1-b)*((1-a)*rgb[y0, x0]+a*rgb[y0, x1])+b*((1-a)*rgb[y1, x0]+a*rgb[y1, x1]))/255.


def ceiling_atlas(source, run, xyzs, domain, low, cell, band, dest, cfg):
    """Sample current ceiling photographs; extrapolate only low-frequency colour."""
    import open3d as o3d
    h, w = domain.shape
    scale = min(1/cfg['texture_metres_per_pixel'], cfg['texture_maximum_side_px']/max(w*cell, h*cell))
    tw, th = max(8, int(w*cell*scale)), max(8, int(h*cell*scale))
    yy, xx = np.indices((th, tw))
    xy = low+np.c_[(xx.ravel()+.5)*w*cell/tw, (yy.ravel()+.5)*h*cell/th]
    rays = np.c_[xy, np.full(len(xy), band[0]-.05), np.zeros((len(xy), 2)), np.ones(len(xy))].astype(np.float32)
    colour = np.zeros((len(xy), 3), np.float32)
    nearest_height = np.full(len(xy), np.inf, np.float32)
    support = np.zeros(len(xy), bool)
    for name, xyz in xyzs.items():
        if name not in ['lobby', 'floor2', 'bridge_12_sub']:
            continue
        normal = np.cross(xyz[:, 1]-xyz[:, 0], xyz[:, 2]-xyz[:, 0])
        nz = abs(normal[:, 2])/np.maximum(np.linalg.norm(normal, axis=1), 1e-9)
        select = (nz > .86) & (xyz[:, :, 2].min(1) > band[0]) & (xyz[:, :, 2].max(1) < band[1])
        if not select.any():
            continue
        filtered = xyz[select]
        mesh = o3d.t.geometry.TriangleMesh(o3d.core.Tensor(filtered.reshape(-1, 3)), o3d.core.Tensor(np.arange(filtered.size//3).reshape(-1, 3).astype(np.uint32)))
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(mesh)
        hit = scene.cast_rays(o3d.core.Tensor(rays))
        distance = hit['t_hit'].numpy()
        valid = np.isfinite(distance) & (distance < nearest_height)
        ids = np.flatnonzero(valid)
        if not len(ids):
            continue
        uv, images, doc, payload = texture_streams(source/name/'clean_textured.glb', select)
        triangles = hit['primitive_ids'].numpy()[ids].astype(int)
        bary = hit['primitive_uvs'].numpy()[ids]
        pixels = uv[triangles, 0]*(1-bary.sum(1))[:, None]+uv[triangles, 1]*bary[:, :1]+uv[triangles, 2]*bary[:, 1:]
        for image in np.unique(images[triangles]):
            which = images[triangles] == image
            if image < 0:
                values = np.full((int(which.sum()), 3), .72)
            else:
                view = doc['bufferViews'][doc['images'][int(image)]['bufferView']]
                start = view.get('byteOffset', 0)
                rgb = np.asarray(Image.open(io.BytesIO(payload[start:start+view['byteLength']])).convert('RGB'))
                values = bilinear_image(rgb, pixels[which])
            colour[ids[which]] = srgb_to_linear(values)
        support[ids] = True
        nearest_height[ids] = distance[ids]
        del payload, doc, scene, mesh, filtered, uv
        gc.collect()
    surface = cv2.resize(domain.astype(np.uint8), (tw, th), interpolation=cv2.INTER_NEAREST) > 0
    support = support.reshape(th, tw) & surface
    colour = colour.reshape(th, tw, 3)
    if support.sum() < 50:
        raise ValueError('Insufficient existing ceiling colour support')
    luma = colour@np.array([.2126, .7152, .0722])
    lo, hi = np.quantile(luma[support], [.25, .90])
    paint = support & (luma >= lo) & (luma <= hi)
    indices = ndimage.distance_transform_edt(~paint, return_distances=False, return_indices=True)
    fill = colour[indices[0], indices[1]]
    pixel = max(w*cell/tw, h*cell/th)
    fill = cv2.GaussianBlur(fill, (0, 0), max(1., cfg['ceiling_colour_fill_radius_m']/pixel))
    blend = np.clip(ndimage.distance_transform_edt(support)*pixel/cfg['texture_blend_m'], 0, 1)
    result = fill*(1-blend[:, :, None])+colour*blend[:, :, None]
    Image.fromarray(np.uint8(linear_to_srgb(np.clip(result, 0, 1))*255+.5)).save(dest)
    Image.fromarray(np.uint8(np.where(surface, np.where(support, 255, 80), 0))).save(dest.with_name(dest.stem+'_support.png'))
    return {'existing_display_ceiling_colour_support_fraction': float(support[surface].mean()),
            'texture_size': [tw, th], 'unknown_colour_fill': 'Smooth nearest non-fixture existing ceiling colour; no invented lights, labels or raw-photograph claim.'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'config/ceiling_expansion_and_closure.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--plan-only', action='store_true')
    a = p.parse_args()
    cfg = json.loads(a.config.read_text())
    source = ROOT/cfg['source_bundle']
    run = json.loads((source/'manifest.json').read_text())
    a.output.mkdir(parents=True, exist_ok=False)
    cell = cfg['cell_m']
    xyzs = {n: glb_triangle_positions(source/n/'clean_textured.glb').astype(np.float32) for n in run['sessions']}
    indoor = [xyzs[n].reshape(-1, 3) for n in ['lobby', 'floor2', 'bridge_12_sub']]
    low = np.floor(np.min([v[:, :2].min(0) for v in indoor], axis=0)/cell)*cell-cell*2
    high = np.ceil(np.max([v[:, :2].max(0) for v in indoor], axis=0)/cell)*cell+cell*2
    shape = tuple(np.rint((high-low)/cell).astype(int)[::-1])
    coverage = np.zeros(shape, np.uint8)
    top_ids = {}
    for name, xyz in xyzs.items():
        bits, ids = complete_floor_cells(xyz, low, shape, cell, cfg['upper_floor_m'])
        if name in ['lobby', 'floor2', 'bridge_12_sub']:
            coverage |= bits
            top_ids[name] = ids
    actual_top = coverage == 3
    atrium = np.load(ROOT/cfg['atrium_mask'])
    core = atrium['core'] if 'core' in atrium else atrium['mask']
    void = remap_mask(core, atrium['low'], float(atrium['cell']), low, cell, shape)
    stair = np.zeros(shape, bool)
    roof_free = np.zeros(shape, bool)
    for name, identity in [('floor2', 'floor2_floor_2f'), ('bridge_12_sub', 'sub_floor_2f')]:
        pack = np.load(ROOT/'outputs/student_union_topology_fixed_v2'/name/(identity+'_mask.npz'))
        stair |= remap_mask(pack['stair_guard'] | pack['free_guard'], pack['low'], float(pack['cell']), low, cell, shape)
    for name, identity in [('floor2', 'floor2_ceiling_2f'), ('bridge_12_sub', 'sub_ceiling_2f')]:
        pack = np.load(ROOT/'outputs/student_union_topology_fixed_v2'/name/(identity+'_mask.npz'))
        roof_free |= remap_mask(pack['free_guard'], pack['low'], float(pack['cell']), low, cell, shape)
    body = remove_diagonal_contacts(actual_top & ~void & ~stair)
    observed = np.load(ROOT/cfg['roof_mask'])
    old_roof = remap_mask(observed['mask'], observed['low'], float(observed['cell']), low, cell, shape)
    roof_height = float(observed['height'])
    envelope = ndimage.binary_fill_holes(actual_top | old_roof)
    radius = max(1, int(round(cfg['roof_close_radius_m']/cell)))
    roof = (ndimage.binary_closing(actual_top | old_roof, structure=disk(radius)) | old_roof | actual_top) & envelope & ~roof_free
    # The double-height lobby remains open at the lower ceiling. At the upper
    # roof, only bounded internal holes of the building envelope are completed.
    roof |= ndimage.binary_fill_holes(roof) & envelope & ~roof_free
    objects = json.loads((source/'semantic_objects.json').read_text())['objects']
    nodes = json.loads((ROOT/'outputs/student_union_roadview_v1/roadview.json').read_text())['nodes']
    model = LevelPresentation(ROOT, run['presentation_config'])
    sign_guard = semantic_roof_guard(objects, nodes, model, low, shape, cell, roof_height, cfg['semantic_ray_guard_m'])
    roof &= ~(sign_guard & ~old_roof)
    np.savez_compressed(a.output/'closure_plan.npz', low=low, cell=cell, actual_top=actual_top, body=body, void=void, stair=stair, old_roof=old_roof, roof=roof, roof_free=roof_free, sign_guard=sign_guard, roof_height=roof_height)
    plan = {'actual_upper_floor_complete_cells_area_m2': float(actual_top.sum()*cell**2),
            'closed_interfloor_body_area_m2': float(body.sum()*cell**2),
            'old_upper_roof_area_m2': float(old_roof.sum()*cell**2),
            'extended_upper_roof_area_m2': float(roof.sum()*cell**2),
            'new_upper_roof_area_m2': float((roof & ~old_roof).sum()*cell**2),
            'lower_ceiling_height_m': cfg['lower_extension_height_m'], 'upper_floor_m': cfg['upper_floor_m'],
            'roof_height_m': roof_height, 'lower_cap_inside_atrium_cells': int((body & void).sum()),
            'lower_cap_inside_stair_guard_cells': int((body & stair).sum()), 'scope': cfg['scope']}
    (a.output/'plan.json').write_text(json.dumps(plan, indent=2)+'\n')
    print(json.dumps(plan), flush=True)
    if a.plan_only:
        return
    lower_texture = a.output/'lower_continuous_ceiling.png'
    upper_texture = a.output/'upper_continuous_ceiling.png'
    texture_reports = {'lower': ceiling_atlas(source, run, xyzs, body, low, cell, [.8, 2.36], lower_texture, cfg),
                       'upper': ceiling_atlas(source, run, xyzs, roof, low, cell, [4.85, 6.3], upper_texture, cfg)}
    import open3d as o3d
    # Reuse the unmodified actual upper-floor geometry as the top skin. Only
    # the underside and perimeter/void skins are appended to the rendering.
    vertices, parts = closed_slab_arrays(body, low, cell, cfg['lower_extension_height_m'], cfg['upper_floor_m'])
    closed_faces = np.concatenate(parts)
    closed_mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(closed_faces))
    closed_mesh.remove_unreferenced_vertices()
    o3d.io.write_triangle_mesh(str(a.output/'analytic_closed_body.ply'), closed_mesh)
    sessions, cleanup = {}, {}
    for name, spec in run['sessions'].items():
        print('Closed ceiling expansion: '+name, flush=True)
        folder = a.output/name
        folder.mkdir()
        xyz = xyzs[name]
        keep = np.ones(len(xyz), bool)
        for mask, height in [(body, cfg['lower_extension_height_m']), (roof, roof_height)]:
            inside = mask_lookup(mask, low, cell, xyz.mean(1)[:, :2])
            exact = np.max(abs(xyz[:, :, 2]-height), axis=1) < 2e-5
            keep &= ~(inside & exact)
        # Existing floors are never in either removal-height band.
        floor = (np.max(abs(xyz[:, :, 2]-cfg['upper_floor_m']), axis=1) < 2e-5) | (np.max(abs(xyz[:, :, 2]+1.55), axis=1) < 2e-5)
        assert keep[floor].all()
        filter_glb(source/name/'clean_textured.glb', folder/'retained.glb', keep)
        mesh = o3d.io.read_triangle_mesh(str(source/name/'clean_structural.ply'))
        mesh.remove_triangles_by_mask(~keep)
        mesh.remove_unreferenced_vertices()
        appended, uvs, textures, reports = [], [], [], []
        if name == 'lobby':
            for label, faces in [('underside', parts[0]), ('perimeter_and_void_sides', parts[2])]:
                patch = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
                patch.compute_vertex_normals()
                uv = (vertices[:, :2]-low)/(np.array(shape[::-1])*cell)
                appended.append(patch)
                uvs.append(uv[faces])
                textures.append(lower_texture)
                reports.append({'part': label, 'triangles': len(faces), 'walkable': False,
                                'role': 'CEILING' if label=='underside' else 'SERVICE_VOID_ENCLOSURE_WALL',
                                'navigation_exported': False})
        if name == 'floor2':
            patch, uv = build_sheet(o3d, roof, low, roof_height, cell, 'ceiling')
            appended.append(patch)
            uvs.append(uv)
            textures.append(upper_texture)
            reports.append({'part': 'expanded_upper_roof', 'triangles': len(patch.triangles),
                            'role': 'CEILING', 'walkable': False, 'navigation_exported': False})
        retained = len(mesh.triangles)
        for patch in appended:
            mesh += patch
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(folder/'clean_structural.ply'), mesh)
        append_planes(folder/'retained.glb', folder/'clean_textured.glb', appended, uvs, textures)
        np.save(folder/'closure_retained_faces.npy', keep)
        for path in (source/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply', 'clean_textured.glb', 'retained.glb']:
                shutil.copyfile(path, folder/path.name)
        sessions[name] = {**spec, 'closure_retained_triangles': retained, 'closure_appended': reports,
                          'bounds_min_m': mesh.get_min_bound().tolist(), 'bounds_max_m': mesh.get_max_bound().tolist()}
        cleanup[name] = {'before_faces': len(keep), 'after_faces': len(mesh.triangles), 'removed_faces': int((~keep).sum()),
                         'ply_sha256': sha256_file(folder/'clean_structural.ply'), 'glb_sha256': sha256_file(folder/'clean_textured.glb')}
    for path in source.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json', 'closure_plan.npz']:
            shutil.copyfile(path, a.output/path.name)
    inputs = {str(p.resolve()): sha256_file(p) for p in [Path(__file__), a.config, source/'manifest.json', ROOT/cfg['roof_mask'], ROOT/cfg['atrium_mask'], ROOT/'src/append_plane_glb.py', ROOT/'src/complete_service_slabs.py', ROOT/'src/clean_structural_fragments.py']}
    for name in run['sessions']:
        for filename in ['clean_structural.ply', 'clean_textured.glb']:
            path = source/name/filename
            inputs[str(path.resolve())] = sha256_file(path)
    manifest = {**run, 'created_utc': utc_now(), 'sessions': sessions, 'inputs_sha256': {**run['inputs_sha256'], **inputs},
                'ceiling_expansion_and_closure': {'config': cfg, 'plan': plan, 'textures': texture_reports},
                'closure_source_manifest': str((source/'manifest.json').resolve()),
                'previous_material_manifest': str((source/'manifest.json').resolve()),
                'floor_material_source_manifest': str((source/'manifest.json').resolve()), 'scope': cfg['scope']}
    (a.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (a.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': sha256_file(a.output/'manifest.json'), 'sessions': cleanup}, indent=2)+'\n')


if __name__ == '__main__':
    main()
