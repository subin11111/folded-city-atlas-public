"""Replace the two remaining visibly rough, depth-verified stair-side wall patches."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sqlite3

import cv2
import numpy as np
from PIL import Image

from append_plane_glb import append_planes
from audit_structural_delivery import glb_triangle_positions
from clean_structural_fragments import filter_glb
from complete_verified_upper_walls import inside_polygon
from export_rtabmap_db import load_pose_csv
from finalize_display_mesh import face_geometry
from finalize_display_mesh import samples_on_faces
from optimize_session import sha256_file, utc_now
from photo_semantic_surfaces import photo_mesh
from presentation_geometry import LevelPresentation
from probe_upper_wall_support import polygon_points
from refine_shadow_colour import correct_shadow_texture
from surface_model_utils import decode_frame, project

ROOT = Path(__file__).resolve().parents[1]


def covered_by_actual_patches(o3d, xyz, guarded, models, records, camera, intrinsic, cfg):
    """Actual exported wall coverage, not an approximate warped-camera ROI."""
    normal, _ = face_geometry(xyz)
    # Generic surfaces keep every horizontal facet. In these two explicitly
    # inspected bare-wall polygons, wrong facet normals are themselves part of
    # the reconstruction noise. Known floors/cavity/objects/handrail remain
    # guarded; every removal still needs seven hits on the real new wall mesh.
    if not cfg.get('verified_bare_wall_volume_override', False):
        guarded = guarded | (abs(normal[:, 2]) > .65)
    centres = xyz.mean(1)
    pixel, _ = project(centres, intrinsic, camera)
    remove = np.zeros(len(xyz), bool)
    band = cfg['maximum_plane_distance_m']
    for mesh, record in zip(models, records):
        v = np.asarray(mesh.vertices)
        n = np.asarray(record['geometry']['plane_normal'])
        c = v.mean(0)
        candidate = (~guarded) & (abs((centres-c)@n) < band+.03)
        candidate &= (abs(normal@n) > cfg['minimum_normal_agreement'])
        candidate &= ((centres >= v.min(0)-band) & (centres <= v.max(0)+band)).all(1)
        for polygon in record.get('protect_photo_polygons', []):
            candidate &= ~inside_polygon(pixel, polygon)
        ids = np.flatnonzero(candidate)
        if not len(ids):
            continue
        scene = o3d.t.geometry.RaycastingScene(nthreads=2)
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
        samples = samples_on_faces(xyz[ids])
        origin = samples-n*band
        direction = np.broadcast_to(n, samples.shape)
        rays = np.concatenate([origin, direction], axis=-1).reshape(-1, 6).astype(np.float32)
        hits = scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy().reshape(samples.shape[:2])
        remove[ids] |= ((hits >= 0) & (hits <= 2*band)).all(1)
    return ~remove


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'config/final_reviewed_wall_patches.json')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    cfg = json.loads(a.config.read_text())
    base = ROOT/cfg['source_bundle']
    run = json.loads((base/'manifest.json').read_text())
    ptr = json.loads((ROOT/'config/current_map.json').read_text())
    mp = ROOT/ptr['manifest']
    selected = json.loads(mp.read_text())
    name = cfg['session']
    s = selected['sessions'][name]
    poses = load_pose_csv(mp.parent/s['trajectory'])
    db = Path(selected['database_integrity'][name]['database'])
    with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as con:
        frame = decode_frame(con, cfg['node'], s['preprocessing'], poses[cfg['node']])
    model = LevelPresentation(ROOT, run['presentation_config'])
    a.output.mkdir(parents=True, exist_ok=False)
    dest = a.output/name
    dest.mkdir()
    import open3d as o3d
    mesh = o3d.io.read_triangle_mesh(str(base/name/'clean_structural.ply'))
    old = np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32)
    centre = old.mean(1)
    normals, _ = face_geometry(old)
    display_camera = frame['world_optical'].copy()
    display_camera[:3, 3] = model.warp(display_camera[:3, 3][None], name)[0]
    display_frame = {**frame, 'world_optical': display_camera}
    pixels, z = project(centre, frame['k_rgb'], display_camera)
    semantics = json.loads((base/'semantic_objects.json').read_text())
    protected = np.zeros(len(old), bool)
    for obj in semantics['objects']:
        if obj['session'] != name or obj.get('class') not in ['door', 'window', 'sign'] or not obj.get('geometry', {}).get('accepted'):
            continue
        boundary = np.asarray(obj['geometry']['world_boundary'])
        protected |= ((old >= boundary.min(0)-.08) & (old <= boundary.max(0)+.08)).all(2).any(1)
    for h in [-1.55, 2.60]:
        protected |= np.max(abs(old[:, :, 2]-h), axis=1) < 2e-5
    # Already verified cavity skins remain entirely untouched.
    prior_source = Path(run['mesh_finish_source_manifest']).parent
    prior = json.loads((prior_source/'manifest.json').read_text())
    source_ids = np.load(base/name/'finish_provenance.npz')['retained_source_faces']
    protected |= source_ids >= prior['sessions'][name]['closure_retained_triangles']
    keep = np.ones(len(old), bool)
    models, uvs, textures, records = [], [], [], []
    shadow_cfg = run['shadow_lift']['config']
    original_photo = dest/'final_wall_lobby445_original.png'
    Image.fromarray(frame['rgb']).save(original_photo)
    o3d.utility.random.seed(20261019)
    for region in cfg['regions']:
        points = polygon_points(frame, region['polygon'])
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        plane, ids = cloud.segment_plane(.035, 3, 1500)
        n = np.asarray(plane[:3])
        c = points[np.asarray(ids)].mean(0)
        residual = abs((points-c)@n)
        assert len(ids)/len(points) >= cfg['minimum_inlier_fraction']
        assert np.quantile(residual, .9) < cfg['maximum_depth_plane_p90_m']
        assert abs(n[2]) < .12, 'Do not turn stairs into a wall plane'
        built, geometry = photo_mesh(o3d, frame, region, (c, n), dest)
        assert built is not None and geometry['accepted']
        patch, uv, texture = built
        # Centroid-only Delaunay clipping can bridge a concave doorway. Check
        # vertices, edge midpoints and centroid of each proposed triangle too.
        polygon = np.asarray(region['polygon'], np.float32)
        lo, hi = np.floor(polygon.min(0)), np.ceil(polygon.max(0))
        pixel_triangles = uv*(hi-lo)+lo
        samples = np.concatenate([pixel_triangles, pixel_triangles.mean(1)[:, None],
            (pixel_triangles+pixel_triangles[:, [1, 2, 0]])/2], axis=1)
        valid = np.array([all(cv2.pointPolygonTest(polygon, tuple(map(float, point)), True) >= -.25
                              for point in triangle) for triangle in samples])
        patch.triangles = o3d.utility.Vector3iVector(np.asarray(patch.triangles)[valid])
        uv = uv[valid]
        geometry['triangles'] = len(uv)
        geometry['concave_photo_boundary_samples_checked'] = True
        patch.vertices = o3d.utility.Vector3dVector(model.warp(np.asarray(patch.vertices), name))
        patch.compute_vertex_normals()
        photo = np.asarray(Image.open(texture).convert('RGB'))
        bright = correct_shadow_texture(photo, np.full(photo.shape[:2], 255, np.uint8), shadow_cfg)
        Image.fromarray(bright).save(texture)
        geometry['texture_sha256_before_shadow_rule'] = geometry['texture_sha256']
        geometry['texture_sha256'] = sha256_file(texture)
        display_c = model.warp(c[None], name)[0]
        eligible = inside_polygon(pixels, region['polygon']) & (z > .25) & (z < 8)
        distance = abs((centre-display_c)@n)
        eligible &= (distance < cfg['maximum_plane_distance_m']) & (abs(normals@n) > cfg['minimum_normal_agreement']) & ~protected
        for polygon in region.get('protect_photo_polygons', []):
            eligible &= ~inside_polygon(pixels, polygon)
        keep &= ~eligible
        geometry['source_world_boundary'] = geometry['world_boundary']
        geometry['world_boundary'] = model.warp(np.asarray(geometry['world_boundary']), name).tolist()
        geometry['world_centre'] = model.warp(np.asarray(geometry['world_centre'])[None], name)[0].tolist()
        records.append({**region, 'session': name, 'node': cfg['node'], 'geometry': geometry,
            'source_rgb': original_photo.name, 'source_rgb_sha256': sha256_file(original_photo),
            'world_optical': frame['world_optical'].tolist(), 'K': frame['k_rgb'].tolist(),
            'source_depth_samples': len(points), 'source_plane_inlier_fraction': len(ids)/len(points),
            'source_plane_p90_m': float(np.quantile(residual, .9)), 'removed_old_wall_faces': int(eligible.sum()),
            'texture_sha256_after_existing_shadow_rule': sha256_file(texture),
            'source_RGB_photo_sha256': hashlib.sha256(frame['rgb'].tobytes()).hexdigest()})
        models.append(patch); uvs.append(uv); textures.append(texture)
    keep = covered_by_actual_patches(o3d, old, protected, models, records, display_camera, frame['k_rgb'], cfg)
    filter_glb(base/name/'clean_textured.glb', dest/'retained_before_wall.glb', keep)
    append_planes(dest/'retained_before_wall.glb', dest/'clean_textured.glb', models, uvs, textures)
    mesh.remove_triangles_by_mask(~keep)
    mesh.remove_unreferenced_vertices()
    for patch in models:
        mesh += patch
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'), mesh)
    actual = glb_triangle_positions(dest/'clean_textured.glb')
    assert np.array_equal(actual, np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32))
    assert np.array_equal(actual[:keep.sum()], old[keep])
    np.save(dest/'reviewed_wall_retained_faces.npy', keep)
    sessions = dict(run['sessions'])
    sessions[name] = {**sessions[name], 'reviewed_wall_finish': records}
    session_removal = {name: int((~keep).sum())}
    # One photographed local surface must own this region across every capture,
    # otherwise an unedited bridge/stair session can still occlude the new wall.
    for other in run['sessions']:
        if other == name:
            continue
        target = a.output/other
        target.mkdir()
        other_mesh = o3d.io.read_triangle_mesh(str(base/other/'clean_structural.ply'))
        xyz = np.asarray(other_mesh.vertices)[np.asarray(other_mesh.triangles)].astype(np.float32)
        centres = xyz.mean(1)
        normal, _ = face_geometry(xyz)
        pixel, optical_z = project(centres, frame['k_rgb'], display_camera)
        guard = np.zeros(len(xyz), bool)
        for obj in semantics['objects']:
            if obj['session'] != other or not obj.get('geometry', {}).get('accepted'):
                continue
            boundary = np.asarray(obj['geometry']['world_boundary'])
            guard |= ((xyz >= boundary.min(0)-.08) & (xyz <= boundary.max(0)+.08)).all(2).any(1)
        for height in [-1.55, 2.60]:
            guard |= np.max(abs(xyz[:, :, 2]-height), axis=1) < 2e-5
        source_face = np.load(base/other/'finish_provenance.npz')['retained_source_faces']
        guard |= source_face >= prior['sessions'][other]['closure_retained_triangles']
        retain = covered_by_actual_patches(o3d, xyz, guard, models, records, display_camera, frame['k_rgb'], cfg)
        filter_glb(base/other/'clean_textured.glb', target/'clean_textured.glb', retain)
        other_mesh.remove_triangles_by_mask(~retain)
        other_mesh.remove_unreferenced_vertices()
        other_mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(target/'clean_structural.ply'), other_mesh)
        assert np.array_equal(glb_triangle_positions(target/'clean_textured.glb'), xyz[retain])
        assert retain[guard].all()
        np.save(target/'reviewed_wall_retained_faces.npy', retain)
        session_removal[other] = int((~retain).sum())
        sessions[other] = {**sessions[other], 'reviewed_wall_overlap_faces_removed': int((~retain).sum())}
    for session in run['sessions']:
        target = a.output/session
        target.mkdir(exist_ok=True)
        for path in (base/session).iterdir():
            if path.is_file() and path.name not in ['clean_textured.glb', 'clean_structural.ply']:
                # Large obsolete historical meshes are still in the source run.
                if path.suffix in ['.glb', '.ply'] and path.name not in ['clean_textured.glb', 'clean_structural.ply']:
                    continue
                shutil.copyfile(path, target/path.name)
    for path in base.iterdir():
        if path.is_file() and path.name not in ['manifest.json', 'cleanup.json', 'geometry_audit.json', 'material_audit.json', 'semantic_objects.json', 'semantic_binding_audit.json']:
            shutil.copyfile(path, a.output/path.name)
    semantics['objects'] += [{**r, 'mesh_face_binding_status': 'UNBOUND_USE_STABLE_ID_AND_WORLD_BOUNDARY', 'passability_verified': False} for r in records]
    (a.output/'semantic_objects.json').write_text(json.dumps(semantics, indent=2, ensure_ascii=False)+'\n')
    result = {**run, 'created_utc': utc_now(), 'sessions': sessions, 'reviewed_wall_finish': cfg,
        'reviewed_wall_source_manifest': str((base/'manifest.json').resolve()),
        'inputs_sha256': {**run['inputs_sha256'], **{str(p.resolve()): sha256_file(p) for p in [base/'manifest.json', a.config, Path(__file__), db]}},
        'scope': run['scope']+' '+cfg['scope']}
    (a.output/'manifest.json').write_text(json.dumps(result, indent=2)+'\n')
    cleanup = {n: {'glb_sha256': sha256_file(a.output/n/'clean_textured.glb'), 'ply_sha256': sha256_file(a.output/n/'clean_structural.ply')} for n in sessions}
    (a.output/'cleanup.json').write_text(json.dumps({'source_manifest_sha256': sha256_file(a.output/'manifest.json'), 'sessions': cleanup}, indent=2)+'\n')
    report = {'retained_coordinates_exact': True, 'actual_GLB_PLY_equal': True,
        'protected_door_sign_floor_and_cavity_faces_not_removed': bool(keep[protected].all()),
        'removed_wall_faces': sum(session_removal.values()), 'removed_wall_faces_by_session': session_removal,
        'new_source_photo_bounded_wall_faces': sum(len(p.triangles) for p in models),
        'regions': records, 'source_RGB_images_changed': False, 'pose_changes': 0,
        'source_geometry_audit': json.loads((base/'geometry_audit.json').read_text()),
        'bundle_manifest_sha256': sha256_file(a.output/'manifest.json'), 'scope': cfg['scope']}
    (a.output/'geometry_audit.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['regions','source_geometry_audit']}), flush=True)


if __name__ == '__main__':
    main()
