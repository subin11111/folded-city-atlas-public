#!/usr/bin/env python3
"""Build actual, separate-session surface representations from frozen RGB-D poses.

This is appearance/reconstruction work, not map optimization or accuracy approval.
Every TSDF is independent per session. Existing map manifests/DBs are read-only.
"""
import argparse
import copy
import gc
import json
from pathlib import Path
import sqlite3
import time

import cv2
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

from export_rtabmap_db import load_pose_csv, voxel_reduce, write_binary_ply
from optimize_session import inverse, sha256_file, utc_now
from surface_model_utils import atlas_uv, decode_frame, detail_candidates, project, select_frames, write_glb

ROOT = Path(__file__).resolve().parents[1]


def dump(path, value):
    if path.exists():
        raise FileExistsError(path)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False)+'\n')


def clean_mesh(mesh, edge_limit=None):
    mesh.remove_degenerate_triangles(); mesh.remove_duplicated_triangles(); mesh.remove_duplicated_vertices()
    if edge_limit is not None:
        v, t = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
        lengths = np.stack([np.linalg.norm(v[t[:, i]]-v[t[:, j]], axis=1) for i, j in [(0, 1), (1, 2), (2, 0)]], axis=1)
        mesh.remove_triangles_by_mask(lengths.max(axis=1)>edge_limit)
    mesh.remove_unreferenced_vertices(); mesh.compute_vertex_normals(); mesh.compute_triangle_normals()
    if not mesh.has_triangles():
        raise ValueError('Surface reconstruction is empty')
    return mesh


def presentation_mesh(mesh, limit):
    result = copy.deepcopy(mesh)
    if len(result.triangles)>limit:
        result = result.simplify_quadric_decimation(limit)
    return clean_mesh(result)


def ray_scene(o3d, mesh):
    scene = o3d.t.geometry.RaycastingScene(nthreads=2)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return scene


def mesh_stats(o3d, mesh, reference, seed):
    v, t = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    edges = np.sort(np.concatenate([t[:, [0, 1]], t[:, [1, 2]], t[:, [2, 0]]]), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    points = np.asarray(reference.points)
    sample = points[np.linspace(0, len(points)-1, min(12000, len(points))).astype(int)]
    scene = ray_scene(o3d, mesh)
    distances = scene.compute_distance(o3d.core.Tensor(sample.astype(np.float32))).numpy()
    o3d.utility.random.seed(seed)
    mesh_samples = np.asarray(mesh.sample_points_uniformly(12000).points)
    reverse = cKDTree(points).query(mesh_samples, workers=2)[0]
    return {'vertices': len(v), 'triangles': len(t), 'surface_area_m2': mesh.get_surface_area(),
        'boundary_edges': int(np.count_nonzero(counts==1)), 'nonmanifold_edges': int(np.count_nonzero(counts>2)),
        'reference_points_within_5cm': float(np.mean(distances<.05)),
        'reference_points_within_10cm': float(np.mean(distances<.10)),
        'reference_to_surface_median_m': float(np.median(distances)),
        'reference_to_surface_p90_m': float(np.percentile(distances, 90)),
        'surface_samples_farther_than_10cm_from_reference': float(np.mean(reverse>.10)),
        'metric_scope': 'Reused same-sensor map self-consistency/coverage, NOT heldout or surveyed accuracy'}


def texture_mesh(mesh, frames, cfg, output):
    vertices = np.asarray(mesh.vertices); triangles = np.asarray(mesh.triangles)
    centers = vertices[triangles].mean(axis=1)
    normals = np.asarray(mesh.triangle_normals)
    best = np.zeros(len(triangles)); assignment = np.full(len(triangles), -1, int)
    ordered = list(frames); pages = (len(ordered)+cfg['columns']*cfg['rows']-1)//(cfg['columns']*cfg['rows'])
    native_shape = frames[ordered[0]]['rgb'].shape
    atlas_paths = []; uv_all = np.zeros((len(triangles), 3, 2), np.float32)
    for k, node in enumerate(ordered):
        frame = frames[node]; h, w = frame['depth'].shape
        uv, z = project(centers, frame['k_depth'], frame['world_optical'])
        x, y = np.rint(uv).astype(int).T
        inside = (z>.25)&(z<4.2)&(x>=0)&(x<w)&(y>=0)&(y<h)
        ids = np.flatnonzero(inside)
        if not len(ids):
            continue
        observed = frame['depth'][y[ids], x[ids]]
        direction = frame['world_optical'][:3, 3]-centers[ids]
        cosine = np.abs(np.sum(normals[ids]*direction, axis=1))/np.maximum(np.linalg.norm(direction, axis=1), 1e-9)
        valid = (observed>0)&(abs(observed-z[ids])<=cfg['depth_tolerance_m']+cfg['relative_depth_tolerance']*z[ids])
        valid &= cosine>=cfg['minimum_abs_view_cosine']
        score = cosine**2/np.maximum(z[ids], .5)
        candidate = ids[valid&(score>best[ids])]
        if not len(candidate):
            continue
        photo_uv, corner_z = project(vertices[triangles[candidate].reshape(-1)], frame['k_rgb'], frame['world_optical'])
        photo_uv = photo_uv.reshape(-1, 3, 2); corner_z = corner_z.reshape(-1, 3)
        ih, iw = frame['rgb'].shape[:2]
        visible = (corner_z>.25).all(axis=1)&(photo_uv[:, :, 0]>=1).all(axis=1)&(photo_uv[:, :, 0]<iw-1).all(axis=1)
        visible &= (photo_uv[:, :, 1]>=1).all(axis=1)&(photo_uv[:, :, 1]<ih-1).all(axis=1)
        candidate = candidate[visible]; photo_uv = photo_uv[visible]
        if not len(candidate):
            continue
        # Reject triangles straddling different depth layers at their corners too.
        cuv, cz = project(vertices[triangles[candidate].reshape(-1)], frame['k_depth'], frame['world_optical'])
        cx, cy = np.rint(cuv).astype(int).T
        cx = np.clip(cx, 0, w-1); cy = np.clip(cy, 0, h-1)
        measured = frame['depth'][cy, cx]
        gate = ((measured>0)&(abs(measured-cz)<.12+.03*cz)).reshape(-1, 3).all(axis=1)
        candidate = candidate[gate]; photo_uv = photo_uv[gate]
        assignment[candidate] = k
        best[candidate] = cosine[np.searchsorted(ids, candidate)]**2/np.maximum(z[candidate], .5)
        uv_all[candidate] = atlas_uv(photo_uv.reshape(-1, 2), k%(cfg['columns']*cfg['rows']),
            native_shape, cfg['atlas_side'], cfg['columns'], cfg['rows']).reshape(-1, 3, 2)
    per_page = cfg['columns']*cfg['rows']
    for page in range(pages):
        atlas = Image.new('RGB', (cfg['atlas_side'], cfg['atlas_side']), (90, 90, 90))
        for slot, node in enumerate(ordered[page*per_page:(page+1)*per_page]):
            image = frames[node]['rgb']
            if image.shape != native_shape:
                raise ValueError('Variable native image shape must not be silently rescaled')
            atlas.paste(Image.fromarray(image), ((slot%cfg['columns'])*native_shape[1], (slot//cfg['columns'])*native_shape[0]))
        path = output/f'atlas_{page:02d}.png'; atlas.save(path); atlas_paths.append(path)
    face_pages = np.where(assignment>=0, assignment//per_page, -1)
    glb = write_glb(output/'photo_textured.glb', vertices, triangles, np.asarray(mesh.vertex_normals),
                    np.asarray(mesh.vertex_colors), face_pages=face_pages, face_uv=uv_all, atlas_paths=atlas_paths)
    return {'glb': glb, 'atlas_pages': len(atlas_paths), 'camera_frames': ordered,
        'native_image_shape': list(native_shape), 'native_photographs_resized': False,
        'textured_face_fraction': float(np.mean(assignment>=0)),
        'unassigned_face_fraction': float(np.mean(assignment<0)),
        'texture_scope': 'Single best photo per face; depth-checked projective UV, no generated or averaged lettering',
        'not_guaranteed': 'No semantic sign recognition, no proof of correct text, remaining pose/visibility/seam errors',
        'camera_assignment': assignment.tolist()}


def build_session(o3d, name, manifest, bundle, cfg, output, inputs):
    start = time.monotonic(); output.mkdir(parents=True, exist_ok=False)
    record = manifest['sessions'][name]
    pose_path = bundle/record['trajectory']; cloud_path = bundle/record['cloud']
    db = Path(manifest['database_integrity'][name]['database'])
    for p in [pose_path, cloud_path, db]:
        inputs[str(p.resolve())] = sha256_file(p)
    if inputs[str(db.resolve())]!=manifest['database_integrity'][name]['sha256']:
        raise ValueError('Original capture differs from the frozen selected-map database')
    poses = load_pose_csv(pose_path)
    connection = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    profile = record['preprocessing']; volumes = {}
    for variant in cfg['tsdf']:
        volumes[variant['id']] = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=variant['voxel_m'], sdf_trunc=variant['truncation_m'],
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    chosen = select_frames(poses, cfg['texture']['maximum_frames'][name], cfg['representative_frames'][name])
    frames = {}; retained = 0
    for index, node in enumerate(sorted(poses)):
        frame = decode_frame(connection, node, profile, poses[node]); depth = frame['depth']; h, w = depth.shape
        retained += np.count_nonzero(depth)
        color = cv2.resize(frame['rgb'], (w, h), interpolation=cv2.INTER_AREA)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.ascontiguousarray(color)),
            o3d.geometry.Image(depth), depth_scale=1., depth_trunc=profile['max_depth']+.001, convert_rgb_to_intensity=False)
        k = frame['k_depth']; intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, k[0, 0], k[1, 1], k[0, 2], k[1, 2])
        for volume in volumes.values():
            volume.integrate(rgbd, intrinsic, inverse(frame['world_optical']))
        if node in chosen:
            frames[node] = frame
        if index%100==0:
            print(json.dumps({'session': name, 'integrating': index+1, 'total': len(poses)}), flush=True)
    reference = o3d.io.read_point_cloud(str(cloud_path)); meshes = {}; result = {}
    for variant, volume in volumes.items():
        mesh = clean_mesh(volume.extract_triangle_mesh())
        if not o3d.io.write_triangle_mesh(str(output/(variant+'.ply')), mesh):
            raise ValueError('Failed mesh export')
        result[variant] = mesh_stats(o3d, mesh, reference, 20261017)
        display = presentation_mesh(mesh, cfg['presentation_max_triangles'])
        o3d.io.write_triangle_mesh(str(output/(variant+'_display.ply')), display)
        write_glb(output/(variant+'.glb'), np.asarray(display.vertices), np.asarray(display.triangles),
                  np.asarray(display.vertex_normals), np.asarray(display.vertex_colors))
        meshes[variant] = display
        del mesh
    del volumes; gc.collect()
    print(name+': ball pivoting', flush=True)
    reference.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=cfg['ball_pivoting']['normal_radius_m'],
        max_nn=cfg['ball_pivoting']['normal_max_nn']))
    points = np.asarray(reference.points); normals = np.asarray(reference.normals)
    cameras = np.array([p[:3, 3] for p in poses.values()])
    _, nearest = cKDTree(cameras).query(points, workers=2)
    normals[np.sum(normals*(cameras[nearest]-points), axis=1)<0] *= -1
    reference.normals = o3d.utility.Vector3dVector(normals)
    bpa = clean_mesh(o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(reference,
        o3d.utility.DoubleVector(cfg['ball_pivoting']['radii_m'])), cfg['ball_pivoting']['maximum_triangle_edge_m'])
    o3d.io.write_triangle_mesh(str(output/'ball_pivoting.ply'), bpa)
    result['ball_pivoting'] = mesh_stats(o3d, bpa, reference, 20261017)
    bdisplay = presentation_mesh(bpa, cfg['presentation_max_triangles'])
    o3d.io.write_triangle_mesh(str(output/'ball_pivoting_display.ply'), bdisplay)
    write_glb(output/'ball_pivoting.glb', np.asarray(bdisplay.vertices), np.asarray(bdisplay.triangles),
              np.asarray(bdisplay.vertex_normals), np.asarray(bdisplay.vertex_colors))
    del bpa, bdisplay; gc.collect()
    print(name+': native photo texture', flush=True)
    texture = texture_mesh(meshes['tsdf_3cm'], frames, cfg['texture'], output)
    # Store the assignment separately: no need to copy a huge index list into summary docs.
    assignment = texture.pop('camera_assignment')
    np.save(output/'texture_frame_assignment.npy', np.asarray(assignment, np.int32))
    detail_points, detail_colors = [], []
    full_fine = o3d.io.read_triangle_mesh(str(output/'tsdf_3cm.ply')); scene = ray_scene(o3d, full_fine)
    for frame in frames.values():
        p, c = detail_candidates(frame, cfg['detail_overlay'])
        if not len(p):
            continue
        distance = scene.compute_distance(o3d.core.Tensor(p.astype(np.float32))).numpy()
        keep = (distance>=cfg['detail_overlay']['minimum_mesh_distance_m'])&(distance<=cfg['detail_overlay']['maximum_mesh_distance_m'])
        detail_points.append(p[keep]); detail_colors.append(c[keep])
    if detail_points:
        pp, cc, _ = voxel_reduce(np.concatenate(detail_points), np.concatenate(detail_colors), cfg['detail_overlay']['voxel_m'])
    else:
        pp, cc = np.empty((0, 3)), np.empty((0, 3), np.uint8)
    write_binary_ply(output/'detail_overlay.ply', pp, cc)
    # Export calibrated native views for optional neural appearance experiments.
    cameras_dir = output/'calibrated_views'; cameras_dir.mkdir()
    neural = {'camera_model': 'OPENCV', 'pose_convention': 'OpenGL camera_to_world; original map is z-up', 'frames': []}
    vision_to_gl = np.diag([1., -1., -1., 1.])
    for frame in frames.values():
        filename = f'{frame["id"]:05d}.png'; Image.fromarray(frame['rgb']).save(cameras_dir/filename)
        k = frame['k_rgb']; h, w = frame['rgb'].shape[:2]
        neural['frames'].append({'file_path': filename, 'w': w, 'h': h, 'fl_x': float(k[0, 0]), 'fl_y': float(k[1, 1]),
            'cx': float(k[0, 2]), 'cy': float(k[1, 2]), 'k1': 0., 'k2': 0., 'p1': 0., 'p2': 0.,
            'transform_matrix': (frame['world_optical']@vision_to_gl).tolist(), 'frame_id': frame['id'],
            'scope': 'Existing RGB/selected pose; not a COLMAP bundle adjustment or trained Gaussian model'})
    dump(cameras_dir/'transforms.json', neural)
    connection.close()
    summary = {'session': name, 'candidate': record['candidate'], 'selected_frames': len(poses),
        'valid_depth_samples_integrated_per_variant': int(retained), 'texture_frames': len(frames),
        'preprocessing': profile, 'models': result, 'texture': texture,
        'presentation_max_triangles': cfg['presentation_max_triangles'],
        'presentation_simplification_scope': 'Quadric-decimated DISPLAY mesh; un-decimated TSDF/BPA PLYs retained. Texture/3cm display use identical decimated geometry.',
        'detail_overlay_points': len(pp), 'detail_overlay_scope': 'Image/depth-edge candidates with valid depth and 2.5-25cm fine-mesh discrepancy; NOT semantic signs',
        'pose_changes': 0, 'cross_session_fusion': False, 'gaussian_training_executed': False,
        'source_inputs': {str(p.resolve()): inputs[str(p.resolve())] for p in [pose_path, cloud_path, db]},
        'seconds': time.monotonic()-start, 'outputs': {str(p.relative_to(output)): {'bytes': p.stat().st_size,
            'sha256': sha256_file(p)} for p in output.rglob('*') if p.is_file()}}
    dump(output/'summary.json', summary)
    print(json.dumps({'session_complete': name, 'models': {k:v['triangles'] for k,v in result.items()},
        'textured_fraction': texture['textured_face_fraction'], 'detail_points': len(pp)}), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--resume', action='store_true', help='Only reuse completed sessions after source/config hashes match')
    parser.add_argument('--sessions', nargs='+')
    args = parser.parse_args()
    import open3d as o3d
    cv2.setNumThreads(2)
    pointer_path = ROOT/'config/current_map.json'; pointer = json.loads(pointer_path.read_text())
    manifest_path = ROOT/pointer['manifest']
    if sha256_file(manifest_path)!=pointer['manifest_sha256']:
        raise ValueError('Selected working map pointer mismatch')
    manifest = json.loads(manifest_path.read_text()); config_path = ROOT/'config/student_union_surface_portfolio.json'
    cfg = json.loads(config_path.read_text()); output = args.output.resolve()
    inputs = {str(p.resolve()): sha256_file(p) for p in [pointer_path, manifest_path, config_path,
        Path(__file__), ROOT/'src/surface_model_utils.py', ROOT/'src/session_preprocessing.py',
        ROOT/'src/export_rtabmap_db.py', ROOT/'src/optimize_session.py']}
    started = {'created_utc': utc_now(), 'input_hashes': inputs, 'config': cfg, 'open3d': o3d.__version__,
        'status': 'BUILDING_DISPLAY_CANDIDATES', 'source_map_replaced': False, 'navigation_ready': False}
    if output.exists():
        if not args.resume:
            raise FileExistsError('Fresh output required unless explicitly resuming')
        prior = json.loads((output/'started.json').read_text())
        if prior['input_hashes']!=inputs:
            raise ValueError('Cannot resume with changed source/config')
    else:
        output.mkdir(parents=True); dump(output/'started.json', started)
    summaries = {}
    for name in args.sessions or manifest['sessions']:
        session_output = output/name
        if session_output.exists():
            if not args.resume or not (session_output/'summary.json').exists():
                raise ValueError('Incomplete session directory must be preserved and inspected, not overwritten')
            summaries[name] = json.loads((session_output/'summary.json').read_text())
            inputs.update(summaries[name]['source_inputs'])
            for rel, record in summaries[name]['outputs'].items():
                if sha256_file(session_output/rel)!=record['sha256']:
                    raise ValueError('Completed surface output hash changed')
            continue
        summaries[name] = build_session(o3d, name, manifest, manifest_path.parent, cfg, session_output, inputs)
    if not all(sha256_file(Path(path))==h for path,h in inputs.items()):
        raise ValueError('Frozen source/map/database changed')
    dump(output/'manifest.json', {**started, 'status': 'DISPLAY_PORTFOLIO_BUILT_NOT_NAVIGATION_GEOMETRY',
        'finished_utc': utc_now(), 'all_inputs_unchanged': True, 'input_hashes_after': inputs, 'sessions': summaries,
        'limitations': ['Same-source geometry statistics are not new accuracy validation.',
            'Remaining selected-map registration errors appear in meshes too.',
            'No smoothing/hole filling/cross-session TSDF; TSDF fusion itself still changes local surfaces.',
            'Image texels preserve observed letters, not readable or correct OCR by guarantee.',
            'Detail candidates are not semantic sign detections.', 'No Gaussian splatting training executed.']})
    print(json.dumps({'complete': True, 'output': str(output), 'sessions': list(summaries)}), flush=True)


if __name__=='__main__':
    main()
