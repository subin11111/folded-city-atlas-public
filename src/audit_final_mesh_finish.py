"""Independently inspect exported cleanup geometry, immutable appearance and cavity."""
import argparse
import json
from pathlib import Path

import numpy as np

from audit_closed_ceiling_extension import welded_edge_counts
from audit_structural_delivery import glb_triangle_positions
from complete_service_slabs import closed_slab_arrays
from optimize_session import sha256_file, utc_now
from refine_display_edges import unpack

ROOT = Path(__file__).resolve().parents[1]


def triangle_keys(xyz, tolerance=1e-5):
    points = np.rint(np.asarray(xyz)/tolerance).astype(np.int64)
    # Lexicographic vertices within each unoriented triangle.
    order = np.lexsort((points[:, :, 2], points[:, :, 1], points[:, :, 0]), axis=1)
    sorted_points = np.take_along_axis(points, order[:, :, None], axis=1)
    return np.ascontiguousarray(sorted_points.reshape(-1, 9)).view(np.dtype((np.void, 72))).ravel()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run = json.loads((args.bundle/'manifest.json').read_text())
    source = Path(run['mesh_finish_source_manifest']).parent
    cfg = run['final_mesh_finish']
    import open3d as o3d
    actual, old_semantic, old_fixed = [], [], []
    sessions = {}
    for name in run['sessions']:
        old = glb_triangle_positions(source/name/'clean_textured.glb').copy()
        new = glb_triangle_positions(args.bundle/name/'clean_textured.glb').copy()
        provenance = np.load(args.bundle/name/'finish_provenance.npz')
        selected = provenance['retained_source_faces']
        vertices, moved = provenance['source_vertices'], provenance['moved_vertices']
        assert (np.diff(selected) > 0).all()
        assert len(selected) == len(new)
        motion = np.linalg.norm(moved-vertices, axis=1)
        assert motion.max(initial=0) <= cfg['seam_join_distance_m']/2+2e-6
        # Exact retained source order is checked before replacing only seam vertices.
        key = np.ascontiguousarray(vertices).view(np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])).ravel()
        retained = old[selected].reshape(-1, 3)
        ids = np.searchsorted(key, np.ascontiguousarray(retained).view(key.dtype).ravel())
        assert np.array_equal(vertices[ids], retained)
        assert np.array_equal(new, moved[ids].reshape(-1, 3, 3))
        normals_old = np.cross(old[selected, 1]-old[selected, 0], old[selected, 2]-old[selected, 0])
        normals_new = np.cross(new[:, 1]-new[:, 0], new[:, 2]-new[:, 0])
        old_sq = np.einsum('ij,ij->i', normals_old, normals_old)
        assert (np.einsum('ij,ij->i', normals_old, normals_new) >= old_sq*(cfg['minimum_triangle_area_fraction']-1e-5)).all()
        semantic, fixed = provenance['semantic_source_faces'], provenance['fixed_source_faces']
        protected = semantic[selected] | fixed[selected]
        assert np.array_equal(new[protected], old[selected][protected])
        old_semantic.append(old[semantic])
        old_fixed.append(old[fixed])
        a, before = unpack(source/name/'clean_textured.glb')
        b, after = unpack(args.bundle/name/'clean_textured.glb')
        assert after[:len(before)] == before, 'Source geometry/UV/colour/image bytes changed'
        for field in ['materials', 'images', 'textures', 'samplers', 'nodes', 'scenes']:
            assert a.get(field) == b.get(field), field+' changed'
        mesh = o3d.io.read_triangle_mesh(str(args.bundle/name/'clean_structural.ply'))
        assert np.array_equal(new, np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32))
        actual.append(new)
        sessions[name] = {'triangles_before': len(old), 'triangles_after': len(new),
            'retained_source_order_and_positions_verified': True,
            'no_triangle_flip_or_area_collapse': True, 'maximum_seam_vertex_motion_m': float(motion.max(initial=0)),
            'retained_semantic_and_fixed_coordinates_exact': True, 'original_binary_UV_images_colour_materials_exact': True,
            'actual_GLB_PLY_equal': True, 'GLB_sha256': sha256_file(args.bundle/name/'clean_textured.glb')}
        print('Verified final mesh and unchanged colour: '+name, flush=True)
    actual = np.concatenate(actual)
    keys = triangle_keys(actual)
    # Only allowed protected-face deletions are duplicates still present elsewhere.
    assert np.isin(triangle_keys(np.concatenate(old_semantic)), keys).all(), 'Semantic geometry lost'
    assert np.isin(triangle_keys(np.concatenate(old_fixed)), keys).all(), 'Fixed floor/ceiling/cavity geometry lost'
    # Inspect actual exported top, underside and wall skins together, not masks alone.
    q = np.load(args.bundle/'closure_plan.npz')
    slab = run['ceiling_expansion_and_closure']['config']
    vertices, parts = closed_slab_arrays(q['body'], q['low'], float(q['cell']), slab['lower_extension_height_m'], slab['upper_floor_m'])
    expected = np.concatenate([vertices[part].astype(np.float32) for part in parts])
    want = triangle_keys(expected)
    order = np.argsort(keys)
    index = np.searchsorted(keys[order], want)
    assert (index < len(keys)).all() and np.array_equal(keys[order[index]], want), 'Exported service cavity skin missing'
    skin = actual[order[index]]
    boundary, nonmanifold = welded_edge_counts(skin)
    assert boundary == 0 and nonmanifold == 0, 'Service cavity reopened'
    scene = o3d.t.geometry.RaycastingScene(nthreads=2)
    scene.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(skin.reshape(-1, 3)),
        o3d.core.Tensor(np.arange(skin.size//3).reshape(-1, 3).astype(np.uint32))))
    y, x = np.where(q['body'])
    xy = q['low']+np.c_[x+.37, y+.41]*float(q['cell'])
    middle = (slab['lower_extension_height_m']+slab['upper_floor_m'])/2
    points = np.c_[xy, np.full(len(xy), middle)].astype(np.float32)
    for direction in np.r_[np.eye(3), -np.eye(3)]:
        rays = np.c_[points, np.broadcast_to(direction, points.shape)].astype(np.float32)
        assert np.isfinite(scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()).all(), 'Cavity leak after cleanup'
    pointer = json.loads((ROOT/'config/current_map.json').read_text())
    assert sha256_file(ROOT/pointer['manifest']) == run['selected_manifest_sha256']
    selected = json.loads((ROOT/pointer['manifest']).read_text())
    db_hashes = {}
    for name, spec in selected['database_integrity'].items():
        db_hashes[name] = sha256_file(Path(spec['database']))
        assert db_hashes[name] == spec['sha256']
    result = {'created_utc': utc_now(), 'bundle_manifest_sha256': sha256_file(args.bundle/'manifest.json'),
        'sessions': sessions, 'all_semantic_and_fixed_triangles_still_exist_globally': True,
        'service_cavity_boundary_edges': boundary, 'service_cavity_nonmanifold_edges': nonmanifold,
        'actual_exported_service_cavity_six_direction_rays_passed': len(points)*6,
        'floor_and_ceiling_heights_not_merged': True, 'new_large_caps': 0, 'selected_map_unchanged': True,
        'raw_database_hashes_unchanged': db_hashes, 'source_pose_changes': 0, 'navigation_ready': False,
        'audit_source_sha256': sha256_file(Path(__file__)),
        'scope': 'Final display cleanup integrity, not whole-building watertightness, surveying, photo restoration or navigation safety. Actual visual renders remain mandatory.'}
    args.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
