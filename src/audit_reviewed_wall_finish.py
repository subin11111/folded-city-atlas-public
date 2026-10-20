"""Read actual final GLBs and verify the photo-bounded cross-session wall cleanup."""
import argparse
import json
from pathlib import Path

import numpy as np

from audit_structural_delivery import glb_triangle_positions
from finalize_display_mesh import face_geometry
from finalize_display_mesh import samples_on_faces
from optimize_session import sha256_file, utc_now
from refine_display_edges import unpack

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    run = json.loads((a.bundle/'manifest.json').read_text())
    base = Path(run['reviewed_wall_source_manifest']).parent
    source_run = json.loads((base/'manifest.json').read_text())
    original_run = json.loads(Path(source_run['mesh_finish_source_manifest']).read_text())
    sessions = {}
    removed_geometry = []
    replacement_geometry = None
    import open3d as o3d
    for name in run['sessions']:
        old = glb_triangle_positions(base/name/'clean_textured.glb')
        current = glb_triangle_positions(a.bundle/name/'clean_textured.glb')
        keep = np.load(a.bundle/name/'reviewed_wall_retained_faces.npy')
        assert len(keep) == len(old)
        assert np.array_equal(current[:keep.sum()], old[keep])
        normal, _ = face_geometry(old)
        horizontal_removed = int(((~keep) & (abs(normal[:, 2]) > .65)).sum())
        if not run['reviewed_wall_finish'].get('verified_bare_wall_volume_override', False):
            assert horizontal_removed == 0, 'Horizontal stair/floor surface lost'
        ids = np.load(base/name/'finish_provenance.npz')['retained_source_faces']
        fixed = ids >= original_run['sessions'][name]['closure_retained_triangles']
        for h in [-1.55, 2.60]:
            fixed |= np.max(abs(old[:, :, 2]-h), axis=1) < 2e-5
        assert keep[fixed].all(), 'Verified floor/roof/cavity skin removed'
        first, before = unpack(base/name/'clean_textured.glb')
        second, after = unpack(a.bundle/name/'clean_textured.glb')
        assert after[:len(before)] == before
        for field in ['materials', 'images', 'textures', 'samplers']:
            assert second.get(field, [])[:len(first.get(field, []))] == first.get(field, [])
        expected_new = sum(r['geometry']['triangles'] for r in run['sessions'][name].get('reviewed_wall_finish', []))
        assert len(current) == int(keep.sum())+expected_new
        removed_geometry.append(old[~keep].copy())
        if expected_new:
            replacement_geometry = current[keep.sum():].copy()
        mesh = o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'))
        assert np.array_equal(current, np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32))
        sessions[name] = {'retained_XYZ_exact': True, 'source_binary_UV_colour_photos_exact': True,
            'known_cavity_and_level_planes_preserved': True,
            'wrong_normal_facets_removed_inside_explicit_photo_wall_volume': horizontal_removed,
            'actual_GLB_PLY_equal': True, 'removed_covered_wall_faces': int((~keep).sum()),
            'source_photo_bounded_new_wall_faces': expected_new}
    old_sem = json.loads((base/'semantic_objects.json').read_text())['objects']
    current_sem = json.loads((a.bundle/'semantic_objects.json').read_text())['objects']
    assert current_sem[:len(old_sem)] == old_sem
    for obj in current_sem[len(old_sem):]:
        assert obj['class'] == 'wall' and obj['label']
        assert sha256_file(a.bundle/obj['session']/obj['source_rgb']) == obj['source_rgb_sha256']
        assert sha256_file(a.bundle/obj['session']/obj['geometry']['texture']) == obj['geometry']['texture_sha256']
        assert obj['source_plane_inlier_fraction'] >= .9 and obj['source_plane_p90_m'] < .025
    # Independent physical coverage check of every actually deleted facet.
    # It also verifies removal of wrong-normal facets only inside these walls,
    # rather than classifying all horizontal-looking triangles as stairs.
    removed_geometry = np.concatenate(removed_geometry)
    samples = samples_on_faces(removed_geometry)
    covered = np.zeros(len(samples), bool)
    cursor = 0
    band = run['reviewed_wall_finish']['maximum_plane_distance_m']
    for obj in current_sem[len(old_sem):]:
        count = obj['geometry']['triangles']
        wall = replacement_geometry[cursor:cursor+count]
        cursor += count
        scene = o3d.t.geometry.RaycastingScene(nthreads=2)
        scene.add_triangles(o3d.t.geometry.TriangleMesh(o3d.core.Tensor(wall.reshape(-1, 3)),
            o3d.core.Tensor(np.arange(wall.size//3).reshape(-1, 3).astype(np.uint32))))
        n = np.asarray(obj['geometry']['plane_normal'])
        rays = np.concatenate([samples-n*band, np.broadcast_to(n, samples.shape)], axis=-1).reshape(-1, 6).astype(np.float32)
        hit = scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy().reshape(samples.shape[:2])
        covered |= ((hit >= 0) & (hit <= 2*band)).all(1)
    assert covered.all(), 'Removed facet is not covered by the actual photographed wall export'
    pointer = json.loads((ROOT/'config/current_map.json').read_text())
    assert sha256_file(ROOT/pointer['manifest']) == run['selected_manifest_sha256']
    source_audit = json.loads((base/'geometry_audit.json').read_text())
    assert source_audit['service_cavity_boundary_edges'] == source_audit['service_cavity_nonmanifold_edges'] == 0
    result = {'created_utc': utc_now(), 'bundle_manifest_sha256': sha256_file(a.bundle/'manifest.json'),
        'source_cleanup_audit_sha256': sha256_file(base/'geometry_audit.json'), 'sessions': sessions,
        'all_existing_semantic_ids_and_world_anchors_exact': True, 'semantic_objects': len(current_sem),
        'new_bounded_photo_wall_regions': len(current_sem)-len(old_sem),
        'all_removed_wall_facets_seven_point_coverage_verified': int(covered.sum()),
        'source_cavity_remains_closed_because_all_actual_skins_retained': True,
        'source_selected_map_unchanged': True, 'source_pose_changes': 0, 'navigation_ready': False,
        'audit_source_sha256': sha256_file(Path(__file__)),
        'scope': 'Cross-session replacement only in two photographed stair-side wall polygons. Not whole-building watertightness, survey accuracy or full removal of remaining texture seams and unobserved gaps.'}
    a.output.write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
