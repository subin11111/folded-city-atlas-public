#!/usr/bin/env python3
"""First-hit diagnostic of reviewed lower-floor signs against display ceilings."""
import argparse
import json
from pathlib import Path
import numpy as np
from optimize_session import sha256_file
from presentation_geometry import LevelPresentation

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    run = json.loads((a.bundle/'manifest.json').read_text())
    objects = json.loads((a.bundle/'semantic_objects.json').read_text())['objects']
    road = json.loads((ROOT/'outputs/student_union_roadview_v1/roadview.json').read_text())['nodes']
    model = LevelPresentation(ROOT, run['presentation_config'])
    import open3d as o3d
    scene = o3d.t.geometry.RaycastingScene()
    lookup = {}
    for name in run['sessions']:
        mesh = o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'))
        mesh.compute_triangle_normals()
        key = scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
        lookup[key] = (name, np.asarray(mesh.triangle_normals))
    records = []
    for obj in objects:
        if obj['class'] != 'sign' or not obj.get('geometry', {}).get('accepted'):
            continue
        boundary = np.asarray(obj['geometry']['world_boundary'])
        if boundary[:, 2].max() > 2.6 or len(boundary) != 4:
            continue
        view = next(v for v in road if v['session'] == obj['session'] and v['node'] == obj['node'])
        eye = model.warp(np.asarray(view['world_optical'])[:3, 3][None], obj['session'])[0]
        y, x = np.indices((12, 30))
        u, v = (x.ravel()+.5)/30, (y.ravel()+.5)/12
        target = ((1-u)*(1-v))[:, None]*boundary[0]+(u*(1-v))[:, None]*boundary[1]+(u*v)[:, None]*boundary[2]+((1-u)*v)[:, None]*boundary[3]
        rays = np.c_[np.broadcast_to(eye, target.shape), target-eye].astype(np.float32)
        hits = scene.cast_rays(o3d.core.Tensor(rays))
        t = hits['t_hit'].numpy()
        blocked = t < .99
        roof = np.zeros(len(t), bool)
        details = []
        for key, (name, normals) in lookup.items():
            ids = np.flatnonzero(blocked & (hits['geometry_ids'].numpy() == key))
            if not len(ids):
                continue
            face_ids = hits['primitive_ids'].numpy()[ids].astype(int)
            points = eye+(target[ids]-eye)*t[ids, None]
            horizontal = abs(normals[face_ids, 2]) > .85
            lower_roof = horizontal & (points[:, 2] > .8) & (points[:, 2] < 2.6)
            roof[ids[lower_roof]] = True
            details.append({'session': name, 'hit_z_minmax': [float(points[:, 2].min()), float(points[:, 2].max())],
                            'horizontal_occluding_samples': int(lower_roof.sum())})
        record = {'id': obj['id'], 'session': obj['session'], 'node': obj['node'],
                  'sign_z_minmax': boundary[:, 2].min().item(), 'sign_top_m': boundary[:, 2].max().item(),
                  'blocked_samples': int(blocked.sum()), 'ceiling_blocked_samples': int(roof.sum()),
                  'upper_quarter_ceiling_blocked_samples': int(roof[v < .25].sum()),
                  'total_samples': len(t), 'occluders': details}
        records.append(record)
        print(json.dumps(record), flush=True)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps({'bundle_manifest_sha256': sha256_file(a.bundle/'manifest.json'), 'signs': records,
                                   'scope': 'Display-model ray/occlusion diagnostic, not sign OCR or surveyed height.'}, indent=2)+'\n')


if __name__ == '__main__':
    main()
