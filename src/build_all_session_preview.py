#!/usr/bin/env python3
"""Combine explicitly exploratory session placements without new graph fitting."""
import argparse
import csv
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from export_rtabmap_db import load_pose_csv, voxel_reduce, write_binary_ply
from optimize_session import sha256_file, trajectory, utc_now
from render_session_report import read_ply
from revalidate_loop_constraints import delta_summary, holdout_score


def checked_rigid(transform):
    value = np.asarray(transform, dtype=float)
    if value.shape != (4, 4) or not np.isfinite(value).all():
        raise ValueError('Expected a finite 4x4 rigid transform')
    if not np.allclose(value[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError('Invalid homogeneous transform')
    rotation = value[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or abs(np.linalg.det(rotation)-1) > 1e-6:
        raise ValueError('Scaling, reflection or deformation is forbidden')
    return value


def compose_outside(joint_from_bridge, outside_from_bridge):
    return checked_rigid(joint_from_bridge) @ np.linalg.inv(checked_rigid(outside_from_bridge))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, default=Path('outputs/student_union_20261017_all_preview'))
    args = parser.parse_args()
    target = args.run / 'bundle'
    if target.exists():
        raise FileExistsError('Refusing to overwrite a preview bundle: ' + str(target))
    reports = {
        'lobby_bridge': args.run/'lobby_bridge/attachment.json',
        'outside_bridge': args.run/'outside_bridge/attachment_candidate.json',
        'substairs': args.run/'substairs/attachment_report.json',
    }
    evidence = {k:json.loads(p.read_text()) for k,p in reports.items()}
    bridge, outside, sub = (evidence[k] for k in ('lobby_bridge','outside_bridge','substairs'))
    if bridge['accepted_by_validation'] is not False or sub['accepted_by_validation'] is not False or outside['strict_accepted'] is not False:
        raise ValueError('This assembly is specifically an unvalidated preview, not an approval exporter')
    t_bridge = checked_rigid(bridge['T_joint_bridge'])
    outside_variant = 'gravity_union_disjoint_4d'
    t_outside_bridge = outside['alternatives'][outside_variant]
    t_outside = compose_outside(t_bridge, t_outside_bridge)
    t_sub = checked_rigid(sub['T_joint_sub'])
    joint = Path('outputs/student_union_20261017_revalidated/joint_disjoint_hypothesis/component_0')
    config = json.loads(Path('config/student_union_all_sessions.json').read_text())['sessions']
    joint_report = json.loads((joint/'joint_report.json').read_text())
    if joint_report['accepted'] is not False:
        raise ValueError('Expected the explicitly rejected parent hypothesis')
    # Cross-check the bridge source coordinate convention before composition.
    if Path(bridge['source_paths']['bridge_poses']).resolve() != Path(outside['pose_csv_b']['path']).resolve():
        raise ValueError('Bridge pose coordinate sources differ')
    specs = {
        'lobby': (joint/'preview_clouds/lobby_candidate.ply', joint/'lobby/candidate_trajectory.csv', np.eye(4), 'existing_joint_hypothesis'),
        'floor2': (joint/'preview_clouds/floor2_candidate.ply', joint/'floor2/candidate_trajectory.csv', np.eye(4), 'existing_joint_hypothesis'),
        'bridge_out1f': (Path(outside['cloud_b']['path']), Path(bridge['source_paths']['bridge_poses']), t_bridge, 'one_doorway_4dof_rigid_hypothesis'),
        'outside': (Path(outside['cloud_a']['path']), Path(outside['pose_csv_a']['path']), t_outside, 'bridge_chained_4dof_rigid_hypothesis'),
        'bridge_12_sub': (Path(sub['source_cloud']), Path(sub['source_pose_csv']), t_sub, 'two_region_4dof_rigid_hypothesis'),
    }
    target.mkdir(parents=True)
    manifest = {'started_utc':utc_now(), 'argv':sys.argv,
        'status':'EXPLORATORY_ALL_SESSION_PREVIEW_NOT_FOR_NAVIGATION',
        'unvalidated_preview':True, 'accepted_by_validation':False, 'navigation_ready':False,
        'frame':'map_all_preview', 'combined_cloud':'combined.ply', 'sessions':{},
        'method':'Rigid placements of separately reconstructed clouds; NO new all-session pose graph optimization',
        'sources':{str(p):sha256_file(p) for p in [*reports.values(),joint/'joint_report.json',Path(__file__)]},
        'connection_reports':{k:str(p.resolve()) for k,p in reports.items()},
        'outside_variant':outside_variant,
        'T_outside_bridge_used':t_outside_bridge,
        'limitations':[
            'Parent lobby/floor2 hypothesis fails its 0.15m held-out connection guard.',
            'Lobby/bridge placement uses one shared doorway, failing the strict reprojection gate and independent-support requirement.',
            'Outside/bridge fit uses correlated shared-entrance observations with failed strict gates; not independent global validation.',
            'Substairs has two fitting regions; audit geometry worsens by about 5cm against direct measurements.',
            'Chained transform uncertainty is not calibrated or propagated into a navigation covariance.',
        ]}
    xyzs, rgbs, overview = [], [], []
    for name, (cloud_path, pose_path, transform, method) in specs.items():
        transform = checked_rigid(transform)
        if not np.allclose(transform[2,:3], [0,0,1], atol=1e-6):
            raise ValueError('Expected gravity-preserving placement, not a tilted or flattened session')
        cloud = read_ply(cloud_path)
        xyz = np.column_stack([cloud[k] for k in ('x','y','z')])
        rgb = np.column_stack([cloud[k] for k in ('r','g','b')])
        moved = (xyz @ transform[:3,:3].T + transform[:3,3]).astype(np.float32)
        if not np.isfinite(moved).all():
            raise ValueError('Nonfinite moved cloud')
        write_binary_ply(target/f'{name}.ply', moved, rgb)
        poses = load_pose_csv(pose_path)
        with pose_path.open(newline='') as stream:
            stamps = {int(row['id']):float(row['stamp']) for row in csv.DictReader(stream)}
        placed = {node:transform @ pose for node,pose in poses.items()}
        trajectory(target/f'{name}_trajectory.csv', placed, stamps)
        manifest['sessions'][name] = {'cloud':f'{name}.ply','trajectory':f'{name}_trajectory.csv',
            'placement':method, 'point_count':len(moved), 'pose_count':len(poses),
            'source_cloud':str(cloud_path.resolve()), 'source_trajectory':str(pose_path.resolve()),
            'T_preview_source':transform.tolist(), 'bounds_min_m':moved.min(axis=0).tolist(),
            'bounds_max_m':moved.max(axis=0).tolist(),
            'source_camera_z_range_m':float(np.ptp([p[2,3] for p in poses.values()])),
            'placed_camera_z_range_m':float(np.ptp([p[2,3] for p in placed.values()])),
            'internal_shape_changed':False, 'vertical_scale':1.0}
        for path in (cloud_path,pose_path):
            manifest['sources'][str(path)] = sha256_file(path)
        xyzs.append(moved); rgbs.append(rgb)
        overview.append((name,moved[np.linspace(0,len(moved)-1,min(50000,len(moved))).astype(int)],placed))
        print(f'placed {name}: {len(moved):,} points',flush=True)
    xyz, rgb, _ = voxel_reduce(np.concatenate(xyzs), np.concatenate(rgbs), .05)
    write_binary_ply(target/'combined.ply',xyz,rgb)
    manifest.update(combined_points=len(xyz),combined_voxel_m=.05,
                    combined_bounds_min_m=xyz.min(axis=0).tolist(),combined_bounds_max_m=xyz.max(axis=0).tolist())
    # Audit another shared-entry pair even if it failed upstream local gates.
    aposes = load_pose_csv(specs['lobby'][1]); bposes = load_pose_csv(specs['bridge_out1f'][1])
    audits = []
    for row in bridge['rows']:
        if 'lobby_node_to_outside_node' not in row:
            continue
        realized = np.linalg.inv(aposes[row['outside_id']]) @ t_bridge @ bposes[row['lobby_id']]
        corr = np.load(row['correspondence_evidence'])
        idx = corr['heldout_indices']
        audits.append({'pair':[row['outside_id'],row['lobby_id']], 'original_reason':row['reason'],
            'used_for_fit':[row['outside_id'],row['lobby_id']] in bridge['fit_pairs'],
            'pose_residual':delta_summary(np.asarray(row['lobby_node_to_outside_node']),realized),
            'reserved_feature_residual':holdout_score(realized,corr['source'][idx],corr['target'][idx])})
    manifest['lobby_bridge_placement_audit'] = audits
    fig, axes = plt.subplots(1,2,figsize=(14,7),constrained_layout=True)
    colors = ['#f59628','#2dd2ff','#f050b4','#5feb64','#aa82ff']
    for (name, points, poses), color in zip(overview,colors):
        values = np.array([p[:3,3] for p in poses.values()])
        for ax,(x,y) in zip(axes,[(0,1),(0,2)]):
            ax.scatter(points[:,x],points[:,y],s=.25,color=color,alpha=.3)
            ax.plot(values[:,x],values[:,y],lw=.7,color=color,label=name)
            ax.set_aspect('equal',adjustable='datalim'); ax.grid(alpha=.15)
    axes[0].set(xlabel='X (m)',ylabel='Y (m)'); axes[1].set(xlabel='X (m)',ylabel='Z (m)')
    axes[1].legend(fontsize=9)
    fig.suptitle('EXPLORATORY ALL-SESSION PREVIEW | placement hypotheses, not a validated map')
    fig.savefig(target/'placement_overview.png',dpi=160); plt.close(fig)
    old = json.loads(Path('docs/experiments/student_union_20261017_revalidation_metrics.json').read_text())['database_integrity']
    manifest['database_integrity'] = {name:{'sha256':sha256_file(Path(entry['database']))} for name,entry in config.items()}
    for name, integrity in manifest['database_integrity'].items():
        integrity['unchanged'] = integrity['sha256'] == old[name]['sha256_at_finalization']
    if not all(v['unchanged'] for v in manifest['database_integrity'].values()):
        raise ValueError('Original database hash changed')
    manifest['finished_utc'] = utc_now()
    (target/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'bundle':str(target.resolve()),'combined_points':len(xyz),'all_databases_unchanged':True},indent=2))


if __name__ == '__main__':
    main()
