#!/usr/bin/env python3
"""Conditional multi-session PGO. Never connects unsupported session components.

Input constraints map node B into node A. Reuses the tested 4-DoF block-Huber
solver; initialization uses training links only, retaining the measured gravity.
Writes candidates even on rejection, but exports no rejected combined map.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from export_rtabmap_db import load_pose_csv
from optimize_session import (Clouds, Edge, connected_components, error_statistics,
    geometric_score, inverse, optimize, relative_error, sha256_file, trajectory, utc_now, write_csv)


def records(path):
    payload = json.loads(path.read_text())
    if not isinstance(payload, list):
        for key in ('edges', 'constraints', 'accepted_edges', 'accepted_frame_edges', 'records'):
            if key in payload:
                payload = payload[key]
                break
        else:
            raise ValueError(f'Unknown constraints schema: {path}')
    rows = []
    for original in payload:
        row = dict(original)
        if 'joint_holdout' in row:
            if 'held_out' in row and row['held_out'] != row['joint_holdout']:
                raise ValueError('Conflicting held-out flags')
            row['held_out'] = bool(row['joint_holdout'])
        if row.get('joint_role') == 'heldout' and not row.get('held_out'):
            raise ValueError('Conflicting held-out role')
        if row.get('held_out') and row.get('initialization_allowed'):
            raise ValueError('Held-out link cannot initialize a map')
        rows.append(row)
    return rows


def eligible_cross(rows, allow_hypotheses=False):
    """Fail closed, including on absent independent-support evidence."""
    return [r for r in rows if (r.get('eligible_for_joint') is True
            or (allow_hypotheses and r.get('hypothesis_eligible') is True))
            and r.get('local_geometry_verified') is True
            and r['session_a'] != r['session_b']
            and r.get('independent_support_count', 0) >= 3]


def session_components(names, rows):
    unseen, groups = set(names), []
    while unseen:
        group, stack = set(), [min(unseen)]
        while stack:
            name = stack.pop()
            if name in group:
                continue
            group.add(name)
            unseen.discard(name)
            for row in rows:
                if row.get('held_out', False):
                    continue
                if row.get('initialization_allowed') is False:
                    continue
                a, b = row['session_a'], row['session_b']
                if a == name and b not in group:
                    stack.append(b)
                if b == name and a not in group:
                    stack.append(a)
        groups.append(sorted(group))
    return sorted(groups, key=lambda g: (-len(g), g))


def yaw_transform(rotation, translation):
    angle = np.arctan2(rotation[1, 0] - rotation[0, 1], rotation[0, 0] + rotation[1, 1])
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec([0, 0, angle]).as_matrix()
    result[:3, 3] = translation
    return result


def initialize_maps(local, rows, root):
    """Training-only breadth-first alignment, with yaw and median translation."""
    transforms = {root: np.eye(4)}
    pending = set(local) - {root}
    while pending:
        advanced = False
        for target in sorted(pending):
            supporting = []
            for row in rows:
                if row.get('held_out', False):
                    continue
                if row.get('initialization_allowed') is False:
                    continue
                a, b = row['session_a'], row['session_b']
                z = np.asarray(row['T_a_b'], dtype=float)
                if a in transforms and b == target:
                    supporting.append((transforms[a] @ local[a][row['node_a']] @ z,
                                       local[b][row['node_b']]))
                elif b in transforms and a == target:
                    supporting.append((transforms[b] @ local[b][row['node_b']] @ inverse(z),
                                       local[a][row['node_a']]))
            if not supporting:
                continue
            angles = []
            for desired, own in supporting:
                r = (desired @ inverse(own))[:3, :3]
                angles.append(np.arctan2(r[1, 0]-r[0, 1], r[0, 0]+r[1, 1]))
            angle = np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
            r = Rotation.from_rotvec([0, 0, angle]).as_matrix()
            t = np.median([d[:3, 3] - r @ o[:3, 3] for d, o in supporting], axis=0)
            transforms[target] = yaw_transform(r, t)
            pending.remove(target)
            advanced = True
            break
        if not advanced:
            raise ValueError('Training links do not connect the requested sessions')
    return transforms


def constraint_edge(row, index, scale=1.):
    a, b = (row['session_a'], row['node_a']), (row['session_b'], row['node_b'])
    z = np.asarray(row['T_a_b'], dtype=float)
    if z.shape != (4, 4) or not np.isfinite(z).all() or not np.allclose(z[3], [0, 0, 0, 1]):
        raise ValueError('Malformed constraint transform')
    if not np.allclose(z[:3, :3].T @ z[:3, :3], np.eye(3), atol=1e-5) or np.linalg.det(z[:3, :3]) < .999:
        raise ValueError('Non-rigid constraint transform')
    # Explicit conservative noise model, not an estimated sensor covariance.
    st = max(.10, float(row.get('sigma_translation_m', .10))) * scale
    sr = np.radians(max(3., float(row.get('sigma_rotation_deg', 3.)))) * scale
    info = np.diag([1/st**2]*3 + [1/sr**2]*3)
    return Edge(index[a], index[b], 3, z, info, info, bool(row.get('held_out', False)))


def residual_rows(rows, poses, index):
    result = []
    for row in rows:
        a, b = index[(row['session_a'], row['node_a'])], index[(row['session_b'], row['node_b'])]
        e = relative_error(np.asarray(row['T_a_b']), poses[a], poses[b])
        result.append({**{k: row[k] for k in ('session_a', 'node_a', 'session_b', 'node_b')},
            'held_out': bool(row.get('held_out', False)),
            'translation_m': float(np.linalg.norm(e[:3])), 'rotation_deg': float(np.degrees(np.linalg.norm(e[3:])))})
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path('config/student_union_all_sessions.json'))
    parser.add_argument('--cross-edges', type=Path, required=True)
    parser.add_argument('--intra-edges', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--constraint-sigma-scale', type=float, default=1.)
    parser.add_argument('--max-evaluations', type=int, default=500)
    parser.add_argument('--allow-drift-hypotheses', action='store_true',
                        help='Experimental local-link hypotheses require whole-link holdout validation')
    args = parser.parse_args()
    if args.constraint_sigma_scale <= 0:
        parser.error('Constraint sigma scale must be positive')
    if args.output.exists():
        parser.error('Choose a fresh output directory')
    args.output.mkdir(parents=True)
    config = json.loads(args.config.read_text())['sessions']
    cross = eligible_cross(records(args.cross_edges), args.allow_drift_hypotheses)
    intra = [r for r in records(args.intra_edges) if r.get('eligible_for_joint') is True] if args.intra_edges else []
    groups = session_components(config, cross)
    report = {'started_utc': utc_now(), 'command': sys.argv, 'components': [],
        'method': 'training-only gravity-preserving initialization; 4DoF block-Huber PGO',
        'noise_model': 'reverified constraints use >=0.10m and >=3deg model sigma, not calibrated sensor covariance',
        'constraint_sigma_scale': args.constraint_sigma_scale,
        'allow_drift_hypotheses': args.allow_drift_hypotheses,
        'source_sha256': {str(p): sha256_file(p) for p in [Path(__file__), Path('src/optimize_session.py'), args.config, args.cross_edges] + ([args.intra_edges] if args.intra_edges else [])},
        'eligible_cross_links': len(cross), 'disconnected_sessions': [g[0] for g in groups if len(g) == 1]}
    for number, names in enumerate(groups):
        if len(names) < 2:
            continue
        output = args.output / f'component_{number}'
        output.mkdir()
        local, stamps, index, reverse, odo, loops = {}, {}, {}, {}, [], []
        stored = {}
        for name in names:
            directory = Path(config[name]['reviewed_dir'])
            for source in (directory/'optimized_main_trajectory.csv', directory/'validated_edges.json', Path(config[name]['database'])):
                report['source_sha256'][str(source)] = sha256_file(source)
            local[name] = load_pose_csv(directory / 'optimized_main_trajectory.csv')
            con = sqlite3.connect(f"file:{Path(config[name]['database']).resolve()}?mode=ro", uri=True)
            stamps[name] = dict(con.execute('SELECT id,stamp FROM Node'))
            con.close()
            stored[name] = json.loads((directory/'validated_edges.json').read_text())
            for node in sorted(local[name]):
                gid = len(index)+1
                index[(name, node)], reverse[gid] = gid, (name, node)
        new = [r for r in cross+intra if r['session_a'] in names and r['session_b'] in names
               and (r['session_a'], r['node_a']) in index and (r['session_b'], r['node_b']) in index]
        replacement = {(r['session_a'], min(r['node_a'], r['node_b']), max(r['node_a'], r['node_b']))
                       for r in new if r['session_a'] == r['session_b'] and not r.get('held_out', False)}
        for name in names:
            for key, target in [('odometry', odo), ('verified_loops', loops)]:
                for e in stored[name][key]:
                    a, b = e['from_id'], e['to_id']
                    if (name, a) not in index or (name, b) not in index:
                        continue
                    if key == 'verified_loops' and (name, min(a,b), max(a,b)) in replacement:
                        if e.get('held_out'):
                            raise ValueError('Attempt to train on an original held-out loop')
                        continue
                    target.append(Edge(index[(name,a)], index[(name,b)], e['type'], np.asarray(e['measurement']),
                        np.asarray(e['information']), np.asarray(e['effective_information']), e.get('held_out', False)))
        new_edges = [constraint_edge(r, index, args.constraint_sigma_scale) for r in new]
        root = 'outside' if 'outside' in names else ('lobby' if 'lobby' in names else names[0])
        transforms = initialize_maps(local, [r for r in new if r['session_a'] != r['session_b']], root)
        initial = {gid: transforms[name] @ local[name][node] for gid, (name,node) in reverse.items()}
        comps = connected_components(list(initial), odo + [e for e in loops+new_edges if not e.held_out])
        anchor = index[(root,min(local[root]))]
        comps = [[anchor]+[node for node in comp if node != anchor] if anchor in comp else comp for comp in comps]
        optimized, solver = optimize(initial, odo, loops+new_edges, comps, max_evaluations=args.max_evaluations)
        guards = []
        if not solver['success']: guards.append('solver_not_converged')
        if solver['active_bounds']: guards.append('active_correction_bounds')
        if solver['final_block_huber_cost'] > solver['initial_block_huber_cost']+1e-8: guards.append('objective_increased')
        rr = residual_rows([r for r in new if r['session_a'] != r['session_b']], optimized, index)
        if args.allow_drift_hypotheses:
            session_pairs = {tuple(sorted((r['session_a'],r['session_b']))) for r in new
                             if r['session_a'] != r['session_b']}
            for pair in session_pairs:
                rows = [r for r in new if tuple(sorted((r['session_a'],r['session_b']))) == pair]
                if sum(not r.get('held_out',False) for r in rows) < 3 or not any(r.get('held_out') for r in rows):
                    guards.append('_'.join(pair)+'_insufficient_training_or_whole_link_holdout')
        for field, median_limit, max_limit in [('translation_m', .15, .35), ('rotation_deg', 3., 8.)]:
            values = [r[field] for r in rr]
            if values and (np.median(values) > median_limit or max(values) > max_limit):
                guards.append('cross_constraint_' + field + '_too_large')
            held_values = [r[field] for r in rr if r['held_out']]
            if held_values and (np.median(held_values) > median_limit or max(held_values) > max_limit):
                guards.append('cross_heldout_' + field + '_too_large')
        geometries, connections, clouds = [], {}, {}
        for name in names:
            connections[name] = sqlite3.connect(f"file:{Path(config[name]['database']).resolve()}?mode=ro", uri=True)
            clouds[name] = Clouds(connections[name], None)
        # Identical voxel-held-out local samples for all before/after comparisons.
        def score_pair(sa, na, sb, nb, poses):
            aa, bb = index[(sa,na)], index[(sb,nb)]
            return geometric_score(clouds[sa].get(na)[1], clouds[sb].get(nb)[1], inverse(poses[aa])@poses[bb], threshold=.20)
        for name in names:
            pairs = [(e['from_id'], e['to_id'], 'heldout_stored_loop') for e in stored[name]['verified_loops'] if e['held_out']]
            ids = sorted(local[name])
            pairs += [(ids[k], ids[k+1], 'adjacent') for k in range(0,len(ids)-1,max(1,len(ids)//60))]
            for a,b,kind in pairs:
                if (name,a) not in index or (name,b) not in index: continue
                for label, poses in [('initial',initial),('candidate',optimized)]:
                    geometries.append({'session':name,'a':a,'b':b,'kind':kind,'trajectory':label,**score_pair(name,a,name,b,poses)})
            for kind in ('heldout_stored_loop','adjacent'):
                before=[r for r in geometries if r['session']==name and r['kind']==kind and r['trajectory']=='initial']
                after=[r for r in geometries if r['session']==name and r['kind']==kind and r['trajectory']=='candidate']
                if before and (np.median([r['clipped_rmse_m'] for r in after]) > np.median([r['clipped_rmse_m'] for r in before])+.005
                               or np.median([r['overlap'] for r in after]) < np.median([r['overlap'] for r in before])-.03):
                    guards.append(name+'_'+kind+'_regression')
        for row in [r for r in new if r['session_a'] != r['session_b']]:
            sa,na,sb,nb=row['session_a'],row['node_a'],row['session_b'],row['node_b']
            acloud,bcloud=clouds[sa].get(na)[1],clouds[sb].get(nb)[1]
            reference=geometric_score(acloud,bcloud,np.asarray(row['T_a_b']),threshold=.20)
            after=score_pair(sa,na,sb,nb,optimized)
            geometries.append({'session':sa+'__'+sb,'a':na,'b':nb,'kind':'cross_heldout' if row.get('held_out') else 'cross_training','trajectory':'candidate',**after})
            if after['clipped_rmse_m'] > reference['clipped_rmse_m']+.02 or after['overlap'] < reference['overlap']-.10:
                guards.append(f'cross_geometry_not_realized_{sa}_{na}_{sb}_{nb}')
        for con in connections.values(): con.close()
        for name in names:
            (output/name).mkdir()
            for label, poses in [('initial',initial),('candidate',optimized)]:
                own={n:poses[index[(name,n)]] for n in local[name]}
                trajectory(output/name/(label+'_trajectory.csv'), own, stamps[name])
        write_csv(output/'cross_residuals.csv', rr)
        write_csv(output/'geometry_validation.csv', geometries)
        write_csv(output/'node_index.csv',[{'global_id':gid,'session':name,'node_id':node} for gid,(name,node) in reverse.items()])
        status={'sessions':names,'root_session':root,'directory':str(output.resolve()),'solver':solver,
            'accepted':not guards,'rejection_reasons':guards,'initial_session_transforms':{n:t.tolist() for n,t in transforms.items()},
            'new_constraints':len(new_edges),'cross_constraints':len(rr),'cross_whole_pair_heldout':sum(r['held_out'] for r in rr),
            'training_errors':error_statistics([e for e in new_edges if not e.held_out],optimized),
            'heldout_errors':error_statistics([e for e in new_edges if e.held_out],optimized),
            'absolute_accuracy_verified':False,'floor_factors':0,'gnss_factors':0}
        (output/'joint_report.json').write_text(json.dumps(status,indent=2)+'\n')
        report['components'].append(status)
    report['finished_utc']=utc_now()
    report['status']='evaluated_connected_components' if report['components'] else 'not_run_no_sufficiently_supported_cross_session_links'
    (args.output/'joint_report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2),flush=True)


if __name__ == '__main__':
    main()
