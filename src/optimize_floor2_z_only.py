#!/usr/bin/env python3
"""Bounded, robust scalar-Z splines with exact selected XY/rotation preservation.

Only pose world-Z offsets vary. Flat-floor regularization uses original depth
patch centres, NEVER camera height. Stair intervals and session attachment
poses are fixed. Historical patch identities remain conditional/reused evidence.
"""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import sqlite3

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import lsq_linear
from scipy.sparse import csr_matrix
from scipy.spatial import cKDTree

from check_session_overlap import Session
from evaluate_map_surfaces import pair_metrics
from evaluate_submap_candidates import fixed_patches
from export_rtabmap_db import load_pose_csv
from optimize_session import Clouds, apply, inverse, sha256_file, utc_now

ROOT = Path(__file__).resolve().parents[1]


def z_basis(ids, interval, fixed, stride):
    lo, hi = interval
    knots = np.array(sorted(set(range(lo, hi, stride)) | set(fixed) | {lo, hi}))
    variable = [int(n) for n in knots if n not in fixed and n not in (lo, hi)]
    basis = np.zeros((len(ids), len(variable)))
    for j, knot in enumerate(variable):
        values = (knots == knot).astype(float)
        basis[:, j] = np.interp(ids, knots, values, left=0., right=0.)
    return knots, variable, basis


def apply_z_offsets(poses, ids, offsets):
    result = {n: p.copy() for n, p in poses.items()}
    for n, dz in zip(ids, offsets):
        result[n][2, 3] += float(dz)
    for n in poses:
        if not np.array_equal(result[n][:2], poses[n][:2]):
            raise AssertionError('XY or rotation changed')
        if not np.array_equal(result[n][:3, :3], poses[n][:3, :3]):
            raise AssertionError('Rotation changed')
    return result


def paired_height_target(pa, pb, ta, tb, radius=.2, minimum=25):
    a, b = apply(pa['training'], ta), apply(pb['training'], tb)
    na, nb = ta[:3, :3] @ pa['normal'], tb[:3, :3] @ pb['normal']
    angle = float(np.degrees(np.arccos(np.clip(abs(na @ nb), 0, 1))))
    if min(abs(na[2]), abs(nb[2])) < .97:
        return None, {'accepted': False, 'reason': 'not near-horizontal', 'angle_deg': angle}
    use_b = cKDTree(a[:, :2]).query(b[:, :2], workers=1)[0] <= radius
    use_a = cKDTree(b[:, :2]).query(a[:, :2], workers=1)[0] <= radius
    detail = {'common_xy_a_points': int(use_a.sum()), 'common_xy_b_points': int(use_b.sum()),
              'angle_deg': angle, 'support_radius_m': radius}
    if min(use_a.sum(), use_b.sum()) < minimum:
        return None, {**detail, 'accepted': False, 'reason': 'insufficient finite common XY support'}
    ca, cb = apply(pa['center'][None], ta)[0], apply(pb['center'][None], tb)[0]
    forward = np.median((b[use_b]-ca) @ na) / na[2]
    reverse = np.median((a[use_a]-cb) @ nb) / nb[2]
    target = float((-forward + reverse)/2)
    return target, {**detail, 'accepted': True, 'delta_z_b_minus_a_target_m': target,
                    'forward_vertical_signed_m': float(forward), 'reverse_vertical_signed_m': float(reverse)}


def solve_scalar_z(basis, ids, poses, observations, floor_rows, cfg, variant):
    lookup = {n: i for i, n in enumerate(ids)}
    train = [r for r in observations if r['used_for_fit']]
    if not train:
        raise ValueError('No geometrically supported height observations')
    groups = Counter(r['region'] for r in train)
    a_data, b_data = [], []
    for r in train:
        scale = variant['pair_sigma_m']*np.sqrt(groups[r['region']])
        a_data.append((basis[lookup[r['b']]]-basis[lookup[r['a']]])/scale)
        b_data.append(r['target']/scale)
    # A shared latent floor level is estimated, not fixed to a guessed building
    # height. Its gauge receives a weak prior to the existing median floor.
    nf = basis.shape[1]
    design = [np.r_[row, 0.] for row in a_data]
    target = list(b_data)
    robust_count = len(design)
    floor_groups = Counter(r['region'] for r in floor_rows)
    level0 = float(np.median([r['height_m'] for r in floor_rows]))
    for r in floor_rows:
        scale = variant['floor_level_sigma_m']*np.sqrt(floor_groups[r['region']])
        design.append(np.r_[basis[lookup[r['node']]], -1.]/scale)
        target.append(-r['height_m']/scale)
    floor_end = len(design)
    for j in range(nf):
        row = np.zeros(nf+1); row[j] = 1/variant['offset_prior_sigma_m']
        design.append(row); target.append(0.)
    positions = np.array([poses[n][:3, 3] for n in ids])
    distance = np.maximum(np.linalg.norm(np.diff(positions, axis=0), axis=1), .10)
    for row, ds in zip(np.diff(basis, axis=0), distance):
        if row.any():
            scale = variant['increment_sigma_m_per_sqrt_m']*np.sqrt(ds)
            design.append(np.r_[row, 0.]/scale); target.append(0.)
    design.append(np.r_[np.zeros(nf), 1/.08]); target.append(level0/.08)
    a, b = np.array(design), np.array(target)
    weights = np.ones(len(b)); history = []; fit = None
    bounds = (np.r_[np.full(nf, -cfg['maximum_absolute_z_correction_m']), level0-.25],
              np.r_[np.full(nf, cfg['maximum_absolute_z_correction_m']), level0+.25])
    for iteration in range(25):
        fit = lsq_linear(csr_matrix(a*np.sqrt(weights)[:, None]), b*np.sqrt(weights),
                         bounds=bounds, method='trf', lsq_solver='lsmr', tol=1e-10,
                         lsmr_tol=1e-11, max_iter=150)
        residual = a @ fit.x-b
        updated = np.ones(len(b))
        updated[:floor_end] = np.minimum(1., cfg['huber_delta']/np.maximum(abs(residual[:floor_end]), 1e-12))
        step = float(np.max(abs(updated-weights)))
        history.append({'iteration': iteration, 'weight_step': step, 'linear_success': bool(fit.success)})
        weights = updated
        if step < 1e-6:
            break
    return basis @ fit.x[:nf], {'linear_success': bool(fit.success), 'optimality': float(fit.optimality),
        'iterations': history, 'variables': nf, 'fitted_floor_level_m': float(fit.x[-1]),
        'initial_floor_level_median_m': level0, 'fit_pairs': len(train),
        'fit_regions': len(groups), 'floor_prior_rows': len(floor_rows),
        'pair_huber_weights': weights[:robust_count].tolist(),
        'floor_prior_huber_weights': weights[robust_count:floor_end].tolist(),
        'sigma_scope': 'Explicit regularization/model choices; not calibrated sensor covariance.'}


def floor_statistics(floor_rows, offsets, lookup):
    values = np.array([r['height_m']+offsets[lookup[r['node']]] for r in floor_rows])
    return {'count': len(values), 'median_m': float(np.median(values)),
        'p10_m': float(np.percentile(values, 10)), 'p90_m': float(np.percentile(values, 90)),
        'p90_minus_p10_m': float(np.percentile(values, 90)-np.percentile(values, 10)),
        'minimum_m': float(values.min()), 'maximum_m': float(values.max()),
        'maximum_minus_minimum_m': float(np.ptp(values)),
        'scope': 'Original below-camera finite depth-patch centres on reviewed upper floor, NOT camera-height or all-building floor accuracy.'}


def write_trajectory_exact(source, dest, offsets):
    # Preserve every untouched CSV field byte-for-byte, including quaternion
    # text; matrix -> quaternion roundtrips are deliberately avoided.
    with source.open(newline='') as f:
        reader = csv.DictReader(f); fields = reader.fieldnames; rows = list(reader)
    for row in rows:
        dz = offsets[int(row['id'])]
        if dz != 0.:
            row['z'] = repr(float(row['z'])+dz)
    with dest.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    with source.open(newline='') as f:
        old = list(csv.DictReader(f))
    with dest.open(newline='') as f:
        new = list(csv.DictReader(f))
    assert all(all(a[k] == b[k] for k in fields if k != 'z') for a, b in zip(old, new))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'config/floor2_z_only.json')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args(); cfg = json.loads(args.config.read_text())
    out = args.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    mp = ROOT/cfg['source_manifest']; selected = json.loads(mp.read_text())
    trajectory = mp.parent/selected['sessions']['floor2']['trajectory']
    db = Path(selected['database_integrity']['floor2']['database'])
    raw_path = ROOT/'outputs/student_union_20261017_added_reviewed/floor2/raw_main_trajectory.csv'
    history = ROOT/'outputs/student_union_20261017_independent_v1/evaluation/report.json'
    paths = [Path(__file__), args.config, mp, trajectory, db, raw_path, history,
        ROOT/'src/evaluate_submap_candidates.py', ROOT/'src/evaluate_map_surfaces.py']
    hashes = {str(p.resolve()): sha256_file(p) for p in paths}
    poses = load_pose_csv(trajectory); raw = load_pose_csv(raw_path); ids = sorted(poses)
    lookup = {n: i for i, n in enumerate(ids)}
    historical = json.loads(history.read_text())['comparisons']
    pairs = [{**r, 'role': 'audit', 'region': 'historical_audit', 'source': 'historical_loop_patch'}
             for r in historical if r['session'] == 'floor2' and {r['a'], r['b']} <= set(ids)]
    pairs += [{**r, 'kind': 'horizontal', 'source': 'reviewed_upper_restroom_photos'} for r in cfg['additional_floor_pairs']]
    trainmap = {(r['a'], r['b']): r['region'] for r in cfg['training_pairs']}
    cache, observations, floor_rows = {}, [], []
    wanted = {n for r in pairs for n in (r['a'], r['b'])} | set(cfg['extra_floor_observation_nodes'])
    with sqlite3.connect(f'file:{db}?mode=ro', uri=True) as con:
        clouds = Clouds(con, None)
        for n in sorted(wanted & set(ids)):
            cache[n] = fixed_patches(clouds, raw, n)
        for r in pairs:
            if r['kind'] != 'horizontal':
                continue
            a, b = cache[r['a']][r['kind'], r['plane_a']], cache[r['b']][r['kind'], r['plane_b']]
            target, support = paired_height_target(a, b, poses[r['a']], poses[r['b']],
                cfg['common_xy_support_radius_m'], cfg['minimum_common_xy_training_points'])
            intended = (r['a'], r['b']) in trainmap or r['role'] == 'train'
            if (r['a'], r['b']) in trainmap:
                r['region'] = trainmap[r['a'], r['b']]
            support_ok = support.get('accepted') and support['angle_deg'] <= cfg['horizontal_normal_max_difference_deg']
            observations.append({**r, 'target': target, 'support': support,
                'used_for_fit': bool(intended and support_ok)})
        lo, hi = cfg['same_level_floor_centre_band_m']; start, end = cfg['adjustable_frame_interval']
        for n in sorted(cache):
            if not start <= n <= end:
                continue
            for (kind, index), patch in cache[n].items():
                if kind != 'horizontal' or index != 0:
                    continue
                centre = apply(patch['center'][None], poses[n])[0]
                if not lo <= centre[2] <= hi:
                    continue
                cell = np.floor(centre[:2]/cfg['floor_centre_region_cell_m']).astype(int)
                floor_rows.append({'node': n, 'plane': index, 'height_m': float(centre[2]),
                    'centre': centre.tolist(), 'region': f'{cell[0]}_{cell[1]}'})
    knots, variables, basis = z_basis(ids, cfg['adjustable_frame_interval'], cfg['hard_fixed_knots'], cfg['knot_stride_frames'])
    baseline_floor = floor_statistics(floor_rows, np.zeros(len(ids)), lookup)
    candidates = {}; offsets_by_code = {'J04': np.zeros(len(ids))}; patches = {}
    for v in cfg['variants']:
        print('Solving scalar Z only: '+v['id'], flush=True)
        offsets, solver = solve_scalar_z(basis, ids, poses, observations, floor_rows, cfg, v)
        candidate = apply_z_offsets(poses, ids, offsets)
        frame_to_offset = dict(zip(ids, map(float, offsets)))
        write_trajectory_exact(trajectory, out/(v['id']+'.csv'), frame_to_offset)
        reread = load_pose_csv(out/(v['id']+'.csv'))
        assert all(np.array_equal(reread[n][:3, :3], poses[n][:3, :3]) and
                   np.array_equal(reread[n][:2, 3], poses[n][:2, 3]) for n in ids)
        floor = floor_statistics(floor_rows, offsets, lookup)
        failures = []
        if not solver['linear_success']: failures.append('linear_solver_failure')
        if floor['p90_minus_p10_m'] > baseline_floor['p90_minus_p10_m']+cfg['maximum_floor_band_regression_m']: failures.append('floor_band_regression')
        if floor['maximum_minus_minimum_m'] > baseline_floor['maximum_minus_minimum_m']+cfg['maximum_floor_band_regression_m']: failures.append('floor_full_range_regression')
        if np.max(abs(np.diff(offsets))) > cfg['maximum_adjacent_correction_step_m']: failures.append('adjacent_Z_correction_step')
        fixed = [n for n in ids if n <= cfg['adjustable_frame_interval'][0] or n >= cfg['adjustable_frame_interval'][1] or n in cfg['hard_fixed_knots']]
        assert all(frame_to_offset[n] == 0 for n in fixed)
        candidates[v['id']] = {'parameters': v, 'solver': solver, 'floor': floor,
            'maximum_abs_offset_m': float(np.max(abs(offsets))),
            'maximum_adjacent_offset_change_m': float(np.max(abs(np.diff(offsets)))),
            'fixed_stair_lower_floor_attachment_nodes': len(fixed), 'fixed_nodes_max_motion_m': 0.,
            'xy_rotation_exactly_unchanged': True, 'all_non_z_csv_fields_identical': True,
            'guard_failures': failures, 'guards_passed': not failures,
            'trajectory': v['id']+'.csv', 'trajectory_sha256': sha256_file(out/(v['id']+'.csv'))}
        offsets_by_code[v['id']] = offsets
    for code, offsets in offsets_by_code.items():
        candidate = apply_z_offsets(poses, ids, offsets)
        rows = []
        for r in pairs:
            a, b = cache[r['a']][r['kind'], r['plane_a']], cache[r['b']][r['kind'], r['plane_b']]
            metric = pair_metrics(a, b, candidate[r['a']], candidate[r['b']])
            used = any(o['a'] == r['a'] and o['b'] == r['b'] and o['used_for_fit'] for o in observations) if r['kind'] == 'horizontal' else False
            rows.append({**r, 'used_for_fit': used, **metric})
        patches[code] = rows
    # Choose the predeclared balanced variant only if the engineering guards
    # pass. Held/development rows are displayed, not used to tune this choice.
    preferred = cfg['preferred_variant_before_audits']
    chosen = preferred if candidates[preferred]['guards_passed'] else None
    for code, record in candidates.items():
        matched = [(a, b) for a, b in zip(patches['J04'], patches[code]) if a['kind'] == 'horizontal']
        record['horizontal_regressions_over_3cm'] = [{'a': a['a'], 'b': a['b'], 'before_m': a['symmetric_plane_abs_median_m'], 'after_m': b['symmetric_plane_abs_median_m']}
            for a, b in matched if b['symmetric_plane_abs_median_m']-a['symmetric_plane_abs_median_m'] > .03]
        for role, predicate in [('fitted', lambda r: r['used_for_fit']), ('development_audit', lambda r: not r['used_for_fit'])]:
            vals = [r['symmetric_plane_abs_median_m'] for r in patches[code] if r['kind'] == 'horizontal' and predicate(r)]
            record[role] = {'count': len(vals), 'median_m': float(np.median(vals)), 'p90_m': float(np.percentile(vals, 90))}
    fig, axes = plt.subplots(2, 2, figsize=(16, 10)); colors = ['#647484', '#58a36d', '#e78533', '#9164ac']
    for (code, offsets), color in zip(offsets_by_code.items(), colors):
        axes[0, 0].plot(ids, offsets*100, color=color, label=code)
        vals = [r['height_m']+offsets[lookup[r['node']]] for r in floor_rows]
        axes[0, 1].scatter([r['node'] for r in floor_rows], vals, color=color, s=16, label=code)
        rr = [r for r in patches[code] if r['kind'] == 'horizontal']
        axes[1, 1].plot(range(len(rr)), [r['symmetric_plane_abs_median_m']*100 for r in rr], '.-', color=color, label=code)
    axes[0, 0].set(title='Scalar Z corrections; XY and ALL rotations fixed', xlabel='Frame ID', ylabel='Z offset (cm)')
    axes[0, 1].set(title='Same-level original floor patch centres (NOT camera Z)', xlabel='Frame ID', ylabel='Floor patch centre Z (m)')
    floor_before = np.array([r['height_m'] for r in floor_rows]); offsets = offsets_by_code[preferred]
    floor_after = floor_before+np.array([offsets[lookup[r['node']]] for r in floor_rows])
    axes[1, 0].boxplot([floor_before, floor_after], labels=['J04', preferred])
    axes[1, 0].set(title='Same-floor spread control; no stair flattening', ylabel='Original patch centre + scalar offset (m)')
    axes[1, 1].set(title='Identical finite horizontal tests (reused/correlated)', xlabel='Patch-pair index', ylabel='Median plane distance (cm)')
    for ax in axes.ravel(): ax.grid(alpha=.15)
    axes[0, 0].legend(); axes[1, 1].legend()
    fig.suptitle('Experimental Z-only pose correction | exact XY/rotation freeze\nOriginal map unchanged; level prior is not a surveyed floor elevation')
    fig.tight_layout(rect=(0, 0, 1, .94)); fig.savefig(out/'z_only_comparison.png', dpi=160); plt.close(fig)
    # Save actual depth-test footprint contact sheets before any reconstruction.
    session = Session(db, 'floor2')
    for page, start in enumerate(range(0, len(floor_rows), 12)):
        selected_rows = floor_rows[start:start+12]
        fig, axes = plt.subplots(3, 4, figsize=(12, 12))
        for ax in axes.ravel(): ax.axis('off')
        for ax, row in zip(axes.ravel(), selected_rows):
            n = row['node']; rgb = cv2.cvtColor(session.image(n), cv2.COLOR_BGR2RGB)
            _, _, width, height, k, local = session.depth(n)
            points = apply(cache[n]['horizontal', row['plane']]['testing'], inverse(local))
            uvw = points @ k.T; uv = uvw[:, :2]/uvw[:, 2:3]*[rgb.shape[1]/width, rgb.shape[0]/height]
            valid = (points[:, 2] > 0)&(uv[:, 0] >= 0)&(uv[:, 0] < rgb.shape[1])&(uv[:, 1] >= 0)&(uv[:, 1] < rgb.shape[0])
            ax.imshow(rgb); ax.scatter(uv[valid, 0], uv[valid, 1], s=2, c='cyan', alpha=.6)
            ax.set_title(f'Original #{n}; Zcentre {row["height_m"]:.3f}m', fontsize=10)
        fig.suptitle('Original photos + fixed depth patch tests | review floor identity; no generated images')
        fig.tight_layout(rect=(0, 0, 1, .96)); fig.savefig(out/f'floor_support_photos_{page}.png', dpi=130); plt.close(fig)
    session.connection.close()
    assert all(sha256_file(Path(p)) == h for p, h in hashes.items())
    result = {'created_utc': utc_now(), 'inputs_sha256': hashes, 'original_inputs_unchanged': True,
        'config': cfg, 'common_pose_count': len(ids), 'knot_nodes': knots.tolist(), 'variable_knots': variables,
        'baseline_floor': baseline_floor, 'floor_observations': floor_rows, 'height_observations': observations,
        'variants': candidates, 'patch_metrics': patches, 'preferred_experimental_candidate': chosen,
        'selection_rule': 'Predeclared Z02 + floor-spread/stair/XY/rotation guards, not best heldout selection.',
        'source_selected_map_replaced': False, 'navigation_ready': False,
        'limitations': ['Z-only cannot correct plane normals, XY mismatches, calibration or nonrigid local depth.',
            'Historical and neighbouring photo/depth tests are correlated development evidence, not surveyed accuracy.',
            'Common upper-floor level regularization assumes reviewed patches share a level; excludes stair time intervals.',
            'Preserved main-stair rise and attachment may limit achievable floor flatness.'],
        'figures': {p.name: sha256_file(p) for p in out.glob('*.png')}}
    (out/'report.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({'output': str(out), 'preferred': chosen, 'floor_before': baseline_floor,
        'variants': {k: {f: v[f] for f in ['floor', 'maximum_abs_offset_m', 'guards_passed', 'guard_failures', 'horizontal_regressions_over_3cm']} for k, v in candidates.items()}}, indent=2), flush=True)


if __name__ == '__main__': main()
