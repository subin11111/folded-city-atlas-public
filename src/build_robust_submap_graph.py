#!/usr/bin/env python3
"""Build an auditable robust-backend graph from quality-screened submaps.

The builder condenses native frame odometry into one edge between neighboring
submap anchors and maps loop/cross-session measurements to those anchors.  It
does not optimize anything and never edits a database.  Temporally correlated
frames receive one primary submap assignment; duplicate constraints in the same
physical-region/submap pair are reduced to one representative.

Pose convention follows the rest of this repository: ``T_w_i`` maps points in
node ``i`` to world, and an edge stores ``inv(T_w_a) @ T_w_b``.  Information
matrices use tangent order ``[tx,ty,tz,rx,ry,rz]``.
"""
from __future__ import annotations

import argparse
import csv
from collections import deque
import json
from pathlib import Path
import shutil
import sys

import numpy as np
from scipy.spatial.transform import Rotation

from export_rtabmap_db import load_pose_csv
from optimize_session import inverse, relative_error, sha256_file, utc_now


ROOT = Path(__file__).resolve().parents[1]


def skew(vector):
    x, y, z = np.asarray(vector, float)
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def adjoint(transform):
    """SE(3) adjoint for tangent order translation, rotation."""
    transform = np.asarray(transform, float)
    rotation, translation = transform[:3, :3], transform[:3, 3]
    result = np.zeros((6, 6))
    result[:3, :3] = rotation
    result[:3, 3:] = skew(translation) @ rotation
    result[3:, 3:] = rotation
    return result


def reverse_measurement(measurement, covariance):
    """Invert a right-perturbed relative measurement and its covariance."""
    transform = inverse(np.asarray(measurement, float))
    mapping = adjoint(np.asarray(measurement, float))
    return transform, mapping @ covariance @ mapping.T


def compose_measurements(first, first_covariance, second, second_covariance):
    """First-order covariance for (A exp(da)) (B exp(db))."""
    mapping = adjoint(inverse(np.asarray(second, float)))
    transform = np.asarray(first, float) @ np.asarray(second, float)
    covariance = mapping @ first_covariance @ mapping.T + second_covariance
    return transform, (covariance + covariance.T) / 2.


def transform_frame_edge(measurement, covariance, local_root_to_a, local_root_to_b):
    """Convert a frame A/B edge to the corresponding submap-root edge."""
    la = np.asarray(local_root_to_a, float)
    lb = np.asarray(local_root_to_b, float)
    root_measurement = la @ np.asarray(measurement, float) @ inverse(lb)
    # E_frame = inv(L_b) E_root L_b; map root tangent into frame tangent.
    mapping = adjoint(inverse(lb))
    information_frame = np.linalg.inv(covariance)
    information_root = mapping.T @ information_frame @ mapping
    root_covariance = np.linalg.inv((information_root + information_root.T) / 2.)
    return root_measurement, (root_covariance + root_covariance.T) / 2.


def robust_scale(rows):
    """Per-axis odometry model discrepancy, with non-zero physical floors."""
    values = np.asarray(rows, float)
    floors = np.r_[np.full(3, .01), np.full(3, np.radians(.2))]
    if not len(values):
        return floors
    center = np.median(values, axis=0)
    mad = 1.4826 * np.median(np.abs(values - center), axis=0)
    return np.maximum(mad, floors)


def odometry_path(adjacency, start, target):
    """Deterministic shortest native-odometry path with propagated covariance."""
    queue = deque([(int(start), np.eye(4), np.zeros((6, 6)), [])])
    visited = {int(start)}
    while queue:
        node, transform, covariance, ids = queue.popleft()
        if node == int(target):
            return transform, covariance, ids
        for neighbor, measurement, edge_covariance, edge_id in sorted(
                adjacency.get(node, []), key=lambda row: (row[0], row[3])):
            if neighbor in visited:
                continue
            visited.add(neighbor)
            combined, combined_covariance = compose_measurements(
                transform, covariance, measurement, edge_covariance)
            queue.append((neighbor, combined, combined_covariance, ids + [edge_id]))
    return None, None, []


def resolve_artifact(path_value, quality_root):
    path = Path(path_value)
    candidates = [path, quality_root / path, quality_root / path.name]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    raise FileNotFoundError(path_value)


def load_quality_session(name, row, quality_root):
    artifacts = row.get('artifacts', {})
    submap_path = resolve_artifact(artifacts.get('submaps_json', row.get('submaps_json')), quality_root)
    frames_path = resolve_artifact(artifacts.get('frames_csv', row.get('frames_csv')), quality_root)
    submap_payload = json.loads(submap_path.read_text())
    submaps = submap_payload.get('submaps', submap_payload)
    frames = {}
    with frames_path.open(newline='') as stream:
        for frame in csv.DictReader(stream):
            node = int(frame['node_id'])
            primary = frame.get('primary_submap_id') or frame.get('primary_submap')
            frames[node] = {**frame, 'primary_submap_id': primary}
    if not submaps or not frames:
        raise ValueError(f'empty quality artifacts for {name}')
    return submaps, frames, submap_path, frames_path


def matrix_from_row(row):
    for key in ('initial_anchor_pose_matrix', 'pose_matrix', 'pose'):
        if key in row:
            value = np.asarray(row[key], float)
            if value.shape == (4, 4):
                return value
    raise ValueError(f'submap has no 4x4 pose: {row.get("submap_id", row.get("id"))}')


def quality_weight(frame):
    """A/B/C endpoint quality, independent of correlation-group size."""
    grade = (frame.get('quality_grade') or frame.get('grade') or frame.get('quality_class') or 'B').upper()
    return {'A': 1., 'B': .5, 'C': .1}.get(grade, .5)


def edge_covariance(row, return_diagnostic=False):
    """Return a positive covariance and disclose exactly where it came from."""
    containers = [('', row)]
    if isinstance(row.get('metrics'), dict):
        containers.append(('metrics.', row['metrics']))
    covariance = None
    source = None
    raw_information_eigenvalues = None
    for prefix, container in containers:
        if 'covariance_6x6' in container:
            covariance = np.asarray(container['covariance_6x6'], float)
            source = prefix + 'covariance_6x6'
            break
        for key in ('information_6x6', 'effective_information', 'information'):
            if key not in container:
                continue
            information = np.asarray(container[key], float)
            if information.shape != (6, 6) or not np.isfinite(information).all():
                raise ValueError(f'invalid {prefix + key}')
            information = (information + information.T) / 2.
            raw_information_eigenvalues = np.linalg.eigvalsh(information)
            if raw_information_eigenvalues[0] <= 0:
                raise ValueError(f'non-positive {prefix + key}')
            covariance = np.linalg.inv(information)
            source = prefix + key
            break
        if covariance is not None:
            break
    if covariance is None:
        metrics = row.get('metrics', {}) if isinstance(row.get('metrics'), dict) else {}
        explicit_sigma = any(key in row or key in metrics for key in (
            'sigma_translation_m', 'sigma_rotation_deg'))
        st = max(.10, float(row.get(
            'sigma_translation_m', metrics.get('sigma_translation_m', .15))))
        sr = np.radians(max(3., float(row.get(
            'sigma_rotation_deg', metrics.get('sigma_rotation_deg', 5.)))))
        covariance = np.diag([st * st] * 3 + [sr * sr] * 3)
        source = ('explicit_sigma_translation_rotation' if explicit_sigma
                  else 'conservative_default_0.15m_5deg')
    if covariance.shape != (6, 6) or not np.isfinite(covariance).all():
        raise ValueError('invalid edge covariance')
    covariance = (covariance + covariance.T) / 2.
    values, vectors = np.linalg.eigh(covariance)
    if values[0] <= 0:
        raise ValueError(f'non-positive edge covariance from {source}')
    used_values = np.clip(values, 1e-8, 1e4)
    used = (vectors * used_values) @ vectors.T
    diagnostic = {
        'source': source,
        'assumed_sigma_translation_m': (float(np.sqrt(covariance[0, 0]))
                                        if 'sigma' in source or 'default' in source else None),
        'assumed_sigma_rotation_deg': (float(np.degrees(np.sqrt(covariance[3, 3])))
                                       if 'sigma' in source or 'default' in source else None),
        'raw_covariance_eigenvalues': values.tolist(),
        'used_covariance_eigenvalues': used_values.tolist(),
        'raw_information_eigenvalues': (None if raw_information_eigenvalues is None
                                        else raw_information_eigenvalues.tolist()),
        'was_clipped': bool(not np.allclose(values, used_values)),
    }
    return (used, diagnostic) if return_diagnostic else used


def extra_constraint_role(row):
    """Validate the curated extra-constraint contract and return its role."""
    positive_evidence = any(row.get(key) is True for key in (
        'hypothesis_eligible', 'local_geometry_verified', 'accepted',
        'eligible_for_joint', 'eligible_for_experimental_graph'))
    if not positive_evidence:
        raise ValueError('extra constraint lacks an explicit positive evidence/eligibility flag')
    role = row.get('joint_role')
    if role not in (None, 'train', 'heldout', 'local_evidence_only'):
        raise ValueError(f'unsupported joint_role {role!r}')
    held = bool(row.get('held_out')) or bool(row.get('joint_holdout')) or role == 'heldout'
    if role == 'train' and held:
        raise ValueError('constraint says joint_role=train but is also marked heldout')
    if role == 'local_evidence_only':
        # Local validation may carry a held_out flag from its source run; that
        # does not promote it to joint fitting or an independent joint holdout.
        return 'local_evidence_only'
    return 'heldout' if held else 'train'


def region_group(session_a, session_b, pose_a, pose_b, spacing=2.):
    pair = '__'.join(sorted((session_a, session_b)))
    midpoint = (np.asarray(pose_a)[:3, 3] + np.asarray(pose_b)[:3, 3]) / 2.
    cell = np.floor(midpoint / spacing + .5).astype(int)
    return f'{pair}:cell_{cell[0]}_{cell[1]}_{cell[2]}'


def deduplicate_edges(edges):
    """One vote per type, endpoints and physical region; heldout stays separate."""
    selected, dropped = {}, []
    for edge in edges:
        key = (edge['type'], tuple(sorted((edge['from'], edge['to']))),
               edge['region_group'], edge['role'])
        covariance = np.asarray(edge['covariance'], float)
        # Do not add metres squared to radians squared.  Normalize against the
        # project gates, then prefer a deliberately remeasured extra over an
        # older native link when both collapse to the same submap relation.
        scale = np.r_[np.full(3, .1 ** 2), np.full(3, np.radians(5.) ** 2)]
        uncertainty = float(np.sum(np.diag(covariance) / scale))
        provenance = edge.get('provenance', {})
        source_rank = (1 if provenance.get('source') == 'validated_native_loop'
                       else 0 if provenance.get('source') else 2)
        score = (source_rank, uncertainty)
        previous = selected.get(key)
        if previous is None or score < previous[0]:
            if previous is not None:
                dropped.append(previous[1]['id'])
            selected[key] = (score, edge)
        else:
            dropped.append(edge['id'])
    return [row[1] for row in selected.values()], sorted(dropped)


def annotate_structural_support(edges):
    """Label whether every switchable transition has an alternate train path."""
    training = [edge for edge in edges if edge['role'] == 'train']

    def connected(source, target, excluded, non_structural_only=False):
        adjacency = {}
        for edge in training:
            if edge['id'] == excluded or (non_structural_only and edge['type'] == 'structural'):
                continue
            adjacency.setdefault(edge['from'], set()).add(edge['to'])
            adjacency.setdefault(edge['to'], set()).add(edge['from'])
        stack, visited = [source], {source}
        while stack:
            node = stack.pop()
            if node == target:
                return True
            for neighbor in adjacency.get(node, ()):
                if neighbor not in visited:
                    visited.add(neighbor)
                    stack.append(neighbor)
        return False

    records = []
    for edge in edges:
        if edge['type'] != 'structural':
            continue
        alternate = connected(edge['from'], edge['to'], edge['id'])
        non_structural = connected(edge['from'], edge['to'], edge['id'], True)
        edge['provenance']['cycle_supported'] = alternate
        edge['provenance']['non_structural_alternate_path'] = non_structural
        edge['provenance']['graph_bridge'] = not alternate
        records.append({'edge_id': edge['id'], 'cycle_supported': alternate,
                        'non_structural_alternate_path': non_structural,
                        'graph_bridge': not alternate})
    return records


def prefer_successful_replacements(edges):
    """Drop a native loop only when its replacement survived all mapping gates.

    Candidate files often contain a remeasured version of a native frame pair.
    Merely declaring that candidate must not erase the native loop: its endpoint
    may later fail the quality/primary-submap gate.  Private bookkeeping fields
    are stripped from every returned edge.
    """
    successful = {
        edge['_frame_key'] for edge in edges
        if edge.get('_source_kind') == 'extra' and edge.get('_frame_key') is not None
    }
    kept, replaced = [], []
    for edge in edges:
        if (edge.get('_source_kind') == 'native'
                and edge.get('_frame_key') in successful):
            replaced.append(edge['id'])
            continue
        kept.append({key: value for key, value in edge.items() if not key.startswith('_')})
    return kept, sorted(replaced)


def build_graph(quality, config, quality_root, initial_graph, constraints_path,
                boundary_covariance_inflation=25., include_structural_transitions=True):
    if not np.isfinite(boundary_covariance_inflation) or boundary_covariance_inflation <= 0:
        raise ValueError('boundary covariance inflation must be finite and positive')
    selected_local, common_initial = {}, {}
    submap_rows, frame_rows, submap_meta = {}, {}, {}
    source_paths = [quality_root/'manifest.json', initial_graph/'report.json', constraints_path]
    backend_submaps = []
    for name, session in quality['sessions'].items():
        local_path = resolve_artifact(session['frozen_pose_csv'], quality_root)
        selected_local[name] = load_pose_csv(local_path)
        common_path = initial_graph/name/'initial.csv'
        common_initial[name] = load_pose_csv(common_path)
        rows, frames, submaps_path, frames_path = load_quality_session(name, session, quality_root)
        source_paths += [local_path, common_path, submaps_path, frames_path]
        expected_pose_hash = session.get('frozen_pose_sha256')
        if expected_pose_hash and sha256_file(local_path) != expected_pose_hash:
            raise ValueError(f'{name}: frozen pose differs from quality-audit hash')
        submap_rows[name], frame_rows[name] = rows, frames
        for index, row in enumerate(rows):
            submap_id = str(row.get('submap_id', row.get('id')))
            root = int(row.get('anchor_node_id', row.get('root_node_id')))
            if root not in common_initial[name] or root not in selected_local[name]:
                raise ValueError(f'{submap_id} anchor is absent from frozen/common poses')
            if not np.allclose(matrix_from_row(row), selected_local[name][root],
                               atol=1e-10, rtol=1e-10):
                raise ValueError(f'{submap_id} declared anchor pose differs from frozen pose')
            run_id = str(row.get('run_id', '0'))
            submap_meta[submap_id] = {'session': name, 'anchor': root, 'run_id': run_id,
                                       'row': row, 'local_pose': selected_local[name][root],
                                       'common_pose': common_initial[name][root]}
            backend_submaps.append({'id': submap_id, 'session_id': name,
                                    'pose': common_initial[name][root].tolist(),
                                    'anchor_node_id': root, 'run_id': run_id,
                                    'quality': {'quality_counts': row.get('quality_counts', {}),
                                                'effective_evidence_groups': row.get('effective_evidence_groups')}})

    config_sessions = config['sessions']
    native_by_session, odometry_models, database_before = {}, {}, {}
    for name in quality['sessions']:
        directory = Path(config_sessions[name]['reviewed_dir'])
        edge_path = directory/'validated_edges.json'
        database = Path(config_sessions[name]['database'])
        source_paths += [edge_path, database]
        native = json.loads(edge_path.read_text())
        native_by_session[name] = native
        database_before[name] = sha256_file(database)
        expected_database_hash = quality['sessions'][name].get('database_sha256_before')
        if expected_database_hash and database_before[name] != expected_database_hash:
            raise ValueError(f'{name}: database differs from quality-audit hash')
        innovations = []
        for edge in native['odometry']:
            a, b = int(edge['from_id']), int(edge['to_id'])
            if a in selected_local[name] and b in selected_local[name]:
                innovations.append(relative_error(np.asarray(edge['measurement']),
                                                  selected_local[name][a], selected_local[name][b]))
        odometry_models[name] = robust_scale(innovations)

    edges, diagnostics = [], {'skipped_internal': [], 'skipped_quality_c': [],
                               'excluded_local_evidence_only': [],
                               'missing_odometry_paths': [], 'replaced_native': [],
                               'deduplicated_edge_ids': [],
                               'omitted_structural_transitions': []}
    # Condensed odometry within each quality run.  Across a hard run boundary,
    # preserve the native path only as a deliberately weak, switchable
    # structural hypothesis.  It may keep a session continuous when consistent,
    # but unlike odometry it is allowed to switch off.
    for name, rows in submap_rows.items():
        native = native_by_session[name]
        adjacency, native_covariance_diagnostics = {}, {}
        for edge in native['odometry']:
            a, b = int(edge['from_id']), int(edge['to_id'])
            measurement = np.asarray(edge['measurement'], float)
            covariance, covariance_diagnostic = edge_covariance(edge, True)
            edge_id = f'{name}:native_odom:{a}:{b}'
            native_covariance_diagnostics[edge_id] = covariance_diagnostic
            adjacency.setdefault(a, []).append((b, measurement, covariance, edge_id))
            reverse, reverse_covariance = reverse_measurement(measurement, covariance)
            adjacency.setdefault(b, []).append((a, reverse, reverse_covariance, edge_id))
        # Run labels are identifiers, not chronology (e.g. r10 vs r2).
        ordered = sorted(rows, key=lambda row:
                         int(row.get('anchor_node_id', row.get('root_node_id'))))
        for first, second in zip(ordered, ordered[1:]):
            same_run = str(first.get('run_id', '0')) == str(second.get('run_id', '0'))
            source = str(first.get('submap_id', first.get('id')))
            target = str(second.get('submap_id', second.get('id')))
            a = int(first.get('anchor_node_id', first.get('root_node_id')))
            b = int(second.get('anchor_node_id', second.get('root_node_id')))
            measurement, covariance, path_ids = odometry_path(adjacency, a, b)
            if measurement is None:
                diagnostics['missing_odometry_paths'].append([source, target])
                continue
            if not same_run and not include_structural_transitions:
                diagnostics['omitted_structural_transitions'].append([source, target])
                continue
            count = len(path_ids)
            model = odometry_models[name]
            covariance += np.diag(model * model * max(1, count))
            correlation_inflation = min(4., 1. + .05 * max(0, count - 1))
            covariance *= correlation_inflation
            predicted = inverse(selected_local[name][a]) @ selected_local[name][b]
            discrepancy = relative_error(measurement, selected_local[name][a], selected_local[name][b])
            covariance += np.diag((.5 * np.abs(discrepancy)) ** 2)
            if not same_run:
                # This is an initialization/continuity hypothesis, not trusted
                # odometry.  Large covariance and a shared switchable type make
                # the assumption explicit and auditable.
                covariance *= boundary_covariance_inflation
            edge_type = 'odometry' if same_run else 'structural'
            edge_prefix = 'odom' if same_run else 'transition'
            group = (f'odometry:{name}:{first.get("run_id", "0")}' if same_run else
                     f'transition:{name}:{first.get("run_id", "0")}__{second.get("run_id", "0")}')
            boundary_start = int(first.get('end_node_id', a))
            boundary_end = int(second.get('start_node_id', b))
            boundary_evidence = [
                {'node_id': node_id, 'boundary_reasons': frame.get('boundary_reasons', '')}
                for node_id, frame in sorted(frame_rows[name].items())
                if boundary_start < node_id <= boundary_end
                and str(frame.get('hard_boundary', '')).lower() in ('true', '1', 'yes')
            ]
            edges.append({'id': f'{edge_prefix}:{source}:{target}', 'from': source, 'to': target,
                          'measurement': measurement.tolist(), 'covariance': covariance.tolist(),
                          'type': edge_type, 'role': 'train', 'region_group': group,
                          'provenance': {'native_path_edge_ids': path_ids, 'native_edge_count': count,
                              'native_path_covariance_sources': sorted({
                                  native_covariance_diagnostics[path_id]['source']
                                  for path_id in path_ids}),
                              'native_path_covariance_clipped_count': sum(
                                  native_covariance_diagnostics[path_id]['was_clipped']
                                  for path_id in path_ids),
                              'correlation_inflation': correlation_inflation,
                              'hard_boundary_transition': not same_run,
                              'hard_boundary_covariance_inflation': (
                                  boundary_covariance_inflation if not same_run else 1.),
                              'hard_boundary_evidence': boundary_evidence if not same_run else [],
                              'app_relative_pose_diagnostic': predicted.tolist(),
                              'native_vs_app_residual': discrepancy.tolist(),
                              'odometry_model_sigma': model.tolist()}})

    extra = json.loads(constraints_path.read_text())
    if isinstance(extra, dict):
        extra = next((extra[key] for key in ('constraints', 'edges', 'records') if key in extra), None)
    if not isinstance(extra, list):
        raise ValueError('unknown constraints schema')
    candidates = []
    for name, native in native_by_session.items():
        for index, row in enumerate(native['verified_loops']):
            candidates.append({'session_a': name, 'node_a': int(row['from_id']),
                               'session_b': name, 'node_b': int(row['to_id']),
                               'T_a_b': row['measurement'], 'held_out': bool(row.get('held_out')),
                               'effective_information': row['effective_information'],
                               '_id': f'{name}:native_loop:{row["from_id"]}:{row["to_id"]}',
                               '_source': 'validated_native_loop', '_source_kind': 'native'})
    for index, row in enumerate(extra):
        validated_role = extra_constraint_role(row)
        if validated_role == 'local_evidence_only':
            diagnostics['excluded_local_evidence_only'].append({
                'edge_id': row.get('edge_id'),
                'frame_endpoints': [row['session_a'], row['node_a'],
                                    row['session_b'], row['node_b']],
                'reason': 'joint_role=local_evidence_only is not eligible for joint fitting or independent holdout evaluation',
            })
            continue
        candidates.append({**row, '_id': str(row.get('edge_id') or
            f'extra:{row["session_a"]}:{row["node_a"]}:{row["session_b"]}:{row["node_b"]}:{index}'),
            '_source': str(row.get('source_file', constraints_path)), '_source_kind': 'extra',
            '_validated_role': validated_role})

    for row in candidates:
        sa, sb = row['session_a'], row['session_b']
        a, b = int(row['node_a']), int(row['node_b'])
        if a not in frame_rows[sa] or b not in frame_rows[sb]:
            diagnostics['skipped_quality_c'].append({'id': row['_id'], 'reason': 'endpoint_not_in_quality_table'})
            continue
        source = frame_rows[sa][a].get('primary_submap_id')
        target = frame_rows[sb][b].get('primary_submap_id')
        if not source or not target:
            diagnostics['skipped_quality_c'].append({'id': row['_id'], 'reason': 'no_primary_submap'})
            continue
        if source == target:
            diagnostics['skipped_internal'].append(row['_id'])
            continue
        wa, wb = quality_weight(frame_rows[sa][a]), quality_weight(frame_rows[sb][b])
        role = row.get('_validated_role') or (
            'heldout' if row.get('held_out') or row.get('joint_holdout') else 'train')
        if role == 'train' and min(wa, wb) < .15:
            diagnostics['skipped_quality_c'].append({'id': row['_id'], 'reason': 'quality_C',
                                                     'endpoint_weights': [wa, wb]})
            continue
        root_a, root_b = submap_meta[source]['anchor'], submap_meta[target]['anchor']
        la = inverse(selected_local[sa][root_a]) @ selected_local[sa][a]
        lb = inverse(selected_local[sb][root_b]) @ selected_local[sb][b]
        frame_covariance, covariance_diagnostic = edge_covariance(row, True)
        measurement, covariance = transform_frame_edge(
            np.asarray(row['T_a_b']), frame_covariance, la, lb)
        covariance /= max(.1, min(wa, wb))
        kind = 'cross_session' if sa != sb else 'loop'
        group = str(row.get('correlation_group') or region_group(
            sa, sb, common_initial[sa][a], common_initial[sb][b]))
        candidates_edge = {'id': row['_id'], 'from': source, 'to': target,
                           'measurement': measurement.tolist(), 'covariance': covariance.tolist(),
                           'type': kind, 'role': role, 'region_group': group,
                           'provenance': {'source': row['_source'], 'frame_endpoints': [sa, a, sb, b],
                                          'endpoint_quality_weights': [wa, wb],
                                          'endpoint_correlation_evidence_weights': [
                                              float(frame_rows[sa][a].get('evidence_weight') or 0.),
                                              float(frame_rows[sb][b].get('evidence_weight') or 0.)],
                                          'frame_edge_covariance': covariance_diagnostic,
                                          'mapped_anchor_nodes': [root_a, root_b]},
                           '_frame_key': tuple(sorted(((sa, a), (sb, b)))),
                           '_source_kind': row.get('_source_kind')}
        edges.append(candidates_edge)

    odometry = [edge for edge in edges if edge['type'] == 'odometry']
    uncertain, replaced = prefer_successful_replacements(
        [edge for edge in edges if edge['type'] != 'odometry'])
    diagnostics['replaced_native'] = replaced
    uncertain, dropped = deduplicate_edges(uncertain)
    diagnostics['deduplicated_edge_ids'] = dropped
    edges = odometry + uncertain
    roles_by_region = {}
    for edge in edges:
        if edge['type'] != 'odometry':
            roles_by_region.setdefault(edge['region_group'], set()).add(edge['role'])
    diagnostics['train_heldout_region_overlap'] = sorted(
        group for group, roles in roles_by_region.items()
        if {'train', 'heldout'} <= roles)
    structural_support = annotate_structural_support(edges)
    diagnostics['structural_support'] = structural_support
    diagnostics['structural_support_counts'] = {
        'total': len(structural_support),
        'graph_bridges': sum(row['graph_bridge'] for row in structural_support),
        'cycle_supported': sum(row['cycle_supported'] for row in structural_support),
        'non_structural_alternate_path': sum(
            row['non_structural_alternate_path'] for row in structural_support),
    }
    database_after = {name: sha256_file(Path(config_sessions[name]['database'])) for name in quality['sessions']}
    if database_after != database_before:
        raise RuntimeError('source database changed while building graph')
    manifest = {'schema_version': 1, 'submaps': backend_submaps, 'edges': edges,
                'metadata': {'created_utc': utc_now(), 'accepted': False, 'navigation': False,
                    'quality_manifest': str((quality_root/'manifest.json').resolve()),
                    'initial_graph': str(initial_graph.resolve()),
                    'constraints': str(constraints_path.resolve()),
                    'constraints_input_contract': (
                        'curated rows with T_a_b plus an explicit positive evidence/eligibility flag; '
                        'joint_role and heldout flags are consistency-checked; '
                        'local_evidence_only rows are excluded, never promoted to train'),
                    'database_sha256_before': database_before,
                    'database_sha256_after': database_after,
                    'originals_unchanged': True,
                    'odometry_model_sigma_by_session': {k: v.tolist() for k, v in odometry_models.items()},
                    'boundary_transition_policy': {
                        'included': bool(include_structural_transitions),
                        'covariance_inflation': float(boundary_covariance_inflation),
                        'switchable_type': 'structural'},
                    'diagnostics': diagnostics,
                    'method': ['native odometry condensed between quality submap anchors',
                        'first-order SE3 covariance propagation',
                        'robust-MAD model discrepancy and explicit temporal-correlation inflation',
                        'frame loop/cross measurements transferred to submap anchors',
                        'hard run boundaries represented only by inflated-covariance switchable structural transitions',
                        'one representative per submap-pair/physical-region/role'],
                    'limitations': ['quality and calibration evidence are correlated within each capture',
                        'odometry temporal-correlation inflation is an explicit model assumption, not calibrated ground truth',
                        'a graph-bridge structural transition cannot be outlier-tested by its residual and is a conditional continuity assumption',
                        'boundary covariance inflation requires sensitivity analysis; it is not a validation result',
                        'plane-only factors are deliberately omitted from this first rank-safe submap graph']}}
    hashes = {str(path.resolve()): sha256_file(path) for path in sorted(set(source_paths)) if path.is_file()}
    manifest['metadata']['source_sha256'] = hashes
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--quality-root', type=Path, required=True)
    parser.add_argument('--initial-graph', type=Path,
                        default=Path('outputs/student_union_20261017_geometry_review/graph_final'))
    parser.add_argument('--constraints', type=Path,
                        default=Path('outputs/student_union_20261017_geometry_review/graph_final/used_constraints.json'))
    parser.add_argument('--config', type=Path, default=Path('config/student_union_all_sessions.json'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--boundary-covariance-inflation', type=float, default=25.)
    parser.add_argument('--omit-structural-transitions', action='store_true')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(f'refusing to overwrite {args.output}')
    quality_manifest = args.quality_root/'manifest.json'
    quality = json.loads(quality_manifest.read_text())
    config = json.loads(args.config.read_text())
    if args.boundary_covariance_inflation <= 0:
        raise ValueError('boundary covariance inflation must be positive')
    manifest = build_graph(
        quality, config, args.quality_root, args.initial_graph, args.constraints,
        boundary_covariance_inflation=args.boundary_covariance_inflation,
        include_structural_transitions=not args.omit_structural_transitions)
    manifest['metadata']['config'] = str(args.config.resolve())
    manifest['metadata']['source_sha256'][str(args.config.resolve())] = sha256_file(args.config)
    args.output.mkdir(parents=True)
    shutil.copyfile(Path(__file__), args.output/'source_snapshot.py')
    (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    report = {'created_utc': utc_now(), 'argv': sys.argv,
              'manifest_sha256': sha256_file(args.output/'manifest.json'),
              'source_sha256': sha256_file(Path(__file__)),
              'counts': {'submaps': len(manifest['submaps']), 'edges': len(manifest['edges']),
                         'odometry': sum(e['type']=='odometry' for e in manifest['edges']),
                         'train_uncertain': sum(e['type']!='odometry' and e['role']=='train' for e in manifest['edges']),
                         'heldout': sum(e['role']=='heldout' for e in manifest['edges'])},
              'accepted': False, 'navigation': False}
    (args.output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'output': str(args.output), **report['counts'], 'accepted': False}))


if __name__ == '__main__':
    main()
