"""Focused safety/algebra tests for the frame-level causal ablation.

No large project outputs or source DBs are opened by these tests.
"""
import copy
from pathlib import Path
import sys

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from build_robust_submap_graph import transform_frame_edge
from optimize_floor2_frames import build_frame_graph, lift_anchor_edge, node_key
from optimize_robust_submaps import (_robust_weights, connected_components,
                                     parse_manifest)
from optimize_session import inverse


def pose(xyz, angles=(0., 0., 0.)):
    result = np.eye(4)
    result[:3, 3] = xyz
    result[:3, :3] = Rotation.from_rotvec(angles).as_matrix()
    return result


@pytest.mark.parametrize('reverse', [False, True])
@pytest.mark.parametrize('role', ['train', 'heldout'])
def test_frame_to_anchor_and_back_preserves_measurement_covariance_role(reverse, role):
    initial = {1: pose([1., 2., -.2], [.2, -.4, .3]),
               2: pose([-1., 1., .5], [-.1, .5, .2]),
               3: pose([2., .5, 1.], [.6, -.2, .4]),
               4: pose([-.3, .1, .9], [.2, .3, -.7])}
    anchors = {'a': 1, 'b': 4}
    a, b, aa, bb = (2, 3, 'a', 'b') if not reverse else (3, 2, 'b', 'a')
    measurement = pose([.5, -.3, 1.], [.3, .8, -.2])
    rng = np.random.default_rng(42)
    random = rng.normal(size=(6, 6))
    # Includes off-diagonal correlations; scalar-only covariance tests would
    # miss adjoint direction errors caused by nonzero translation/rotation.
    covariance = (random @ random.T + np.eye(6)) * .001
    la = inverse(initial[anchors[aa]]) @ initial[a]
    lb = inverse(initial[anchors[bb]]) @ initial[b]
    root_measurement, root_covariance = transform_frame_edge(measurement, covariance, la, lb)
    edge = {'id': 'test', 'from': aa, 'to': bb, 'type': 'loop', 'role': role,
            'region_group': 'reserved_region', 'measurement': root_measurement.tolist(),
            'covariance': root_covariance.tolist(),
            'provenance': {'frame_endpoints': ['floor2', a, 'floor2', b]}}
    original = copy.deepcopy(edge)
    lifted = lift_anchor_edge(edge, anchors, initial)
    assert edge == original
    np.testing.assert_allclose(lifted['measurement'], measurement, atol=1e-12)
    np.testing.assert_allclose(lifted['covariance'], covariance, atol=1e-12)
    assert lifted['role'] == role
    assert lifted['region_group'] == edge['region_group']
    assert (lifted['from'], lifted['to']) == (node_key(a), node_key(b))


def fixture_inputs():
    initial = {n: pose([.2 * (n - 100), 1., 4.]) for n in range(100, 105)}
    frames = {n: {'quality_grade': 'A', 'hard_boundary': 'False',
                  'primary_submap_id': 's1' if n < 103 else 's2'} for n in initial}
    submaps = {'submaps': [{'submap_id': 's1', 'run_id': 'run1'},
                           {'submap_id': 's2', 'run_id': 'run1'}]}
    canonical = {'submaps': [
        {'id': 's1', 'session_id': 'floor2', 'anchor_node_id': 100,
         'pose': initial[100].tolist()},
        {'id': 's2', 'session_id': 'floor2', 'anchor_node_id': 104,
         'pose': initial[104].tolist()},
        {'id': 'unrelated', 'session_id': 'lobby', 'anchor_node_id': 1,
         'pose': np.eye(4).tolist()}],
        'edges': [{'id': 'cross', 'type': 'cross_session', 'from': 'unrelated', 'to': 's1'}],
        'metadata': {'odometry_model_sigma_by_session': {'floor2': [.01]*6}}}
    native = {'odometry': [
        {'from_id': a, 'to_id': a+1,
         'measurement': (inverse(initial[a]) @ initial[a+1]).tolist(),
         'covariance_6x6': np.diag([.01]*6).tolist()}
        for a in range(100, 104)], 'verified_loops': []}
    return canonical, initial, native, frames, submaps


def local_measurements(a=101, b=102):
    return {'complete': True, 'inputs_unchanged': True, 'records': [
        {'a': a, 'b': b, 'accepted': True, 'T_a_b': pose([.21, 0, 0]).tolist(),
         'covariance_6x6': np.diag([.02]*6).tolist()}]}


def test_refined_preserves_fixed_data_and_only_adds_new_local_factor():
    inputs = fixture_inputs()
    before = copy.deepcopy(inputs)
    baseline = build_frame_graph(*inputs)
    refined = build_frame_graph(*inputs, local_measurements())
    assert inputs[0] == before[0]
    assert baseline['submaps'] == refined['submaps']
    assert baseline['edges'] == refined['edges'][:-1]
    assert len(refined['edges']) == len(baseline['edges']) + 1
    assert refined['edges'][-1]['type'] == 'registration'
    assert refined['edges'][-1]['role'] == 'train'
    assert all(r['session_id'] == 'floor2' for r in refined['submaps'])
    assert all(e['type'] != 'cross_session' for e in refined['edges'])
    assert refined['metadata']['bundle_adjustment'] is False
    assert refined['metadata']['intrinsics_or_depth_scale_optimized'] is False


def test_c_frame_with_pose_is_retained_with_weak_not_strong_continuity():
    inputs = fixture_inputs()
    inputs[3][102]['quality_grade'] = 'C'
    graph = build_frame_graph(*inputs)
    assert len(graph['submaps']) == 5
    for edge in graph['edges']:
        touches_c = node_key(102) in (edge['from'], edge['to'])
        assert edge['type'] == ('structural' if touches_c else 'odometry')
        if touches_c:
            assert edge['provenance']['uncertainty_multiplier'] == 25.
            assert edge['provenance']['low_quality_endpoint'] is True
    assert 'omit from dense cloud export' in graph['metadata']['quality_C_policy']


def test_missing_c_pose_is_omitted_but_missing_usable_pose_rejected():
    inputs = fixture_inputs()
    inputs[3][99] = {'quality_grade': 'C', 'hard_boundary': 'True', 'primary_submap_id': ''}
    graph = build_frame_graph(*inputs)
    assert graph['metadata']['quality_frames_without_initial_pose_omitted'] == [99]
    assert node_key(99) not in {n['id'] for n in graph['submaps']}
    inputs[3][99]['quality_grade'] = 'A'
    with pytest.raises(ValueError, match='inventories differ'):
        build_frame_graph(*inputs)


@pytest.mark.parametrize('case', ['boundary', 'run', 'quality', 'reserved', 'duplicate', 'incomplete'])
def test_unsafe_local_factors_are_rejected(case):
    inputs = fixture_inputs()
    measurements = local_measurements()
    if case == 'boundary': inputs[3][102]['hard_boundary'] = 'true'
    if case == 'run':
        inputs[3][102]['primary_submap_id'] = 's2'
        inputs[4]['submaps'][1]['run_id'] = 'different_run'
    if case == 'quality': inputs[3][102]['quality_grade'] = 'C'
    if case == 'reserved':
        # Endpoint 101 is within the explicit +/- 10 frame exclusion.
        inputs[2]['verified_loops'] = [{'from_id': 111, 'to_id': 800, 'held_out': True}]
    if case == 'duplicate': measurements['records'] *= 2
    if case == 'incomplete': measurements['complete'] = False
    with pytest.raises(ValueError):
        build_frame_graph(*inputs, measurements)


def test_registration_is_robustly_weighted_not_quadratic_odometry():
    graph = build_frame_graph(*fixture_inputs(), local_measurements())
    graph['edges'][-1]['measurement'] = pose([5., 0, 0]).tolist()
    nodes, edges = parse_manifest(graph)
    weights, _ = _robust_weights('baseline_huber', edges,
                                {n.id: n.initial for n in nodes}, 2.5, 5.991, 10.)
    assert 0. < weights['local_rgbd:101:102'] < .2
    assert all(weights[e.id] == 1. for e in edges if e.is_odometry)


def test_heldout_loop_is_not_a_component_connection_and_role_is_preserved():
    inputs = fixture_inputs()
    inputs[2]['odometry'] = inputs[2]['odometry'][:2]
    inputs[0]['edges'].append({'id': 'heldout', 'from': 's1', 'to': 's2', 'type': 'loop',
                              'role': 'heldout', 'region_group': 'reserve',
                              'measurement': pose([.8, 0, 0]).tolist(),
                              'covariance': np.eye(6).tolist(),
                              'provenance': {'frame_endpoints': ['floor2', 100, 'floor2', 104]}})
    graph = build_frame_graph(*inputs)
    nodes, edges = parse_manifest(graph)
    assert graph['edges'][-1]['role'] == 'heldout'
    components = connected_components([n.id for n in nodes], edges)
    assert len(components) == 3
    assert {node_key(104)} in [set(c) for c in components]


def test_lift_rejects_cross_session_endpoint_metadata():
    with pytest.raises(ValueError, match='Cross-session'):
        lift_anchor_edge({'provenance': {'frame_endpoints': ['lobby', 1, 'floor2', 2]}}, {}, {})


def test_canonical_anchor_initialization_must_match():
    inputs = fixture_inputs()
    inputs[0]['submaps'][0]['pose'][0][3] += .5
    with pytest.raises(ValueError, match='initialization differ'):
        build_frame_graph(*inputs)
