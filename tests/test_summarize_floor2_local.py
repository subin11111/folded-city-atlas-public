"""Synthetic receipt/binding tests; never open project databases or outputs."""
import copy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from summarize_floor2_local import (canonical_manifest_sha256, graph_ablation,
                                    sha256_file, verify_hashes, verify_solve)


def graph():
    return {'schema_version': 1, 'submaps': [
        {'id': name, 'frame_id': i, 'session_id': 'floor2',
         'quality': grade, 'pose': [i-1., 0, 0, 0, 0, 0, 1]}
        for name, i, grade in [('a',1,'A'),('b',2,'B'),('c',3,'C')]],
        'edges': [{'id':'ab','from':'a','to':'b','type':'odometry','role':'train'},
                  {'id':'bc','from':'b','to':'c','type':'structural','role':'train'},
                  {'id':'ac_holdout','from':'a','to':'c','type':'loop','role':'heldout'}],
        'metadata': {'bundle_adjustment':False}}


def refined_graph(base):
    refined=copy.deepcopy(base)
    refined['metadata']['different_created_time']='allowed'
    refined['edges'] += [{'id':f'new{i}','from':'a','to':'c','type':'registration','role':'train'}
                         for i in range(3)]
    return refined


def write_json(path,value):
    path.write_text(json.dumps(value,indent=2)+'\n')


def write_poses(path,ids=(1,2,3),offset=0.):
    path.write_text('id,stamp,x,y,z,qx,qy,qz,qw\n'+''.join(
        f'{i},0,{i-1+offset},0,0,0,0,0,1\n' for i in ids))


def solve_fixture(tmp_path):
    base=graph()
    write_json(tmp_path/'manifest.json',base)
    write_poses(tmp_path/'trajectory.csv')
    write_poses(tmp_path/'trajectory_AB.csv',(1,2))
    source=tmp_path/'source.py'
    source.write_text('# original source\n')
    variant={'fit_edge_ids':['ab','bc'],'heldout_edge_count_in_objective':0,
             'heldout_edge_ids':['ac_holdout'],'component_count':1,
             'poses':{n['id']:n['pose'] for n in base['submaps']},
             'solver_iterations':[{'success':True,'nfev':5,'maximum_weight_change':0.,
                                   'maximum_parameter_step':1e-8,'active_bounds':0}],
             'edges':[{'id':'ab','type':'odometry'},{'id':'bc','type':'structural'},
                      {'id':'ac_holdout','type':'loop'}]}
    report={'input_hashes_unchanged':True,'source_hashes':{str(source):sha256_file(source)},
            'provenance':{'canonical_manifest_sha256':canonical_manifest_sha256(base)},
            'graph_manifest_sha256':sha256_file(tmp_path/'manifest.json'),
            'trajectory_sha256':sha256_file(tmp_path/'trajectory.csv'),
            'trajectory_AB_sha256':sha256_file(tmp_path/'trajectory_AB.csv'),
            'export_AB_count':2,'variants':{'baseline_huber':variant}}
    write_json(tmp_path/'report.json',report)
    return base,report


def test_only_three_registration_ablation_and_bridge_caveat():
    base=graph()
    result=graph_ablation(base,refined_graph(base))
    assert result['all_shared_factors_identical']
    assert result['added_registration_ids']==['new0','new1','new2']
    assert result['baseline_components']==result['refined_components']==1
    assert result['baseline_components_without_structural']==2
    # The heldout a-c edge must not hide that b-c is a training bridge.
    assert result['structural_graph_bridge_ids']==['bc']
    assert 'cannot independently validate' in result['limitation']


@pytest.mark.parametrize('change',['node','shared_factor','missing_factor','duplicate',
                                   'wrong_type','wrong_role','too_few'])
def test_ablation_rejects_noncausal_factor_or_inventory_changes(change):
    base=graph();refined=refined_graph(base)
    if change=='node': refined['submaps'][0]['pose'][0]=9.
    if change=='shared_factor': refined['edges'][0]['unexpected_change']=True
    if change=='missing_factor': refined['edges'].pop(0)
    if change=='duplicate': refined['edges'].append(copy.deepcopy(refined['edges'][0]))
    if change=='wrong_type': refined['edges'][-1]['type']='loop'
    if change=='wrong_role': refined['edges'][-1]['role']='heldout'
    if change=='too_few': refined['edges'].pop()
    with pytest.raises(ValueError): graph_ablation(base,refined)


def test_hash_failure_is_detected(tmp_path):
    path=tmp_path/'input'
    path.write_text('before')
    hashes={str(path):sha256_file(path)}
    verify_hashes(hashes)
    path.write_text('after')
    with pytest.raises(ValueError,match='Hash mismatch'): verify_hashes(hashes)


def test_correct_solve_graph_trajectory_binding(tmp_path):
    base,_=solve_fixture(tmp_path)
    result=verify_solve(tmp_path,base)
    assert result['report_graph_trajectory_binding_verified']
    assert result['full_pose_count']==3
    assert result['AB_pose_count']==2
    assert result['outer_strict_stopping_condition_met']


@pytest.mark.parametrize('change',['source','graph_binding','graph_file','trajectory_file',
                                   'full_pose_value','full_coverage','fit_ids','heldout_fit',
                                   'ab_pose_value','ab_coverage','ab_report_count',
                                   'heldout_ids','solved_pose_ids'])
def test_solve_binding_rejects_mismatch_even_with_updated_trajectory_hash(tmp_path,change):
    base,report=solve_fixture(tmp_path)
    variant=report['variants']['baseline_huber']
    if change=='source': (tmp_path/'source.py').write_text('# edited\n')
    if change=='graph_binding': report['provenance']['canonical_manifest_sha256']='not_the_graph'
    if change=='graph_file':
        path=tmp_path/'manifest.json';path.write_text(path.read_text()+'\n')
    if change=='trajectory_file': write_poses(tmp_path/'trajectory.csv',offset=.5)
    if change in ('full_pose_value','full_coverage'):
        write_poses(tmp_path/'trajectory.csv',ids=(1,2) if change=='full_coverage' else (1,2,3),
                    offset=.5 if change=='full_pose_value' else 0.)
        report['trajectory_sha256']=sha256_file(tmp_path/'trajectory.csv')
    if change=='fit_ids': variant['fit_edge_ids']=['ab']
    if change=='heldout_fit': variant['heldout_edge_count_in_objective']=1
    if change in ('ab_pose_value','ab_coverage'):
        write_poses(tmp_path/'trajectory_AB.csv',ids=(1,3) if change=='ab_coverage' else (1,2),
                    offset=.5 if change=='ab_pose_value' else 0.)
        report['trajectory_AB_sha256']=sha256_file(tmp_path/'trajectory_AB.csv')
    if change=='ab_report_count': report['export_AB_count']=999
    if change=='heldout_ids': variant['heldout_edge_ids']=[]
    if change=='solved_pose_ids': variant['poses']['unrelated']=[0,0,0,0,0,0,1]
    write_json(tmp_path/'report.json',report)
    with pytest.raises(ValueError): verify_solve(tmp_path,base)


def test_iteration_budget_is_not_reported_as_strict_convergence(tmp_path):
    base,report=solve_fixture(tmp_path)
    last=report['variants']['baseline_huber']['solver_iterations'][-1]
    last.update(success=False,maximum_parameter_step=.2,nfev=200)
    write_json(tmp_path/'report.json',report)
    result=verify_solve(tmp_path,base)
    assert not result['all_inner_solves_success']
    assert not result['outer_strict_stopping_condition_met']
