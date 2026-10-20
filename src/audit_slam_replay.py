#!/usr/bin/env python3
"""Independent prefix causality, real source sample and immutable-final checks."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from optimize_session import apply,sha256_file,utc_now
from slam_replay_core import load_sessions,decode_rgbd,ROOT


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--replay',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    events=json.loads((a.replay/'events.json').read_text());manifest=json.loads((a.replay/'manifest.json').read_text())
    sessions=load_sessions();seen={n:[] for n in sessions};local={n:{} for n in sessions}
    rng=np.random.default_rng(20261019);sample_count=0;max_distance=0.;rows=[]
    accepted={n:set() for n in sessions}
    for sequence,event in enumerate(events):
        assert event['sequence']==sequence
        name=event['session'];session=sessions[name]
        seen[name].extend(event['ids'])
        assert seen[name]==session.ids[:len(seen[name])], 'Not a causal source prefix'
        assert len(seen[name])==event['seen_count'] and len(set(seen[name]))==len(seen[name])
        z=np.load(event['snapshot']);assert sha256_file(Path(event['snapshot']))==event['snapshot_sha256']
        ids=list(map(int,z['ids']));assert set(ids)==set(seen[name])
        assert np.array_equal(z['raw_poses'],np.array([session.raw[n] for n in ids]))
        for check in event['loop_checks']:
            assert check['from_id'] in seen[name] and check['to_id'] in seen[name]
            assert check['available_at_seen_count']==len(seen[name])
            if check['accepted']:accepted[name].add((check['from_id'],check['to_id']))
        assert set(map(tuple,z['loops']))==accepted[name]
        solver=event['solver']
        if solver['applied']:assert solver['success'] and solver['active_bounds']==0
        else:assert np.array_equal(z['poses'],z['before_poses']), 'Unaccepted solve altered output'
        delta=np.linalg.norm(z['poses'][:,:3,3]-z['before_poses'][:,:3,3],axis=1).max(initial=0)
        assert abs(delta-solver['maximum_pose_update_m'])<1e-8
        for n in event['ids']:
            try:local[name][n]=decode_rgbd(session.row(n))[0]
            except ValueError:local[name][n]=np.empty((0,3),np.float32)
        # Inspect real samples without depending on the writer's iteration order.
        expected=np.concatenate([apply(local[name][n],pose) for n,pose in zip(ids,z['poses'])])
        indices=rng.choice(len(z['points']),min(128,len(z['points'])),replace=False)
        distances=cKDTree(expected).query(z['points'][indices],workers=1)[0]
        assert distances.max(initial=0)<2e-5,'Displayed sample lacks arrived RGB-D support'
        max_distance=max(max_distance,float(distances.max(initial=0)));sample_count+=len(indices)
        rows.append({'sequence':sequence,'session':name,'seen_count':len(ids),'applied':solver['applied'],
            'solver_seconds':solver['seconds'],'maximum_pose_update_m':float(delta)})
        if sequence%10==0:print('AUDIT_PREFIX '+str(sequence+1)+'/'+str(len(events)),flush=True)
    pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest']
    assert sha256_file(mp)==pointer['manifest_sha256']
    selected=json.loads(mp.read_text());db_hashes={}
    for name,session in sessions.items():
        assert len(seen[name])==len(session.ids)
        db_hashes[name]=sha256_file(session.database)
        assert db_hashes[name]==selected['database_integrity'][name]['sha256']
    final=json.loads((ROOT/'config/current_local_view.json').read_text())
    assert sha256_file(ROOT/final['structural_manifest'])==final['structural_manifest_sha256']
    for file,digest in manifest['code_sha256'].items():
        snapshot=a.replay/(Path(file).stem+'_source.py')
        if not snapshot.exists():snapshot=a.replay/'source_snapshot'/Path(file).name
        assert sha256_file(snapshot)==digest,'Missing exact executed source snapshot'
    solvers=[e['solver'] for e in events if e['solver']['ran']]
    result={'created_utc':utc_now(),'prefixes_checked':len(events),'source_frames':sum(map(len,seen.values())),
        'future_node_or_loop_use':False,'all_sessions_completed':True,
        'source_supported_point_samples_checked':sample_count,'maximum_sample_roundoff_m':max_distance,
        'real_solve_attempts':len(solvers),'applied_solutions':sum(s['applied'] for s in solvers),
        'solver_seconds_median':float(np.median([s['seconds'] for s in solvers])),
        'solver_seconds_max':max(s['seconds'] for s in solvers),
        'verified_stored_loop_pairs':sum(map(len,accepted.values())),
        'raw_database_sha256_unchanged':db_hashes,'selected_map_sha256_unchanged':pointer['manifest_sha256'],
        'final_display_sha256_unchanged':final['structural_manifest_sha256'],
        'executed_source_snapshots_verified':True,'replay_wall_seconds':manifest['wall_seconds'],
        'scope':'Causal recorded-VIO/stored-candidate replay, sampled point provenance. Not new online place discovery, live sensor benchmark, final mesh generation or navigation approval.',
        'rows':rows,'audit_source_sha256':sha256_file(Path(__file__))}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    for session in sessions.values():session.connection.close()
    print(json.dumps({k:result[k] for k in ['prefixes_checked','source_frames','applied_solutions','verified_stored_loop_pairs','solver_seconds_max']}),flush=True)


if __name__=='__main__':main()
