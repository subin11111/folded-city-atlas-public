#!/usr/bin/env python3
"""Verify portfolio provenance, actual image files, matched observations and DBs."""
import argparse
import csv
import json
from pathlib import Path
import shutil

import numpy as np
from PIL import Image

from build_session_portfolios import ROOT, read, dump
from export_rtabmap_db import load_pose_csv
from optimize_robust_submaps import parse_manifest
from optimize_session import sha256_file, utc_now
from revalidate_loop_constraints import SPECS


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--assets',type=Path,required=True);a=p.parse_args();run=a.run.resolve();assets=a.assets.resolve()
    master=read(assets/'manifest.json');rows=[];checks=[];snapshots=run/'verified_source_snapshots';snapshots.mkdir(exist_ok=False)
    image_count=0;fresh_count=0;input_checks={};summaries={}
    for name in SPECS:
        summary=read(assets/name/'summary.json');manifest=read(assets/name/'manifest.json');source=read(run/name/'report.json')
        assert sha256_file(assets/name/'summary.json')==manifest['summary_sha256']
        assert sha256_file(run/name/'report.json')==summary['source_report_sha256']
        assert sha256_file(Path(source['database']))==source['database_sha256']
        for key,value in source['input_hashes'].items():
            assert sha256_file(Path(key))==value,key;input_checks[key]=value
        graph=read(run/name/'graph.json');nodes,edges=parse_manifest(graph)
        assert {n.session_id for n in nodes}=={name}
        assert all(e.edge_type!='cross_session' for e in edges)
        stage_map={s['id']:s for s in summary['stages']};exports={s['id']:s for s in summary['exports']}
        same=load_pose_csv(Path(stage_map['O00']['trajectory']));pre=load_pose_csv(Path(stage_map['P00']['trajectory']))
        assert same.keys()==pre.keys()
        assert all(np.array_equal(same[k],pre[k]) for k in same),'Preprocessing-only changed poses'
        reference=exports['P00']
        for stage in summary['stages']:
            key=stage['id'];export=exports[key];evaluation=summary['evaluation'][key]
            assert sha256_file(Path(stage['trajectory']))==stage['trajectory_sha256']
            assert sha256_file(Path(export['cloud']))==export['sha256']
            if key not in ('O00','O_FULL'):
                assert export['frame_ids']==reference['frame_ids']
                assert export['camera_points_sha256']==reference['camera_points_sha256']
                assert export['input_points']==reference['input_points']
            solver=stage.get('solver') or {}
            if stage['newly_computed']:fresh_count+=1
            rows.append({'session':name,'candidate_id':key,'label':stage['label'],
                'fresh_solve':stage['newly_computed'],'frame_count':len(export['frame_ids']),
                'camera_input_points':export['input_points'],'output_voxel_points':export['points'],
                'fixed_patch_count':evaluation['count'],'fixed_patch_median_cm':evaluation['median_cm'],
                'fixed_patch_p90_cm':evaluation['p90_cm'],'fixed_patch_maximum_cm':evaluation['maximum_cm'],
                'inner_success':solver.get('inner_success',solver.get('success')),
                'strict_outer_converged':solver.get('strict_outer_converged'),'active_bounds':solver.get('active_bounds'),
                'image':str(assets/name/(key+'.png')),'cloud':export['cloud']})
        for figure in manifest['figures']:
            path=assets/name/figure['file'];assert sha256_file(path)==figure['sha256']
            with Image.open(path) as im:im.verify()
            image_count+=1
        summaries[name]={'full_app_frames':summary['full_app_frame_count'],'common_AB_frames':summary['frame_count'],
            'profile':summary['selected_profile'],'stages':len(summary['stages']),
            'pixel_retention_on_common_frames':reference['input_points']/exports['O00']['input_points'],
            'new_local_refinements':source['new_refinement_count'],'normal_factors':source['normal_factor_count'],
            'default_map_replaced':False,'winner':None}
        checks.append(name)
    assert sha256_file(assets/'overview.png')==master['overview_sha256']
    with Image.open(assets/'overview.png') as im:im.verify()
    image_count+=1
    for key,value in input_checks.items():
        path=Path(key)
        if path.suffix=='.py':shutil.copyfile(path,snapshots/(path.stem+'_'+value[:16]+'.py'))
    csvpath=assets/'candidate_metrics.csv'
    if csvpath.exists():raise FileExistsError(csvpath)
    with csvpath.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    result={'created_utc':utc_now(),'sessions_verified':checks,'candidate_count':len(rows),'fresh_solves':fresh_count,
        'image_count':image_count,'original_databases_unchanged':True,'pose_only_candidates_have_identical_preprocessed_observations':True,
        'preprocessing_only_preserves_exact_app_poses':True,'all_source_hashes_verified':input_checks,
        'candidate_metrics_sha256':sha256_file(csvpath),'sessions':summaries,'user_winners':None,
        'default_rviz_changed':False,'accepted':False,'navigation':False,
        'note':'Passing provenance/tests is not surveyed map accuracy, loop identity validation or user selection.'}
    dump(assets/'verification.json',result);dump(run/'verification.json',result)
    print(json.dumps({k:result[k] for k in ('candidate_count','fresh_solves','image_count','sessions')}))


if __name__=='__main__':main()
