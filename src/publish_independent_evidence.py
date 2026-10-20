#!/usr/bin/env python3
"""Package actual capture figures, development metrics and input hashes."""
import argparse
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from optimize_session import sha256_file, utc_now
from render_session_report import read_ply
from export_submap_candidate_clouds import cloud_bounds, render_projection


def read(path):return json.loads(path.read_text())


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--test-result',default='Test result not supplied to this publisher');a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    a.output.mkdir(parents=True)
    review=read(a.run/'review.json')
    manual=read(a.run/'manual_structure_soft_v2/lobby_39__bridge_out1f_79/report.json')
    local=read(a.run/'floor2_structure_planes_v4/floor2_548__floor2_981/report.json')
    integration=read(a.run/'structure_integration/report.json')
    final=read(a.run/'structure_integration_evaluation/report.json')
    exports=read(a.run/'clouds/manifest.json')
    sweep=manual['depth_threshold_sweep']['sweep']
    fig,ax=plt.subplots(figsize=(9,4.5))
    ax.plot([r['range_gate_m']*100 for r in sweep],[r['valid_both'] for r in sweep],'o-',label='Both-depth corner observations')
    ax.axhline(8,color='#d47a32',ls='--',label='RGB identity observations retained: all 8')
    ax.set(xlabel='Allowed local depth range (cm)',ylabel='Number of corners',ylim=(0,9),
           title='Depth validity is different from matching the same corner')
    ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(a.output/'depth_threshold_sweep.png',dpi=170);plt.close(fig)
    records=[]
    images={}
    for s in review['sessions']:
        images[f'{s}_clouds.png']=(a.run/'figures'/f'{s}_clouds.png',f'{s}: selected incumbent and independent candidate; identical A/B frames and plotting limits.')
    images['independent_fixed_patches.png']=(a.run/'figures/fixed_patch_summary.png','Three independent session portfolios on frozen historical finite patches; bridges have no matching patch inventory.')
    for name in ('directional_comparison','horizontal_same_samples','wall_same_samples'):
        images[name+'.png']=(a.run/'directional_diagnostics'/(name+'.png'),'Same wall/floor patch observations; structure_joint fits these planes, so this is training/development evidence.')
    for folder,prefix in [('manual_structure_soft_v2/lobby_39__bridge_out1f_79','doorway'),
                          ('floor2_structure_planes_v4/floor2_548__floor2_981','floor2_structure')]:
        for name in ('train_tracked_reprojection','test_tracked_reprojection','withheld_doorway_cloud'):
            path=a.run/folder/(name+'.png')
            if path.exists():images[f'{prefix}_{name}.png']=(path,'Actual RGB or depth observations. Train is fitted; test uses separate frames but remains correlated development evidence.')
    for name in ('common_support_all_stages','floor2_548_981_wall_0_0','floor2_656_1055_wall_0_1','upper_corridor_time_windows_xy'):
        images['integrated_'+name+'.png']=(a.run/'structure_integration_evaluation/figures'/(name+'.png'),
                                           'Full floor2 frame graph after ONE weak structure factor; remaining defects are preserved.')
    for name,(source,caption) in images.items():
        with Image.open(source) as im:im.verify()
        target=a.output/name;shutil.copyfile(source,target)
        if sha256_file(source)!=sha256_file(target):raise ValueError('Image copy changed bytes')
        with Image.open(target) as im:size=list(im.size)
        records.append(dict(file=name,source=str(source.resolve()),sha256=sha256_file(target),
                            size_pixels=size,caption=caption,synthetic=False))
    # Render the latest dense candidate alongside the incumbent, keeping its
    # source poses' common frozen world gauge and exactly 925 A/B observations.
    paths=[a.run/'clouds/floor2/selected/cloud.ply',a.run/'clouds/floor2/weak_structure/cloud.ply']
    clouds=[read_ply(p) for p in paths];bounds=cloud_bounds(clouds)
    fig,axes=plt.subplots(2,2,figsize=(13,10))
    for col,(name,cloud) in enumerate(zip(('incumbent_preview','weak_structure_full_graph'),clouds)):
        for row,(x,y) in enumerate((('x','y'),('x','z'))):render_projection(axes[row,col],cloud,x,y,bounds,title=name)
    fig.suptitle('Actual regenerated floor2 point clouds; same 925 RGB-D frames\nOne weak structure factor improves a wall; map-wide residual errors remain')
    fig.tight_layout(rect=(0,0,1,.94));latest=a.output/'integrated_dense_cloud.png';fig.savefig(latest,dpi=160);plt.close(fig)
    records.append(dict(file=latest.name,sha256=sha256_file(latest),source_clouds=[dict(path=str(p.resolve()),sha256=sha256_file(p)) for p in paths],
                        synthetic=False,caption='Actual dense RGB-D reconstruction; identical source frames; common view bounds.'))
    threshold=a.output/'depth_threshold_sweep.png'
    records.append(dict(file=threshold.name,sha256=sha256_file(threshold),synthetic=False,
                        source=str((a.run/'manual_structure_soft_v2/lobby_39__bridge_out1f_79/report.json').resolve())))
    numerical={}
    for session in review['sessions']:
        d=read(a.run/'optimization'/session/'report.json')
        numerical[session]={m:dict(outer_iterations=len(v['solver_iterations']),
                                  final_iteration=v['solver_iterations'][-1],
                                  heldout_edges_in_objective=v['heldout_edge_count_in_objective']) for m,v in d['variants'].items()}
    sessions={s:{k:v[k] for k in ('used_AB_frames','submaps','native_loop_count','retained_loop_roles',
                 'common_fixed_patch_count','fixed_patch_statistics','decisions','selected_label','best_attempt_for_visual_comparison',
                 'selected_trajectory','selected_trajectory_sha256','database_unchanged','database_sha256_after')} for s,v in review['sessions'].items()}
    def small(d):
        return {k:d.get(k) for k in ('training_observations','test_observations','training_unique_metric_landmarks',
                'training_after','test_before','test_after','withheld_cloud_before','withheld_cloud_after',
                'delta_from_incumbent','local_support_passed','local_rejection_reasons','depth_threshold_sweep')}
    summary={'created_utc':utc_now(),'accepted':False,'navigation':False,'run':str(a.run.resolve()),
             'sessions':sessions,'independent_numerical_runs':numerical,'manual_doorway':small(manual),
             'floor2_structure_local':small(local),'floor2_structure_full_graph':{
                'common_support':final['common_support_all_stages'],
                'critical_patches':[r for r in final['comparisons'] if (r['a'],r['b']) in [(548,981),(656,1055)]],
                'final_iteration':integration['variants']['baseline_huber']['solver_iterations'][-1],
                'strict_converged':integration['variants']['baseline_huber']['strict_converged'],
                'weak_factor':read(a.run/'structure_integration/manifest.json')['metadata']['weak_manual_structure_factor'],
                'cloud_points':read(a.run/'clouds/floor2/weak_structure/summary.json')['output_points']},
             'cloud_exports':[dict(session=x['session'],label=x['label'],points=x['summary']['output_points'],
                                  frames=x['summary']['decoded_depth_frames'],cloud=x['cloud'],sha256=x['cloud_sha256']) for x in exports['exports']],
             'merge_consideration':review['merge_consideration'],
             'test_result':a.test_result,
             'limitations':['Repeated development observations, not surveyed ground truth.',
                            'The strongest local wall/floor numbers use fitted surfaces.',
                            'Incumbent lobby/floor2 poses inherit a prior joint hypothesis; the 15 new graph solves contain no cross-session factors.',
                            'Bridge sessions lack enough independent within-session validation.',
                            'Latest weak-structure capture is a comparison candidate; default RViz is not replaced.']}
    input_paths=[a.run/'review.json',a.run/'clouds/manifest.json',a.run/'structure_integration/report.json',
                 a.run/'structure_integration_evaluation/report.json',a.run/'directional_diagnostics/report.json']
    summary['input_hashes']={str(p.resolve()):sha256_file(p) for p in input_paths}
    (a.output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
    (a.output/'manifest.json').write_text(json.dumps(dict(created_utc=utc_now(),source_sha256=sha256_file(Path(__file__)),
        figures=records,summary_sha256=sha256_file(a.output/'summary.json'),accepted=False,navigation=False),indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(dict(output=str(a.output),images=len(records))))


if __name__=='__main__':main()
