#!/usr/bin/env python3
"""Directional diagnostics for the exact floor2 548/981 comparison.

Floor and wall normal errors are observable; finite patch centroid differences
along a wall are NOT treated as corresponding-point X/Y truth. Camera reprojection
of manual landmarks is evaluated separately in the structure-fitting report.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import sqlite3

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from evaluate_submap_candidates import fixed_patches, plot_patch
from evaluate_map_surfaces import pair_metrics
from export_rtabmap_db import load_pose_csv
from optimize_session import Clouds, inverse, sha256_file, utc_now

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--structure-report',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    a.output.mkdir(parents=True)
    graph=ROOT/'outputs/student_union_20261017_geometry_review/graph_final/floor2'
    paths={'previous_rviz':ROOT/'outputs/student_union_20261017_all_preview/bundle/floor2_trajectory.csv',
           'frozen_initial':graph/'initial.csv','4dof':graph/'pgo4.csv','finite_planes':graph/'pgo6_planes.csv'}
    transforms={}
    for label,path in paths.items():
        poses=load_pose_csv(path);transforms[label]=inverse(poses[548])@poses[981]
    structure=json.loads(a.structure_report.read_text())
    transforms['structure_joint']=np.array(structure['T_a_b'])
    database=ROOT/'data/raw/rtabmap/261017-013501-2f.db';before=sha256_file(database)
    raw=load_pose_csv(ROOT/'outputs/student_union_20261017_added_reviewed/floor2/raw_main_trajectory.csv')
    with sqlite3.connect(f'file:{database}?mode=ro',uri=True) as con:
        clouds=Clouds(con,None)
        pa,pb=fixed_patches(clouds,raw,548),fixed_patches(clouds,raw,981)
    metrics={}
    for kind in ('horizontal','wall'):
        first,second=pa[kind,0],pb[kind,0]
        metrics[kind]={label:pair_metrics(first,second,np.eye(4),t) for label,t in transforms.items()}
        plot_patch(a.output/f'{kind}_same_samples.png',first,second,transforms,metrics[kind],
                   f'floor2 548/981 {kind}; finite plane is a training factor in structure_joint')
    fig,axes=plt.subplots(1,2,figsize=(12,5))
    for ax,kind in zip(axes,('horizontal','wall')):
        labels=list(transforms);values=[100*metrics[kind][m]['symmetric_plane_abs_median_m'] for m in labels]
        ax.bar(labels,values,color=['#657b95','#999999','#4393c3','#de8f45','#47a76a'])
        ax.set(title='Floor-normal separation' if kind=='horizontal' else 'Wall-normal separation',
               ylabel='Same finite-patch distance (cm)');ax.tick_params(axis='x',rotation=30);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Complementary measured directions; not interchangeable XYZ coordinates\nRepeated development observations; structure candidate uses these planes for fitting')
    fig.tight_layout();fig.savefig(a.output/'directional_comparison.png',dpi=180);plt.close(fig)
    result={'created_utc':utc_now(),'source_sha256':sha256_file(Path(__file__)),
            'database_unchanged':sha256_file(database)==before,'metrics':metrics,
            'source_pose_hashes':{m:sha256_file(path) for m,path in paths.items()},
            'structure_report_sha256':sha256_file(a.structure_report),
            'physical_interpretation':'Floor patch constrains floor-normal distance and tilt; wall patch constrains wall-normal distance and tilt. Their intersection tangent needs feature/line observations. World XYZ from different optimizers are never spliced.',
            'independent_ground_truth':False,'structure_planes_used_for_training':True,
            'figures':{p.name:sha256_file(p) for p in a.output.glob('*.png')}}
    (a.output/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:{m:round(100*v['symmetric_plane_abs_median_m'],3) for m,v in vv.items()} for k,vv in metrics.items()}))


if __name__=='__main__':main()
