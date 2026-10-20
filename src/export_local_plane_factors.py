#!/usr/bin/env python3
"""Export finite same-surface hypotheses without tangential pose constraints."""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from evaluate_geometry_stage import fixed_patches
from evaluate_map_surfaces import Settings, patch_correspondence, pair_metrics
from export_rtabmap_db import calibration_from_blob, load_pose_csv
from optimize_session import Clouds, apply, checked_transform, inverse, sha256_file, utc_now
from refine_geometry_surfaces import ROOT,SPECS

# Original RGB + calibrated projection overlays inspected by the assistant.
REVIEWED={
    (548,981): 'Both finite patches project onto the same mint-green lower parapet beneath the dark handrail on the right of the shutter/exit-lamp landing. They do not project onto the end wall or grille. Same railing supports, corner and black floor skirting are visible.',
    (503,961): 'Both finite horizontal patches project onto the near tiled floor of the same level corridor, identifiable by the left exhibit/photo wall, exit lamp and right railing. No stair tread, threshold or level transition is present within either finite projected support. This is a local repeated floor patch, not an entire-floor constraint.',
}


def projected_points(connection,node,points):
    image_blob,calibration=connection.execute('SELECT image,calibration FROM Data WHERE id=?',(node,)).fetchone()
    rgb=cv2.cvtColor(cv2.imdecode(np.frombuffer(image_blob,np.uint8),cv2.IMREAD_COLOR),cv2.COLOR_BGR2RGB)
    width,height,k,local=calibration_from_blob(calibration)
    camera=apply(points,inverse(local))
    positive=camera[:,2]>.05
    pixels=camera[positive,:2]/camera[positive,2,None]
    pixels=pixels*np.diag(k)[:2]+k[:2,2]
    pixels*=np.array([rgb.shape[1]/width,rgb.shape[0]/height])
    visible=(pixels[:,0]>=0)&(pixels[:,0]<rgb.shape[1])&(pixels[:,1]>=0)&(pixels[:,1]<rgb.shape[0])
    return rgb,pixels[visible]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError('Use fresh evidence directory')
    args.output.mkdir(parents=True)
    name='floor2';filename,prior_root,_=SPECS[name]
    database=ROOT/'data/raw/rtabmap'/filename
    original_hash=sha256_file(database)
    connection=sqlite3.connect(f'file:{database}?mode=ro',uri=True)
    clouds=Clouds(connection,None)
    previous=ROOT/'outputs'/prior_root
    raw=load_pose_csv(previous/name/'raw_main_trajectory.csv')
    app=load_pose_csv(previous/'audit'/name/'app_optimized_trajectory.csv')
    records=[]
    for a,b,kind in [(548,981,'wall'),(503,961,'horizontal')]:
        pa=fixed_patches(clouds,raw,a)[kind,0]
        pb=fixed_patches(clouds,raw,b)[kind,0]
        stored=checked_transform(connection.execute('SELECT transform FROM Link WHERE from_id=? AND to_id=? AND type IN (1,2)',(a,b)).fetchone()[0])
        match=patch_correspondence(pa,pb,stored,Settings())
        fig,axes=plt.subplots(2,2,figsize=(10,12))
        for col,(node,patch) in enumerate([(a,pa),(b,pb)]):
            rgb,train=projected_points(connection,node,patch['training'])
            _,test=projected_points(connection,node,patch['testing'])
            axes[0,col].imshow(rgb);axes[0,col].set_title(f'Original frame {node}')
            axes[1,col].imshow(rgb)
            axes[1,col].scatter(train[:,0],train[:,1],s=5,c='#ffe600',alpha=.6,label='fixed fit samples')
            axes[1,col].scatter(test[:,0],test[:,1],s=7,c='#ff2458',alpha=.7,label='withheld depth samples')
            axes[1,col].set_title(f'{kind} patch 0: projection through stored calibration')
            axes[1,col].legend(fontsize=8)
            for row in range(2):axes[row,col].axis('off')
        fig.suptitle('Candidate same finite surface: projected support requires visual confirmation',fontsize=13)
        fig.tight_layout();fig.savefig(args.output/f'{a}_{b}_{kind}_patch_projection.png',dpi=150);plt.close(fig)
        record={'session':name,'session_a':name,'session_b':name,'node_a':a,'node_b':b,
                'kind':kind,'plane_a':0,'plane_b':0,'held_out':False,
                'accepted':bool(match is not None and (a,b) in REVIEWED),
                'accepted_for_experiment':bool(match is not None and (a,b) in REVIEWED),
                'final_map_approved':False,
                'reason':'manually_reviewed_same_finite_surface_with_stored_measurement_overlap' if match is not None else 'finite_patch_support_failed',
                'manual_visual_evidence':REVIEWED.get((a,b)),
                'normal_a':pa['normal'].tolist(),'normal_b':pb['normal'].tolist(),
                'center_a':pa['center'].tolist(),'center_b':pb['center'].tolist(),
                'training_points_a':len(pa['training']),'training_points_b':len(pb['training']),
                'test_points_a':len(pa['testing']),'test_points_b':len(pb['testing']),
                'patch_validation_a':pa['diagnostic'],'patch_validation_b':pb['diagnostic'],
                'finite_support_under_stored_measurement':match,
                'baseline_patch_metrics':pair_metrics(pa,pb,app[a],app[b]),
                'observability':{'constrained':'Two normal-angle DOF plus plane-normal displacement',
                                 'unconstrained':'Two tangential translations and rotation about plane normal',
                                 'maximum_constraint_rank':3,'not_full_se3_measurement':True},
                'suggested_model':{'normal_displacement_sigma_m':.08,'normal_angle_sigma_degrees':3.,
                                   'loss':'block Huber; model scales, not calibrated covariance'},
                'source_image':str((args.output/f'{a}_{b}_{kind}_patch_projection.png').resolve()),
                'T_a_b_stored':stored.tolist()}
        sign=1. if pa['normal']@(stored[:3,:3]@pb['normal'])>=0 else -1.
        record['normal_b']=(sign*pb['normal']).tolist()
        record['normal_b_sign_aligned_to_a_under_stored_measurement']=sign
        np.savez_compressed(args.output/f'{a}_{b}_{kind}_patches.npz',
                            train_a=pa['training'],train_b=pb['training'],test_a=pa['testing'],test_b=pb['testing'])
        records.append(record)
    connection.close()
    if sha256_file(database)!=original_hash:raise RuntimeError('Source DB changed')
    result={'created_utc':utc_now(),'source_sha256':sha256_file(Path(__file__)),
            'database_sha256':original_hash,'original_database_unchanged':True,'plane_factors':records,
            'warning':'Historical native-training loop places, not independent graph holdouts; no infinite global floor equation.'}
    (args.output/'plane_candidates.json').write_text(json.dumps(result,indent=2)+'\n')
    (args.output/'constraints.json').write_text(json.dumps({**result,'constraints':records},indent=2)+'\n')


if __name__=='__main__':main()
