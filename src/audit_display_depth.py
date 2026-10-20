#!/usr/bin/env python3
"""Separate camera-pose accuracy from display geometry and changing hit masks."""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np

from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def depth_image(o3d,mesh,frame):
    scene=o3d.t.geometry.RaycastingScene(nthreads=2)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    d=frame['depth'];k=frame['k_depth'];y,x=np.indices(d.shape)
    directions=np.stack([(x-k[0,2])/k[0,0],(y-k[1,2])/k[1,1],np.ones_like(x)],axis=-1)
    directions=directions@frame['world_optical'][:3,:3].T
    origins=np.broadcast_to(frame['world_optical'][:3,3],directions.shape)
    rays=np.concatenate([origins,directions],axis=-1).astype(np.float32)
    return scene.cast_rays(o3d.core.Tensor(rays))['t_hit'].numpy()


def residual_stats(values):
    values=np.asarray(values)
    if not len(values):return {'pixels':0,'median_m':None,'p90_m':None}
    return {'pixels':len(values),'median_m':float(np.median(abs(values))),
        'p90_m':float(np.percentile(abs(values),90)),'signed_median_m':float(np.median(values))}


def compare_depths(before,after,observed):
    valid=np.isfinite(observed)&(observed>.25)
    old=valid&np.isfinite(before)&(before>.25);new=valid&np.isfinite(after)&(after>.25)
    common=old&new;added=new&~old;lost=old&~new
    improvement=abs(after[common]-observed[common])-abs(before[common]-observed[common])
    return {'valid_pixels':int(valid.sum()),'old_all_hits':residual_stats(before[old]-observed[old]),
        'new_all_hits':residual_stats(after[new]-observed[new]),
        'same_pixels_before':residual_stats(before[common]-observed[common]),
        'same_pixels_after':residual_stats(after[common]-observed[common]),
        'newly_hit':residual_stats(after[added]-observed[added]),
        'lost_hits':residual_stats(before[lost]-observed[lost]),
        'display_depth_change_on_same_pixels':residual_stats(after[common]-before[common]),
        'paired_worse_by_over_3cm_fraction':float(np.mean(improvement>.03)) if len(improvement) else None,
        'paired_better_by_over_3cm_fraction':float(np.mean(improvement<-.03)) if len(improvement) else None,
        'scope':'First-hit display/sensor disagreement, NOT pose error; common-pixel masks explicitly matched.'}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--repair',type=Path,default=ROOT/'outputs/student_union_surface_continuity_v1')
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    import open3d as o3d
    pointer_path=ROOT/'config/current_map.json';pointer=json.loads(pointer_path.read_text());mp=ROOT/pointer['manifest']
    selected=json.loads(mp.read_text());repair=json.loads((a.repair/'manifest.json').read_text())
    hashes={str(path):sha256_file(path) for path in [pointer_path,mp,Path(__file__)]};sessions={}
    for n,record in selected['sessions'].items():
        paths=[ROOT/'outputs/student_union_surface_portfolio_v1'/n/'tsdf_3cm_display.ply',a.repair/n/'supported_poisson.ply',
            mp.parent/record['trajectory'],Path(selected['database_integrity'][n]['database'])]
        for path in paths:hashes[str(path)]=sha256_file(path)
        poses=load_pose_csv(paths[2]);connection=sqlite3.connect(f'file:{paths[3]}?mode=ro',uri=True)
        meshes=[o3d.io.read_triangle_mesh(str(path)) for path in paths[:2]]
        sessions[n]={}
        for label,node in repair['sessions'][n]['camera_nodes'].items():
            frame=decode_frame(connection,node,record['preprocessing'],poses[node])
            depths=[depth_image(o3d,m,frame) for m in meshes]
            sessions[n][label]={'node':node,**compare_depths(*depths,frame['depth'])}
        connection.close();print(json.dumps({n:sessions[n]['audit']}),flush=True)
    for path,h in hashes.items():
        if sha256_file(Path(path))!=h:raise ValueError('Frozen input changed')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps({'created_utc':utc_now(),'sessions':sessions,'input_hashes':hashes,
        'all_inputs_unchanged':True,'camera_pose_changes':0,'pose_accuracy_measured':False},indent=2)+'\n')


if __name__=='__main__':main()
