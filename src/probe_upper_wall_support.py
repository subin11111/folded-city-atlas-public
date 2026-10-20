#!/usr/bin/env python3
"""Read-only original depth support diagnostic for photo-bounded wall regions."""
import argparse,json,sqlite3
from pathlib import Path
import cv2
import numpy as np
from surface_model_utils import decode_frame,unproject
from export_rtabmap_db import load_pose_csv
from optimize_session import apply

ROOT=Path(__file__).resolve().parents[1]


def polygon_points(frame,polygon):
    mask=np.zeros(frame['rgb'].shape[:2],np.uint8);cv2.fillPoly(mask,[np.asarray(polygon,np.int32)],1)
    mask=cv2.resize(mask,(frame['depth'].shape[1],frame['depth'].shape[0]),interpolation=cv2.INTER_NEAREST)>0
    world=apply(unproject(frame['depth'],frame['k_depth']).reshape(-1,3),frame['world_optical'])
    return world[(mask&frame['valid']).ravel()]


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/verified_upper_wall_completion.json');a=p.parse_args();cfg=json.loads(a.config.read_text())
    pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text())
    for region in cfg['wall_regions']+cfg['ceiling_regions']:
        name=region['session'];poses=load_pose_csv(mp.parent/selected['sessions'][name]['trajectory']);con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True)
        f=decode_frame(con,region['node'],selected['sessions'][name]['preprocessing'],poses[region['node']]);con.close();points=polygon_points(f,region['polygon']);camera=f['world_optical'][:3,3];v=points-camera
        report={'id':region['id'],'valid_depth_samples':len(points),'camera':camera.tolist()}
        if len(points):
            centre=points.mean(0);_,_,vt=np.linalg.svd(points-centre,full_matrices=False);normal=vt[-1];residual=abs((points-centre)@normal)
            theta=np.arctan2(v[:,1],v[:,0]);report.update(xyz_min=points.min(0).tolist(),xyz_max=points.max(0).tolist(),normal=normal.tolist(),plane_p90=float(np.percentile(residual,90)),azimuth_span_deg=float(np.rad2deg(theta.max()-theta.min())))
        print(json.dumps(report),flush=True)


if __name__=='__main__':main()
