#!/usr/bin/env python3
"""Same-photo handover diagnostic before/after the four-pixel photo feather."""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np
from PIL import Image

from blend_photographic_floors import grid_xy,prepare_frame,sample_floor
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from presentation_geometry import LevelPresentation
from refine_surface_display import srgb_to_linear

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());cfg=run['floor_material_config'];pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text());model=LevelPresentation(ROOT,run['presentation_config'])
    poses={s:load_pose_csv(mp.parent/selected['sessions'][s]['trajectory']) for s in ['lobby','floor2','bridge_12_sub']};reports={}
    for group in cfg['groups']:
        report=next(r['group_report'] for r in run['floor_material_reports'] if r['group']==group);domain=np.load(a.bundle/'floor_charts'/(group+'_photo_domain.npz'));labels=domain['labels'];h,w=labels.shape;xy=grid_xy(domain['low'],domain['extent'],w,h);hard=np.zeros((h*w,3),np.float32);valid=np.zeros(h*w,bool)
        for index,v in enumerate(report['selected_photos']):
            ids=np.flatnonzero((labels==index).ravel());session=v['session'];spec=selected['sessions'][session];database=selected['database_integrity'][session]['database']
            with sqlite3.connect(f'file:{database}?mode=ro',uri=True) as con:frame=prepare_frame(con,v['node'],spec['preprocessing'],poses[session][v['node']],session,model,group,cfg)
            rgb,score=sample_floor(frame,xy[ids],model,group,cfg);good=score>0;hard[ids[good]]=srgb_to_linear(rgb[good])*v['scalar_gain'];valid[ids[good]]=True
        hard=hard.reshape(h,w,3);valid=valid.reshape(h,w);soft=srgb_to_linear(np.asarray(Image.open(a.bundle/'floor_charts'/(group+'_photo_mosaic.png')).convert('RGB'))/255.);before=np.log(np.maximum(hard@np.array([.2126,.7152,.0722]),.004));after=np.log(np.maximum(soft@np.array([.2126,.7152,.0722]),.004));old=[];new=[]
        for axis in [0,1]:
            aa=(slice(None,-1),slice(None)) if axis==0 else (slice(None),slice(None,-1));bb=(slice(1,None),slice(None)) if axis==0 else (slice(None),slice(1,None))
            good=valid[aa]&valid[bb]&(labels[aa]!=labels[bb]);good&=(labels[aa]==np.roll(labels,1,axis)[aa])&(labels[bb]==np.roll(labels,-1,axis)[bb]);good&=(abs(before[aa]-np.roll(before,1,axis)[aa])<.10)&(abs(before[bb]-np.roll(before,-1,axis)[bb])<.10)
            jump=abs(before[aa]-before[bb]);good&=(jump>.035)&(jump<1.)&(before[aa]>np.log(.015))&(before[bb]>np.log(.015));old.extend(jump[good].tolist());new.extend(abs(after[aa]-after[bb])[good].tolist())
        reports[group]={'same_selected_boundary_pixel_pairs':len(old),'median_abs_log_luminance_jump_hard_handover':float(np.median(old)) if old else None,'median_abs_log_luminance_jump_after_feather':float(np.median(new)) if new else None,'p90_hard_handover':float(np.percentile(old,90)) if old else None,'p90_after_feather':float(np.percentile(new,90)) if new else None,'same_original_photos_and_scalar_gains':True};print('Actual photo boundary diagnostic: '+group,flush=True)
    output={'created_utc':utc_now(),'material_bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'groups':reports,'audit_source_sha256':sha256_file(Path(__file__)),'scope':'Same selected supported photo-label boundary pairs, excluding high within-photo gradients and tiny/very large steps. Isolates feather from a hard photo switch using the same photos/gains. Not independent quality validation, all floor boundaries, sensor accuracy or unknown-region completion.'};a.output.write_text(json.dumps(output,indent=2)+'\n')


if __name__=='__main__':main()
