#!/usr/bin/env python3
"""Probe actual photographed ceiling / bank sign support before local completion."""
import argparse,json,sqlite3
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from build_unified_presentation import ceiling_fit
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file
from probe_upper_wall_support import polygon_points
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/verified_service_slabs.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True);cfg=json.loads(a.config.read_text());ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];run=json.loads(mp.read_text());rows=[]
    for region in cfg['ceiling_regions']+cfg['bank_objects']:
        name=region['session'];s=run['sessions'][name];poses=load_pose_csv(mp.parent/s['trajectory']);profile={**s['preprocessing'],'max_depth':5.5};db=run['database_integrity'][name]['database']
        with sqlite3.connect(f'file:{db}?mode=ro',uri=True) as con:f=decode_frame(con,region['node'],profile,poses[region['node']])
        points=polygon_points(f,region['polygon']);plane,fit=ceiling_fit(points) if region.get('kind')=='ceiling' else (None,{})
        row={**region,'source_depth_points':len(points),'fit':fit};rows.append(row);print(json.dumps(row),flush=True)
        np.savez_compressed(a.output/(region['id']+'.npz'),points=points,world_optical=f['world_optical'],k_rgb=f['k_rgb'])
        im=Image.fromarray(f['rgb']);poly=[tuple(p) for p in region['polygon']];ImageDraw.Draw(im).line(poly+[poly[0]],fill='cyan',width=3);im.save(a.output/(region['id']+'.png'))
    (a.output/'report.json').write_text(json.dumps({'config_sha256':sha256_file(a.config),'regions':rows},indent=2)+'\n')


if __name__=='__main__':main()
