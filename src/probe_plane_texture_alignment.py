#!/usr/bin/env python3
"""Run and save a single photo-alignment trial before rebuilding all charts."""
import argparse
import json
from pathlib import Path
import sqlite3
import numpy as np
from aligned_photo_charts import texture_plane
from export_rtabmap_db import load_pose_csv
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--session',default='floor2');p.add_argument('--index',type=int,default=1);a=p.parse_args()
    a.output.mkdir(exist_ok=False,parents=True)
    import open3d as o3d
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    s=selected['sessions'][a.session];run=json.loads((a.source/a.session/'summary.json').read_text())
    pack=np.load(a.source/a.session/f'mask_{a.index:03d}.npz');plane={k:pack[k] for k in ['centre','basis','low']};mask=pack['mask']
    report=next(v for v in run['continuous_planes'] if np.allclose(v['centre'],plane['centre']) and v['shape']==list(mask.shape))
    poses=load_pose_csv(mp.parent/s['trajectory']);con=sqlite3.connect(f'file:{selected["database_integrity"][a.session]["database"]}?mode=ro',uri=True)
    frames={n:decode_frame(con,n,s['preprocessing'],poses[n]) for n in report['selected_photo_nodes']};con.close()
    cloud=o3d.io.read_point_cloud(str(mp.parent/s['cloud']))
    path,stats=texture_plane(plane,mask,frames,report['selected_photo_nodes'],cloud,a.output,a.index)
    (a.output/'summary.json').write_text(json.dumps(stats,indent=2)+'\n')
    print(json.dumps({'texture':str(path),'coverage':stats['photo_coverage_on_surface_fraction'],
        'alignment_edges':len(stats['texture_alignment']['accepted_edges']),
        'max_offset_m':stats['texture_alignment']['maximum_offset_m']}),flush=True)


if __name__=='__main__':main()
