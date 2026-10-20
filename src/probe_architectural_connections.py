#!/usr/bin/env python3
"""Original RGB-D joint probes for photographed architectural presentation surfaces."""
import argparse,json,sqlite3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from complete_verified_upper_walls import robust_plane
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file
from photo_semantic_surfaces import lift_pixels
from probe_upper_wall_support import polygon_points
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'config/architectural_connections.json');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    cfg=json.loads(a.config.read_text());pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text());rows=[]
    for region in cfg['regions']:
        name=region['session'];poses=load_pose_csv(mp.parent/selected['sessions'][name]['trajectory']);db=Path(selected['database_integrity'][name]['database']);con=sqlite3.connect(f'file:{db}?mode=ro',uri=True);profile=dict(selected['sessions'][name]['preprocessing']);profile['max_depth']=cfg['ceiling_depth_limit_m'];frame=decode_frame(con,region['node'],profile,poses[region['node']]);con.close();pts=polygon_points(frame,region['polygon'])
        plane,fit=robust_plane(pts,region['kind']=='ceiling') if len(pts)>=30 else (None,{'accepted':False,'support_points':len(pts)})
        row={**region,'depth_points':len(pts),'fit':fit}
        if plane is not None and region.get('front_edge'):
            xyz,t=lift_pixels(np.asarray(region['front_edge']),frame['k_rgb'],frame['world_optical'],*plane);row.update(edge_xyz=xyz.tolist(),edge_depth=t.tolist())
        np.savez_compressed(a.output/(region['id']+'_points.npz'),points=pts,world_optical=frame['world_optical'],k_rgb=frame['k_rgb'])
        im=Image.fromarray(frame['rgb']);draw=ImageDraw.Draw(im);poly=[tuple(v) for v in region['polygon']];draw.line(poly+[poly[0]],fill='cyan',width=3);im.save(a.output/(region['id']+'.png'));rows.append(row)
        print(json.dumps(row),flush=True)
    (a.output/'report.json').write_text(json.dumps({'regions':rows,'config_sha256':sha256_file(a.config),'pose_changes':0},indent=2)+'\n')


if __name__=='__main__':main()
