#!/usr/bin/env python3
"""Save original photographs covering a specific mapped wall for visual annotation."""
import argparse,json,sqlite3
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
from export_rtabmap_db import load_pose_csv
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]

def main():
    p=argparse.ArgumentParser();p.add_argument('--session',default='floor2');p.add_argument('--index',type=int,default=1);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(exist_ok=False,parents=True)
    base=ROOT/'outputs/student_union_continuous_selected_v4';pack=np.load(base/a.session/f'mask_{a.index:03d}.npz')
    summary=json.loads((base/a.session/'summary.json').read_text())
    plane=next(v for v in summary['continuous_planes'] if np.allclose(v['centre'],pack['centre']) and v['shape']==list(pack['mask'].shape))
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text());s=selected['sessions'][a.session]
    poses=load_pose_csv(mp.parent/s['trajectory']);con=sqlite3.connect(f'file:{selected["database_integrity"][a.session]["database"]}?mode=ro',uri=True)
    panels=[]
    for node in sorted(plane['selected_photo_nodes']):
        f=decode_frame(con,node,s['preprocessing'],poses[node]);im=Image.fromarray(f['rgb']);im.save(a.output/f'{node:05d}.png')
        im.thumbnail((350,460));panel=Image.new('RGB',(360,490),'white');panel.paste(im,((360-im.width)//2,30));ImageDraw.Draw(panel).text((10,7),f'{a.session}:{node} / plane {a.index}',fill='black');panels.append(panel)
    for start in range(0,len(panels),8):
        sheet=Image.new('RGB',(1440,980),'white')
        for i,panel in enumerate(panels[start:start+8]):sheet.paste(panel,((i%4)*360,(i//4)*490))
        sheet.save(a.output/f'contact_{start//8}.jpg',quality=95)

if __name__=='__main__':main()
