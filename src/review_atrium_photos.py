#!/usr/bin/env python3
"""Original RGB views towards candidate double-height lobby cells."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from surface_model_utils import project
from optimize_session import sha256_file

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--audit',type=Path,default=ROOT/'outputs/student_union_topology_audit_v1');p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    grid=np.load(a.audit/'multilevel_rasters.npz');yy,xx=np.nonzero(grid['possible_double_height']);world=np.c_[grid['low'][0]+(xx+.5)*float(grid['cell']),grid['low'][1]+(yy+.5)*float(grid['cell']),np.full(len(xx),3.6)]
    world=world[::max(1,len(world)//600)];road=ROOT/'outputs/student_union_roadview_v1';nodes=json.loads((road/'roadview.json').read_text())['nodes'];font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',17);records=[]
    for name,lower in [('lobby',True),('floor2',True),('floor2',False)]:
        ranked=[]
        for node in nodes:
            if node['session']!=name:continue
            pose=np.asarray(node['world_optical']);zcam=pose[2,3]
            if lower and zcam>1.:continue
            if not lower and zcam<3.5:continue
            targets=world.copy();targets[:,2]=3.6 if lower else 1.2
            uv,z=project(targets,np.asarray(node['K']),pose);inside=(z>.5)&(z<8)&(uv[:,0]>20)&(uv[:,0]<700)&(uv[:,1]>20)&(uv[:,1]<940)
            score=inside.sum()/max(.5,np.median(z[inside])) if inside.any() else 0
            if score>0:ranked.append((score,node))
        ranked.sort(key=lambda x:-x[0]);chosen=[]
        for _,node in ranked:
            pos=np.asarray(node['world_optical'])[:3,3];direction=np.asarray(node['world_optical'])[:3,2]
            if all(np.linalg.norm(pos-np.asarray(v['world_optical'])[:3,3])>1. or direction@np.asarray(v['world_optical'])[:3,2]<.65 for v in chosen):chosen.append(node)
            if len(chosen)==16:break
        panels=[]
        for node in chosen:
            im=Image.open(road/node['photo']).convert('RGB');dest=a.output/f'{name}_{node["node"]:05d}.png';im.save(dest)
            im.thumbnail((350,460));panel=Image.new('RGB',(360,500),'white');panel.paste(im,((360-im.width)//2,35));ImageDraw.Draw(panel).text((8,5),f'{name}:{node["node"]} / Zcam {np.asarray(node["world_optical"])[2,3]:.2f}',font=font,fill='black');panels.append(panel)
            records.append({'session':name,'node':node['node'],'file':dest.name,'source_photo':node['photo'],'source_sha256':sha256_file(road/node['photo']),'world_optical':node['world_optical'],'K':node['K']})
        for start in range(0,len(panels),8):
            sheet=Image.new('RGB',(1440,1000),'white')
            for i,im in enumerate(panels[start:start+8]):sheet.paste(im,((i%4)*360,(i//4)*500))
            sheet.save(a.output/f'{name}_{"lower" if lower else "upper"}_{start//8}.png')
    (a.output/'photos.json').write_text(json.dumps(records,indent=2)+'\n')
    print('Original reviewed photo candidates: '+str(len(records)),flush=True)


if __name__=='__main__':main()
