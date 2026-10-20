#!/usr/bin/env python3
"""Find original RGB views of remaining wall holes for explicit visual review."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy import ndimage
from PIL import Image,ImageDraw,ImageFont
from surface_model_utils import project
from optimize_session import sha256_file

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--source',type=Path,default=ROOT/'outputs/student_union_continuous_selected_v4');a=p.parse_args()
    a.output.mkdir(exist_ok=False,parents=True)
    road=ROOT/'outputs/student_union_roadview_v1';nodes=json.loads((road/'roadview.json').read_text())['nodes']
    records=[];font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',17)
    for name in ['floor2','lobby']:
        candidates=[]
        for path in sorted((a.source/name).glob('mask_*.npz')):
            m=np.load(path);holes=ndimage.binary_fill_holes(m['mask'])&~m['mask'];lab,num=ndimage.label(holes)
            for i in range(1,num+1):
                chosen=lab==i;area=chosen.sum()*.06**2
                if area<.12:continue
                xy=np.argwhere(chosen);uv=(xy[:,::-1]+.5)*.06+m['low'];points=m['centre']+uv@ m['basis'][:,:2].T
                centre=points.mean(0);normal=m['basis'][:,2]
                views=[]
                for node in nodes:
                    if node['session']!=name:continue
                    pose=np.asarray(node['world_optical']);pixels,z=project(points,np.asarray(node['K']),pose)
                    inside=(z>.3)&(z<5.)&(pixels[:,0]>12)&(pixels[:,0]<708)&(pixels[:,1]>12)&(pixels[:,1]<948)
                    direction=pose[:3,3]-centre;cosine=abs(direction@normal)/max(np.linalg.norm(direction),1e-9)
                    score=inside.mean()*cosine**2/max(np.linalg.norm(direction),.5)
                    if inside.mean()>.65 and cosine>.35:views.append((score,node))
                views.sort(key=lambda x:-x[0]);selected=[]
                for score,node in views:
                    if all(np.linalg.norm(np.asarray(node['position_map_m'])-np.asarray(v['position_map_m']))>.4 for v in selected):selected.append(node)
                    if len(selected)==2:break
                veto=float((m['free'][chosen]>=2).mean())
                candidates.append({'session':name,'plane_index':int(path.stem.split('_')[-1]),'hole':i,
                    'area_m2':float(area),'free_vote_fraction':veto,'world_centre':centre.tolist(),'views':selected})
        # Keep both low-depth unknown holes and positively observed openings.
        candidates=sorted(candidates,key=lambda r:(r['free_vote_fraction']>.6,-r['area_m2']))[:8]
        panels=[]
        for item in candidates:
            for node in item['views']:
                image=Image.open(road/node['photo']).convert('RGB');draw=ImageDraw.Draw(image)
                uv,z=project(np.asarray(item['world_centre'])[None],np.asarray(node['K']),np.asarray(node['world_optical']))
                x,y=uv[0];draw.ellipse((x-22,y-22,x+22,y+22),outline='red',width=5)
                dest=a.output/f'{name}_p{item["plane_index"]:03d}_h{item["hole"]}_n{node["node"]}.jpg';image.save(dest,quality=95)
                record={k:v for k,v in item.items() if k!='views'};record.update(node=node['node'],photo=dest.name,original_photo=str(road/node['photo']),original_photo_sha256=sha256_file(road/node['photo']))
                records.append(record);image.thumbnail((350,460));panel=Image.new('RGB',(360,510),'white');panel.paste(image,((360-image.width)//2,50))
                label=f'{name}:{node["node"]} p{item["plane_index"]} h{item["hole"]} | {item["area_m2"]:.2f}m² free {item["free_vote_fraction"]:.0%}'
                ImageDraw.Draw(panel).text((5,8),label,font=font,fill='black');panels.append(panel)
        for start in range(0,len(panels),8):
            sheet=Image.new('RGB',(1440,1020),'white')
            for j,panel in enumerate(panels[start:start+8]):sheet.paste(panel,((j%4)*360,(j//4)*510))
            sheet.save(a.output/f'{name}_review_{start//8}.jpg',quality=95)
    (a.output/'candidates.json').write_text(json.dumps(records,indent=2)+'\n')
    print('Saved candidate photo views: '+str(len(records)))


if __name__=='__main__':main()
