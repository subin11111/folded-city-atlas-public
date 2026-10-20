#!/usr/bin/env python3
"""Original-photo contact sheets around both levels of the main-stair restroom."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from surface_model_utils import project
from optimize_session import sha256_file

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    road=ROOT/'outputs/student_union_roadview_v1';nodes=json.loads((road/'roadview.json').read_text())['nodes'];font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',18);records=[]
    for label,target in [('restroom_lower',[-2.25,-9.6,.3]),('restroom_upper',[-3.5,-9.,4.5]),('atrium_wall',[-5.,-7.5,2.])]:
        ranked=[]
        for node in nodes:
            if node['session'] not in ['lobby','floor2']:continue
            pose=np.asarray(node['world_optical']);uv,z=project(np.asarray([target]),np.asarray(node['K']),pose);u,v=uv[0]
            if .5<z[0]<14 and -80<u<800 and -80<v<1040:
                score=((u-360)/720)**2+((v-420)/960)**2+z[0]*.06;ranked.append((score,node))
        ranked.sort(key=lambda x:x[0]);chosen=[]
        for score,node in ranked:
            pose=np.asarray(node['world_optical'])
            if all(np.linalg.norm(pose[:3,3]-np.asarray(v['world_optical'])[:3,3])>.65 or pose[:3,2]@np.asarray(v['world_optical'])[:3,2]<.85 for v in chosen):chosen.append(node)
            if len(chosen)==24:break
        panels=[]
        for node in chosen:
            im=Image.open(road/node['photo']).convert('RGB');im.thumbnail((300,400));panel=Image.new('RGB',(310,450),'white');panel.paste(im,((310-im.width)//2,43));pose=np.asarray(node['world_optical'])
            ImageDraw.Draw(panel).text((5,4),f'{node["session"]}:{node["node"]} Zcam {pose[2,3]:.2f}',font=font,fill='black');panels.append(panel)
            records.append({'label':label,'session':node['session'],'node':node['node'],'source_photo':node['photo'],'sha256':sha256_file(road/node['photo']),'world_optical':node['world_optical'],'K':node['K']})
        for start in range(0,len(panels),12):
            sheet=Image.new('RGB',(1240,1350),'white')
            for i,im in enumerate(panels[start:start+12]):sheet.paste(im,((i%4)*310,(i//4)*450))
            sheet.save(a.output/f'{label}_{start//12}.png')
    (a.output/'photos.json').write_text(json.dumps(records,indent=2)+'\n')


if __name__=='__main__':main()
