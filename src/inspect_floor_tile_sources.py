#!/usr/bin/env python3
"""Original floor photograph coordinate grid and dark tile component review."""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);reports=[]
    for node in [720,892]:
        path=ROOT/'outputs/student_union_roadview_v1/floor2/photos'/f'{node:05d}.jpg';rgb=np.asarray(Image.open(path).convert('RGB'));im=Image.fromarray(rgb);draw=ImageDraw.Draw(im)
        for x in range(0,720,50):draw.line([(x,0),(x,959)],fill='#40c0ff',width=1);draw.text((x+2,480),str(x),fill='cyan')
        for y in range(500,960,50):draw.line([(0,y),(719,y)],fill='#40c0ff',width=1);draw.text((2,y+2),str(y),fill='cyan')
        gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);mask=((gray<65)&(np.indices(gray.shape)[0]>500)).astype(np.uint8);count,labels,stats,centres=cv2.connectedComponentsWithStats(mask)
        components=[]
        for i in range(1,count):
            x,y,w,h,area=stats[i]
            if 50<area<4000 and w>7 and h>5 and .15<w/h<6:
                components.append({'centre':centres[i].tolist(),'bbox':[int(x),int(y),int(w),int(h)],'area':int(area)});draw.rectangle((x,y,x+w,y+h),outline='red',width=2);draw.text((x,y-12),str(len(components)),fill='red')
        im.save(a.output/f'floor_{node}_coordinates.png');reports.append({'node':node,'dark_components':components})
    (a.output/'components.json').write_text(json.dumps(reports,indent=2)+'\n')


if __name__=='__main__':main()
