#!/usr/bin/env python3
"""Contact sheets of unchanged local roadview photographs, labeled by source node."""
import argparse,json,math
from pathlib import Path
from PIL import Image,ImageDraw


def main():
    p=argparse.ArgumentParser();p.add_argument('--roadpack',type=Path,default=Path('outputs/student_union_roadview_v1'));p.add_argument('--session',default='lobby');p.add_argument('--stride',type=int,default=6);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);nodes=[n for n in json.loads((a.roadpack/'roadview.json').read_text())['nodes'] if n['session']==a.session][::a.stride]
    for start in range(0,len(nodes),48):
        part=nodes[start:start+48];sheet=Image.new('RGB',(1440,260*math.ceil(len(part)/8)),'white');draw=ImageDraw.Draw(sheet)
        for i,n in enumerate(part):
            im=Image.open(a.roadpack/n['photo']).convert('RGB');im.thumbnail((180,240));x=(i%8)*180;y=(i//8)*260;sheet.paste(im,(x,y+20));draw.text((x+3,y+3),f'{a.session}:{n["node"]}',fill='black')
        sheet.save(a.output/f'contact_{start//48}.jpg',quality=95)


if __name__=='__main__':main()
