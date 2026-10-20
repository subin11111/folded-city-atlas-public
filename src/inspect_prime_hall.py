#!/usr/bin/env python3
"""Original-photo contact sheets for manual PRIME HALL source review."""
import argparse
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    photos=ROOT/'outputs/student_union_roadview_v1/floor2/photos'
    for group,(lo,hi) in enumerate([(740,810),(870,915),(190,240)]):
        files=[f for f in sorted(photos.glob('*.jpg')) if lo<=int(f.stem)<=hi]
        files=[files[i] for i in np.linspace(0,len(files)-1,min(12,len(files))).astype(int)]
        sheet=Image.new('RGB',(4*360,3*270),'#f4f4f4');draw=ImageDraw.Draw(sheet)
        for index,file in enumerate(files):
            im=Image.open(file).convert('RGB').crop((0,120,720,620)).resize((360,250));x=index%4*360;y=index//4*270;sheet.paste(im,(x,y+20));draw.text((x+5,y+4),f'Original floor2:{int(file.stem)}',fill='black')
        sheet.save(a.output/f'source_review_{group}.png')


if __name__=='__main__':main()
