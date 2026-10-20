#!/usr/bin/env python3
"""Scientific XY footprint evidence for bounded ceiling/top-floor/side volumes."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());fig,axes=plt.subplots(1,2,figsize=(12,5),constrained_layout=True)
    for ax,slab in zip(axes,run['sessions']['lobby']['service_slabs']):
        d=np.load(a.bundle/'lobby'/slab['mask_file']);mask=d['mask'];photo=d['photographed'];low=d['low'];cell=float(d['cell']);h,w=mask.shape;extent=[low[0],low[0]+w*cell,low[1],low[1]+h*cell];y,x=np.where(photo|mask);bounds=(low+np.c_[x,y]*cell);lo=bounds.min(0)-.6;hi=bounds.max(0)+.6;grey=np.ma.masked_where(~d['allowed'],np.ones_like(mask));ax.imshow(grey,origin='lower',extent=extent,cmap='Greys',vmin=0,vmax=4,alpha=.5);ax.contour(low[0]+(np.arange(w)+.5)*cell,low[1]+(np.arange(h)+.5)*cell,photo,[.5],colors=['#ee9b00'],linewidths=1.2);ax.imshow(np.ma.masked_where(~mask,np.ones_like(mask)),origin='lower',extent=extent,cmap='Blues',vmin=0,vmax=1.5,alpha=.85);ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='X (m)',ylabel='Y (m)',title=f'{slab["id"]}: {slab["area_m2"]:.2f} m² / shared closed footprint');ax.set_aspect('equal');ax.grid(alpha=.2)
    fig.suptitle('Gray: existing complete upper-floor support | Orange: photographed ceiling extent | Blue: closed slab\nNo outward footprint dilation. Stair/atrium guards retained. Model dimensions, not surveyed slabs.',fontsize=10);a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=160);plt.close(fig)


if __name__=='__main__':main()
