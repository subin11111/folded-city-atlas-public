#!/usr/bin/env python3
"""Scientific footprint comparison; open atrium/stairs are not low ceilings."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser();p.add_argument('--plan',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();q=np.load(a.plan);low=q['low'];cell=float(q['cell']);shape=q['body'].shape;extent=[low[0],low[0]+shape[1]*cell,low[1],low[1]+shape[0]*cell]
    fig,axes=plt.subplots(1,3,figsize=(16,6),constrained_layout=True)
    panels=[('Existing 2F floor cells / actual source geometry','actual_top'),('Closed service cavity / underside + edge skins','body'),('Upper ceiling / previous vs expanded','roof')]
    for ax,(title,key) in zip(axes,panels):
        ax.imshow(np.ma.masked_where(~q[key],np.ones(shape)),origin='lower',extent=extent,cmap='Blues',vmin=0,vmax=1.5)
        if key=='roof':ax.imshow(np.ma.masked_where(~q['old_roof'],np.ones(shape)),origin='lower',extent=extent,cmap='Greens',vmin=0,vmax=1.5)
        else:
            ax.imshow(np.ma.masked_where(~q['void'],np.ones(shape)),origin='lower',extent=extent,cmap='Oranges',vmin=0,vmax=1.5,alpha=.6)
            ax.imshow(np.ma.masked_where(~q['stair'],np.ones(shape)),origin='lower',extent=extent,cmap='Reds',vmin=0,vmax=1.5,alpha=.3)
        ax.set(title=title,xlabel='X (m)',ylabel='Y (m)',aspect='equal');ax.grid(alpha=.15)
    fig.suptitle('Floors and camera poses fixed. Orange: open atrium at 1F. Red: stair/free-space guards.\nAdded ceilings and service-cavity walls are non-walkable presentation surfaces, not navigation export.',fontsize=11)
    a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=160);plt.close(fig)


if __name__=='__main__':main()
