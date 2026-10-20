#!/usr/bin/env python3
"""Measured-source roof footprint and actual same-area continuity diagnostics."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());audit=json.loads((a.bundle/'geometry_audit.json').read_text());roof=np.load(a.bundle/'roof_mask.npz');obs=json.loads(Path(run['roof_observations']).read_text());mask=roof['mask'];old=roof['observed'];h,w=mask.shape;low=roof['low'];cell=float(roof['cell']);extent=[low[0],low[0]+w*cell,low[1],low[1]+h*cell];fig,axes=plt.subplots(1,3,figsize=(15,5),constrained_layout=True);layer=np.full(mask.shape,np.nan);layer[old]=1;layer[mask&~old]=2;axes[0].imshow(layer,origin='lower',extent=extent,cmap='coolwarm',vmin=0,vmax=2);axes[0].set(title=f'Observed roof domain: {mask.sum()*cell**2:.2f} m²\nRed: {np.sum(mask&~old)*cell**2:.2f} m² bounded interior fill',xlabel='X (m)',ylabel='Y (m)');axes[0].set_aspect('equal');heights=[r['original_display_height_m'] for r in obs['records']];axes[1].hist(heights,bins=20,color='#6085a5');axes[1].axvline(float(roof['height']),color='#c74734',label='Existing reviewed joint anchor');axes[1].legend(fontsize=8);axes[1].set(title=f'{len(heights)} original RGB-D roof fits\nDisplay regularization, NOT pose correction',xlabel='Source-plane centre display Z (m)',ylabel='Frames');r=audit['fixed_upward_ray_diagnostic'];values=[r['before']['surface_hit_in_1p5m_fraction']*100,r['after']['surface_hit_in_1p5m_fraction']*100];axes[2].bar(['Before','After'],values,color=['#a87962','#438b67']);axes[2].set(ylim=(0,110),ylabel='Same selected-area ray hit (%)',title='Actual mesh coverage in selected roof domain\nNot building-wide or surveyed accuracy')
    for i,value in enumerate(values):axes[2].text(i,value+2,f'{value:.2f}%',ha='center')
    a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=160);plt.close(fig)


if __name__=='__main__':main()
