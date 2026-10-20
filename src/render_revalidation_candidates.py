#!/usr/bin/env python3
"""Render experimental alternatives without silently replacing a canonical map."""
from pathlib import Path
import json
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from render_session_report import read_ply, trajectory


def main():
    root=Path('outputs/student_union_20261017_revalidated')
    session=root/'intra_session/bridge_12_sub'
    previous=Path('outputs/student_union_20261017_added_reviewed/bridge_12_sub')
    variants=[('Previous independent 4DoF',previous/'cloud_optimized_filtered.ply',previous/'optimized_main_trajectory.csv'),
              ('Reverified loop / 4DoF',session/'candidate_nominal/cloud.ply',session/'candidate_nominal/main_trajectory.csv'),
              ('Bounded SE3 / attitude sigma 5deg',session/'candidate_attitude_5deg/cloud.ply',session/'candidate_attitude_5deg/main_trajectory.csv')]
    samples=[]
    for _,path,_ in variants:
        cloud=read_ply(path)
        samples.append(cloud[np.linspace(0,len(cloud)-1,min(160000,len(cloud))).astype(int)])
    bounds={k:(min(np.percentile(c[k],.5) for c in samples),max(np.percentile(c[k],99.5) for c in samples)) for k in ('x','y','z')}
    fig,axes=plt.subplots(2,3,figsize=(16,9),constrained_layout=True)
    for col,(sample,(label,_,_)) in enumerate(zip(samples,variants)):
        for row,(x,y) in enumerate([('x','y'),('x','z')]):
            ax=axes[row,col]
            ax.scatter(sample[x],sample[y],c=sample['z'],cmap='viridis',s=.12,vmin=bounds['z'][0],vmax=bounds['z'][1])
            ax.set(xlabel=x+' (m)',ylabel=y+' (m)',xlim=bounds[x],ylim=bounds[y],title=label)
            ax.set_aspect('equal',adjustable='box')
    fig.suptitle('Secondary-stair candidates | Same 211 frames / 5cm voxel | NOT independently verified')
    fig.savefig(session/'revalidation_cloud_comparison.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4),constrained_layout=True)
    for label,_,path in variants:
        stamp,xyz=trajectory(path)
        axes[0].plot(xyz[:,0],xyz[:,1],lw=1,label=label)
        axes[1].plot(stamp-stamp[0],xyz[:,2],lw=1,label=label)
    axes[0].set(xlabel='X (m)',ylabel='Y (m)');axes[0].set_aspect('equal',adjustable='datalim')
    axes[1].set(xlabel='Elapsed time (s)',ylabel='Camera Z (m), not surveyed floor height')
    axes[1].legend(fontsize=8)
    for ax in axes:ax.grid(alpha=.25)
    fig.savefig(session/'revalidation_trajectory_comparison.png',dpi=160);plt.close(fig)
    pairdir=root/'cross_session_planar_interior/lobby__floor2'
    edges=json.loads((pairdir/'accepted_frame_edges.json').read_text())
    tiles=[]
    for e in edges:
        a,b=e['node_a'],e['node_b']
        path=pairdir/'matches'/f'{a:04d}_{b:04d}.jpg'
        frame=cv2.imread(str(path))
        if frame is None:raise ValueError(f'Missing pair visualization: {path}')
        tile=np.zeros((260,360,3),np.uint8)
        tile[:240]=cv2.resize(frame,(360,240))
        cv2.putText(tile,f'lobby {a} / floor2 {b}',(6,254),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
        tiles.append(tile)
    while len(tiles)%2:tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(str(pairdir/'accepted_pair_review.jpg'),np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles),2)]))
    print(session/'revalidation_cloud_comparison.png')
    print(pairdir/'accepted_pair_review.jpg')


if __name__=='__main__':main()
