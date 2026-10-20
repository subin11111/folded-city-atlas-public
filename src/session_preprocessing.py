"""Strict RGB-D decoding and removable-pixel preprocessing, not surface synthesis."""
import sqlite3
import zlib
from pathlib import Path

import cv2
import numpy as np
from scipy.ndimage import median_filter, maximum_filter, minimum_filter
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from export_rtabmap_db import calibration_from_blob
from optimize_session import apply, geometric_score, inverse, trimmed_icp


def decode_confidence(blob,shape):
    raw=zlib.decompress(blob) if blob else b'';size=int(np.prod(shape))
    if len(raw) not in (size,size+12):raise ValueError(f'Invalid confidence encoding: {len(raw)} for {shape}')
    # The optional 12-byte trailer is metadata, never a reason to invent confidence=100.
    return np.frombuffer(raw[:size],np.uint8).reshape(shape)


def depth_mask(depth,confidence,profile):
    valid=np.isfinite(depth)&(depth>=.25)&(depth<=profile['max_depth'])&(confidence>=profile['confidence'])
    finite=np.where(np.isfinite(depth)&(depth>0),depth,0.)
    if profile.get('median_gate',0):
        center=median_filter(finite,size=3,mode='nearest')
        valid &= (np.abs(depth-center)<=profile['median_gate']+.015*depth)&(center>0)
    if profile.get('edge_gate',0):
        high=maximum_filter(finite,size=3,mode='nearest')
        low=minimum_filter(np.where(finite>0,finite,np.inf),size=3,mode='nearest')
        valid &= (high-low)<=profile['edge_gate']+.02*depth
    return valid


BASE={'name':'base','confidence':50,'max_depth':4.,'median_gate':0.,'edge_gate':0.}


class RGBD:
    def __init__(self,database):
        self.connection=sqlite3.connect(f'file:{Path(database).resolve()}?mode=ro',uri=True)
        self.cache={}

    def get(self,node,profile=BASE,stride=2,cache=False):
        key=(node,profile['name'],stride)
        if key in self.cache:return self.cache[key]
        row=self.connection.execute('SELECT image,depth,depth_confidence,calibration FROM Data WHERE id=?',(int(node),)).fetchone()
        color=cv2.imdecode(np.frombuffer(row[0],np.uint8),cv2.IMREAD_COLOR)
        packed=cv2.imdecode(np.frombuffer(row[1],np.uint8),cv2.IMREAD_UNCHANGED)
        if packed is None or packed.ndim!=3 or packed.shape[2]!=4:raise ValueError('Unexpected depth PNG')
        depth=np.ascontiguousarray(packed).view('<f4').reshape(packed.shape[:2]);h,w=depth.shape
        conf=decode_confidence(row[2],depth.shape);cw,ch,k,local=calibration_from_blob(row[3])
        baseline=depth_mask(depth,conf,BASE)[::stride,::stride]
        valid=depth_mask(depth,conf,profile)[::stride,::stride]
        v,u=np.mgrid[0:h:stride,0:w:stride];z=depth[::stride,::stride][valid].astype(float)
        points=np.c_[(u[valid]-k[0,2]*w/cw)*z/(k[0,0]*w/cw),
                     (v[valid]-k[1,2]*h/ch)*z/(k[1,1]*h/ch),z]
        points=apply(points,local).astype(np.float32)
        rgb=cv2.cvtColor(cv2.resize(color,(w,h),interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB)[::stride,::stride][valid]
        vox=np.floor(points/.06).astype(np.int64);_,ids=np.unique(vox,axis=0,return_index=True)
        held=(vox[ids]@np.array([73856093,19349663,83492791]))%5==0
        result={'points':points,'colors':rgb,'train':points[ids][~held],'test':points[ids][held],
                'baseline_count':int(baseline.sum()),'retained_count':len(points),
                'confidence_histogram':{str(x):int((conf==x).sum()) for x in np.unique(conf)}}
        if cache:self.cache[key]=result
        return result


def local_refinement(a_train,b_train,a_test,b_test,initial):
    """Existing trimmed point-to-point ICP, two directions and unchanged test voxels.

    This refines an existing local association, not image-based place discovery.
    Fixed test voxels are always from BASE, never the cleaned fitting set.
    """
    if min(len(a_train),len(b_train),len(a_test),len(b_test))<50:
        return {'passed':False,'reason':'insufficient_points'}
    fitted=trimmed_icp(a_train,b_train,initial,iterations=20)
    reverse=trimmed_icp(b_train,a_train,inverse(initial),iterations=20)
    # Existing function returns transformation and fit diagnostics.
    transform=fitted[0];back=reverse[0]
    before=geometric_score(a_test,b_test,initial,.20);after=geometric_score(a_test,b_test,transform,.20)
    cycle=transform@back;change=inverse(initial)@transform
    cycle_t=float(np.linalg.norm(cycle[:3,3]));cycle_r=float(np.degrees(Rotation.from_matrix(cycle[:3,:3]).magnitude()))
    dt=float(np.linalg.norm(change[:3,3]));dr=float(np.degrees(Rotation.from_matrix(change[:3,:3]).magnitude()))
    passed=(after['overlap']>=.30 and after['clipped_rmse_m']<=before['clipped_rmse_m']+.001
            and after['overlap']>=before['overlap']-.01 and cycle_t<.05 and cycle_r<2.
            and dt<.35 and dr<5. and (after['clipped_rmse_m']<before['clipped_rmse_m']-.001 or after['overlap']>before['overlap']+.02))
    return {'passed':bool(passed),'reason':'conditional_local_refinement' if passed else 'fixed_test_or_cycle_or_local_change_gate',
            'measurement':transform.tolist(),'before':before,'after':after,'cycle_translation_m':cycle_t,
            'cycle_rotation_deg':cycle_r,'change_m':dt,'change_deg':dr,
            'limitation':'Pose-initialized geometry refinement; same-sensor sample holdout is correlated, not a new place recognition proof.'}
