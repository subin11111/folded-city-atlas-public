"""Registration sensitivity only. Does not replace the map or calibrate indoor fixes."""
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'docs/assets/gnss-slam-audit-20261020'
v=json.loads((OUT/'comparison.json').read_text(encoding='utf-8'))
g=v['georeference'];R0=np.asarray(g['R_map_xy_to_EN']);t0=np.asarray(g['t_map_xy_to_EN_m']);o=np.asarray(v['manual_preview_offset_en_m'])
rows=[r for r in v['sessions']['outside']['records'] if r['reference']]
X=np.array([r['reference'][:2] for r in rows]);Y=np.array([r['raw'] for r in rows])@R0.T+t0+o
accuracy=np.array([r['reported_accuracy'] for r in rows])


def fit(x,y,a):
    base=1/np.maximum(a,3)**2;w=base.copy()
    for _ in range(20):
        w=w/w.sum();cx=(x*w[:,None]).sum(0);cy=(y*w[:,None]).sum(0)
        u,_,vt=np.linalg.svd((x-cx).T@(w[:,None]*(y-cy)))
        R=vt.T@np.diag([1,np.linalg.det(vt.T@u.T)])@u.T;t=cy-R@cx
        residual=np.linalg.norm(x@R.T+t-y,axis=1);w=base*np.minimum(1,5/np.maximum(residual,1e-9))
    return R,t


def rmse(a):return float(np.sqrt(np.mean(a*a)))
result={'applied_to_map':False,'indoor_fixes_used_for_fit':False,'independent_control_points':False,
        'source':'337 timestamp-matched exterior fixes and reconstructed web poses',
        'exterior_slam_span_xy_m':np.ptp(X,axis=0).tolist(),'existing_yaw_deg':g['yaw_deg'],'fits':[]}
splits=[('all_exterior_in_sample',np.arange(len(X))),('first_temporal_half',np.arange(len(X)//2)),('second_temporal_half',np.arange(len(X)//2,len(X)))]
for name,idx in splits:
    r,t=fit(X[idx],Y[idx],accuracy[idx]);res=np.linalg.norm(X@r.T+t-Y,axis=1);rest=np.setdiff1d(np.arange(len(X)),idx)
    item={'name':name,'training_count':len(idx),'yaw_deg':float(np.rad2deg(np.arctan2(r[1,0],r[0,0]))),
          'translation_en_m':t.tolist(),'rotation':r.tolist(),'scale':1.,'training_rmse_m':rmse(res[idx]),
          'other_temporal_half_rmse_m':rmse(res[rest]) if len(rest) else None,'session_discrepancy_rmse_m':{}}
    for session,s in v['sessions'].items():
        rows=[a for a in s['records'] if a['reference']]
        x=np.array([a['reference'][:2] for a in rows]);y=np.array([a['raw'] for a in rows])@R0.T+t0+o
        item['session_discrepancy_rmse_m'][session]=rmse(np.linalg.norm(x@r.T+t-y,axis=1))
    result['fits'].append(item)
result['temporal_half_yaw_difference_deg']=abs(result['fits'][1]['yaw_deg']-result['fits'][2]['yaw_deg'])
(OUT/'registration-sensitivity.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
