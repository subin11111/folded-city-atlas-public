"""Reanalyse existing evidence; no new survey or participant data is invented."""
from pathlib import Path
import collections
import hashlib
import json
import sys

import argparse
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo-root',type=Path,default=Path(__file__).resolve().parents[1])
parser.add_argument('--font-family',default='Malgun Gothic',help='A font with Korean glyphs, e.g. NanumGothic')
args=parser.parse_args()
R=args.repo_root.resolve()
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT=R/'docs/assets/critical-review-20261021'
OUT.mkdir(parents=True,exist_ok=True)
plt.rcParams.update({'font.family':args.font_family,'axes.unicode_minus':False,'font.size':12})
selected={'outside':'J01','lobby':'N05','bridge_out1f':'H03','floor2':'J04','bridge_12_sub':'H03'}
labels=dict(zip(selected,['실외','1층 로비','실외 연결','메인 계단·2층','서브계단']))
result={'scope':'Existing development-patch consistency, not independent survey accuracy or full RAW/PGO/BA ablation',
        'sessions':{},'pooled':{},'input_sha256':{}}
pooled={k:[] for k in ['O00','P00','selected']}
def index(items):
    counts=collections.Counter();out={}
    for p in items:
        base=(p['a'],p['b'],p['kind']);i=counts[base];counts[base]+=1
        out[(*base,i)]=p
    return out
def metrics(values):
    return {'n':len(values),**({'median_cm':float(np.median(values)*100),'p90_cm':float(np.percentile(values,90)*100),
        'max_cm':float(max(values)*100)} if len(values) else {})}
for name,candidate in selected.items():
    path=R/f'docs/assets/session-portfolios/{name}/summary.json'
    data=json.loads(path.read_text('utf-8'))
    result['input_sha256'][str(path.relative_to(R))]=hashlib.sha256(path.read_bytes()).hexdigest()
    sources={k:index(data['evaluation'][v]['patches']) for k,v in [('O00','O00'),('P00','P00'),('selected',candidate)]}
    common=set.intersection(*(set(v) for v in sources.values()))
    assert all(set(v)==common for v in sources.values()),'Changed patch coverage must not be hidden'
    keys=sorted(common)
    rows=[]
    for key in keys:
        row={'session':name,'pair':[key[0],key[1]],'kind':key[2],'patch_index':key[3]}
        for method in sources:
            row[method]=sources[method][key]['value_m']
            pooled[method].append(row)
        rows.append(row)
    result['sessions'][name]={'label':labels[name],'candidate':candidate,
        'candidate_entries':len(data['stages']),
        'unique_trajectory_hashes':len({s.get('trajectory_sha256') for s in data['stages']}),
        'matched_patches':len(rows),
        'metrics':{k:metrics([v[k] for v in rows]) for k in sources},
        'improved_count':sum(v['selected']<v['O00']-1e-10 for v in rows),
        'worsened_count':sum(v['selected']>v['O00']+1e-10 for v in rows),'patches':rows}
for method,rows in pooled.items():result['pooled'][method]=metrics([r[method] for r in rows])
(OUT/'surface-aggregate.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
fig,axes=plt.subplots(1,2,figsize=(11.8,3.6),layout='constrained')
for ax,kind,title in zip(axes,['wall','horizontal'],['벽 패치','수평 패치']):
    for method,label,col,ls in [('O00','앱 저장 자세','#80868E','-'),('P00','깊이 전처리만','#A2CC80','--'),('selected','선택 후처리','#17804A','-')]:
        a=np.sort([r[method]*100 for r in pooled[method] if r['kind']==kind])
        if len(a):ax.step(a,np.arange(1,len(a)+1)/len(a),where='post',label=f'{label} (n={len(a)})',color=col,linestyle=ls,lw=2)
    ax.set(title=title,xlabel='관측면 사이 거리 (cm)',ylabel='누적 비율',ylim=(0,1.03))
    ax.grid(alpha=.2);ax.legend(fontsize=9)
fig.savefig(OUT/'surface-distribution.png',dpi=240);plt.close(fig)

g=json.loads((OUT/'gnss-settings-replay.json').read_text('utf-8'))
fig,axes=plt.subplots(1,2,figsize=(12,3.5),layout='constrained')
names=list(selected);x=np.arange(len(names));width=.26
for i,(key,label,col) in enumerate([('relative_rmse_m','지도와 원 수신 차이','#828993'),('bias_norm_m','평균 차이 벡터 크기','#A2CC80'),('mean_removed_rmse_m','평균 제거 후 잔차','#17804A')]):
    y=[g['sessions'][n]['unspecified_radius']['metrics']['raw'][key] for n in names]
    bars=axes[0].bar(x+(i-1)*width,y,width,label=label,color=col)
    axes[0].bar_label(bars,fmt='%.1f',fontsize=8,padding=2)
axes[0].set(xticks=x,xticklabels=[labels[n] for n in names],ylabel='수평 차이 (m)',title='고정 편향과 나머지 변동을 분리')
axes[0].tick_params(axis='x',labelsize=9);axes[0].legend(fontsize=8)
outside=g['sessions']['outside']['unspecified_radius']['metrics']
axes[1].bar(range(3),[outside[k]['relative_rmse_m'] for k in ['raw','kalman','rts']],color=['#828993','#A2CC80','#17804A'])
axes[1].set(xticks=range(3),xticklabels=['원 수신','실시간 칼만','RTS 후보정'],ylabel='동시각 지도와 차이 RMSE (m)',title='실외 공통 337시각의 동일 입력 비교',ylim=(0,6))
for i,k in enumerate(['raw','kalman','rts']):axes[1].text(i,outside[k]['relative_rmse_m']+.08,f"{outside[k]['relative_rmse_m']:.2f}",ha='center')
fig.savefig(OUT/'gnss-bias-outdoor.png',dpi=240);plt.close(fig)

# Rigid translation changes the mean vector, not the centered residual.
source=json.loads((R/'docs/assets/gnss-slam-audit-20261020/comparison.json').read_text('utf-8'))
rotation=np.asarray(source['georeference']['R_map_xy_to_EN'])
offset=np.asarray(source['manual_preview_offset_en_m'])
translation=rotation.T@offset
invariance={}
for n,s in source['sessions'].items():
    e=np.asarray([np.asarray(p['raw'])-np.asarray(p['reference'])[:2] for p in s['records'] if p['reference'] is not None])
    a=e-e.mean(axis=0);b=e+translation-(e+translation).mean(axis=0)
    invariance[n]={'mean_delta_map_xy_m':e.mean(axis=0).tolist(),
        'mean_removed_rmse_m':float(np.sqrt(np.mean(np.sum(a*a,axis=1)))),
        'centered_change_after_removing_preview_offset_max_m':float(np.max(np.abs(a-b)))}
(OUT/'offset-mean-separation.json').write_text(json.dumps({'translation_map_xy_m':translation.tolist(),
    'not_applied_to_operational_map':True,'sessions':invariance},ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'surface':{n:{k:v for k,v in s.items() if k not in ['patches','metrics']} for n,s in result['sessions'].items()},
    'pooled':result['pooled'],'offset_invariance_max_m':max(s['centered_change_after_removing_preview_offset_max_m'] for s in invariance.values())},ensure_ascii=False))
