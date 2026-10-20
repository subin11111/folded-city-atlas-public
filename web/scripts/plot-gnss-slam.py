"""Scientific figures from recorded measurements and the published map only."""
import os
import sys
from pathlib import Path
if os.environ.get('GNSS_PLOT_DEPS'): sys.path.insert(0, os.environ['GNSS_PLOT_DEPS'])
import base64
import json
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'docs/assets/gnss-slam-audit-20261020'
DATA=json.loads((OUT/'comparison.json').read_text(encoding='utf-8'))
NAV=json.loads((ROOT/'web/public/data/navigation.json').read_text(encoding='utf-8'))
plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False,'font.size':10,
    'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'white','savefig.facecolor':'white'})
C={'reference':'#26372f','raw':'#c36b3d','kalman':'#2876b5','rts':'#6f4f94'}


def background(ax,level=None):
    for k,layer in NAV['levels'].items():
        if level and k!=level: continue
        for ring in layer['floor_rings']: ax.add_patch(Polygon(ring,facecolor='#eef1ec',edgecolor='#b3bcb1',lw=.5,zorder=0))
        for ring in layer['obstacle_rings']: ax.add_patch(Polygon(ring,facecolor='#d0d5cf',edgecolor='none',alpha=.6,zorder=1))
    ax.set_aspect('equal',adjustable='box');ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.grid(alpha=.17)


def save(fig,name):
    fig.savefig(OUT/name,dpi=180,bbox_inches='tight');plt.close(fig)


def plan(ax,s):
    background(ax,'2' if s['label'].startswith('2층') else '1')
    ref=np.array([r['position'] for r in s['reference_trajectory']]);ax.plot(ref[:,0],ref[:,1],color=C['reference'],lw=1.6,label='SLAM 지도 위치 기준')
    for kind,label in [('raw','원본 GNSS'),('kalman','칼만'),('rts','RTS 후보정')]:
        xy=np.array([r[kind] for r in s['records'] if kind in r]);ax.plot(xy[:,0],xy[:,1],color=C[kind],lw=.8,alpha=.8,label=label)
    paired=[r for r in s['records'] if r['reference']]
    for r in paired[::max(1,len(paired)//12)]:ax.plot([r['raw'][0],r['reference'][0]],[r['raw'][1],r['reference'][1]],color='#b0a9a1',lw=.6,alpha=.6)
    ax.set_title(s['label']);ax.legend(fontsize=8,loc='best')


fig,axes=plt.subplots(2,3,figsize=(17,11),layout='constrained')
for ax,s in zip(axes.flat,DATA['sessions'].values()):plan(ax,s)
axes.flat[-1].axis('off')
axes.flat[-1].text(.03,.88,'실측 5개 세션\n고유 GNSS 시각 1,099개\n동시각 SLAM 대응 1,098개',fontsize=15,va='top',transform=axes.flat[-1].transAxes)
axes.flat[-1].text(.03,.57,'같은 미터 좌표와 동일한 축 비율\n회색 선은 같은 시각의 위치 차이\n층은 SLAM 기준이며 GNSS에서 추정하지 않음\nSLAM 기준도 독립 측량 정답은 아님\n세션별 임의 이동과 경로 강제 부착 없음',fontsize=11,va='top',linespacing=1.9,transform=axes.flat[-1].transAxes)
fig.suptitle('GNSS와 SLAM 지도 위치의 동시각 대조',fontsize=19)
save(fig,'all-session-overlay.png')

fig,axes=plt.subplots(5,1,figsize=(13,13),layout='constrained')
for ax,s in zip(axes,DATA['sessions'].values()):
    rows=[r for r in s['records'] if r['reference'] and 'kalman' in r];t0=s['reference_trajectory'][0]['t'];t=[r['gnss_stamp']-t0 for r in rows]
    for kind,label in [('raw','원본'),('kalman','칼만'),('rts','RTS')]:
        error=[np.linalg.norm(np.array(r[kind])-np.array(r['reference'][:2])) for r in rows];ax.plot(t,error,label=label,color=C[kind],lw=1)
    ax.set_title(s['label'],loc='left',fontsize=11);ax.set_ylabel('수평 차이 (m)');ax.grid(alpha=.2);ax.set_ylim(bottom=0)
axes[0].legend(ncol=3,fontsize=9);axes[-1].set_xlabel('촬영 시작 후 시간 (s)');fig.suptitle('촬영 궤적 대비 위치 차이의 시간 변화',fontsize=17)
save(fig,'residual-time.png')

names=[s['label'] for s in DATA['sessions'].values()];idx=np.arange(5)
fig,ax=plt.subplots(figsize=(12,5),layout='constrained')
for j,(key,title,col) in enumerate([('raw_common_residual_m','원본',C['raw']),('kalman_residual_m','칼만',C['kalman']),('rts_residual_m','RTS',C['rts'])]):
    vals=[s['metrics'][key]['rmse'] for s in DATA['sessions'].values()];bars=ax.bar(idx+(j-1)*.25,vals,.24,label=title,color=col)
    ax.bar_label(bars,fmt='%.2f',fontsize=9,padding=3)
ax.set_xticks(idx,names);ax.set_ylim(0,39);ax.set_ylabel('동일 시각의 수평 차이 RMSE (m)');ax.legend(ncol=3);ax.grid(axis='y',alpha=.18)
ax.set_title('평활화만으로 실내 위치 편향이 사라지지 않았다')
save(fig,'residual-summary.png')

fig,axes=plt.subplots(1,2,figsize=(12,5.4),layout='constrained')
for session,ax in zip(['lobby','floor2'],axes):
    s=DATA['sessions'][session];rows=[r for r in s['records'] if 'raw_walkable_distance_m' in r]
    xy=[r['raw_residual_m'] for r in rows];wall=[r['raw_walkable_distance_m'] for r in rows]
    ax.scatter(wall,xy,s=10,alpha=.5,color=C['raw']);ax.set_xlabel('가장 가까운 보행 격자까지 거리 (m)');ax.set_ylabel('동시각 SLAM 위치와의 차이 (m)');ax.grid(alpha=.2);ax.set_title(s['label'])
    count=sum(a<1 and b>10 for a,b in zip(wall,xy));ax.text(.04,.94,f'보행면 1m 이내여도\n실제 기준과 10m 초과 {count}/{len(rows)}개',transform=ax.transAxes,va='top',fontsize=11)
fig.suptitle('복도에 가깝다는 이유만으로 현재 위치가 맞지는 않는다',fontsize=16)
save(fig,'corridor-ambiguity.png')

fig,axes=plt.subplots(1,2,figsize=(12,5),layout='constrained')
before=[s['metrics']['raw_residual_m']['rmse'] for s in DATA['sessions'].values()]
after=[s['metrics']['without_preview_offset_residual_m']['rmse'] for s in DATA['sessions'].values()]
axes[0].barh(idx-.15,before,.3,label='현재 화면 이동 포함',color=C['raw']);axes[0].barh(idx+.15,after,.3,label='동 2m 남 2m 이동 제거',color='#6e8976');axes[0].set_yticks(idx,names);axes[0].invert_yaxis();axes[0].legend(fontsize=8);axes[0].set_xlabel('SLAM 대비 RMSE (m)');axes[0].grid(axis='x',alpha=.2)
for name,s in DATA['sessions'].items():
    d=np.asarray(s['metrics']['median_delta_map_xy_m']);R=np.asarray(DATA['georeference']['R_map_xy_to_EN']);d=R@d
    axes[1].quiver(0,0,d[0],d[1],angles='xy',scale_units='xy',scale=1,color=C['raw'],width=.006)
    axes[1].text(d[0]+.5,d[1],s['label'],fontsize=9)
axes[1].set_xlim(-25,10);axes[1].set_ylim(-4,23);axes[1].set_aspect('equal');axes[1].set_xlabel('동쪽 (m)');axes[1].set_ylabel('북쪽 (m)');axes[1].grid(alpha=.2);axes[1].set_title('세션별 위치차 성분 중앙값의 방향')
fig.suptitle('지도 표시 이동과 기록별 편차를 구분한 진단',fontsize=16)
save(fig,'registration-diagnostics.png')

fig=plt.figure(figsize=(12,8));ax=fig.add_subplot(111,projection='3d')
for key in ['lobby','floor2','bridge_12_sub']:
    s=DATA['sessions'][key];ref=np.array([r['position'] for r in s['reference_trajectory']]);ax.plot(*ref.T,color=C['reference'],lw=.9)
    rows=[r for r in s['records'] if r['reference']];xyz=np.array([[*r['raw'],r['reference'][2]] for r in rows]);ax.scatter(*xyz.T,s=4,alpha=.45,color=C['raw'])
ax.set_xlabel('지도 X (m)');ax.set_ylabel('지도 Y (m)');ax.set_zlabel('SLAM 카메라 Z (m)');ax.view_init(elev=28,azim=-52);ax.set_box_aspect((2,2,.65));ax.set_title('층간 기록의 수평 위치 차이\nGNSS 점의 Z는 비교용 SLAM 높이이며 GNSS 고도가 아니다')
save(fig,'multilevel-discrepancy.png')

# Compact figure for one report page; original and post-filter RMSE remain distinguishable.
fig,axes=plt.subplots(1,2,figsize=(12,5.1),layout='constrained')
for key,ax in zip(['lobby','floor2'],axes):plan(ax,DATA['sessions'][key])
fig.suptitle('실내 GNSS와 동시각 SLAM 위치 대조',fontsize=15)
save(fig,'indoor-report.png')
print('Created 7 figures from recorded data')
