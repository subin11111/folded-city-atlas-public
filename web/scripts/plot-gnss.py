"""Matplotlib figures from saved filter outputs; no generated scene imagery."""
import argparse
import json
import os
from pathlib import Path
import sys
if os.environ.get('GNSS_PLOT_DEPS'):sys.path.insert(0,os.environ['GNSS_PLOT_DEPS'])
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.ticker import FuncFormatter
from PIL import Image

root=Path(__file__).resolve().parents[2]
out=root/'docs/assets/gnss-navigation-20261020'
font=Path('C:/Windows/Fonts/malgun.ttf')
if font.exists():
    font_manager.fontManager.addfont(str(font));plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name()
plt.rcParams.update({'font.size':15,'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False,'figure.facecolor':'white','savefig.facecolor':'white'})
colors={'raw':'#8c969e','kalman':'#265d89','rts':'#16815b','truth':'#202c26','reject':'#af4c34'}
read=lambda name:json.loads((out/name).read_text('utf-8'))
synthetic=read('filter_results.json');recorded=read('recorded-filter-results.json')
def save(fig,name):
    fig.savefig(out/name,dpi=180,bbox_inches='tight');plt.close(fig)
def grid(ax):ax.grid(True,color='#dde2df',linewidth=.65,alpha=.9);ax.set_axisbelow(True)
def plan(data,name,measured=False):
    h=data['history'];raw=np.array([r['raw'] for r in h]);kf=np.array([r['x'][:2] for r in h]);rts=np.array([r['position'] for r in data['smooth']])
    fig,axs=plt.subplots(1,2,figsize=(12,5.2),layout='constrained')
    axs[0].plot(*raw.T,'.-',color=colors['raw'],ms=3,lw=.8,label='원 수신 위치')
    if not measured:
        truth=np.array([r['position'] for r in data['truth']]);axs[0].plot(*truth.T,'--',color=colors['truth'],lw=1,label='합성 기준 궤적');axs[1].plot(*truth.T,'--',color=colors['truth'],lw=1,label='합성 기준 궤적')
    axs[1].plot(*raw.T,'.',color=colors['raw'],ms=2,alpha=.55,label='원 수신 위치')
    # Break the display at missing-observation segments; never draw a recovered path across an outage.
    starts=[i for i,r in enumerate(h) if r['segment']]+[len(h)]
    for k,(a,b) in enumerate(zip(starts,starts[1:])):
        axs[1].plot(*kf[a:b].T,color=colors['kalman'],lw=1.3,label='실시간 칼만' if k==0 else None)
        axs[1].plot(*rts[a:b].T,color=colors['rts'],lw=1.4,label='RTS 후보정' if k==0 else None)
    rejected=np.array([r['raw'] for r in h if not r['accepted']])
    if len(rejected):axs[0].scatter(*rejected.T,c=colors['reject'],marker='x',s=55,label='갱신 거부')
    low=raw.min(axis=0)-6;high=raw.max(axis=0)+6
    for ax,title in zip(axs,['원본','실시간 보정과 후보정']):
        ax.set(xlim=(low[0],high[0]),ylim=(low[1],high[1]),xlabel='지도 x [m]',ylabel='지도 y [m]',title=title);ax.set_aspect('equal',adjustable='box');grid(ax);ax.legend(fontsize=13,loc='best')
    fig.suptitle('실측 iPhone 위치 로그 / 독립 기준 위치 없음' if measured else '합성 GNSS 재현 / 고정 seed 20261020',fontweight='bold')
    save(fig,name)
plan(synthetic,'synthetic-plan.png')
plan(recorded,'recorded-plan.png',True)
h=synthetic['history'];t0=h[0]['t'];t=np.array([r['t']-t0 for r in h]);truth={r['timestamp']/1000:r['position'] for r in synthetic['truth']};target=np.array([truth[r['t']] for r in h])
fig,axs=plt.subplots(3,1,figsize=(11.7,8),sharex=True,layout='constrained')
for series,label,c in [(np.array([r['raw'] for r in h]),'원본',colors['raw']),(np.array([r['x'][:2] for r in h]),'칼만',colors['kalman']),(np.array([r['position'] for r in synthetic['smooth']]),'RTS',colors['rts'])]:axs[0].plot(t,np.linalg.norm(series-target,axis=1),label=label,color=c,lw=1.3)
axs[0].set_ylabel('합성 기준 대비 오차 [m]');axs[0].legend(ncol=3,fontsize=10)
axs[1].plot(t,[r['nis'] for r in h],color=colors['kalman'],lw=1);axs[1].axhline(9.21034,color=colors['reject'],ls='--',label='χ²(2), 99% = 9.21');axs[1].set(yscale='symlog',ylabel='혁신 NIS');axs[1].yaxis.set_major_formatter(FuncFormatter(lambda v,_:f'{v:g}'));axs[1].set_ylim(bottom=0);axs[1].legend(fontsize=13)
statuses=json.loads((root/'web/public/data/gnss_demo.json').read_text())['records'];tx=[r['timestamp']/1000-t0 for r in statuses]
axs[2].plot(tx,[r['coords']['accuracy'] for r in statuses],color=colors['raw'],label='입력 정확도 반경');axs[2].axhline(50,ls='--',color=colors['reject'],label='수신 반경 문턱 50m');axs[2].set(ylabel='보고 반경 [m]',xlabel='재현 시간 [s]');axs[2].legend(fontsize=10)
for ax in axs:ax.axvspan(70,88,color='#b6bdb7',alpha=.2,label=None);grid(ax)
fig.suptitle('위치 튐 판정과 수신 공백 / 회색 구간은 관측 없음',fontweight='bold');save(fig,'synthetic-time.png')
fig,axs=plt.subplots(1,2,figsize=(11.7,4.1),layout='constrained')
for values,label,c in [(np.array([r['raw'] for r in h]),'원본',colors['raw']),(np.array([r['x'][:2] for r in h]),'칼만',colors['kalman']),(np.array([r['position'] for r in synthetic['smooth']]),'RTS',colors['rts'])]:axs[0].plot(t,np.linalg.norm(values-target,axis=1),label=label,color=c,lw=1.3)
axs[0].set(xlabel='재현 시간 [s]',ylabel='기준 대비 오차 [m]');axs[0].legend(fontsize=12,ncol=3)
axs[1].plot(t,[r['nis'] for r in h],color=colors['kalman']);axs[1].axhline(9.21034,color=colors['reject'],ls='--',label='판정 문턱 9.21');axs[1].set(xlabel='재현 시간 [s]',ylabel='혁신 NIS',yscale='symlog',ylim=(0,None));axs[1].yaxis.set_major_formatter(FuncFormatter(lambda v,_:f'{v:g}'));axs[1].legend(fontsize=12)
for ax in axs:ax.axvspan(70,88,color='#b6bdb7',alpha=.2);grid(ax)
fig.suptitle('합성 시간에 따른 오차와 이상치 판정',fontweight='bold');save(fig,'synthetic-report.png')
h=recorded['history'];t=np.array([r['t']-h[0]['t'] for r in h]);fig,axs=plt.subplots(3,1,figsize=(11.7,7),sharex=True,layout='constrained')
for j,label in enumerate(['x [m]','y [m]']):
    for values,name,c in [([r['raw'][j] for r in h],'원본',colors['raw']),([r['x'][j] for r in h],'칼만',colors['kalman']),([r['position'][j] for r in recorded['smooth']],'RTS',colors['rts'])]:axs[j].plot(t,values,label=name,color=c,lw=1.1)
    axs[j].set_ylabel(label);grid(axs[j])
axs[0].legend(ncol=3,fontsize=10)
axs[2].plot(t,[r['accuracy'] for r in h],color=colors['raw']);axs[2].set(ylabel='iOS 보고 반경 [m]',xlabel='첫 GNSS 시각 이후 [s]');grid(axs[2]);fig.suptitle('실측 야외 338개 고유 시각 / 절대 정확도 개선율은 계산하지 않음',fontweight='bold');save(fig,'recorded-time.png')
fig,ax=plt.subplots(figsize=(11.7,4.5),layout='constrained');ax.axis('off')
labels=['WGS84 위치 수신\n시각과 정확도 검사','ECEF → ENU → 지도 XY\n등속도 예측 F, Q','혁신과 속도 판정\nNIS ≤ 9.21','칼만 갱신\nJoseph 공분산','현재 위치와 헤딩\n실외 경로 진행']
for i,label in enumerate(labels):
    x=.02+i*.197;ax.text(x+.083,.73,label,ha='center',va='center',fontsize=10,bbox={'boxstyle':'square,pad=.8','facecolor':'#f2f4f3','edgecolor':'#a3afa7'},transform=ax.transAxes)
    if i<4:ax.annotate('',xy=(x+.194,.73),xytext=(x+.175,.73),xycoords='axes fraction',arrowprops={'arrowstyle':'->','color':'#354b40'})
ax.text(.5,.35,'원 수신과 공분산 저장 → 정방향 필터 기록 → RTS 역방향 평활화 → 원본 / 칼만 / 후보정 비교',ha='center',fontsize=12,transform=ax.transAxes)
ax.text(.5,.12,'실시간 안내에는 미래 관측을 사용하지 않음     |     15초 초과 공백은 새 구간     |     GNSS로 실내 층을 자동 확정하지 않음',ha='center',fontsize=10,transform=ax.transAxes)
ax.set_title('실시간 위치 보정과 로그 후보정의 분리',fontweight='bold',pad=16);save(fig,'filter-pipeline.png')
frames=sorted((out/'frames').glob('*.png'))
if frames:
    images=[Image.open(p).convert('RGB').resize((390,844),Image.Resampling.LANCZOS).quantize(colors=128) for p in frames]
    images[0].save(out/'gnss-replay.gif',save_all=True,append_images=images[1:],duration=300,loop=0,optimize=False)
print(json.dumps({'figures':6,'gif_frames':len(frames)},ensure_ascii=False))
