"""Plot measured-log replay and particle ancestry, without a fabricated truth path."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation,PillowWriter
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--font-family',default='Malgun Gothic')
args=parser.parse_args()
R=Path(__file__).resolve().parents[1];out=R/'docs/assets/map-pdr-20261021'
d=json.loads((out/'comparison.json').read_text('utf-8'))
nav=json.loads((R/'web/public/data/navigation.json').read_text('utf-8'))
catalog=json.loads((R/'web/public/data/catalog.json').read_text('utf-8'))
plt.rcParams.update({'font.family':args.font_family,'axes.unicode_minus':False,'font.size':11,
    'axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
green='#17804A';dark='#063D2A';gray='#8C9790';orange='#A8693A'
def run(config,seed=872341):
    return next(r for r in d['runs'] if r['config']==config and r['frontend']=='gyro' and r['seed']==seed and abs(r['floorStart']-68.841865234375)<1e-8)
baseline,selected=run('ekf_map'),run('dense_pf')
def background(ax):
    shape=nav['shape'];raw=np.frombuffer(__import__('base64').b64decode(nav['levels']['2']['walkable_bits']),dtype=np.uint8)
    bits=np.unpackbits(raw,bitorder='little')[:shape[0]*shape[1]].reshape(shape)
    o=nav['origin_xy'];c=nav['cell_m']
    ax.imshow(np.where(bits,1,np.nan),origin='lower',extent=(o[0]-c/2,o[0]+(shape[1]-.5)*c,o[1]-c/2,o[1]+(shape[0]-.5)*c),
        cmap=matplotlib.colors.ListedColormap(['#E6ECE7']),vmin=0,vmax=1,interpolation='nearest')
    for p in catalog:
        if p['id'] not in ['prime_hall','career_office','classical_music']:continue
        ax.scatter(*p['position'][:2],marker='s',s=18,color='#596960',zorder=4)
        label={'prime_hall':'PRIME HALL','career_office':'학생취업처','classical_music':'고전음악감상실'}[p['id']]
        ax.annotate(label,p['position'][:2],xytext=(4,4),textcoords='offset points',fontsize=8,color='#596960')
    ax.set(xlim=(-34,-7),ylim=(-14,20),xlabel='지도 X (m)',ylabel='지도 Y (m)');ax.set_aspect('equal');ax.grid(alpha=.14)
def points(r):return np.array([p['position'][:2] for p in r['trace'] if p['level']=='2'])
fig,axes=plt.subplots(1,3,figsize=(13,6.1),layout='constrained')
for ax in axes:background(ax)
for ax,r,label,color in [(axes[0],baseline,'보행 EKF + 벽 제약',orange),(axes[1],selected,'지도 보조 적응형 PF · 실시간',green)]:
    a=points(r);ax.plot(a[:,0],a[:,1],color=color,lw=2)
    ax.scatter(*a[0],color=dark,marker='o',s=35);ax.scatter(*a[-1],color=color,marker='X',s=65,zorder=5)
    ax.set_title(f"{label}\n갱신 {r['phase']['second']['moved']}/{r['phase']['second']['steps']}, 안내 가능 {r['usable']}",fontsize=11)
for seed in [193,893,20261021]:
    a=points(run('dense_pf',seed));axes[1].plot(a[:,0],a[:,1],color=green,alpha=.19,lw=.8,zorder=0)
a=np.array([p['position'][:2] for p in selected['ancestry']]);axes[2].plot(a[:,0],a[:,1],color=dark,lw=2)
axes[2].scatter(*a[0],marker='o',color=dark,s=35);axes[2].scatter(*a[-1],marker='X',color=dark,s=65)
axes[2].set_title('입자 계보 역추적 · 후보정\n후속 벽 관측을 사용, 실시간 아님',fontsize=11)
fig.supxlabel('같은 지도·센서와 층 시작 조건. 원=시작, X=종료. 위치 정답이 아닌 추정 궤적이며 사진의 목적지 좌표로 피팅하지 않았다.',fontsize=9)
fig.savefig(out/'floor2-comparison.png',dpi=220);plt.close(fig)

# Single-axis comparison sized for a readable portrait report figure.
fig,ax=plt.subplots(figsize=(6.4,4.4),layout='constrained');background(ax)
for item,color,label in [(baseline,orange,'보행 EKF 21/67회'),(selected,green,'지도 보조 PF 57/67회')]:
    xy=points(item);ax.plot(xy[:,0],xy[:,1],color=color,lw=2,label=label)
    ax.scatter(*xy[-1],marker='X',color=color,s=55)
ax.set_title('같은 실측 센서와 2층 시작점의 인과적 추정',fontsize=11)
ax.legend(fontsize=10,loc='upper left');fig.savefig(out/'floor2-overlay.png',dpi=220);plt.close(fig)

groups=[g for g in d['groups'] if g['frontend']=='gyro']
fig,axes=plt.subplots(1,2,figsize=(11.6,3.9),layout='constrained')
labels=['보행 EKF','고정 입자 PF','적응형 PF','넓은 방향 PF','고밀도 PF'];x=np.arange(len(groups))
for ax,key,title,color in [(axes[0],'medianRejected','2층 위치 갱신 거부 횟수 · 중앙값',orange),(axes[1],'medianUsable','불확실성 기준을 통과한 갱신 · 중앙값',green)]:
    values=[g[key] for g in groups];bars=ax.bar(x,values,color=color,width=.62);ax.bar_label(bars,padding=3)
    ax.set(xticks=x,xticklabels=labels,ylabel='걸음 후보 수',title=title,ylim=(0,max(values)+12));ax.grid(axis='y',alpha=.15)
fig.supxlabel('시작 시각 3개 × 난수 시드 4개. 동일 기록의 민감도 시험이며 독립 사용자 12명의 결과가 아니다.',fontsize=9)
fig.savefig(out/'method-comparison.png',dpi=220);plt.close(fig)

trace=[p for p in selected['trace'] if p['level']=='2'];t=np.array([p['t'] for p in trace]);a=points(selected)
fig,axes=plt.subplots(2,1,figsize=(10.5,5.3),layout='constrained',sharex=True)
axes[0].plot(t,[p['uncertainty'] for p in trace],color=green);axes[0].axhline(7,color=orange,ls='--',label='안내 대기 기준')
axes[0].set(ylabel='모델 불확실성 (m)',title='2층 재생의 불확실성과 입자 수');axes[0].legend()
axes[1].step(t,[p['particles'] for p in trace],where='post',color=dark);axes[1].set(ylabel='입자 수',xlabel='기록 시작 후 시간 (s)')
for ax in axes:ax.grid(alpha=.2)
fig.savefig(out/'uncertainty.png',dpi=220);plt.close(fig)

fig,ax=plt.subplots(figsize=(6.4,7.0),layout='constrained');background(ax)
line,=ax.plot([],[],color=green,lw=2);dot,=ax.plot([],[],'o',color=green,ms=9);title=ax.set_title('')
def update(i):
    line.set_data(a[:i+1,0],a[:i+1,1]);dot.set_data([a[i,0]],[a[i,1]])
    title.set_text(f"실측 센서 로그 재생 · 2층 · {t[i]:.1f}초\n지도 보조 PF / 현장 위치 정답 아님")
    return line,dot,title
animation=FuncAnimation(fig,update,frames=range(0,len(a),2),interval=140,blit=False)
animation.save(out/'floor2-replay.gif',writer=PillowWriter(fps=7),dpi=100);plt.close(fig)
manifest={'run_config':selected['config'],'frontend':selected['frontend'],'seed':selected['seed'],
    'seed_selection':'Fixed production default, not best seed','floorStart':selected['floorStart'],
    'total_combinations':len(d['runs']),'second':selected['phase']['second'],'first':selected['phase']['first'],
    'guidance_usable_second':selected['usable'],'uncertainty_pause_second':selected['paused'],
    'ground_truth_available':False,'same_sensor_different_conditions_not_independent_subjects':True,
    'retrospective_ancestry_not_used_for_live_location':True,'map_registration_changed':False}
(out/'selection.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(manifest,ensure_ascii=False))

# Assemble actual browser captures when present; no scene synthesis or retouching.
frames=sorted(out.glob('web-frame-*.png'))
if frames:
    from PIL import Image
    images=[Image.open(p).convert('RGB').resize((960,600)) for p in frames]
    images[0].save(out/'web-floor2-replay.gif',save_all=True,append_images=images[1:],duration=550,loop=0)
