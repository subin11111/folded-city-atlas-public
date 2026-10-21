"""Audit recorded route coverage separately from saved pedestrian updates.

The original ZIP is read-only. No trajectory fitting, floor reassignment or
filter tuning is performed. Optional figures only redraw measured barometer
samples and the already saved causal estimator traces.
"""
import argparse
import base64
import csv
import hashlib
import io
import json
from pathlib import Path
from statistics import median
import zipfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--zip', type=Path, required=True)
    parser.add_argument('--no-plot', action='store_true')
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    out = repo/'docs/assets/map-pdr-20261021'
    observations = json.loads((out/'observations.json').read_text('utf-8'))
    comparison = json.loads((out/'comparison.json').read_text('utf-8'))
    selection = json.loads((out/'selection.json').read_text('utf-8'))
    digest = hashlib.sha256(args.zip.read_bytes()).hexdigest()
    assert digest == observations['source_sha256'] == comparison['source_sha256']
    with zipfile.ZipFile(args.zip) as archive:
        tables = {}
        for name in ['AccelerometerUncalibrated', 'Gyroscope', 'Gravity', 'Orientation', 'Barometer', 'Location']:
            rows = list(csv.DictReader(io.StringIO(archive.read(name+'.csv').decode('utf-8-sig'))))
            tables[name] = [{key: float(value) for key,value in row.items()} for row in rows]
        photos = [name for name in archive.namelist() if name.startswith('Camera/') and name.lower().endswith('.jpg')]
    run = next(r for r in comparison['runs'] if r['config']==selection['run_config']
               and r['frontend']==selection['frontend'] and r['seed']==selection['seed']
               and abs(r['floorStart']-selection['floorStart'])<1e-8)
    ekf = next(r for r in comparison['runs'] if r['config']=='ekf_map'
               and r['frontend']==selection['frontend'] and r['seed']==selection['seed']
               and abs(r['floorStart']-selection['floorStart'])<1e-8)
    entry, stairs, upper = observations['entry_s'], observations['stairs_start_s'], selection['floorStart']
    duration = max(r['seconds_elapsed'] for r in tables['AccelerometerUncalibrated'])

    def updates(label, start, end):
        trace = [r for r in run['trace'] if start<=r['t']<end]
        return {'label':label, 'start_s':start, 'end_s':end, 'step_candidates':len(trace),
                'position_updates':sum(r['moved'] for r in trace),
                'updates_are_not_position_accuracy':True}

    lower_height = median(r['relativeAltitude'] for r in tables['Barometer'] if 42<=r['seconds_elapsed']<=47)
    upper_height = median(r['relativeAltitude'] for r in tables['Barometer'] if 75<=r['seconds_elapsed']<=99)
    gyrotimes = [r['seconds_elapsed'] for r in tables['Gyroscope']]
    locations = [r for r in tables['Location'] if r['seconds_elapsed']>=0]
    body = {
        'source_zip_sha256':digest, 'duration_s':duration,
        'streams':{name:{'samples':len(rows), 'start_s':rows[0]['seconds_elapsed'],
                        'end_s':rows[-1]['seconds_elapsed']} for name,rows in tables.items()},
        'gyroscope_median_hz':1/median(b-a for a,b in zip(gyrotimes,gyrotimes[1:])),
        'photo_count':len(photos),
        'photo_observations':[
            {'t':20.139, 'place':'building exterior'},
            {'t':30.617, 'place':'outside the entrance doors'},
            {'t':40.149, 'place':'inside the lobby; CU sign'},
            {'t':50.240, 'place':'ascending the main staircase'},
            {'t':60.154, 'place':'upper main staircase'},
            {'t':70.295, 'place':'second-floor PRIME HALL frontage'},
            {'t':110.272, 'place':'second-floor corridor'},
        ],
        'actual_entry_crossing_photo_bracket_s':[entry,40.149],
        'entry_initialization_assumption_s':entry,
        'entry_button_click_recorded':False,
        'stairs_barometer_ascent_interval_s':[stairs,upper],
        'barometer_lower_window_s':[42,47], 'barometer_upper_window_s':[75,99],
        'barometer_floor_rise_m':upper_height-lower_height,
        'barometer_is_independent_position_ground_truth':False,
        'replay_skips_pre_entry_samples':True,
        'replay_disables_pedestrian_steps_on_stairs':True,
        'second_floor_reset_s':upper,
        'second_floor_reset_is_assumed_confirmation_not_automatic_detection':True,
        'segments':[
            updates('entry-initialized interval',entry,stairs),
            updates('entry photo bracket subset',entry,40.149),
            updates('confirmed lobby photo until stairs subset',40.149,stairs),
            updates('stairs: updates deliberately disabled',stairs,upper),
            updates('upper floor after reset',upper,duration+.001),
        ],
        'ekf_upper_floor':ekf['phase']['second'],
        'pf_upper_floor_uncertainty_above_7m':run['paused'],
        'location_samples_in_session':len(locations),
        'cached_location_samples_before_start':len(tables['Location'])-len(locations),
        'last_location_s':locations[-1]['seconds_elapsed'],
        'max_location_gap_s':max(b['seconds_elapsed']-a['seconds_elapsed'] for a,b in zip(locations,locations[1:])),
        'location_tail_without_new_sample_s':duration-locations[-1]['seconds_elapsed'],
        'independent_position_reference_available':False,
        'native_sensor_recording_not_safari_field_test':True,
        'new_filter_experiment':False,
        'map_registration_changed':False,
        'module_hashes_match_saved_comparison':{
            name:hashlib.sha256((repo/f'web/public/{name}.js').read_bytes()).hexdigest()==value
            for name,value in comparison['source_sha256_by_module'].items()},
    }
    assert body['segments'][0]['position_updates']==9 and body['segments'][0]['step_candidates']==30
    assert body['segments'][2]['position_updates']==0 and body['segments'][2]['step_candidates']==15
    assert body['segments'][4]['position_updates']==57 and body['segments'][4]['step_candidates']==67
    assert len(photos)==11
    (out/'recording-scope.json').write_text(json.dumps(body,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    if not args.no_plot:
        plot(out, repo, body, tables, run, ekf)
    assert hashlib.sha256(args.zip.read_bytes()).hexdigest()==digest
    print(json.dumps({'duration_s':duration,'photo_count':len(photos),'barometer_floor_rise_m':upper_height-lower_height,
                      'segments':body['segments'],'source_unchanged':True},ensure_ascii=False))


def plot(out, repo, body, tables, run, ekf):
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False,'font.size':11,
                         'axes.spines.top':False,'axes.spines.right':False,'savefig.facecolor':'white'})
    fig, (left, right) = plt.subplots(1,2,figsize=(13,4.6),gridspec_kw={'width_ratios':[1.65,1]},layout='constrained')
    spans = [(0,30.617,'실외 기록','#DDE7E4'),(30.617,48.5757434,'입구 초기화 가정','#B5D5A5'),
             (48.5757434,68.841865234375,'계단 · 갱신 중단','#D8C4AA'),
             (68.841865234375,112.01044213867188,'2층 · 재초기화','#A2CC80')]
    for a,b,label,color in spans:
        left.axvspan(a,b,color=color,alpha=.35)
        left.text((a+b)/2,5.85,label,ha='center',va='center',fontsize=9)
    heights=tables['Barometer']
    baseline=median(r['relativeAltitude'] for r in heights if 42<=r['seconds_elapsed']<=47)
    left.plot([r['seconds_elapsed'] for r in heights],[r['relativeAltitude']-baseline for r in heights],
              color='#063D2A',lw=2,label='기압 상대높이 · 1층 기준값 차감')
    left.axvspan(30.617,40.149,facecolor='none',edgecolor='#7B817D',hatch='///',linewidth=.6)
    for t, y, label in [(20.139,-.9,'20.139 실외'),(40.149,-1.8,'40.149 실내 확인'),
                        (50.240,1.2,'50.240 계단'),(70.295,4.8,'70.295 PRIME HALL')]:
        left.plot([t,t],[0,y],color='#7B817D',lw=.7)
        left.text(t,y,label,fontsize=9,ha='center',va='bottom' if y>=0 else 'top')
    left.text(39.5,3.35,'초기화 구간 9/30\n실내 확인 이후 0/15',ha='center',fontsize=10,color='#7A4D24')
    left.text(91,2.2,'2층 PF 위치 갱신 57/67\n평가 지표: 갱신 횟수',ha='center',fontsize=10,color='#17804A')
    left.set(xlim=(0,112.1),ylim=(-2.6,6.35),xlabel='기록 시작 후 경과 시간 (초)',ylabel='상대높이 (m)',
             title='실외 → 1층 → 메인 계단 → 2층의 실제 연속 기록')
    left.grid(alpha=.15);left.legend(fontsize=9,loc='lower right')
    nav=json.loads((repo/'web/public/data/navigation.json').read_text('utf-8'))
    shape=nav['shape'];raw=np.frombuffer(base64.b64decode(nav['levels']['2']['walkable_bits']),dtype=np.uint8)
    walk=np.unpackbits(raw,bitorder='little')[:shape[0]*shape[1]].reshape(shape)
    origin=nav['origin_xy'];cell=nav['cell_m']
    right.imshow(np.where(walk,1,np.nan),origin='lower',
                 extent=(origin[0]-cell/2,origin[0]+(shape[1]-.5)*cell,origin[1]-cell/2,origin[1]+(shape[0]-.5)*cell),
                 cmap=matplotlib.colors.ListedColormap(['#E6ECE7']),vmin=0,vmax=1,interpolation='nearest')
    for r,color,label in [(ekf,'#A8693A','EKF 21/67'),(run,'#17804A','PF 57/67')]:
        xy=np.array([v['position'][:2] for v in r['trace'] if v['level']=='2'])
        right.plot(xy[:,0],xy[:,1],lw=2,color=color,label=label)
        right.scatter(*xy[0],color=color,s=30,marker='o')
        right.scatter(*xy[-1],color=color,s=45,marker='X')
    right.set(xlim=(-32,-8),ylim=(-14,20),xlabel='지도 X (m)',ylabel='지도 Y (m)',title='2층 계단 끝에서 다시 시작한 추정')
    right.set_aspect('equal');right.grid(alpha=.15);right.legend(fontsize=10,loc='upper left')
    fig.supxlabel('사선 영역은 사진으로 정한 출입문 통과 시각의 범위다. 위치 초기화 시각과 실제 통과 시각을 구분한다.',fontsize=10)
    fig.savefig(out/'recording-scope.png',dpi=220)
    plt.close(fig)


if __name__=='__main__':
    main()
