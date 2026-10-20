#!/usr/bin/env python3
"""Explain a human visual shortlist from immutable raw rows, poses and surfaces.

No optimization, source-map replacement or automatic candidate promotion.
The evidence distinguishes measured differences from causal interpretation.
"""
import argparse
from collections import Counter
import csv
from itertools import combinations
import json
from pathlib import Path
import shutil
import sqlite3

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.spatial.transform import Rotation

from audit_constraint_conflicts import graph_bridges
from check_session_overlap import Session
from evaluate_submap_candidates import fixed_patches
from evaluate_map_surfaces import pair_metrics
from export_rtabmap_db import load_pose_csv
from export_submap_candidate_clouds import render_projection
from optimize_robust_submaps import (_as_transform, _edge_chi2, _robust_weights,
    connected_components, parse_manifest, pose_error)
from optimize_session import Clouds, apply, geometric_score, inverse, sha256_file, utc_now
from render_session_report import read_ply
from revalidate_loop_constraints import SPECS

ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT/'outputs/student_union_20261017_independent_v1'
PORTFOLIO=ROOT/'docs/assets/session-portfolios'
NEW=ROOT/'outputs/student_union_20261017_session_portfolios_v1'
QUALITY=ROOT/'outputs/student_union_20261017_submap_quality'
CONFIG=ROOT/'config/student_union_visual_shortlist_20261017.json'
RAW_FIELDS=['node_id','stamp','quality_grade','quality_score','quality_reasons','critical_reasons',
    'confidence_100_fraction','confidence_50_fraction','confidence_0_fraction','usable_depth_fraction',
    'blur_laplacian_variance_640px','dark_clipped_fraction','bright_clipped_fraction',
    'angular_step_deg','linear_step_m','timestamp_step_s','intrinsic_jump_depth_px','primary_submap_id']


def read(path):return json.loads(Path(path).read_text())


def write(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():raise FileExistsError(path)
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')


def csv_write(path,rows):
    with path.open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(rows)


def distribution(values):
    values=np.asarray(values,float)
    return {'count':len(values),'median':float(np.median(values)) if len(values) else None,
            'p90':float(np.percentile(values,90)) if len(values) else None,
            'maximum':float(values.max()) if len(values) else None}


def pose_difference(reference,candidate):
    ids=sorted(reference)
    if set(candidate)!=set(reference):raise ValueError('Pose observation support differs')
    translation=np.array([candidate[n][:3,3]-reference[n][:3,3] for n in ids])
    rotations=[float(np.degrees(Rotation.from_matrix(reference[n][:3,:3].T@candidate[n][:3,:3]).magnitude())) for n in ids]
    return {'translation_m':distribution(np.linalg.norm(translation,axis=1)),
            'absolute_z_change_m':distribution(np.abs(translation[:,2])),
            'rotation_deg':distribution(rotations),'identical_to_1mm_and_0p01deg':bool(
                np.linalg.norm(translation,axis=1).max()<.001 and max(rotations)<.01)}


def loss_activity(graph,report,method):
    nodes,edges=parse_manifest(graph);v=report['variants'][method]
    poses={k:_as_transform(t,k) for k,t in v['poses'].items()};train=[e for e in edges if e.used_for_fit]
    weights,_=_robust_weights(method,train,poses,**{k:v['parameters'][k] for k in ('huber_delta','dcs_phi','group_switch_prior')})
    groups=Counter(e.region_group for e in train if not e.is_odometry)
    rows=[]
    for edge in train:
        error,chi2=_edge_chi2(edge,poses)
        robust=weights[edge.id]*(groups[edge.region_group] if not edge.is_odometry else 1.)
        rows.append({'id':edge.id,'type':edge.edge_type,'chi2':chi2,'weight_before_region_average':robust,
            'translation_m':float(np.linalg.norm(error[:3])),'rotation_deg':float(np.degrees(np.linalg.norm(error[3:])))})
    switchable=[r for r in rows if r['type']!='odometry']
    bridges=graph_bridges([n.id for n in nodes],train)
    return {'parameters':v['parameters'],'nodes':len(nodes),'training_edges':len(train),
        'edge_types':dict(Counter(e.edge_type for e in train)),
        'graph_bridges':len(bridges),'strict_outer_step_met':v['solver_iterations'][-1]['maximum_parameter_step']<1e-7,
        'last_solver_iteration':v['solver_iterations'][-1],
        'switchable_count':len(switchable),'robust_weights_below_0p99':sum(r['weight_before_region_average']<.99 for r in switchable),
        'maximum_switchable_chi2':max((r['chi2'] for r in switchable),default=None),'rows':rows}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--assets',type=Path,required=True);a=p.parse_args();out=a.output.resolve();assets=a.assets.resolve()
    if out.exists() or assets.exists():raise FileExistsError('Use fresh evidence directories')
    out.mkdir(parents=True);assets.mkdir(parents=True);shutil.copyfile(Path(__file__),out/'source_snapshot.py')
    config=read(CONFIG);quality=read(QUALITY/'manifest.json');historical=read(OLD/'evaluation/report.json')['comparisons']
    old_review=read(OLD/'review.json');joint_path=ROOT/'outputs/student_union_20261017_geometry_review/graph_final/report.json'
    joint=read(joint_path);normal_path=joint_path.parent/'normal_observations.json';normal=read(normal_path)
    input_paths={CONFIG,QUALITY/'manifest.json',OLD/'evaluation/report.json',OLD/'review.json',joint_path,normal_path,Path(__file__)}
    data={};raw_rows=[];summary_rows=[];pair_rows=[];patch_rows=[];figures=[];live={}
    # Establish immutable sources before geometry analysis.
    for name,spec in config['sessions'].items():
        directory=PORTFOLIO/name;summary=read(directory/'summary.json');stages={s['id']:s for s in summary['stages']}
        codes=list(dict.fromkeys(spec['shortlist']+spec['controls']))
        source_native=ROOT/'outputs'/SPECS[name][1]/name/'validated_edges.json'
        source_raw=ROOT/'outputs'/SPECS[name][1]/name/'raw_main_trajectory.csv'
        q_csv=QUALITY/name/'frames.csv';graph=OLD/'isolation'/name/'manifest.json';backend=OLD/'optimization'/name/'report.json'
        input_paths.update([directory/'summary.json',source_native,source_raw,q_csv,graph,backend,NEW/name/'graph.json'])
        poses={key:load_pose_csv(Path(stages[key]['trajectory'])) for key in codes}
        for code in codes:
            stage=stages[code];input_paths.add(Path(stage['trajectory']))
            assert sha256_file(Path(stage['trajectory']))==stage['trajectory_sha256']
        db=Path(quality['sessions'][name]['source_database']);input_paths.add(db)
        assert sha256_file(db)==quality['sessions'][name]['database_sha256_before']
        live[name]={'summary':summary,'stages':stages,'codes':codes,'poses':poses,'db':db,'native':read(source_native),
                    'raw':load_pose_csv(source_raw),'qrows':list(csv.DictReader(q_csv.open())),
                    'graph':read(graph),'backend':read(backend)}
    hashes={str(p.resolve()):sha256_file(p) for p in input_paths}
    def save(fig,name,caption):
        fig.tight_layout(rect=(0,0,1,.94));path=assets/(name+'.png');fig.savefig(path,dpi=160);plt.close(fig)
        with Image.open(path) as im:im.verify()
        figures.append({'file':path.name,'sha256':sha256_file(path),'caption':caption,'synthetic':False})
    for name,session in live.items():
        summary=session['summary'];pp=session['poses'];codes=session['codes'];ids=set(next(iter(pp.values())))
        native=session['native'];q=quality['sessions'][name]
        # A common exact-pair reserve excludes anything fitted in the newer
        # graphs; this is still reused/development data, not an independent survey.
        trained=set()
        for g in [session['graph'],read(NEW/name/'graph.json')]:
            for edge in g['edges']:
                endpoint=edge.get('provenance',{}).get('frame_endpoints')
                if edge['role']=='train' and endpoint and endpoint[0]==endpoint[2]==name:
                    trained.add(tuple(sorted((int(endpoint[1]),int(endpoint[3])))))
        reserve=[tuple(pair) for pair in old_review['sessions'][name]['geometry_pairs']['heldout']
                 if tuple(sorted(pair)) not in trained and set(pair)<=ids]
        odometry=[e for e in native['odometry'] if {int(e['from_id']),int(e['to_id'])}<=ids]
        picks=np.unique(np.linspace(0,len(odometry)-1,min(24,len(odometry))).astype(int)) if odometry else []
        adjacent=[(int(odometry[i]['from_id']),int(odometry[i]['to_id'])) for i in picks]
        closure=[(int(e['from_id']),int(e['to_id'])) for e in native['verified_loops']
                 if {int(e['from_id']),int(e['to_id'])}<=ids]
        surfaces=[r for r in historical if r['session']==name and {r['a'],r['b']}<=ids]
        cache={};candidate_evidence={}
        with sqlite3.connect(f'file:{session["db"]}?mode=ro',uri=True) as con:
            clouds=Clouds(con,None)
            for row in surfaces:
                for node in (row['a'],row['b']):
                    if node not in cache:cache[node]=fixed_patches(clouds,session['raw'],node)
            for code in codes:
                pose=pp[code];patches=[]
                for row in surfaces:
                    aa,bb,kind=row['a'],row['b'],row['kind']
                    metric=pair_metrics(cache[aa][kind,row['plane_a']],cache[bb][kind,row['plane_b']],pose[aa],pose[bb])
                    patches.append({'a':aa,'b':bb,'kind':kind,'plane_a':row['plane_a'],'plane_b':row['plane_b'],**metric})
                    patch_rows.append({'session':name,'candidate':code,**patches[-1]})
                geometry={}
                for group,pairs in [('adjacent_local',adjacent),('common_exact_pair_reserve',reserve),('existing_closure_diagnostic',closure)]:
                    rows=[]
                    for aa,bb in pairs:
                        score=geometric_score(clouds.get(aa)[1],clouds.get(bb)[1],inverse(pose[aa])@pose[bb],.20)
                        rows.append({'a':aa,'b':bb,**score})
                    geometry[group]={'count':len(rows),'clipped_rmse_m':distribution([r['clipped_rmse_m'] for r in rows]),
                                     'overlap':distribution([r['overlap'] for r in rows]),'rows':rows}
                odom_errors=[pose_error(np.asarray(e['measurement']),pose[e['from_id']],pose[e['to_id']]) for e in odometry]
                framez=[pose[n][2,3] for n in sorted(pose)]
                e=summary['evaluation'][code]
                frozen=e['patches']
                if len(frozen)!=len(patches) or any(
                    (old['a'],old['b'],old['kind'])!=(new['a'],new['b'],new['kind'])
                    or abs(old['value_m']-new['symmetric_plane_abs_median_m'])>1e-6
                    for old,new in zip(frozen,patches)):
                    raise ValueError('Recomputed fixed surfaces differ from frozen portfolio; do not mix evidence')
                perkind={kind:{'plane_distance_m':distribution([r['symmetric_plane_abs_median_m'] for r in patches if r['kind']==kind]),
                               'normal_difference_deg':distribution([r['normal_difference_degrees'] for r in patches if r['kind']==kind])}
                         for kind in ('wall','horizontal')}
                candidate_evidence[code]={'user_shortlisted':code in config['sessions'][name]['shortlist'],
                    'fixed_patch':{k:e[k] for k in ('count','median_cm','p90_cm','maximum_cm')},'per_kind':perkind,
                    'geometry':geometry,'native_odometry_translation_m':distribution([np.linalg.norm(x[:3]) for x in odom_errors]),
                    'native_odometry_rotation_deg':distribution([np.degrees(np.linalg.norm(x[3:])) for x in odom_errors]),
                    'camera_z_span_m':float(np.ptp(framez)),'difference_from_app':pose_difference(pp['O00'],pose),
                    'solver':session['stages'][code].get('solver'),'patches':patches}
                summary_rows.append({'session':name,'candidate':code,'user_shortlisted':code in config['sessions'][name]['shortlist'],
                    'fixed_patch_count':e['count'],'patch_median_cm':e['median_cm'],'patch_p90_cm':e['p90_cm'],'patch_maximum_cm':e['maximum_cm'],
                    'wall_normal_p90_deg':perkind['wall']['normal_difference_deg']['p90'],
                    'horizontal_normal_p90_deg':perkind['horizontal']['normal_difference_deg']['p90'],
                    'common_reserve_pairs':len(reserve),'reserve_rmse_median_m':geometry['common_exact_pair_reserve']['clipped_rmse_m']['median'],
                    'reserve_overlap_median':geometry['common_exact_pair_reserve']['overlap']['median'],
                    'local_adjacent_rmse_median_m':geometry['adjacent_local']['clipped_rmse_m']['median'],
                    'native_odom_translation_p90_cm':candidate_evidence[code]['native_odometry_translation_m']['p90']*100 if odom_errors else None,
                    'camera_z_span_m':float(np.ptp(framez))})
            # Fixed original camera points expose why XY can look good while
            # another surface is displaced in depth/height.
            if name=='floor2':
                for aa,bb,kind in [(548,981,'wall'),(656,1055,'wall'),(642,1031,'horizontal')]:
                    row=next(r for r in surfaces if (r['a'],r['b'],r['kind'])==(aa,bb,kind))
                    selected=['H00','J04','F03','N06'];world={}
                    for code in selected:
                        world[code]=[apply(cache[node][kind,plane]['testing'],pp[code][node])
                                     for node,plane in [(aa,row['plane_a']),(bb,row['plane_b'])]]
                    joined=np.concatenate([v for pair in world.values() for v in pair]);fig,axes=plt.subplots(2,4,figsize=(18,8))
                    for col,code in enumerate(selected):
                        metric=next(r for r in candidate_evidence[code]['patches'] if (r['a'],r['b'],r['kind'])==(aa,bb,kind))
                        for rr,(i,j) in enumerate([(0,1),(0,2)]):
                            ax=axes[rr,col]
                            for points,color,label in zip(world[code],['#367eb0','#e28c43'],[str(aa),str(bb)]):
                                ax.scatter(points[:,i],points[:,j],s=3,color=color,alpha=.6,label=label)
                            ax.set(xlim=(joined[:,i].min()-.1,joined[:,i].max()+.1),ylim=(joined[:,j].min()-.1,joined[:,j].max()+.1),
                                xlabel='X (m)',ylabel=['Y (m)','Z (m)'][rr]);ax.set_aspect('equal',adjustable='box');ax.grid(alpha=.15)
                        axes[0,col].set_title(f'{code}: gap {metric["symmetric_plane_abs_median_m"]*100:.2f} cm\nnormal {metric["normal_difference_degrees"]:.2f} deg')
                    axes[0,0].legend();fig.suptitle(f'floor2 {aa}/{bb} {kind}: identical finite test points, common axes\nJ04 is a visual preference, not an all-region accuracy winner')
                    save(fig,f'floor2_{aa}_{bb}_{kind}','Same unmodified original depth patch; no refit per candidate or wall deletion. Gaps are repeated development diagnostics, not surveyed accuracy.')
        pairdiff={}
        for left,right in combinations(codes,2):
            delta=pose_difference(pp[left],pp[right]);pairdiff[left+'__'+right]=delta
            pair_rows.append({'session':name,'left':left,'right':right,
                'maximum_translation_m':delta['translation_m']['maximum'],'p90_translation_m':delta['translation_m']['p90'],
                'maximum_rotation_deg':delta['rotation_deg']['maximum'],'maximum_z_change_m':delta['absolute_z_change_m']['maximum'],
                'identical_to_1mm_and_0p01deg':delta['identical_to_1mm_and_0p01deg']})
        losses={method:loss_activity(session['graph'],session['backend'],method) for method in ('baseline_huber','dcs','group_switch')}
        for row in session['qrows']:raw_rows.append({'session':name,**{k:row.get(k) for k in RAW_FIELDS}})
        keep=[r for r in session['summary']['exports'] if r['id'] in codes and r['id']!='O00']
        assert len({r['camera_points_sha256'] for r in keep})==1
        assert len({r['input_points'] for r in keep})==1
        data[name]={'shortlist':config['sessions'][name]['shortlist'],'controls':config['sessions'][name]['controls'],
            'source_quality':{k:q[k] for k in ('frame_count','quality_grade_counts','hard_boundary_frame_count','metric_summaries','reason_counts')},
            'same_preprocessed_camera_point_support_verified':True,'candidates':candidate_evidence,
            'candidate_pose_differences':pairdiff,'historical_robust_loss_activity':losses,
            'common_reserve_pairs':reserve,'raw_native_loop_count':len(native['verified_loops'])}
        # Newly plotted shortlists plus explicit controls; original input cloud
        # bounds/height scale are reused to prevent view-dependent approval.
        display=list(dict.fromkeys(config['sessions'][name]['shortlist']+(['N02'] if name!='floor2' else ['H00','F03','N06'])))
        clouds={key:read_ply(NEW/name/'clouds'/(key+'.ply')) for key in display}
        fig,axes=plt.subplots(2,len(display),figsize=(4.6*len(display),9),squeeze=False)
        for col,key in enumerate(display):
            for rr,(first,second) in enumerate([('x','y'),('x','z')]):
                render_projection(axes[rr,col],clouds[key],first,second,summary['export_settings']['common_full_cloud_bounds'],
                    title=key+(' [visual shortlist]' if key in config['sessions'][name]['shortlist'] else ' [control]') if rr==0 else '',max_points=200000)
        fig.suptitle(name+': user visual shortlist versus controls; XY and XZ\nIdentical filtered observations, voxel size, common bounds and height color scale')
        save(fig,name+'_shortlist','Actual existing candidate point clouds. Brackets mark human visual preference, not navigation approval; counterexamples are retained.')
        # Plot camera Z only as an observable proxy, never label span stair height.
        if name in ('bridge_12_sub','bridge_out1f','lobby'):
            fig,axes=plt.subplots(1,2,figsize=(13,5))
            for code in display:
                ordered=sorted(pp[code]);xyz=np.array([pp[code][n][:3,3] for n in ordered])
                axes[0].scatter(xyz[:,0],xyz[:,1],s=4,label=code,alpha=.65)
                axes[1].scatter(ordered,xyz[:,2],s=4,label=code,alpha=.65)
            axes[0].set(xlabel='X (m)',ylabel='Y (m)');axes[0].set_aspect('equal',adjustable='datalim')
            axes[1].set(xlabel='Stored node ID (not uniform time)',ylabel='Camera Z (m)');axes[1].legend(ncol=2,fontsize=8)
            for ax in axes:ax.grid(alpha=.2)
            fig.suptitle(name+': XY and camera-height traces; retained frames only, gaps are NOT interpolated\nCamera Z span is not a surveyed floor-to-floor or stair rise')
            save(fig,name+'_trajectory','A/B node samples without lines across rejected-frame gaps. Traces show differences, not metric truth or measured stair rise.')
    # All raw-quality summaries on one consistent set of stored rows.
    names=list(data);fig,axes=plt.subplots(1,4,figsize=(17,5));x=np.arange(len(names))
    metrics=[('usable_depth_fraction',100,'Median valid depth (%)'),('confidence_100_fraction',100,'Median high confidence (%)'),
             ('blur_laplacian_variance_640px',1,'Median sharpness proxy'),('angular_step_deg',1,'Median rotation / stored node (deg)')]
    for ax,(key,scale,title) in zip(axes,metrics):
        ax.bar(x,[quality['sessions'][n]['metric_summaries'][key]['median']*scale for n in names],color='#5888ac')
        ax.set_xticks(x,names,rotation=30,ha='right',fontsize=8);ax.set_title(title,fontsize=10);ax.grid(axis='y',alpha=.2)
    fig.suptitle('Original capture-quality rows: different scene/coverage/blur regimes\nThese are diagnostics, not calibration, angular velocity or proof of a wrong loop')
    save(fig,'raw_capture_profiles','Derived per-node raw RGB-D/pose audit rows; all nodes, before candidate choice. Some quality measures are correlated.')
    fig,axes=plt.subplots(1,3,figsize=(18,6))
    for ax,name in zip(axes,['outside','lobby','floor2']):
        codes=live[name]['codes'];values=[data[name]['candidates'][key]['fixed_patch']['p90_cm'] for key in codes]
        colors=['#b8773e' if k in data[name]['shortlist'] else '#6c8fa7' for k in codes]
        ax.bar(np.arange(len(codes)),values,color=colors);ax.set_xticks(np.arange(len(codes)),codes,rotation=30)
        ax.set(title=name,ylabel='Same finite-patch P90 (cm)');ax.grid(axis='y',alpha=.2)
    fig.suptitle('Human visual preference (brown) and fixed 3D patch measurements are different criteria\nRepeated development observations; lower here does not prove a globally correct map')
    save(fig,'visual_preference_vs_patch_p90','Controls are blue. The plot deliberately retains cases where the visually preferred method has worse local metrics.')
    # A scale-disparity panel directly exposes the J04 target/holdout tradeoff.
    fig,ax=plt.subplots(figsize=(11,5));targets=[(548,981,'wall'),(656,1055,'wall'),(642,1031,'horizontal')];x=np.arange(3)
    for offset,code in zip([-.27,-.09,.09,.27],['H00','J04','F03','N06']):
        values=[next(r['symmetric_plane_abs_median_m']*100 for r in data['floor2']['candidates'][code]['patches'] if (r['a'],r['b'],r['kind'])==key) for key in targets]
        ax.bar(x+offset,values,width=.18,label=code)
    ax.set_xticks(x,['548 / 981 wall\nJ04 training surface','656 / 1055 wall\nJ04 reserved; later development','642 / 1031 horizontal\nreused reserve diagnostic'])
    ax.set(ylabel='Finite-patch plane distance (cm)',title='floor2 J04: sharp target wall does not guarantee other walls or heights');ax.legend();ax.grid(axis='y',alpha=.2)
    save(fig,'floor2_local_tradeoff','J04 fits one local wall/floor combination; F03/N06 use different graph inputs. This is not a loss-only controlled ablation.')
    # Preserve the distinction between sources and interpretations in machine-readable output.
    normal_counts=Counter(r['session'] for r in normal['observations'])
    result={'created_utc':utc_now(),'status':'VISUAL_SHORTLIST_EXPLANATION_NOT_NEW_OPTIMIZATION','preferences':config,
        'sessions':data,'historical_joint_model':{'components':joint['components'],'relative_edges':joint['relative_edges'],
            'normal_observations_by_session':dict(normal_counts),'same_finite_plane_factors':joint['same_finite_plane_factors'],
            'variants':{key:{'solver':r['solver'],'accepted':r['accepted'],'failures':r['failures']} for key,r in joint['variants'].items()}},
        'input_hashes':hashes,'all_inputs_unchanged':all(sha256_file(Path(path))==value for path,value in hashes.items()),
        'source_database_or_default_rviz_modified':False,'winner_selected':False,
        'limitations':['Human visual shortlist is not a final operational map selection.',
            'SE3 is ambiguous and provisionally includes N05/N06; no unstated unique winner.',
            'Fixed patch and same-sensor depth tests are reused/correlated development observations, not surveyed truth.',
            'Native odometry agreement and camera height span are not physical ground truth or measured stair rise.',
            'H/N/J differ in nodes, inputs, constraints, initialization and solver models; causal attribution to algorithm alone is not established.',
            'Why a map looks better is an interpretation; exact pose equality, support, raw rows and residuals are measured evidence.']}
    if not result['all_inputs_unchanged']:raise RuntimeError('Input changed during analysis')
    write(out/'report.json',result);write(assets/'summary.json',result)
    csv_write(assets/'candidate_rows.csv',summary_rows);csv_write(assets/'pose_pair_differences.csv',pair_rows)
    csv_write(assets/'raw_quality_rows.csv',raw_rows);csv_write(assets/'fixed_patch_rows.csv',patch_rows)
    write(assets/'manifest.json',{'created_utc':utc_now(),'figures':figures,'summary_sha256':sha256_file(assets/'summary.json'),
        'csv_hashes':{p.name:sha256_file(p) for p in assets.glob('*.csv')},'source_sha256':sha256_file(Path(__file__)),
        'source_report':str((out/'report.json').resolve()),'default_map_changed':False,'synthetic':False})
    print(json.dumps({'complete':True,'figures':len(figures),'candidate_rows':len(summary_rows),'raw_quality_rows':len(raw_rows),'assets':str(assets)}))


if __name__=='__main__':main()
