#!/usr/bin/env python3
"""Local perspective-photo roadview inventory and coarse GNSS/OSM placement.

No OSM edits/uploads, no navigation approval, no fabricated 360 photographs.
"""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np

from audit_sessions import decode_gps,horizontal_enu,valid_gps_coordinate
from export_rtabmap_db import load_pose_csv,calibration_from_blob
from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def fit_se2(source,target,accuracy,iterations=20):
    source=np.asarray(source,float);target=np.asarray(target,float);base=1/np.maximum(accuracy,3.)**2;w=base.copy()
    for _ in range(iterations):
        w=w/w.sum();x=(source*w[:,None]).sum(0);y=(target*w[:,None]).sum(0)
        u,_,vt=np.linalg.svd((source-x).T@(w[:,None]*(target-y)))
        rotation=vt.T@np.diag([1.,np.linalg.det(vt.T@u.T)])@u.T;translation=y-rotation@x
        residual=np.linalg.norm(source@rotation.T+translation-target,axis=1)
        w=base*np.minimum(1.,5./np.maximum(residual,1e-9))
    return rotation,translation,residual


def enu_to_lonlat(east,north,origin):
    lat,lon=origin;r=6378137.
    return lon+np.rad2deg(np.asarray(east)/(r*np.cos(np.deg2rad(lat)))),lat+np.rad2deg(np.asarray(north)/r)


def keyframes(poses,stamps,spacing=1.2,angle_deg=22.,maximum_gap=3.):
    kept=[];last=None
    for node,pose in sorted(poses.items()):
        if last is None:kept.append(node);last=node;continue
        delta=poses[last][:3,:3].T@pose[:3,:3]
        angle=np.rad2deg(np.arccos(np.clip((np.trace(delta)-1)/2,-1,1)))
        if np.linalg.norm(pose[:3,3]-poses[last][:3,3])>=spacing or angle>=angle_deg or stamps[node]-stamps[last]>=maximum_gap:
            kept.append(node);last=node
    if list(sorted(poses))[-1] not in kept:kept.append(list(sorted(poses))[-1])
    return kept


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    a.output.mkdir(parents=True)
    pointer_path=ROOT/'config/current_map.json';pointer=json.loads(pointer_path.read_text());mp=ROOT/pointer['manifest']
    if sha256_file(mp)!=pointer['manifest_sha256']:raise ValueError('Selected map changed')
    selected=json.loads(mp.read_text());hashes={str(p):sha256_file(p) for p in [pointer_path,mp,Path(__file__)]}
    # Only exterior fixes establish world placement. Indoor GPS is NOT used.
    outside=selected['sessions']['outside'];poses=load_pose_csv(mp.parent/outside['trajectory'])
    db=Path(selected['database_integrity']['outside']['database'])
    con=sqlite3.connect(f'file:{db}?mode=ro',uri=True);fixes=[];seen=set()
    for node,stamp,blob in con.execute('SELECT id,stamp,gps FROM Node ORDER BY id'):
        if node not in poses or not blob:continue
        gps=decode_gps(blob)
        if not valid_gps_coordinate(gps) or gps[0] in seen:continue
        seen.add(gps[0])
        if not(0<gps[4]<=25 and -.1<=stamp-gps[0]<=2.):continue
        fixes.append({'node':node,'stamp':gps[0],'lat':gps[2],'lon':gps[1],'accuracy_m':gps[4]})
    con.close()
    if len(fixes)<6:raise ValueError('Insufficient GNSS fixes even for coarse preview')
    origin=(float(np.median([f['lat'] for f in fixes])),float(np.median([f['lon'] for f in fixes])))
    xy=np.array([poses[f['node']][:2,3] for f in fixes]);enu=np.array([horizontal_enu(f['lat'],f['lon'],origin)[:2] for f in fixes])
    accuracy=np.array([f['accuracy_m'] for f in fixes]);r,t,residual=fit_se2(xy,enu,accuracy)
    rng=np.random.default_rng(20261018);angles=[]
    for _ in range(100):
        ids=rng.integers(0,len(fixes),len(fixes));rr,_,_=fit_se2(xy[ids],enu[ids],accuracy[ids],iterations=8)
        angles.append(np.arctan2(rr[1,0],rr[0,0]))
    yaw=np.arctan2(r[1,0],r[0,0]);angle_delta=np.rad2deg(np.angle(np.exp(1j*(np.array(angles)-yaw))))
    georef={'origin_latitude_deg':origin[0],'origin_longitude_deg':origin[1],'R_map_xy_to_EN':r.tolist(),
        't_map_xy_to_EN_m':t.tolist(),'scale':1.,'yaw_deg':float(np.rad2deg(yaw)),
        'unique_exterior_fixes':len(fixes),'reported_accuracy_median_m':float(np.median(accuracy)),
        'fit_residual_median_m':float(np.median(residual)),'fit_residual_p90_m':float(np.percentile(residual,90)),
        'bootstrap_yaw_delta_p05_p95_deg':np.percentile(angle_delta,[5,95]).tolist(),
        'status':'COARSE_GNSS_PREVIEW_NOT_SURVEYED','indoor_gps_used':False,'GNSS_altitude_used':False,
        'limits':'Bootstrap reuses correlated fixes and is not calibrated confidence; rigid XY placement does not prove footprint accuracy.'}
    (a.output/'georeference.json').write_text(json.dumps(georef,indent=2)+'\n')
    nodes=[];edges=[];sessions={};trajectory_features=[]
    for name,s in selected['sessions'].items():
        dest=a.output/name;dest.mkdir();(dest/'photos').mkdir();(dest/'thumbnails').mkdir()
        db=Path(selected['database_integrity'][name]['database']);pp=mp.parent/s['trajectory']
        for path in [db,pp]:hashes[str(path)]=sha256_file(path)
        pose=load_pose_csv(pp);con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
        stamps=dict(con.execute('SELECT id,stamp FROM Node'));chosen=keyframes(pose,stamps)
        own=[];all_positions=[]
        for node,p in sorted(pose.items()):
            en=r@p[:2,3]+t;lon,lat=enu_to_lonlat(*en,origin);all_positions.append([float(lon),float(lat)])
            if node not in chosen:continue
            blob,calib=con.execute('SELECT image,calibration FROM Data WHERE id=?',(node,)).fetchone()
            bgr=cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR);ih,iw=bgr.shape[:2]
            cw,ch,k,local=calibration_from_blob(calib);k=k.copy();k[0]*=iw/cw;k[1]*=ih/ch
            camera=p@local;forward=camera[:3,2];identity=f'{name}:{node}'
            photo=dest/'photos'/f'{node:05d}.jpg';thumb=dest/'thumbnails'/f'{node:05d}.jpg'
            cv2.imwrite(str(photo),bgr,[cv2.IMWRITE_JPEG_QUALITY,95])
            cv2.imwrite(str(thumb),cv2.resize(bgr,(180,240)),[cv2.IMWRITE_JPEG_QUALITY,85])
            record={'id':identity,'session':name,'node':node,'timestamp':stamps[node],
                'photo':str(photo.relative_to(a.output)),'thumbnail':str(thumb.relative_to(a.output)),
                'photo_sha256':sha256_file(photo),'width':iw,'height':ih,'K':k.tolist(),
                'world_optical':camera.tolist(),'position_map_m':camera[:3,3].tolist(),
                'heading_in_map_deg':float(np.rad2deg(np.arctan2(forward[1],forward[0]))),
                'horizontal_fov_deg':float(np.rad2deg(2*np.arctan(iw/(2*k[0,0])))),
                'longitude_deg_coarse':float(lon),'latitude_deg_coarse':float(lat),
                'image_type':'perspective_not_360','privacy_status':'LOCAL_ONLY_NOT_REDACTED_NEEDS_REVIEW'}
            if own:
                edges.append({'from':own[-1]['id'],'to':identity,'kind':'capture_sequence_not_navigation',
                    'distance_3d_m':float(np.linalg.norm(np.array(own[-1]['position_map_m'])-camera[:3,3])),
                    'bidirectional':True,'survey_validated':False})
            own.append(record);nodes.append(record)
        con.close();sessions[name]={'selected_pose_count':len(pose),'roadview_keyframes':len(own)}
        trajectory_features.append({'type':'Feature','properties':{'session':name,'georef_status':'coarse_GNSS'},
            'geometry':{'type':'LineString','coordinates':all_positions}})
        print(json.dumps({'prepared':name,'roadview_keyframes':len(own)}),flush=True)
    # Proximity alone is not enough to authorize cross-floor/session transitions.
    candidates=[]
    for i,n in enumerate(nodes):
        for m in nodes[i+1:]:
            if n['session']==m['session']:continue
            delta=np.asarray(n['position_map_m'])-m['position_map_m']
            if np.linalg.norm(delta)<1.2 and abs(delta[2])<.4:
                candidates.append({'from':n['id'],'to':m['id'],'distance_m':float(np.linalg.norm(delta)),
                    'status':'REQUIRES_VISUAL_REVIEW_NOT_TRAVERSABLE'})
    (a.output/'roadview.json').write_text(json.dumps({'nodes':nodes,'edges':edges,
        'cross_session_candidates':candidates,'coordinate_frame':'selected map, metres, optical camera +Z forward',
        'panoramas_360_available':False,'public_release_ready':False,
        'release_requirements':['Verify floor/session connectors','Face/license-plate redaction and review',
            'Survey/confirm OSM placement','Measure gaps in viewing-direction coverage; no invented 360 content']},indent=2)+'\n')
    (a.output/'trajectories.geojson').write_text(json.dumps({'type':'FeatureCollection','features':trajectory_features})+'\n')
    for path,h in hashes.items():
        if sha256_file(Path(path))!=h:raise ValueError('Original input changed')
    (a.output/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':sessions,
        'nodes':len(nodes),'sequence_edges':len(edges),'unvalidated_cross_session_candidates':len(candidates),
        'georeference':georef,'inputs_sha256':hashes,'all_original_inputs_unchanged':True,
        'source_map_replaced':False,'public_release_ready':False,'OSM_edits_or_uploads':False},indent=2)+'\n')


if __name__=='__main__':main()
