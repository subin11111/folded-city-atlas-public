"""Read-only five-session GNSS audit against the positions used by the web map.

The original selected-pose CSVs are absent from the Windows transfer. Reproduce
the documented Admin pose/gauge/placement/keyframe translation interpolation,
then verify every published photograph position. Never fit indoor GNSS to SLAM.
"""
import argparse
import base64
import csv
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import struct
import zipfile
import zlib

import numpy as np

SESSIONS = {
    'outside': '261016-234854-outside.db',
    'lobby': '261017-000622-1f.db',
    'bridge_out1f': '261017-011331-bridgeout1f.db',
    'floor2': '261017-013501-2f.db',
    'bridge_12_sub': '261017-013904-bridg12_2.db',
}
LABELS = dict(zip(SESSIONS, ['실외', '1층', '실외와 1층 연결', '2층과 메인 계단', '서브 계단']))


def stats(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    return {'n': len(a), **({k: float(v) for k, v in zip(
        ['median', 'p90', 'max', 'rmse'],
        [np.median(a), np.percentile(a, 90), a.max(), np.sqrt(np.mean(a*a))])} if len(a) else {})}


def ecef(lat, lon):
    lat, lon = np.deg2rad([lat, lon]); a = 6378137.; e2 = 6.6943799901413165e-3
    n = a / np.sqrt(1-e2*np.sin(lat)**2)
    return np.array([n*np.cos(lat)*np.cos(lon), n*np.cos(lat)*np.sin(lon), n*(1-e2)*np.sin(lat)])


def enu(lat, lon, g):
    phi, lam = np.deg2rad([g['origin_latitude_deg'], g['origin_longitude_deg']])
    rot = np.array([[-np.sin(lam), np.cos(lam), 0],
                    [-np.sin(phi)*np.cos(lam), -np.sin(phi)*np.sin(lam), np.cos(phi)]])
    return rot @ (ecef(lat, lon)-ecef(g['origin_latitude_deg'], g['origin_longitude_deg']))


def match_pose(stamp, stamps, xyz, max_gap=5.):
    if stamp < stamps[0] or stamp > stamps[-1]: return None
    j = int(np.searchsorted(stamps, stamp))
    if j == 0: return xyz[0].copy()
    if stamps[j] == stamp: return xyz[j].copy()
    if stamps[j]-stamps[j-1] > max_gap: return None
    u = (stamp-stamps[j-1])/(stamps[j]-stamps[j-1])
    return xyz[j-1]*(1-u)+xyz[j]*u


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, required=True)
    parser.add_argument('--development', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    web = Path(__file__).resolve().parents[1]
    read = lambda n: json.loads((web/'public/data'/f'{n}.json').read_text(encoding='utf-8'))
    m, photos, nav = read('map'), read('photos'), read('navigation')
    g = m['georeference']; rotation = np.asarray(g['R_map_xy_to_EN']); shift = np.asarray(g['t_map_xy_to_EN_m'])
    offset = np.asarray(m.get('manual_preview_offset_en_m', [2., -2.]))
    args.out.mkdir(parents=True, exist_ok=True)
    output = {'schema': 'gnss-slam-audit-v1', 'reference': 'web_map_pose_reconstruction_not_independent_ground_truth',
              'original_selected_pose_csv_available': False, 'gnss_altitude_used': False,
              'time_matching': 'unique GNSS timestamp to linear SLAM position interpolation; no extrapolation; max bracket 5s',
              'georeference': g, 'manual_preview_offset_en_m': offset.tolist(), 'sessions': {}}
    masks = {k: np.unpackbits(np.frombuffer(base64.b64decode(v['walkable_bits']), dtype='u1'), bitorder='little')[:math.prod(nav['shape'])].reshape(nav['shape']).astype(bool) for k, v in nav['levels'].items()}
    walk_xy = {k: np.column_stack(np.where(v))[:, ::-1]*nav['cell_m']+np.asarray(nav['origin_xy']) for k, v in masks.items()}
    # Distances use published cell centres, exactly as navigation.py constructs XY.
    def walk_distance(point, level):
        return float(np.sqrt(np.min(np.sum((walk_xy[level]-point)**2, axis=1))))
    all_rows = []
    with zipfile.ZipFile(args.development) as archive:
        zread = lambda name: json.loads(archive.read(name))
        selected = zread('docs/assets/selected-map-review/summary.json')
        renders = zread('docs/assets/final-mesh-cleanup-20261019/render_manifest.json')
        for session, name in SESSIONS.items():
            path = args.data/name
            digest_before = hashlib.sha256(path.read_bytes()).hexdigest()
            with sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True) as db:
                rows = db.execute('SELECT id,stamp,gps FROM Node ORDER BY stamp').fetchall()
                ids_blob, poses_blob = db.execute('SELECT opt_ids,opt_poses FROM Admin').fetchone()
            ids = np.frombuffer(zlib.decompress(ids_blob), '<i4')
            matrices = np.frombuffer(zlib.decompress(poses_blob), '<f4').reshape(-1, 3, 4)
            gauge = np.asarray(zread(f'docs/assets/session-portfolios/{session}/summary.json')['stages'][0]['display_gauge'])
            placement = np.asarray(selected['sessions'][session]['T_working_selected'])
            poses = {}
            for node, matrix in zip(ids, matrices):
                p = np.eye(4); p[:3] = matrix; poses[int(node)] = placement @ gauge @ p
            anchors = []
            for figure in renders['figures']:
                camera = figure['camera']; node = camera['node']
                if camera['session'] == session and node in poses:
                    anchors.append((node, np.asarray(camera['source_world_optical'])[:3, 3]-poses[node][:3, 3]))
            anchors.sort(key=lambda a:a[0])
            for node, pose in poses.items():
                if anchors:
                    pose[:3, 3] += [np.interp(node, [a[0] for a in anchors], [a[1][axis] for a in anchors]) for axis in range(3)]
            checks = [float(np.linalg.norm(np.asarray(photo['position'])-poses[photo['node']][:3, 3])) for photo in photos if photo['session']==session and photo['node'] in poses]
            if not checks or max(checks) > .001: raise ValueError(f'{session}: published photo frame mismatch {checks}')
            # Pose time is from the raw DB. Do not equate the storage-node time and GPS measurement time.
            lookup = {node: stamp for node, stamp, _ in rows}
            ordered = sorted((lookup[node], node, pose[:3,3]) for node, pose in poses.items() if node in lookup)
            stamps = np.array([v[0] for v in ordered]); xyz = np.array([v[2] for v in ordered])
            unique = {}; duplicate = 0; age_values = []; report_accuracy = []
            for node, stamp, blob in rows:
                if not blob or len(blob)!=48: continue
                t, lon, lat, alt, accuracy, bearing = struct.unpack('<6d', blob)
                if not (all(math.isfinite(v) for v in [t,lon,lat,alt,accuracy,bearing]) and t>0 and accuracy>0 and -180<=lon<=180 and -90<=lat<=90): continue
                age_values.append(stamp-t); report_accuracy.append(accuracy)
                if t in unique: duplicate += 1; continue
                en = enu(lat, lon, g); raw = rotation.T @ (en-shift-offset)
                reference = match_pose(t, stamps, xyz)
                row = {'node': node, 'node_stamp': stamp, 'gnss_stamp': t, 'age_s': stamp-t,
                       'longitude': lon, 'latitude': lat, 'altitude_raw': alt, 'reported_accuracy': accuracy,
                       'raw': raw.tolist(), 'reference': reference.tolist() if reference is not None else None}
                if reference is not None:
                    delta = raw-reference[:2]; without_offset = rotation.T @ (en-shift)-reference[:2]
                    row.update(raw_residual_m=float(np.linalg.norm(delta)), raw_delta_xy=delta.tolist(),
                               without_preview_offset_residual_m=float(np.linalg.norm(without_offset)))
                    # Expected floor comes ONLY from SLAM; this is not GNSS floor inference.
                    candidates = [(abs(reference[2]-(v['z']+1.5)),k) for k,v in nav['levels'].items()]
                    dz, level = min(candidates)
                    row['reference_level'] = level if dz <= .65 and session!='outside' else None
                    if row['reference_level']:
                        row['raw_walkable_distance_m'] = walk_distance(raw, level)
                        row['slam_walkable_distance_m'] = walk_distance(reference[:2], level)
                unique[t] = row
                all_rows.append({'session':session,**{k:v for k,v in row.items() if not isinstance(v,list)}})
            records = [unique[t] for t in sorted(unique)]
            paired = [r for r in records if r['reference'] is not None]
            deltas = np.array([r['raw_delta_xy'] for r in paired])
            bias = np.median(deltas,axis=0)
            metrics = {'nodes':len(rows), 'reference_poses':len(poses), 'valid_gps_rows':len(age_values),
                       'unique_fixes':len(records), 'duplicate_timestamp_rows':duplicate, 'paired_fixes':len(paired),
                       'unpaired_fixes':len(records)-len(paired), 'reported_accuracy_m':stats(report_accuracy),
                       'measurement_age_s':stats(age_values), 'raw_residual_m':stats([r['raw_residual_m'] for r in paired]),
                       'without_preview_offset_residual_m':stats([r['without_preview_offset_residual_m'] for r in paired]),
                       'median_delta_map_xy_m':bias.tolist(),
                       'diagnostic_after_per_session_translation_m':stats(np.linalg.norm(deltas-bias,axis=1)),
                       'per_session_translation_applied':False,
                       'raw_distance_to_walkable_m':stats([r['raw_walkable_distance_m'] for r in paired if 'raw_walkable_distance_m' in r]),
                       'reference_distance_to_walkable_m':stats([r['slam_walkable_distance_m'] for r in paired if 'slam_walkable_distance_m' in r]),
                       'published_photo_checks':len(checks), 'published_photo_max_difference_m':max(checks)}
            output['sessions'][session] = {'label': LABELS[session], 'source':name, 'sha256':digest_before,
                'metrics':metrics, 'reference_trajectory':[{'node':int(node),'t':float(t),'position':p.tolist()} for t,node,p in ordered],
                'records':records}
            # Public replay retains the measurement timestamp, not each duplicated storage-node stamp.
            replay = {'schema':'folded-city-gnss-v1', 'simulated':False, 'source':name, 'sha256':digest_before,
                      'description':'Recorded iOS OS location; unique measurement times. SLAM comparison reference is not phone navigation or independent ground truth.',
                      'session':session, 'comparison_reference':'web-published-map-pose-proxy',
                      'records':[{'timestamp':r['gnss_stamp']*1000,
                          'coords':{'latitude':r['latitude'],'longitude':r['longitude'],'accuracy':r['reported_accuracy'],
                                    'accuracyConvention':'unspecified_radius','heading':None,'speed':None},
                          'node':r['node'],'nodeTimestamp':r['node_stamp']*1000,
                          'reference':r['reference']} for r in records]}
            (web/'public/data'/f'gnss_{session}_recorded.json').write_text(json.dumps(replay,ensure_ascii=False,separators=(',',':'))+'\n',encoding='utf-8',newline='\n')
            if hashlib.sha256(path.read_bytes()).hexdigest()!=digest_before: raise ValueError('Input changed')
            print(json.dumps({'session':session,**metrics},ensure_ascii=False),flush=True)
    output['source_hashes']={'development_zip':hashlib.sha256(args.development.read_bytes()).hexdigest(),
                            'map_json':hashlib.sha256((web/'public/data/map.json').read_bytes()).hexdigest(),
                            'photos_json':hashlib.sha256((web/'public/data/photos.json').read_bytes()).hexdigest()}
    (args.out/'paired-input.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
    with (args.out/'unique-fixes.csv').open('w',encoding='utf-8-sig',newline='') as stream:
        fields=sorted({key for row in all_rows for key in row});writer=csv.DictWriter(stream,fieldnames=fields,lineterminator='\n');writer.writeheader();writer.writerows(all_rows)


if __name__ == '__main__': main()
