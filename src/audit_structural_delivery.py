#!/usr/bin/env python3
"""Validate actual textured GLB triangles, display PLY and unchanged source DBs."""
import argparse
import json
from pathlib import Path
import struct
import shutil

import numpy as np
from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def glb_triangle_positions(path):
    raw=path.read_bytes();length=struct.unpack_from('<I',raw,12)[0];doc=json.loads(raw[20:20+length]);start=28+length
    def accessor(index):
        a=doc['accessors'][index];v=doc['bufferViews'][a['bufferView']]
        dtype={5126:'<f4',5125:'<u4',5123:'<u2',5121:'u1'}[a['componentType']]
        width={'VEC3':3,'SCALAR':1}[a['type']]
        return np.frombuffer(raw,dtype=dtype,count=a['count']*width,offset=start+v.get('byteOffset',0)+a.get('byteOffset',0)).reshape(a['count'],width)
    result=[]
    for p in doc['meshes'][0]['primitives']:
        xyz=accessor(p['attributes']['POSITION'])
        if 'indices' in p:xyz=xyz[accessor(p['indices']).ravel()]
        result.append(xyz)
    return np.concatenate(result).reshape(-1,3,3)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--native-evidence',type=Path);a=p.parse_args()
    import open3d as o3d
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest']
    if sha256_file(mp)!=ptr['manifest_sha256']:raise ValueError('Selected map changed')
    selected=json.loads(mp.read_text());run=json.loads((a.bundle/'manifest.json').read_text());cleanup=json.loads((a.bundle/'cleanup.json').read_text())
    inputs={}
    for n,s in selected['sessions'].items():
        path=Path(selected['database_integrity'][n]['database']);h=sha256_file(path)
        if h!=selected['database_integrity'][n]['sha256']:raise ValueError('Original DB changed')
        inputs[str(path)]=h
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Source changed: '+path)
        inputs[path]=h
    results={}
    for n in selected['sessions']:
        folder=a.bundle/n;ply=folder/'clean_structural.ply';glb=folder/'clean_textured.glb'
        if sha256_file(ply)!=cleanup['sessions'][n]['ply_sha256'] or sha256_file(glb)!=cleanup['sessions'][n]['glb_sha256']:raise ValueError('Final output hash changed')
        mesh=o3d.io.read_triangle_mesh(str(ply));xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32)
        actual=glb_triangle_positions(glb)
        if not np.array_equal(xyz,actual):raise ValueError('GLB and PLY disagree: '+n)
        results[n]={'triangles':len(mesh.triangles),'GLB_PLY_triangle_coordinates_equal_float32':True,
            'glb_sha256':sha256_file(glb),'ply_sha256':sha256_file(ply)}
        geometry_key='flat_sheets' if run.get('flat_horizontal_display') else 'heightfields'
        results[n][geometry_key]=run['sessions'][n][geometry_key]
        if run.get('verified_upper_wall_completion'):
            results[n]['upper_wall_review']=run['sessions'][n]['upper_wall_review']
            results[n]['floor_occlusion_trim']=run['sessions'][n].get('floor_occlusion_trim')
        print('Verified actual geometry: '+n,flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    native=None
    if a.native_evidence:
        policy_path=ROOT/'config/structural_display_policy.json';policy=json.loads(policy_path.read_text())
        if policy['structural_manifest_sha256']!=sha256_file(a.bundle/'manifest.json'):raise ValueError('Display policy mismatch')
        loaded=json.loads((a.native_evidence/'native_loaded.json').read_text())
        expected=[]
        for n in selected['sessions']:
            path=a.bundle/n/'clean_textured.glb' if policy['sessions'][n]['method']=='structural' else ROOT/'outputs/student_union_appearance_v2'/n/'balanced_only.glb'
            expected.append(str(path.resolve()))
        if loaded['loaded_files']!=expected:raise ValueError('Actual native model selection differs from policy')
        native_image=a.native_evidence/'native_scene_render.png';target=a.output.parent/'selected_overview.png';shutil.copyfile(native_image,target)
        native={'loaded':loaded,'scene_render_sha256':sha256_file(target),'scene_render':'selected_overview.png',
            'scope':'Native scene render callback, not a desktop screenshot','selection_policy_sha256':sha256_file(policy_path)}
    a.output.write_text(json.dumps({'created_utc':utc_now(),'sessions':results,'inputs_sha256':inputs,
        'selected_manifest_sha256':ptr['manifest_sha256'],'raw_databases_unchanged':True,
        'pose_changes':0,'modeled_display_geometry_changes':True,'navigation_ready':False,
        'manifest_sha256':sha256_file(a.bundle/'manifest.json'),'cleanup_sha256':sha256_file(a.bundle/'cleanup.json'),
        'audit_source_sha256':sha256_file(Path(__file__)),'native_selected_view':native},indent=2)+'\n')


if __name__=='__main__':main()
