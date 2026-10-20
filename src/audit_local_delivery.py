#!/usr/bin/env python3
"""Verify frozen input hashes, default original-face geometry, and local photo pack."""
import argparse
import json
from pathlib import Path
import struct
import numpy as np
from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def glb_positions(path):
    raw=path.read_bytes();length=struct.unpack_from('<I',raw,12)[0]
    doc=json.loads(raw[20:20+length]);start=28+length
    attrs=doc['meshes'][0]['primitives'][0]['attributes'];acc=doc['accessors'][attrs['POSITION']]
    view=doc['bufferViews'][acc['bufferView']]
    return np.frombuffer(raw,dtype='<f4',count=acc['count']*3,offset=start+view.get('byteOffset',0)+acc.get('byteOffset',0)).reshape(-1,3)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import open3d as o3d
    ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest']
    if sha256_file(mp)!=ptr['manifest_sha256']:raise ValueError('Selected map hash mismatch')
    selected=json.loads(mp.read_text());bundle=ROOT/'outputs/student_union_appearance_v2'
    manifest=json.loads((bundle/'manifest.json').read_text());review=json.loads((bundle/'review.json').read_text())
    roadpath=ROOT/'outputs/student_union_roadview_v1';roadmanifest=json.loads((roadpath/'manifest.json').read_text())
    checked={}
    for d in [manifest['inputs_sha256'],roadmanifest['inputs_sha256']]:
        for name,h in d.items():
            if name in checked:continue
            if sha256_file(Path(name))!=h:raise ValueError('Frozen input changed: '+name)
            checked[name]=h
    sessions={}
    for n,s in selected['sessions'].items():
        raw=Path(selected['database_integrity'][n]['database'])
        if sha256_file(raw)!=selected['database_integrity'][n]['sha256']:raise ValueError('Raw DB changed')
        before=o3d.io.read_triangle_mesh(str(ROOT/'outputs/student_union_preserved_display_v1'/n/'preserved_geometry.ply'))
        xyz=np.asarray(before.vertices)[np.asarray(before.triangles)].reshape(-1,3).astype(np.float32)
        actual=glb_positions(bundle/n/'balanced_only.glb')
        if not np.array_equal(xyz,actual):raise ValueError('Default display changed original faces: '+n)
        if sha256_file(bundle/n/'balanced_only.glb')!=review['sessions'][n]['sha256']:raise ValueError('Display hash changed')
        sessions[n]={'default_original_face_coordinates_exact_at_float32':True,'default_added_faces':0,
            'raw_DB_hash_unchanged':True,'original_faces':len(before.triangles)}
    road=json.loads((roadpath/'roadview.json').read_text())
    for node in road['nodes']:
        if sha256_file(roadpath/node['photo'])!=node['photo_sha256']:raise ValueError('Roadview photo changed')
    if any(e['kind']!='capture_sequence_not_navigation' for e in road['edges']):raise ValueError('Unreviewed traversal edges')
    viewport=json.loads((ROOT/'outputs/student_union_local_view_v1/osm_viewport.json').read_text())
    a.output.parent.mkdir(parents=True,exist_ok=True)
    report={'created_utc':utc_now(),'sessions':sessions,'input_files_verified':len(checked),'inputs_sha256':checked,
        'photos_verified':len(road['nodes']),'sequence_edges_not_navigation':len(road['edges']),
        'automatic_cross_session_edges':0,'panoramas_360':False,'public_release_ready':False,
        'OSM_viewport_tiles':len(viewport['tiles']),'OSM_uploads':False,
        'selected_manifest_sha256':ptr['manifest_sha256'],
        'test_suite':'Run separately with pytest; this auditor does not execute or certify the test suite',
        'repair_review_sha256':sha256_file(bundle/'review.json'),'auditor_sha256':sha256_file(Path(__file__))}
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items() if k!='inputs_sha256'},indent=2))


if __name__=='__main__':main()
