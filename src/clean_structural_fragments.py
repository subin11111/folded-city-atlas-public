#!/usr/bin/env python3
"""Remove tiny leftover surface shards after finite-patch replacement.

Only display derivatives change. Existing image atlases/materials are retained;
new index accessors hide rejected triangles without reprojecting photographs.
"""
import argparse
import json
from pathlib import Path
import struct

import numpy as np
from optimize_session import sha256_file,utc_now


def filter_glb(source,target,keep):
    if target.exists():raise FileExistsError(target)
    raw=source.read_bytes();length=struct.unpack_from('<I',raw,12)[0]
    doc=json.loads(raw[20:20+length]);payload=bytearray(raw[28+length:]);offset=0;primitives=[]
    for primitive in doc['meshes'][0]['primitives']:
        if 'indices' in primitive:
            acc=doc['accessors'][primitive['indices']];view=doc['bufferViews'][acc['bufferView']]
            dtype={5125:'<u4',5123:'<u2',5121:'u1'}[acc['componentType']]
            base_indices=np.frombuffer(payload,dtype=dtype,count=acc['count'],offset=view.get('byteOffset',0)+acc.get('byteOffset',0)).astype('<u4')
            faces=len(base_indices)//3
        else:
            acc=doc['accessors'][primitive['attributes']['POSITION']];faces=acc['count']//3
            base_indices=np.arange(acc['count'],dtype='<u4')
        mask=keep[offset:offset+faces];offset+=faces
        if not len(mask) or not mask.any():continue
        if not mask.all():
            ids=base_indices.reshape(-1,3)[mask].ravel()
            payload.extend(b'\0'*(-len(payload)%4));start=len(payload);payload.extend(ids.tobytes())
            doc['bufferViews'].append({'buffer':0,'byteOffset':start,'byteLength':ids.nbytes,'target':34963})
            doc['accessors'].append({'bufferView':len(doc['bufferViews'])-1,'componentType':5125,'count':len(ids),'type':'SCALAR'})
            primitive['indices']=len(doc['accessors'])-1
        primitives.append(primitive)
    if offset!=len(keep):raise ValueError('Triangle ordering mismatch')
    doc['meshes'][0]['primitives']=primitives;doc['buffers'][0]['byteLength']=len(payload)
    encoded=json.dumps(doc,separators=(',',':')).encode();encoded+=b' '*(-len(encoded)%4)
    with target.open('wb') as stream:
        stream.write(struct.pack('<4sII',b'glTF',2,12+8+len(encoded)+8+len(payload)))
        stream.write(struct.pack('<I4s',len(encoded),b'JSON'));stream.write(encoded)
        stream.write(struct.pack('<I4s',len(payload),b'BIN\0'));stream.write(payload)
    return int(keep.sum())


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('bundle',type=Path);p.add_argument('--resume',action='store_true');a=p.parse_args()
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());prior=a.bundle/'cleanup.json'
    if prior.exists() and not a.resume:raise FileExistsError(prior)
    results=json.loads(prior.read_text())['sessions'] if prior.exists() else {}
    for n in run['sessions']:
        if n in results:continue
        folder=a.bundle/n;mesh=o3d.io.read_triangle_mesh(str(folder/'structural.ply'))
        labels,counts,areas=mesh.cluster_connected_triangles();areas=np.asarray(areas);keep=areas[np.asarray(labels)]>=.02
        before=len(mesh.triangles);filter_glb(folder/'structural_textured.glb',folder/'clean_textured.glb',keep)
        mesh.remove_triangles_by_mask(~keep);mesh.remove_unreferenced_vertices();mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(folder/'clean_structural.ply'),mesh)
        results[n]={'removed_faces':int((~keep).sum()),'removed_components':int(np.sum(areas<.02)),
            'removed_area_m2':float(areas[areas<.02].sum()),'before_faces':before,'after_faces':len(mesh.triangles),
            'minimum_component_area_m2':.02,'atlases_and_vertex_coordinates_changed':False,
            'ply_sha256':sha256_file(folder/'clean_structural.ply'),'glb_sha256':sha256_file(folder/'clean_textured.glb')}
        prior.write_text(json.dumps({'created_utc':utc_now(),'sessions':results,'source_manifest_sha256':sha256_file(a.bundle/'manifest.json'),
            'source_sha256':sha256_file(Path(__file__)),'navigation_ready':False,
            'limitation':'A real tiny isolated detail can also be removed; source surfaces and original photos remain available.'},indent=2)+'\n')
        print(json.dumps({'cleaned':n,**results[n]}),flush=True)


if __name__=='__main__':main()
