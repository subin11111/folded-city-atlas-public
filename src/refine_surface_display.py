#!/usr/bin/env python3
"""Derivative display exports: standards-correct colour and native RGB-D sheets.

Preserves the reconstruction outputs and source map. glTF vertex COLOR_0 is
linear, while the RGB-D/Open3D arrays are sRGB; only colour encoding is changed.
An additional depth-sheet model keeps each photograph on its own depth grid,
without averaging photos or welding different visits into invented surfaces.
"""
import argparse
import json
from pathlib import Path
import sqlite3
import struct

import numpy as np

from export_rtabmap_db import load_pose_csv
from optimize_session import apply, sha256_file, utc_now
from surface_model_utils import atlas_uv, decode_frame, project, unproject, write_glb

ROOT=Path(__file__).resolve().parents[1]


def srgb_to_linear(value):
    value=np.asarray(value)
    return np.where(value<=.04045,value/12.92,((value+.055)/1.055)**2.4)


def standardize_glb(source,target):
    if target.exists():
        raise FileExistsError(target)
    raw=source.read_bytes();size=struct.unpack('<I',raw[12:16])[0]
    document=json.loads(raw[20:20+size]);binary=bytearray(raw[28+size:])
    converted=set()
    for mesh in document['meshes']:
        for primitive in mesh['primitives']:
            index=primitive['attributes'].get('COLOR_0')
            if index is None or index in converted:
                continue
            acc=document['accessors'][index]
            if acc['type']!='VEC3' or acc['componentType']!=5126:
                raise ValueError('Unexpected RGB colour accessor')
            view=document['bufferViews'][acc['bufferView']]
            values=np.ndarray((acc['count'],3),dtype='<f4',buffer=binary,
                offset=view.get('byteOffset',0)+acc.get('byteOffset',0))
            values[:]=srgb_to_linear(values)
            acc['min']=values.min(axis=0).astype(float).tolist();acc['max']=values.max(axis=0).astype(float).tolist()
            converted.add(index)
    encoded=json.dumps(document,separators=(',',':')).encode();encoded+=b' '*(-len(encoded)%4)
    with target.open('wb') as stream:
        stream.write(struct.pack('<4sII',b'glTF',2,12+8+len(encoded)+8+len(binary)))
        stream.write(struct.pack('<I4s',len(encoded),b'JSON'));stream.write(encoded)
        stream.write(struct.pack('<I4s',len(binary),b'BIN\0'));stream.write(binary)
    return {'source':str(source),'source_sha256':sha256_file(source),'sha256':sha256_file(target),
            'colour_accessors_converted':len(converted),'geometry_and_atlas_pixels_changed':False}


def depth_sheet(frame,stride=4):
    depth=frame['depth'][::stride,::stride];h,w=depth.shape
    kd=frame['k_depth'].copy();kd[0]/=stride;kd[1]/=stride
    optical=unproject(depth,kd).reshape(-1,3);points=apply(optical,frame['world_optical'])
    index=np.arange(h*w).reshape(h,w)
    a=index[:-1,:-1].ravel();b=index[:-1,1:].ravel();c=index[1:,:-1].ravel();d=index[1:,1:].ravel()
    triangles=np.concatenate([np.c_[a,b,c],np.c_[b,d,c]])
    zz=optical[triangles,2]
    valid=(zz>.25).all(axis=1)&(np.ptp(zz,axis=1)<(.10+.02*zz.mean(axis=1)))
    triangles=triangles[valid]
    uv,_=project(points,frame['k_rgb'],frame['world_optical'])
    ih,iw=frame['rgb'].shape[:2]
    good=(uv[:,0]>=0)&(uv[:,0]<=iw-1)&(uv[:,1]>=0)&(uv[:,1]<=ih-1)
    triangles=triangles[good[triangles].all(axis=1)]
    face_norm=np.cross(points[triangles[:,1]]-points[triangles[:,0]],points[triangles[:,2]]-points[triangles[:,0]])
    normals=np.zeros_like(points)
    for corner in range(3):
        np.add.at(normals,triangles[:,corner],face_norm)
    normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
    return points,triangles,normals,uv


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--sessions',nargs='+');p.add_argument('--resume',action='store_true')
    a=p.parse_args();run=a.run.resolve();output=a.output.resolve()
    if output.exists() and not a.resume:
        raise FileExistsError(output)
    output.mkdir(parents=True,exist_ok=True)
    pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest']
    if sha256_file(mp)!=pointer['manifest_sha256']:
        raise ValueError('Selected map changed')
    selected=json.loads(mp.read_text());cfg=json.loads((ROOT/'config/student_union_surface_portfolio.json').read_text())
    records={}
    for name in a.sessions or selected['sessions']:
        dest=output/name
        if dest.exists():
            if not a.resume:
                raise FileExistsError(dest)
            records[name]=json.loads((dest/'manifest.json').read_text());continue
        dest.mkdir();base=run/name;summary=json.loads((base/'summary.json').read_text())
        converted={}
        for method in ['tsdf_5cm','tsdf_3cm','ball_pivoting','photo_textured']:
            converted[method]=standardize_glb(base/(method+'.glb'),dest/(method+'.glb'))
        record=selected['sessions'][name];poses=load_pose_csv(mp.parent/record['trajectory'])
        db=Path(selected['database_integrity'][name]['database']);before=sha256_file(db)
        if before!=selected['database_integrity'][name]['sha256']:
            raise ValueError('Original DB changed')
        con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
        vertices=[];triangles=[];normals=[];uvs=[];pages=[];offset=0
        for i,node in enumerate(summary['texture']['camera_frames']):
            frame=decode_frame(con,node,record['preprocessing'],poses[node]);v,t,n,uv=depth_sheet(frame)
            vertices.append(v);triangles.append(t+offset);normals.append(n)
            per_page=cfg['texture']['columns']*cfg['texture']['rows']
            uvs.append(atlas_uv(uv[t].reshape(-1,2),i%per_page,frame['rgb'].shape,
                cfg['texture']['atlas_side'],cfg['texture']['columns'],cfg['texture']['rows']).reshape(-1,3,2))
            pages.append(np.full(len(t),i//per_page,int));offset+=len(v)
        con.close()
        glb=write_glb(dest/'depth_sheet_textured.glb',np.concatenate(vertices),np.concatenate(triangles),
            np.concatenate(normals),np.ones((offset,3)),face_pages=np.concatenate(pages),face_uv=np.concatenate(uvs),
            atlas_paths=sorted(base.glob('atlas_*.png')))
        assert sha256_file(db)==before
        records[name]={'created_utc':utc_now(),'session':name,'source_surface_summary_sha256':sha256_file(base/'summary.json'),
            'colour_standardized_models':converted,'depth_sheet':glb,'camera_frames':summary['texture']['camera_frames'],
            'depth_sheet_scope':'Native-photo textured per-keyframe depth-grid sheets; no inter-visit welding. Overlaps/seams/registration errors remain.',
            'database_sha256':before,'original_database_unchanged':True,'source_map_replaced':False,'navigation_ready':False}
        (dest/'manifest.json').write_text(json.dumps(records[name],indent=2)+'\n')
        print(json.dumps({'display_layers':name,'depth_sheet_triangles':glb['triangles']}),flush=True)
    (output/'manifest.json').write_text(json.dumps({'sessions':records,'source_map_replaced':False,
        'source_sha256':sha256_file(Path(__file__)),'selected_manifest_sha256':pointer['manifest_sha256']},indent=2)+'\n')


if __name__=='__main__':
    main()
