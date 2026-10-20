#!/usr/bin/env python3
"""Identify actual visible GLB materials at visually reviewed floor pixels."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np
import cv2
from PIL import Image
from audit_floor_material_continuity import unpack,image_hashes


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--review',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());review=json.loads((a.review/'manifest.json').read_text());known={r['new_texture_sha256']:r['id'] for r in run['floor_material_reports']};scene_records={};geometries={}
    import open3d as o3d
    scene=o3d.t.geometry.RaycastingScene()
    for name in run['sessions']:
        mesh=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'));gid=scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh));geometries[gid]=name;doc,binary=unpack(a.bundle/name/'clean_textured.glb');hashes=image_hashes(doc,binary);entries=[];cursor=0
        for primitive in doc['meshes'][0]['primitives']:
            count=doc['accessors'][primitive['indices']]['count']//3 if 'indices' in primitive else doc['accessors'][primitive['attributes']['POSITION']]['count']//3;material=doc['materials'][primitive['material']];texture=material.get('pbrMetallicRoughness',{}).get('baseColorTexture',{}).get('index');digest=hashes[doc['textures'][texture]['source']] if texture is not None else None;entries.append({'start':cursor,'stop':cursor+count,'material':primitive['material'],'image_sha256':digest,'floor_chart':known.get(digest),'vertex_colours':'COLOR_0' in primitive['attributes']});cursor+=count
        scene_records[name]=(entries,doc,binary)
    reports=[]
    for figure in review['figures']:
        view=figure['camera'];pose=np.asarray(view['world_optical']);k=np.asarray(view['K']);pixels=np.array([[180,730],[300,650],[600,650],[650,730],[400,730],[250,520],[600,520]],float);local=np.c_[(pixels[:,0]-k[0,2])/k[0,0],(pixels[:,1]-k[1,2])/k[1,1],np.ones(len(pixels))];direction=local@pose[:3,:3].T;rays=np.c_[np.broadcast_to(pose[:3,3],direction.shape),direction].astype(np.float32);hit=scene.cast_rays(o3d.core.Tensor(rays));rows=[]
        rendered=np.asarray(Image.open(a.review/f'{view["session"]}_{view["node"]}_after.png')) if (a.review/f'{view["session"]}_{view["node"]}_after.png').exists() else None
        for pixel,gid,primitive,bary in zip(pixels,hit['geometry_ids'].numpy(),hit['primitive_ids'].numpy(),hit['primitive_uvs'].numpy()):
            name=geometries.get(int(gid));entry=next((v for v in scene_records[name][0] if v['start']<=primitive<v['stop']),None) if name else None;row={'pixel':pixel.tolist(),'session':name,'triangle':int(primitive),'material':entry}
            if entry and entry['floor_chart']:
                entries,doc,binary=scene_records[name];prim=doc['meshes'][0]['primitives'][entries.index(entry)]
                def data(index,width,dtype):
                    acc=doc['accessors'][index];v=doc['bufferViews'][acc['bufferView']];offset=v.get('byteOffset',0)+acc.get('byteOffset',0);return np.ndarray((acc['count'],width),dtype=dtype,buffer=binary,offset=offset,strides=(v.get('byteStride',np.dtype(dtype).itemsize*width),np.dtype(dtype).itemsize))
                local=int(primitive)-entry['start'];ids=data(prim['indices'],1,{5125:'<u4',5123:'<u2',5121:'u1'}[doc['accessors'][prim['indices']]['componentType']]).ravel()[local*3:local*3+3] if 'indices' in prim else np.arange(local*3,local*3+3);coords=data(prim['attributes']['TEXCOORD_0'],2,'<f4')[ids];uv=(coords*np.array([1-bary.sum(),bary[0],bary[1]])[:,None]).sum(0);texture=doc['materials'][prim['material']]['pbrMetallicRoughness']['baseColorTexture']['index'];image=doc['images'][doc['textures'][texture]['source']];v=doc['bufferViews'][image['bufferView']];start=v.get('byteOffset',0);rgb=cv2.cvtColor(cv2.imdecode(np.frombuffer(binary[start:start+v['byteLength']],np.uint8),cv2.IMREAD_COLOR),cv2.COLOR_BGR2RGB);h,w=rgb.shape[:2];x=int(np.clip(uv[0]*w,0,w-1));y=int(np.clip(uv[1]*h,0,h-1));pack=np.load(Path('/home/subin/folded-city-atlas/outputs/student_union_topology_fixed_v2')/next(r['session'] for r in run['floor_material_reports'] if r['id']==entry['floor_chart'])/(entry['floor_chart']+'_mask.npz'));mh,mw=pack['mask'].shape;mx=int(np.clip(uv[0]*mw,0,mw-1));my=int(np.clip(uv[1]*mh,0,mh-1));row.update(UV=uv.tolist(),top_left_sample=rgb[y,x].tolist(),flipped_V_sample=rgb[h-1-y,x].tolist(),mask_at_UV=bool(pack['mask'][my,mx]),mask_at_flipped_V=bool(pack['mask'][mh-1-my,mx]),rendered_RGB=rendered[int(pixel[1]),int(pixel[0])].tolist() if rendered is not None else None)
            rows.append(row)
        reports.append({'view':f'{view["session"]}:{view["node"]}','pixels':rows})
    a.output.write_text(json.dumps(reports,indent=2)+'\n');print(json.dumps(reports,indent=2))


if __name__=='__main__':main()
