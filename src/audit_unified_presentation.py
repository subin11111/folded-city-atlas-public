#!/usr/bin/env python3
"""Verify delivered presentation geometry, original XY/photos, and the open atrium."""
import argparse,hashlib,json,struct
from pathlib import Path
import numpy as np
from audit_structural_delivery import glb_triangle_positions
from optimize_session import sha256_file,utc_now
from presentation_geometry import LevelPresentation

ROOT=Path(__file__).resolve().parents[1]


def embedded_images(path):
    raw=Path(path).read_bytes();length=struct.unpack_from('<I',raw,12)[0];doc=json.loads(raw[20:20+length]);binary=raw[28+length:];result=[]
    for image in doc.get('images',[]):
        view=doc['bufferViews'][image['bufferView']];start=view.get('byteOffset',0);result.append(hashlib.sha256(binary[start:start+view['byteLength']]).hexdigest())
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());base=Path(run['base_display_manifest']).parent;prior=json.loads((base/'manifest.json').read_text());model=LevelPresentation(ROOT,run['presentation_config']);atrium=np.load(a.bundle/'reviewed_atrium.npz');results={}
    import open3d as o3d
    for name,s in run['sessions'].items():
        source=o3d.io.read_triangle_mesh(str(base/name/'clean_structural.ply'));source.compute_triangle_normals();old=np.asarray(source.vertices)[np.asarray(source.triangles)].astype(np.float32);keep=np.load(a.bundle/name/'retained_source_faces.npy');after=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'));xyz=np.asarray(after.vertices)[np.asarray(after.triangles)].astype(np.float32);count=s['retained_source_triangles'];expected=model.warp(old[keep].reshape(-1,3),name).reshape(-1,3,3)
        if not np.array_equal(xyz[:count],expected):raise ValueError('Unexpected retained geometry changes: '+name)
        if not np.array_equal(xyz[:count,:,:2],old[keep,:,:2]):raise ValueError('Retained XY changed: '+name)
        if not np.array_equal(xyz,glb_triangle_positions(a.bundle/name/'clean_textured.glb')):raise ValueError('GLB/PLY differ: '+name)
        images=embedded_images(base/name/'clean_textured.glb');new_images=embedded_images(a.bundle/name/'clean_textured.glb')
        if images!=new_images[:len(images)]:raise ValueError('Original photographs changed')
        prefix=prior['sessions'][name]['retained_previous_prefix_faces'];floors=[]
        for plane in prior['sessions'][name]['flat_sheets']:
            if not plane.get('accepted'):continue
            end=prefix+plane['triangles'];source_indices=np.arange(prefix,end);remaining=source_indices[keep[prefix:end]];ranks=np.cumsum(keep)[remaining]-1;z=xyz[ranks,:,2]
            if plane['kind']=='floor' and name in ['lobby','floor2','bridge_12_sub']:
                target=model.upper if '2f' in plane['id'] else model.lower
                deviation=float(abs(z-target).max(initial=0));span=float(np.ptp(z)) if z.size else None
                if deviation>2e-6:raise ValueError('Shared floor is not flat: '+plane['id']+' '+str(deviation))
                floors.append({'id':plane['id'],'display_level_m':target,'maximum_deviation_m':deviation,'height_span_m':span,'scope':'Flat by model construction, not measured accuracy.'})
            prefix=end
        new=xyz[count:];normal=np.cross(new[:,1]-new[:,0],new[:,2]-new[:,0]) if len(new) else np.zeros((0,3));length=np.linalg.norm(normal,axis=1);valid=length>1e-9;normal[valid]/=length[valid,None];centres=new.mean(1) if len(new) else np.zeros((0,3));horizontal=(abs(normal[:,2])>.8)&(centres[:,2]<model.upper-.2);q=centres[horizontal];ij=np.floor((q[:,:2]-atrium['low'])/atrium['cell']).astype(int);x,y=ij.T;h,w=atrium['core'].shape;inside=(x>=0)&(x<w)&(y>=0)&(y<h);hits=int(atrium['core'][y[inside],x[inside]].sum())
        if hits:raise ValueError('New low ceiling covers atrium core: '+str(hits))
        results[name]={'actual_GLB_PLY_equal':True,'retained_source_XY_exact':True,'only_declared_display_Z_warp_applied':True,'original_embedded_images_unchanged':len(images),'superseded_triangles':int((~keep).sum()),'source_retained_triangles':count,'added_triangles':len(new),'floors':floors,'new_low_horizontal_faces_inside_atrium_core':hits,'ply_sha256':sha256_file(a.bundle/name/'clean_structural.ply'),'glb_sha256':sha256_file(a.bundle/name/'clean_textured.glb')}
        print('Presentation verified: '+name,flush=True)
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Source binding changed: '+path)
    pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text());databases={}
    for n,r in selected['database_integrity'].items():
        digest=sha256_file(Path(r['database']));assert digest==r['sha256'];databases[n]=digest
    result={'created_utc':utc_now(),'sessions':results,'input_bindings_valid':True,'source_selected_map_replaced':False,'selected_manifest_sha256':sha256_file(mp),'database_sha256_unchanged':databases,'bundle_manifest_sha256':sha256_file(a.bundle/'manifest.json'),'audit_source_sha256':sha256_file(Path(__file__)),'scope':'Architectural presentation, explicit modeled completion and vertical normalization. Original poses/maps retained. Not surveyed or navigation-ready.'};a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
