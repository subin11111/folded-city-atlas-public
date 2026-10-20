#!/usr/bin/env python3
"""Withdraw modeled ceilings; restore only unchanged depth-based TSDF faces.

No new height estimate, footprint completion, wall extension or ceiling plane is
created. Existing floors, walls, stairs and mapped vertical semantics stay exact.
"""
import argparse,copy,json,shutil,struct
from pathlib import Path
import numpy as np
from audit_continuous_planes import face_keys
from audit_structural_delivery import glb_triangle_positions
from clean_structural_fragments import filter_glb
from optimize_session import sha256_file,utc_now

ROOT=Path(__file__).resolve().parents[1]


def retained_without_modeled_ceilings(count,prefix,planes):
    keep=np.ones(count,bool);cursor=prefix
    for plane in planes:
        if not plane.get('accepted'):continue
        stop=cursor+plane['triangles']
        if plane['kind']=='ceiling':keep[cursor:stop]=False
        cursor=stop
    if cursor!=count:raise ValueError('Unexpected source triangle layout')
    return keep


def merge_glbs(first,second,target):
    """Join two local single-mesh GLBs without changing triangles or textures."""
    def read(path):
        raw=path.read_bytes();length=struct.unpack_from('<I',raw,12)[0];return json.loads(raw[20:20+length]),bytearray(raw[28+length:])
    if target.exists():raise FileExistsError(target)
    doc,payload=read(first);other,extra=read(second)
    if len(doc['meshes'])!=1 or len(other['meshes'])!=1:raise ValueError('Only standardized single-mesh GLBs are supported')
    offsets={k:len(doc.get(k,[])) for k in ['bufferViews','accessors','images','samplers','textures','materials']}
    payload.extend(b'\0'*(-len(payload)%4));byte_offset=len(payload);payload.extend(extra)
    for key in offsets:doc.setdefault(key,[])
    for v in other['bufferViews']:
        v=copy.deepcopy(v);v['buffer']=0;v['byteOffset']=v.get('byteOffset',0)+byte_offset;doc['bufferViews'].append(v)
    for v in other['accessors']:
        v=copy.deepcopy(v)
        if 'sparse' in v:raise ValueError('Sparse accessor is not supported')
        v['bufferView']+=offsets['bufferViews'];doc['accessors'].append(v)
    for v in other.get('images',[]):
        v=copy.deepcopy(v);v['bufferView']+=offsets['bufferViews'];doc['images'].append(v)
    doc['samplers']+=copy.deepcopy(other.get('samplers',[]))
    for v in other.get('textures',[]):
        v=copy.deepcopy(v);v['source']+=offsets['images']
        if 'sampler' in v:v['sampler']+=offsets['samplers']
        doc['textures'].append(v)
    def remap_textures(value):
        if isinstance(value,dict):
            for k,v in value.items():
                if k.endswith('Texture') and isinstance(v,dict) and 'index' in v:v['index']+=offsets['textures']
                else:remap_textures(v)
        elif isinstance(value,list):
            for v in value:remap_textures(v)
    for v in other.get('materials',[]):
        v=copy.deepcopy(v);remap_textures(v);doc['materials'].append(v)
    for v in other['meshes'][0]['primitives']:
        v=copy.deepcopy(v);v['attributes']={k:i+offsets['accessors'] for k,i in v['attributes'].items()}
        if 'indices' in v:v['indices']+=offsets['accessors']
        if 'material' in v:v['material']+=offsets['materials']
        doc['meshes'][0]['primitives'].append(v)
    doc['extensionsUsed']=sorted(set(doc.get('extensionsUsed',[])+other.get('extensionsUsed',[])));doc['buffers'][0]['byteLength']=len(payload)
    encoded=json.dumps(doc,separators=(',',':')).encode();encoded+=b' '*(-len(encoded)%4)
    with target.open('wb') as stream:
        stream.write(struct.pack('<4sII',b'glTF',2,12+8+len(encoded)+8+len(payload)));stream.write(struct.pack('<I4s',len(encoded),b'JSON'));stream.write(encoded)
        stream.write(struct.pack('<I4s',len(payload),b'BIN\0'));stream.write(payload)


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=ROOT/'outputs/student_union_topology_fixed_v2');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    import open3d as o3d
    source=json.loads((a.source/'manifest.json').read_text());a.output.mkdir(parents=True,exist_ok=False)
    run={**source,'sessions':{},'inputs_sha256':dict(source['inputs_sha256']),'inferred_ceilings_disabled':True,
        'base_display_manifest':str((a.source/'manifest.json').resolve()),'base_display_policy':{'sessions':{n:{'method':'structural'} for n in source['sessions']}},
        'pose_changes':0,'scope':'User rejected arbitrary ceiling heights. No modeled ceiling completion; only existing depth-based TSDF face coordinates restored. Floors/walls/stairs/vertical semantics unchanged.'}
    for path in [Path(__file__),a.source/'manifest.json',ROOT/'src/clean_structural_fragments.py',ROOT/'outputs/student_union_preserved_display_v1/manifest.json']:
        path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
    semantics=json.loads((a.source/'semantic_objects.json').read_text());cleanup={};audit={}
    for name,s in source['sessions'].items():
        dest=a.output/name;dest.mkdir();folder=a.source/name;old=o3d.io.read_triangle_mesh(str(folder/'clean_structural.ply'))
        before=np.asarray(old.vertices)[np.asarray(old.triangles)].astype(np.float32);prefix=s['retained_previous_prefix_faces'];planes=s['flat_sheets']
        keep=retained_without_modeled_ceilings(len(before),prefix,planes);np.save(dest/'retained_previous_faces.npy',keep)
        filter_glb(folder/'clean_textured.glb',dest/'retained.glb',keep);old.remove_triangles_by_mask(~keep);old.remove_unreferenced_vertices()
        raw_ply=ROOT/'outputs/student_union_preserved_display_v1'/name/'preserved_geometry.ply';raw_glb=ROOT/'outputs/student_union_appearance_v2'/name/'balanced_only.glb'
        raw=o3d.io.read_triangle_mesh(str(raw_ply));raw.compute_triangle_normals();xyz=np.asarray(raw.vertices)[np.asarray(raw.triangles)].astype(np.float32)
        if not np.array_equal(xyz,glb_triangle_positions(raw_glb)):raise ValueError('Observed PLY/GLB mismatch')
        centres=xyz.mean(1);band=np.zeros(len(xyz),bool)
        for plane in planes:
            if plane['kind']=='ceiling':band|=(centres[:,2]>=plane['height_band_m'][0])&(centres[:,2]<plane['height_band_m'][1])
        # A band only selects a subset of EXISTING vertices for restoration;
        # it never supplies a height, extrapolation, new boundary or new face.
        restore=band&(abs(np.asarray(raw.triangle_normals)[:,2])>.65)&~np.isin(face_keys(raw),face_keys(old))
        np.save(dest/'restored_observed_faces.npy',restore)
        combined=o3d.geometry.TriangleMesh(old)
        if restore.any():
            filter_glb(raw_glb,dest/'restored_observed.glb',restore);merge_glbs(dest/'retained.glb',dest/'restored_observed.glb',dest/'clean_textured.glb')
            raw.remove_triangles_by_mask(~restore);raw.remove_unreferenced_vertices();combined+=raw
        else:shutil.copyfile(dest/'retained.glb',dest/'clean_textured.glb')
        combined.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'),combined)
        actual=np.asarray(combined.vertices)[np.asarray(combined.triangles)].astype(np.float32)
        if not np.array_equal(actual,glb_triangle_positions(dest/'clean_textured.glb')):raise ValueError('Actual delivered GLB/PLY mismatch')
        if not np.array_equal(actual[:keep.sum()],before[keep]) or not np.array_equal(actual[keep.sum():],xyz[restore]):raise ValueError('Geometry or restoration moved')
        objects=[o for o in semantics['objects'] if o['session']==name]
        for obj in objects:
            for file in [obj.get('source_rgb'),obj.get('geometry',{}).get('texture')]:
                if file and (folder/file).exists():shutil.copyfile(folder/file,dest/file)
            if obj.get('surface_added') and obj['class'] not in ['floor','ceiling']:
                ids=np.asarray(obj['current_triangle_indices'])
                if ids.max()>=prefix or not np.array_equal(actual[ids],before[ids]):raise ValueError('Vertical semantics moved')
            if sha256_file(dest/obj['source_rgb'])!=obj['source_rgb_sha256']:raise ValueError('Source RGB changed')
        reports=[{**p0,'accepted':False,'withdrawn_by_user_request':True,'reason':'No inferred ceiling height or unsupported completion permitted'} if p0['kind']=='ceiling' else p0 for p0 in planes]
        for p0 in planes:
            if p0.get('accepted') and p0['kind']=='floor':shutil.copyfile(folder/p0['mask_file'],dest/p0['mask_file'])
        for path in [raw_ply,raw_glb,folder/'clean_structural.ply',folder/'clean_textured.glb']:
            path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
        s={**s,'flat_sheets':reports,'source_mesh':str((folder/'clean_structural.ply').resolve()),'source_texture':str((folder/'clean_textured.glb').resolve()),
            'inferred_ceiling_faces':0,'removed_modeled_ceiling_faces':int((~keep).sum()),'observed_ceiling_restored_faces':int(restore.sum()),
            'observed_ceiling_source_ply':str(raw_ply),'observed_ceiling_source_glb':str(raw_glb),'unchanged_retained_faces':int(keep.sum())}
        run['sessions'][name]=s;(dest/'summary.json').write_text(json.dumps(s,indent=2)+'\n')
        audit[name]={'new_inferred_ceiling_faces':0,'all_nonwithdrawn_coordinates_and_textures_retained':True,
            'all_original_walls_and_stair_prefix_coordinates_exact':True,'original_prefix_faces':prefix,
            'removed_modeled_ceiling_faces':int((~keep).sum()),'restored_observed_TSDF_faces':int(restore.sum()),
            'restoration_height_change_m':0.,'new_geometry_not_in_either_source_faces':0,'GLB_PLY_exact_float32':True}
        cleanup[name]={'removed_faces':int((~keep).sum()),'before_faces':len(before),'after_faces':len(actual),
            'ply_sha256':sha256_file(dest/'clean_structural.ply'),'glb_sha256':sha256_file(dest/'clean_textured.glb')}
        print(name+' / withdrew '+str((~keep).sum())+' modeled ceiling faces / restored '+str(restore.sum())+' unchanged observed faces',flush=True)
    run['created_utc']=utc_now();(a.output/'manifest.json').write_text(json.dumps(run,indent=2)+'\n');(a.output/'semantic_objects.json').write_text(json.dumps(semantics,indent=2,ensure_ascii=False)+'\n')
    (a.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.output/'manifest.json'),'source_sha256':sha256_file(Path(__file__))},indent=2)+'\n')
    (a.output/'ceiling_withdrawal_audit.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':audit,'pose_changes':0,'new_ceiling_planes':0,
        'scope':'Only unchanged existing TSDF ceiling-band faces restored; original depth-based reconstruction is not sensor truth or independent surveyed accuracy.',
        'manifest_sha256':sha256_file(a.output/'manifest.json'),'source_sha256':sha256_file(Path(__file__))},indent=2)+'\n')
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input changed: '+path)


if __name__=='__main__':main()
