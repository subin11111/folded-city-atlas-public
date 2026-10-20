#!/usr/bin/env python3
"""Trim only modeled floor cells contradicted by two reviewed opaque-wall photos."""
import argparse,json,shutil
from pathlib import Path
import numpy as np
from audit_structural_delivery import glb_triangle_positions
from clean_structural_fragments import filter_glb
from optimize_session import sha256_file,utc_now


def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--audit',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run=json.loads((a.bundle/'manifest.json').read_text());audit=json.loads(a.audit.read_text())
    if audit['source_manifest_sha256']!=sha256_file(a.bundle/'manifest.json'):raise ValueError('Audit targets a different wall candidate')
    a.output.mkdir(parents=True,exist_ok=False);source_manifest=a.bundle/'manifest.json';run['pre_occlusion_manifest']=str(source_manifest.resolve());run['modeled_floor_occlusion_trim']=audit;cleanup={}
    import open3d as o3d
    for name,s in run['sessions'].items():
        dest=a.output/name;dest.mkdir();before=o3d.io.read_triangle_mesh(str(a.bundle/name/'clean_structural.ply'));xyz=np.asarray(before.vertices)[np.asarray(before.triangles)];keep=np.ones(len(xyz),bool);cursor=s['retained_previous_prefix_faces'];regions=[]
        for plane in s['flat_sheets']:
            if not plane.get('accepted'):continue
            if plane['kind']!='floor':raise ValueError('Unexpected accepted global ceiling')
            count=plane['triangles'];pack=np.load(Path(s['source_mesh']).parent/plane['mask_file']);cell=float(pack['cell']);ij=np.floor((xyz[cursor:cursor+count].mean(1)[:,:2]-pack['low'])/cell).astype(int);mask=pack['mask'].copy();removed_cells=0
            for entry in audit['two_view_contradicted_modeled_floor_cells']:
                if entry['session']!=name or entry['plane']!=plane['id']:continue
                if len(set(entry['original_photo_nodes']))<2:raise ValueError('Cell lacks two distinct source photographs')
                selected=(ij[:,0]==entry['x'])&(ij[:,1]==entry['y']);keep[cursor:cursor+count]&=~selected
                if selected.any():mask[entry['y'],entry['x']]=False;removed_cells+=1
            removed_triangles=int((~keep[cursor:cursor+count]).sum());regions.append({'id':plane['id'],'removed_modeled_cells':removed_cells,'removed_modeled_triangles':removed_triangles,'removed_area_xy_m2':removed_cells*cell*cell})
            plane['triangles']-=removed_triangles;plane['area_xy_m2']=float(mask.sum()*cell*cell)
            plane['model_inferred_area_m2']=float((mask&~pack['observed']).sum()*cell*cell)
            data={k:pack[k] for k in pack.files};data['mask']=mask;np.savez_compressed(dest/plane['mask_file'],**data);cursor+=count
        # Floor-face indices follow the unchanged original vertical/semantic
        # prefix. No source stair, wall or original TSDF surface is removed.
        if not keep[:s['retained_previous_prefix_faces']].all():raise ValueError('A retained wall/stair/semantic face was removed')
        removed=int((~keep).sum());np.save(dest/'retained_pre_occlusion_faces.npy',keep);filter_glb(a.bundle/name/'clean_textured.glb',dest/'clean_textured.glb',keep)
        before.remove_triangles_by_mask(~keep);before.remove_unreferenced_vertices();o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'),before)
        actual=np.asarray(before.vertices)[np.asarray(before.triangles)].astype(np.float32)
        if not np.array_equal(actual,xyz[keep].astype(np.float32)) or not np.array_equal(actual,glb_triangle_positions(dest/'clean_textured.glb')):raise ValueError('Coordinate changes during floor trim')
        # Source photos and texture crops remain bit-for-bit; GLB embeds the
        # actual materials, these files are explicit provenance evidence.
        for path in (a.bundle/name).glob('*.png'):shutil.copyfile(path,dest/path.name)
        for path in (a.bundle/name).glob('*_profile.npz'):shutil.copyfile(path,dest/path.name)
        s['original_source_faces']-=removed;s['floor_occlusion_trim']={'removed_modeled_floor_triangles':removed,'planes':regions,'all_other_geometry_coordinates_unchanged':True}
        (dest/'summary.json').write_text(json.dumps(s,indent=2)+'\n');cleanup[name]={'removed_faces':removed,'before_faces':len(keep),'after_faces':len(actual),'ply_sha256':sha256_file(dest/'clean_structural.ply'),'glb_sha256':sha256_file(dest/'clean_textured.glb')}
        for path in [a.bundle/name/'clean_structural.ply',a.bundle/name/'clean_textured.glb']:
            path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
        print(name+' / trimmed '+str(removed)+' contradicted modeled-floor triangles',flush=True)
    for path in [Path(__file__),source_manifest,a.audit,Path(__file__).parent/'audit_restroom_floor_occlusion.py']:
        path=path.resolve();run['inputs_sha256'][str(path)]=sha256_file(path)
    run['created_utc']=utc_now();(a.output/'manifest.json').write_text(json.dumps(run,indent=2)+'\n');shutil.copyfile(a.bundle/'semantic_objects.json',a.output/'semantic_objects.json')
    (a.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(a.output/'manifest.json'),'source_sha256':sha256_file(Path(__file__))},indent=2)+'\n')
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input changed: '+path)


if __name__=='__main__':main()
