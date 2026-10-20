#!/usr/bin/env python3
"""Continuity display constrained by frozen-map visibility, not raw sensor fitting.

Uses old full-resolution TSDF as the display reference; does not refit cameras.
Keeps the Poisson surface where it agrees, restores reference faces at missing
foreground/features, and removes new foreground occluders at sampled cameras.
"""
import argparse
import gc
import json
from pathlib import Path
import sqlite3

import numpy as np

from audit_display_depth import compare_depths,residual_stats
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from reconstruct_surface_portfolio import clean_mesh,texture_mesh
from refine_surface_display import standardize_glb
from repair_surface_fragmentation import export_rgb,topology
from surface_model_utils import decode_frame,select_frames

ROOT=Path(__file__).resolve().parents[1]


def visibility_masks(reference,candidate,tolerance=.05):
    ref=np.isfinite(reference)&(reference>.25);new=np.isfinite(candidate)&(candidate>.25)
    front=new&(~ref|(candidate<reference-tolerance))
    missing=ref&(~new|(candidate>reference+tolerance))
    return front,missing


def ray_scene(o3d,mesh):
    scene=o3d.t.geometry.RaycastingScene(nthreads=2)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh));return scene


def rays(o3d,frame):
    depth=frame['depth'];k=frame['k_depth'];y,x=np.indices(depth.shape)
    directions=np.stack([(x-k[0,2])/k[0,0],(y-k[1,2])/k[1,1],np.ones_like(x)],axis=-1)
    directions=directions@frame['world_optical'][:3,:3].T
    origin=np.broadcast_to(frame['world_optical'][:3,3],directions.shape)
    return o3d.core.Tensor(np.concatenate([origin,directions],axis=-1).astype(np.float32))


def cast(scene,tensor):
    result=scene.cast_rays(tensor)
    return result['t_hit'].numpy(),result['primitive_ids'].numpy()


def assemble(o3d,candidate,keep,reference,backup):
    first=o3d.geometry.TriangleMesh(candidate)
    first.remove_triangles_by_mask(~keep);first.remove_unreferenced_vertices()
    if backup.any():
        second=o3d.geometry.TriangleMesh(reference)
        second.remove_triangles_by_mask(~backup);second.remove_unreferenced_vertices()
        first+=second
    first.remove_degenerate_triangles();first.remove_duplicated_triangles();first.remove_unreferenced_vertices()
    first.compute_vertex_normals();return first


def fidelity_stats(ref,pred):
    valid=np.isfinite(ref)&(ref>.25);hit=np.isfinite(pred)&(pred>.25);common=valid&hit
    delta=pred[common]-ref[common]
    return {'reference_hits':int(valid.sum()),'common_hits':int(common.sum()),
        'lost_reference_hit_fraction':float(np.sum(valid&~hit)/max(valid.sum(),1)),
        'new_hits_without_reference_fraction':float(np.sum(hit&~valid)/max(hit.sum(),1)),
        'reference_depth_difference':residual_stats(delta),
        'over_5cm_on_common_fraction':float(np.mean(abs(delta)>.05)) if len(delta) else 1.}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--sessions',nargs='+');p.add_argument('--resume',action='store_true');a=p.parse_args()
    if a.output.exists() and not a.resume:raise FileExistsError(a.output)
    a.output.mkdir(parents=True,exist_ok=True)
    import open3d as o3d
    pointer_path=ROOT/'config/current_map.json';pointer=json.loads(pointer_path.read_text());mp=ROOT/pointer['manifest']
    if sha256_file(mp)!=pointer['manifest_sha256']:raise ValueError('Selected map changed')
    selected=json.loads(mp.read_text());cfg_path=ROOT/'config/student_union_surface_portfolio.json';cfg=json.loads(cfg_path.read_text())
    source_run=ROOT/'outputs/student_union_surface_portfolio_v1';repair_run=ROOT/'outputs/student_union_surface_continuity_v1'
    previous=json.loads((a.output/'manifest.json').read_text()) if a.resume else {}
    hashes=previous.get('inputs_sha256',{});records=previous.get('sessions',{})
    for path in [pointer_path,mp,cfg_path,Path(__file__),ROOT/'src/audit_display_depth.py',
        ROOT/'src/surface_model_utils.py',ROOT/'src/reconstruct_surface_portfolio.py',ROOT/'src/refine_surface_display.py']:
        h=sha256_file(path)
        if str(path) in hashes and hashes[str(path)]!=h:raise ValueError('Source changed during resume')
        hashes[str(path)]=h
    for name in a.sessions or selected['sessions']:
        if name in records:continue
        print(name+': frozen-map visibility guard',flush=True)
        out=a.output/name;out.mkdir();record=selected['sessions'][name]
        paths=[source_run/name/'tsdf_3cm.ply',repair_run/name/'supported_poisson.ply',
            mp.parent/record['trajectory'],Path(selected['database_integrity'][name]['database'])]
        for path in paths:hashes[str(path)]=sha256_file(path)
        poses=load_pose_csv(paths[2]);con=sqlite3.connect(f'file:{paths[3]}?mode=ro',uri=True)
        source_summary=json.loads((source_run/name/'summary.json').read_text())
        preferred=list(cfg['representative_frames'][name])
        if name=='floor2':preferred+=[1078]
        if name=='bridge_12_sub':preferred+=[108]
        ids=select_frames(poses,14,preferred)
        guards={n:decode_frame(con,n,record['preprocessing'],poses[n]) for n in ids}
        ref=o3d.io.read_triangle_mesh(str(paths[0]));candidate=o3d.io.read_triangle_mesh(str(paths[1]))
        scene=ray_scene(o3d,ref);ray_arrays={n:rays(o3d,f) for n,f in guards.items()}
        ref_depths={n:cast(scene,r) for n,r in ray_arrays.items()}
        del scene;gc.collect()
        keep=np.ones(len(candidate.triangles),bool);backup=np.zeros(len(ref.triangles),bool);history=[]
        for iteration in range(3):
            combined=assemble(o3d,candidate,keep,ref,backup)
            scene=ray_scene(o3d,combined);kept_ids=np.flatnonzero(keep)
            # assemble does not reorder the initial candidate triangle block.
            remove=set();restore=set()
            for node,r in ray_arrays.items():
                z,primitive=cast(scene,r);rz,rprimitive=ref_depths[node]
                front,missing=visibility_masks(rz,z)
                ids_bad=primitive[front];ids_bad=ids_bad[ids_bad<len(kept_ids)]
                remove.update(kept_ids[ids_bad].tolist())
                restore.update(rprimitive[missing].tolist())
            if remove:keep[list(remove)]=False
            if restore:backup[list(restore)]=True
            history.append({'iteration':iteration+1,'new_candidate_faces_removed':len(remove),
                'reference_faces_requested':len(restore),'remaining_candidate_faces':int(keep.sum()),
                'restored_reference_faces':int(backup.sum())})
            print(json.dumps({'session':name,**history[-1]}),flush=True)
            del scene,combined;gc.collect()
        combined=assemble(o3d,candidate,keep,ref,backup);scene=ray_scene(o3d,combined)
        guard_stats={n:fidelity_stats(ref_depths[n][0],cast(scene,r)[0]) for n,r in ray_arrays.items()}
        # Additional nearby cameras were not used to remove/restore triangles.
        audit_ids=[n for n in sorted(poses) if n not in ids]
        audit_ids=select_frames(audit_ids,6,[]) if audit_ids else []
        audit_frames={n:decode_frame(con,n,record['preprocessing'],poses[n]) for n in audit_ids}
        ref_scene=ray_scene(o3d,ref);audits={}
        for node,f in audit_frames.items():
            r=rays(o3d,f);rz,_=cast(ref_scene,r);pz,_=cast(scene,r)
            audits[node]={'display_fidelity':fidelity_stats(rz,pz),
                'raw_sensor_diagnostic':compare_depths(rz,pz,f['depth'])}
        del scene,ref_scene;gc.collect()
        o3d.io.write_triangle_mesh(str(out/'fidelity_display.ply'),combined);export_rgb(combined,out/'fidelity_display.glb')
        # Native camera frames and atlas policy identical to previous runs.
        frames={n:decode_frame(con,n,record['preprocessing'],poses[n]) for n in source_summary['texture']['camera_frames']}
        con.close();print(name+': native photo texture',flush=True)
        texture=texture_mesh(combined,frames,cfg['texture'],out);assignment=texture.pop('camera_assignment')
        np.save(out/'texture_frame_assignment.npy',np.asarray(assignment,np.int32))
        standardize_glb(out/'photo_textured.glb',out/'fidelity_textured.glb')
        summary={'session':name,'method':'Poisson continuity + frozen full TSDF visibility repair; no raw depth/camera fitting',
            'reference':'Original full-resolution 3cm TSDF using frozen selected poses',
            'camera_pose_changes':0,'guard_camera_nodes':ids,'audit_camera_nodes':audit_ids,'history':history,
            'topology':topology(combined),'guard_fidelity':guard_stats,'audit_fidelity':audits,'texture':texture,
            'navigation_ready':False,'guard_tolerance_m':.05,
            'outputs':{p.name:{'sha256':sha256_file(p),'bytes':p.stat().st_size} for p in out.iterdir() if p.is_file()}}
        (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');records[name]=summary
        for path,h in hashes.items():
            if sha256_file(Path(path))!=h:raise ValueError('Frozen input changed')
        (a.output/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':records,
            'inputs_sha256':hashes,'all_inputs_unchanged':True,'selected_manifest_sha256':pointer['manifest_sha256'],
            'source_map_replaced':False,'navigation_ready':False,
            'limitations':['Visibility tested only at listed same-capture cameras, not all viewpoints.',
                'Reference TSDF is itself an approximation of the selected point cloud.',
                'Restored original faces can reintroduce holes/fragments; no claim of perfect continuity.',
                'Does not correct existing map registration or sensor-depth disagreement.']},indent=2)+'\n')
        print(json.dumps({'session_complete':name,'triangles':len(combined.triangles)}),flush=True)
        del ref,candidate,combined,frames,guards,ref_depths;gc.collect()


if __name__=='__main__':main()
