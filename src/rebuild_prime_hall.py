#!/usr/bin/env python3
"""Photo-supported whole PRIME HALL fascia, with explicit old-fragment cleanup."""
import argparse
import json
from pathlib import Path
import shutil
import sqlite3

import cv2
import numpy as np
from PIL import Image
from scipy.spatial import cKDTree

from append_plane_glb import append_planes
from clean_structural_fragments import filter_glb
from export_rtabmap_db import load_pose_csv
from level_floor_materials import replace_floor_references
from optimize_session import apply, sha256_file, utc_now
from photo_semantic_surfaces import fitted_plane, photo_mesh
from presentation_geometry import LevelPresentation
from surface_model_utils import decode_frame, project, unproject

ROOT=Path(__file__).resolve().parents[1]


def inside_polygon(pixels,polygon):
    polygon=np.asarray(polygon,np.float32)
    return np.array([cv2.pointPolygonTest(polygon,tuple(p),False)>=0 for p in np.asarray(pixels,np.float32)])


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,default=ROOT/'config/prime_hall_photo_finish.json');parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    cfg=json.loads(args.config.read_text());base=ROOT/cfg['source_bundle'];prior=json.loads((base/'manifest.json').read_text());pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text());model=LevelPresentation(ROOT,prior['presentation_config']);args.output.mkdir(parents=True,exist_ok=False)
    import open3d as o3d
    inputs={str(p.resolve()):sha256_file(p) for p in [Path(__file__),args.config,base/'manifest.json',mp,ROOT/'src/photo_semantic_surfaces.py',ROOT/'src/append_plane_glb.py',ROOT/'src/level_floor_materials.py',ROOT/'src/clean_structural_fragments.py']}
    semantics=json.loads((base/'semantic_objects.json').read_text());sessions={};cleanup={}
    objects=[]
    # Build the owner first so every other session can remove only its own
    # overlapping observations of the same finite photographed sign band.
    owners={obj['session'] for obj in cfg['objects']}
    session_names=sorted(prior['sessions'],key=lambda name:name not in owners)
    for name in session_names:
        spec=prior['sessions'][name]
        dest=args.output/name;dest.mkdir();mesh=o3d.io.read_triangle_mesh(str(base/name/'clean_structural.ply'));mesh.compute_triangle_normals();xyz=np.asarray(mesh.vertices)[np.asarray(mesh.triangles)].astype(np.float32);centres=xyz.mean(1);normals=np.asarray(mesh.triangle_normals);keep=np.ones(len(xyz),bool);patches=[];uvs=[];textures=[];records=[]
        for annotation in [o for o in cfg['objects'] if o['session']==name]:
            session=selected['sessions'][name];poses=load_pose_csv(mp.parent/session['trajectory']);database=Path(selected['database_integrity'][name]['database']);inputs[str(database)]=sha256_file(database);inputs[str((mp.parent/session['trajectory']).resolve())]=sha256_file(mp.parent/session['trajectory'])
            with sqlite3.connect(f'file:{database}?mode=ro',uri=True) as con:frame=decode_frame(con,annotation['node'],{**session['preprocessing'],'max_depth':5.5},poses[annotation['node']])
            summary=ROOT/'outputs/student_union_continuous_selected_v4'/name/'summary.json';planes=json.loads(summary.read_text())['continuous_planes'];inputs[str(summary.resolve())]=sha256_file(summary)
            depth_points=apply(unproject(frame['depth'],frame['k_depth']).reshape(-1,3),frame['world_optical']);mask=np.zeros(frame['rgb'].shape[:2],np.uint8)
            for poly in annotation['plane_support_polygons']:cv2.fillPoly(mask,[np.asarray(poly,np.int32)],1)
            small=cv2.resize(mask,frame['depth'].shape[::-1],interpolation=cv2.INTER_NEAREST)>0;points=depth_points[(small&(frame['depth']>0)).ravel()]
            if len(points)<20:raise ValueError('Too few original fascia-context depth samples')
            vertical=[p for p in planes if abs(np.asarray(p['basis'])[2,2])<.4]
            initial=min(vertical,key=lambda p:np.median(abs((points-np.asarray(p['centre']))@np.asarray(p['basis'])[:,2])))
            cloud_path=mp.parent/session['cloud'];cloud=o3d.io.read_point_cloud(str(cloud_path));inputs[str(cloud_path.resolve())]=sha256_file(cloud_path)
            plane,fit=fitted_plane(frame,annotation,initial,planes,cKDTree(np.asarray(cloud.points)))
            if plane is None:raise ValueError('PRIME HALL plane unsupported: '+str(fit))
            built,geometry=photo_mesh(o3d,frame,annotation,plane,dest)
            if built is None:raise ValueError('PRIME HALL photo surface rejected: '+str(geometry))
            patch,uv,texture=built;patch.vertices=o3d.utility.Vector3dVector(model.warp(np.asarray(patch.vertices),name));patch.compute_vertex_normals();patches.append(patch);uvs.append(uv);textures.append(texture)
            # Test all sessions against the finite photographed band in the later pass.
            boundary=model.warp(np.asarray(geometry['world_boundary']),name);source_image=dest/(annotation['id']+'_original.png');Image.fromarray(frame['rgb']).save(source_image)
            record={**annotation,'geometry':{**geometry,'source_world_boundary':geometry['world_boundary'],'world_boundary':boundary.tolist(),'world_centre':boundary.mean(0).tolist()},'plane_fit':fit,'K':frame['k_rgb'].tolist(),'world_optical':frame['world_optical'].tolist(),'source_rgb':source_image.name,'source_RGB_sha256':sha256_file(source_image),'presentation_geometry_only':True,'passability_verified':False}
            records.append(record);objects.append(record)
        # This dataset's sign owner is floor2; apply the same finite replacement
        # to any overlapping observations in other sessions, not their whole walls.
        for record in objects:
            boundary=np.asarray(record['geometry']['world_boundary']);centre=boundary.mean(0);_,_,vt=np.linalg.svd(boundary-centre,full_matrices=False);normal=vt[-1];axis=boundary[1]-boundary[0];axis/=np.linalg.norm(axis);other=np.cross(normal,axis);poly=np.c_[(boundary-centre)@axis,(boundary-centre)@other]
            distance=abs((centres-centre)@normal);near=(distance<record['maximum_old_face_plane_distance_m'])&(abs(normals[:,2])<.65)&(abs(normals@normal)>.45);ids=np.flatnonzero(near);uv=np.c_[(centres[ids]-centre)@axis,(centres[ids]-centre)@other];clear=np.zeros(len(keep),bool);clear[ids]=inside_polygon(uv,poly);keep&=~clear
        np.save(dest/'finish_retained_source_faces.npy',keep);filter_glb(base/name/'clean_textured.glb',dest/'finish_retained.glb',keep);mesh.remove_triangles_by_mask(~keep);mesh.remove_unreferenced_vertices();retained=len(mesh.triangles)
        for patch in patches:mesh+=patch
        mesh.compute_vertex_normals();o3d.io.write_triangle_mesh(str(dest/'clean_structural.ply'),mesh);append_planes(dest/'finish_retained.glb',dest/'finish_appended.glb',patches,uvs,textures);replace_floor_references(dest/'finish_appended.glb',dest/'clean_textured.glb',{})
        for path in (base/name).iterdir():
            if path.is_file() and path.name not in ['clean_structural.ply','clean_textured.glb','retained.glb']:shutil.copyfile(path,dest/path.name)
        sessions[name]={**spec,'finish_retained_triangles':retained,'photo_finish_surfaces':records,'finish_removed_triangles':int((~keep).sum()),'bounds_min_m':mesh.get_min_bound().tolist(),'bounds_max_m':mesh.get_max_bound().tolist()};cleanup[name]={'before_faces':len(keep),'after_faces':len(mesh.triangles),'removed_faces':int((~keep).sum()),'ply_sha256':sha256_file(dest/'clean_structural.ply'),'glb_sha256':sha256_file(dest/'clean_textured.glb')}
        for p in [base/name/'clean_structural.ply',base/name/'clean_textured.glb']:inputs[str(p.resolve())]=sha256_file(p)
        print(f'Photographed sign finish {name}: {int((~keep).sum())} old fragments removed, {len(patches)} full photo band',flush=True)
    # Preserve other semantic source files and the finished roof/floor masks.
    for path in base.iterdir():
        if path.is_file() and path.name not in ['manifest.json','cleanup.json','semantic_objects.json','geometry_audit.json','material_audit.json']:shutil.copyfile(path,args.output/path.name)
    semantics['objects']+=objects;(args.output/'semantic_objects.json').write_text(json.dumps(semantics,indent=2,ensure_ascii=False)+'\n');sessions['floor2']['semantic_objects']+=objects
    manifest={**prior,'created_utc':utc_now(),'sessions':sessions,'inputs_sha256':{**prior['inputs_sha256'],**inputs},'photo_sign_finish':True,'photo_finish_source_manifest':str((base/'manifest.json').resolve()),'prime_hall_config':cfg,'scope':'Original RGB-D photo floor seams and full photographed PRIME HALL band. No new lettering, uniform floor paint or source-pose changes. Existing continuous ceilings retained; local photographed facade fragments replaced only.'}
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(args.output/'cleanup.json').write_text(json.dumps({'sessions':cleanup,'source_manifest_sha256':sha256_file(args.output/'manifest.json')},indent=2)+'\n')
    for path,digest in manifest['inputs_sha256'].items():assert sha256_file(Path(path))==digest,'Input changed: '+path


if __name__=='__main__':main()
