#!/usr/bin/env python3
"""Describe existing first-hit occluders of reviewed source-photo polygons."""
import json
from pathlib import Path
import cv2
import numpy as np
import open3d as o3d

ROOT=Path(__file__).resolve().parents[1]
objects=json.loads((ROOT/'outputs/student_union_photo_semantics_v8/semantic_objects.json').read_text())['objects']
for name in ['floor2','lobby']:
    mesh=o3d.io.read_triangle_mesh(str(ROOT/'outputs/student_union_continuous_selected_v4'/name/'clean_structural.ply'));mesh.compute_triangle_normals()
    scene=o3d.t.geometry.RaycastingScene(nthreads=2);scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    vertices=np.asarray(mesh.vertices);triangles=np.asarray(mesh.triangles);normals=np.asarray(mesh.triangle_normals)
    for o in objects:
        if o['session']!=name or o['id'] not in ['f2_mens_restroom_door','f2_mens_restroom_sign','f2_counselling_sign','f2_exit_sign','f2_fire_equipment_front']:continue
        poly=np.asarray(o['polygon'],np.float32);low=poly.min(0).astype(int);high=poly.max(0).astype(int)
        y,x=np.mgrid[low[1]:high[1]:4,low[0]:high[0]:4];uv=np.c_[x.ravel(),y.ravel()];uv=uv[[cv2.pointPolygonTest(poly,tuple(p),True)>3 for p in uv.astype(np.float32)]]
        k=np.asarray(o['K_rgb']);pose=np.asarray(o['world_optical']);direction=np.c_[(uv[:,0]-k[0,2])/k[0,0],(uv[:,1]-k[1,2])/k[1,1],np.ones(len(uv))]@pose[:3,:3].T
        origin=np.broadcast_to(pose[:3,3],direction.shape);hit=scene.cast_rays(o3d.core.Tensor(np.c_[origin,direction].astype(np.float32)))
        ids=hit['primitive_ids'].numpy();valid=ids<len(triangles);ids=ids[valid];point=origin[valid]+hit['t_hit'].numpy()[valid,None]*direction[valid]
        g=o['geometry'];n=np.asarray(g['plane_normal']);c=np.asarray(g['plane_centre'])
        print(o['id'],json.dumps({'plane_signed_distance_hit_q':np.quantile((point-c)@n,[.1,.5,.9]).tolist(),
            'normal_dot_q':np.quantile(abs(normals[ids]@n),[.1,.5,.9]).tolist(),'normal_z_q':np.quantile(abs(normals[ids,2]),[.1,.5,.9]).tolist(),
            'fit':o['plane_fit'],'world_centre':g['world_centre']},ensure_ascii=False))
