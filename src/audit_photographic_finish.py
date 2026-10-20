#!/usr/bin/env python3
"""Verify retained geometry, actual sign pixels and white-letter visibility."""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
from PIL import Image

from audit_floor_material_continuity import unpack,image_hashes
from audit_structural_delivery import glb_triangle_positions
from export_rtabmap_db import load_pose_csv
from optimize_session import sha256_file,utc_now
from photo_semantic_surfaces import lift_pixels
from presentation_geometry import LevelPresentation
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def glyph_components(rgb,polygon):
    region=np.zeros(rgb.shape[:2],np.uint8);cv2.fillPoly(region,[np.asarray(polygon,np.int32)],1)
    gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY);threshold,_=cv2.threshold(gray[region>0],0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU);mask=(gray>max(30,threshold))&(region>0);count,labels,stats,_=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    ids=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=35 and stats[i,cv2.CC_STAT_HEIGHT]>=12]
    ids.sort(key=lambda i:int(stats[i,cv2.CC_STAT_LEFT]))
    return labels,ids,stats


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--bundle',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();run=json.loads((args.bundle/'manifest.json').read_text());base=Path(run['photo_finish_source_manifest']).parent;prior=json.loads((base/'manifest.json').read_text());model=LevelPresentation(ROOT,run['presentation_config']);pointer=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text());results={}
    import open3d as o3d
    scene=o3d.t.geometry.RaycastingScene();geometry_ids={}
    for name,session in run['sessions'].items():
        old=o3d.io.read_triangle_mesh(str(base/name/'clean_structural.ply'));new=o3d.io.read_triangle_mesh(str(args.bundle/name/'clean_structural.ply'));old.compute_triangle_normals();old_xyz=np.asarray(old.vertices)[np.asarray(old.triangles)].astype(np.float32);xyz=np.asarray(new.vertices)[np.asarray(new.triangles)].astype(np.float32);keep=np.load(args.bundle/name/'finish_retained_source_faces.npy');retained=session['finish_retained_triangles'];assert np.array_equal(xyz[:retained],old_xyz[keep]);assert np.array_equal(xyz,glb_triangle_positions(args.bundle/name/'clean_textured.glb'));assert retained==int(keep.sum());assert len(xyz)-retained==sum(o['geometry']['triangles'] for o in session['photo_finish_surfaces'])
        if (~keep).any():assert (abs(np.asarray(old.triangle_normals)[~keep,2])<.65).all(),'Horizontal floor or ceiling was removed'
        before_doc,before_bin=unpack(base/name/'clean_textured.glb');after_doc,after_bin=unpack(args.bundle/name/'clean_textured.glb',strict=True);assert after_bin[:len(before_bin)]==before_bin;before_images=image_hashes(before_doc,before_bin);assert before_images==image_hashes(after_doc,after_bin)[:len(before_images)]
        geometry_ids[name]=scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(new));results[name]={'retained_XYZ_exact':True,'GLB_PLY_equal':True,'floor_ceiling_geometry_retained':True,'old_binary_and_images_preserved':True,'removed_facade_triangles':int((~keep).sum()),'added_photographic_sign_triangles':len(xyz)-retained,'ply_sha256':sha256_file(args.bundle/name/'clean_structural.ply'),'glb_sha256':sha256_file(args.bundle/name/'clean_textured.glb')}
        print('Photographic finish geometry verified: '+name,flush=True)
    signs=[]
    for name,session in run['sessions'].items():
        cursor=session['finish_retained_triangles']
        for obj in session['photo_finish_surfaces']:
            spec=selected['sessions'][name];poses=load_pose_csv(mp.parent/spec['trajectory']);database=selected['database_integrity'][name]['database']
            with sqlite3.connect(f'file:{database}?mode=ro',uri=True) as con:frame=decode_frame(con,obj['node'],{**spec['preprocessing'],'max_depth':5.5},poses[obj['node']])
            assert np.array_equal(frame['rgb'],np.asarray(Image.open(args.bundle/name/obj['source_rgb']).convert('RGB')))
            poly=np.asarray(obj['polygon']);low=np.maximum(np.floor(poly.min(0)).astype(int),0);high=np.minimum(np.ceil(poly.max(0)).astype(int),[719,959]);texture=np.asarray(Image.open(args.bundle/name/obj['geometry']['texture']).convert('RGB'));assert np.array_equal(texture,frame['rgb'][low[1]:high[1]+1,low[0]:high[0]+1]),'Sign letters were painted or altered'
            labels,ids,stats=glyph_components(frame['rgb'],obj['text_polygon']);assert len(ids)==9,f'Expected 9 reviewed PRIME HALL letter components, got {len(ids)}';letters=[];camera=model.warp(frame['world_optical'][:3,3][None],name)[0]
            for letter,label in zip('PRIMEHALL',ids):
                yy,xx=np.where(labels==label);pixels=np.c_[xx,yy];points,_=lift_pixels(pixels,frame['k_rgb'],frame['world_optical'],obj['geometry']['plane_centre'],obj['geometry']['plane_normal']);points=model.warp(points,name);rays=np.c_[np.broadcast_to(camera,points.shape),points-camera].astype(np.float32);hit=scene.cast_rays(o3d.core.Tensor(rays));g=hit['geometry_ids'].numpy();primitive=hit['primitive_ids'].numpy();visible=(g==geometry_ids[name])&(primitive>=cursor)&(primitive<cursor+obj['geometry']['triangles']);letters.append({'letter':letter,'source_pixel_bbox':stats[label,:4].tolist(),'glyph_pixels':len(pixels),'actual_sign_first_hit_fraction':float(visible.mean())})
            h=letters[5];signs.append({'id':obj['id'],'all_original_RGB_pixels_exact':True,'source_node':obj['node'],'letter_components':letters,'H_first_hit_fraction':h['actual_sign_first_hit_fraction'],'plane_fit':obj['plane_fit'],'scope':'Photo-plane glyph visibility from display-height-adjusted source camera; not surveyed positional accuracy.'});cursor+=obj['geometry']['triangles']
    for path,digest in run['inputs_sha256'].items():assert sha256_file(Path(path))==digest,'Input binding changed: '+path
    assert sha256_file(mp)==run['selected_manifest_sha256']==pointer['manifest_sha256'];dbs={name:sha256_file(Path(v['database'])) for name,v in selected['database_integrity'].items()};assert all(dbs[name]==v['sha256'] for name,v in selected['database_integrity'].items())
    result={'created_utc':utc_now(),'bundle_manifest_sha256':sha256_file(args.bundle/'manifest.json'),'sessions':results,'prime_hall':signs,'selected_map_unchanged':True,'source_database_hashes_unchanged':dbs,'input_bindings_valid':True,'audit_source_sha256':sha256_file(Path(__file__)),'scope':'Retained geometry, exact source sign crop and visible photo glyphs. No claimed surveyed accuracy or removal of all unknown surface gaps.'};args.output.write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
