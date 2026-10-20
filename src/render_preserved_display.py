#!/usr/bin/env python3
"""Calibrated before/Poisson/geometry-preserving comparisons, no camera refitting."""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont

from export_rtabmap_db import load_pose_csv
from optimize_session import inverse,sha256_file,utc_now
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--assets',type=Path,required=True);a=p.parse_args()
    a.assets.mkdir(exist_ok=True,parents=True)
    if (a.assets/'manifest.json').exists():raise FileExistsError(a.assets/'manifest.json')
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());pointer=json.loads((ROOT/'config/current_map.json').read_text())
    mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text())
    renderer=o3d.visualization.rendering.OffscreenRenderer(540,720)
    renderer.scene.set_background([.06,.08,.10,1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(
        o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18);figures=[]
    for name,s in run['sessions'].items():
        dest=a.assets/name;dest.mkdir();r=selected['sessions'][name]
        poses=load_pose_csv(mp.parent/r['trajectory'])
        con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True)
        frames={label:decode_frame(con,node,r['preprocessing'],poses[node]) for label,node in s['camera_nodes'].items()};con.close()
        methods=[('Previous 3cm / 180k display',ROOT/'outputs/student_union_surface_layers_v1'/name/'photo_textured.glb',
                  ROOT/'outputs/student_union_surface_portfolio_v1'/name/'tsdf_3cm_display.ply'),
                 ('Poisson continuity',ROOT/'outputs/student_union_surface_continuity_v1'/name/'supported_poisson_textured.glb',
                  ROOT/'outputs/student_union_surface_continuity_v1'/name/'supported_poisson.ply'),
                 ('Preserved 5cm / no decimation',a.bundle/name/'preserved_textured.glb',a.bundle/name/'preserved_geometry.ply')]
        for kind in ['photo','geometry']:
            pictures={label:[] for label in frames}
            for title,glb,ply in methods:
                renderer.scene.clear_geometry()
                if kind=='photo':
                    model=o3d.io.read_triangle_model(str(glb))
                    for material in model.materials:material.shader='defaultUnlit'
                    renderer.scene.add_model('model',model)
                else:
                    mesh=o3d.io.read_triangle_mesh(str(ply));mesh.compute_vertex_normals()
                    mesh.vertex_colors=o3d.utility.Vector3dVector(np.full((len(mesh.vertices),3),.55))
                    mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultLit';mat.sRGB_color=True
                    renderer.scene.add_geometry('model',mesh,mat)
                    renderer.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.])
                for label,f in frames.items():
                    k=f['k_rgb'].copy();k[0]*=540/f['rgb'].shape[1];k[1]*=720/f['rgb'].shape[0]
                    renderer.setup_camera(k,inverse(f['world_optical']),540,720)
                    pictures[label].append((title,np.asarray(renderer.render_to_image()).copy()))
            for label,items in pictures.items():
                if kind=='photo':items=[('Original RGB',cv2.resize(frames[label]['rgb'],(540,720),interpolation=cv2.INTER_AREA))]+items
                panel=Image.new('RGB',(540*len(items),805),'white');draw=ImageDraw.Draw(panel)
                draw.text((8,8),f'{name} #{frames[label]["id"]} | {label} | fixed camera; display geometry is NOT surveyed truth',font=font,fill='black')
                for i,(title,pixels) in enumerate(items):
                    draw.text((i*540+8,45),title,font=font,fill='black');panel.paste(Image.fromarray(pixels),(i*540,80))
                path=dest/(label+'_'+kind+'.png');panel.save(path)
                figures.append({'file':str(path.relative_to(a.assets)),'sha256':sha256_file(path),'node':frames[label]['id'],'session':name,'kind':kind})
        print(json.dumps({'rendered':name}),flush=True)
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Source input changed')
    verified=0
    for n,s in run['sessions'].items():
        for relative,item in s['outputs'].items():
            if sha256_file(a.bundle/n/relative)!=item['sha256']:raise ValueError('Display file changed')
            verified+=1
    for path in a.assets.glob('*.jpg'):
        figures.append({'file':path.name,'sha256':sha256_file(path),'kind':'actual_browser_screenshot'})
    (a.assets/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'figures':figures,
        'all_original_inputs_unchanged':True,'selected_manifest_sha256':pointer['manifest_sha256'],
        'renderer_sha256':sha256_file(Path(__file__)),'display_files_verified':verified,
        'sessions':{n:{k:v for k,v in s.items() if k!='outputs'} for n,s in run['sessions'].items()},
        'depth_audit':json.loads((ROOT/'outputs/student_union_display_depth_audit_v1/report.json').read_text()),
        'navigation_ready':False,'source_map_replaced':False},indent=2)+'\n')


if __name__=='__main__':main()
