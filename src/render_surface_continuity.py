#!/usr/bin/env python3
"""Actual-data before/after continuity figures with unchanged calibrated poses."""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image,ImageDraw,ImageFont

from export_rtabmap_db import load_pose_csv
from optimize_session import inverse,sha256_file,utc_now
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True)
    p.add_argument('--assets',type=Path,required=True);p.add_argument('--sessions',nargs='+')
    p.add_argument('--resume',action='store_true');a=p.parse_args()
    if a.assets.exists() and not a.resume:raise FileExistsError(a.assets)
    a.assets.mkdir(parents=True,exist_ok=True)
    import open3d as o3d
    run=json.loads((a.run/'manifest.json').read_text());pointer=json.loads((ROOT/'config/current_map.json').read_text())
    mp=ROOT/pointer['manifest'];selected=json.loads(mp.read_text())
    display_inputs={}
    for name in run['sessions']:
        path=ROOT/'outputs/student_union_surface_layers_v1'/name/'photo_textured.glb'
        display_inputs[str(path)]=sha256_file(path)
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Frozen input changed')
    renderer=o3d.visualization.rendering.OffscreenRenderer(540,720)
    renderer.scene.set_background([.055,.065,.08,1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(
        o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    records=[];font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',19)
    methods=[('Previous textured TSDF',None),('Clean / larger display budget','clean_high_budget'),
        ('Supported Poisson RGB','supported_poisson'),('Supported Poisson + photo','supported_poisson_textured')]
    for name in a.sessions or run['sessions']:
        dest=a.assets/name
        if dest.exists():
            if not a.resume:raise FileExistsError(dest)
            records+=json.loads((dest/'manifest.json').read_text())['figures'];continue
        dest.mkdir();record=selected['sessions'][name];summary=run['sessions'][name]
        poses=load_pose_csv(mp.parent/record['trajectory'])
        con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True)
        views=dict(summary['camera_nodes'])
        if name=='floor2':views['stairs']=1078
        if name=='bridge_12_sub':views['stairs']=108
        frames={label:decode_frame(con,n,record['preprocessing'],poses[n]) for label,n in views.items()};con.close()
        cloud=o3d.io.read_point_cloud(str(mp.parent/record['cloud']));bounds=cloud.get_axis_aligned_bounding_box()
        centre=bounds.get_center();extent=np.linalg.norm(bounds.get_extent());camera=centre+np.array([.65,-.9,.8])*extent
        rendered={label:[] for label in [*views,'oblique']}
        for title,method in methods:
            renderer.scene.clear_geometry()
            path=ROOT/'outputs/student_union_surface_layers_v1'/name/'photo_textured.glb' if method is None else a.run/name/(method+'.glb')
            model=o3d.io.read_triangle_model(str(path))
            for material in model.materials:material.shader='defaultUnlit'
            renderer.scene.add_model('model',model)
            for label,frame in frames.items():
                k=frame['k_rgb'].copy();k[0]*=540/frame['rgb'].shape[1];k[1]*=720/frame['rgb'].shape[0]
                renderer.setup_camera(k,inverse(frame['world_optical']),540,720)
                rendered[label].append((title,np.asarray(renderer.render_to_image()).copy()))
            renderer.setup_camera(55.,centre,camera,[0,0,1.])
            rendered['oblique'].append((title,np.asarray(renderer.render_to_image()).copy()))
        for label,panels in rendered.items():
            if label!='oblique':
                original=cv2.resize(frames[label]['rgb'],(540,720),interpolation=cv2.INTER_AREA)
                panels=[('Original RGB',original)]+panels
            sheet=Image.new('RGB',(540*3,(720+48)*2+40),'white');draw=ImageDraw.Draw(sheet)
            for i,(title,pixels) in enumerate(panels):
                x=(i%3)*540;y=40+(i//3)*768
                draw.text((x+7,y+8),title,font=font,fill='black');sheet.paste(Image.fromarray(pixels),(x,y+40))
            note=f'{name} | {label} | fixed poses / interpolation is not measured truth'
            draw.text((8,10),note,font=font,fill='black')
            path=dest/(label+'_comparison.png');sheet.save(path)
            records.append({'file':str(path.relative_to(a.assets)),'sha256':sha256_file(path),
                'session':name,'view':label,'camera_node':views.get(label),'actual_data':True})
        # Equal grey material separates actual holes from photograph seams.
        geometry={label:[] for label in [*views,'oblique']}
        for title,filename in [('Previous 180k geometry',ROOT/'outputs/student_union_surface_portfolio_v1'/name/'tsdf_3cm_display.ply'),
                ('Clean / larger budget',a.run/name/'clean_high_budget.ply'),
                ('Supported Poisson',a.run/name/'supported_poisson.ply')]:
            renderer.scene.clear_geometry();mesh=o3d.io.read_triangle_mesh(str(filename));mesh.compute_vertex_normals()
            mesh.vertex_colors=o3d.utility.Vector3dVector(np.tile([.55,.55,.55],(len(mesh.vertices),1)))
            material=o3d.visualization.rendering.MaterialRecord();material.shader='defaultLit';material.sRGB_color=True
            renderer.scene.add_geometry('geometry',mesh,material)
            renderer.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.])
            for label,frame in frames.items():
                k=frame['k_rgb'].copy();k[0]*=540/frame['rgb'].shape[1];k[1]*=720/frame['rgb'].shape[0]
                renderer.setup_camera(k,inverse(frame['world_optical']),540,720)
                geometry[label].append((title,np.asarray(renderer.render_to_image()).copy()))
            renderer.setup_camera(55.,centre,camera,[0,0,1.])
            geometry['oblique'].append((title,np.asarray(renderer.render_to_image()).copy()))
        for label,panels in geometry.items():
            sheet=Image.new('RGB',(1620,805),'white');draw=ImageDraw.Draw(sheet)
            draw.text((8,8),name+' | '+label+' | identical neutral material, fixed camera',font=font,fill='black')
            for i,(title,pixels) in enumerate(panels):
                draw.text((i*540+8,45),title,font=font,fill='black');sheet.paste(Image.fromarray(pixels),(i*540,80))
            path=dest/(label+'_geometry.png');sheet.save(path)
            records.append({'file':str(path.relative_to(a.assets)),'sha256':sha256_file(path),
                'session':name,'view':label,'camera_node':views.get(label),'actual_data':True,'equal_neutral_material':True})
        own=[r for r in records if r['session']==name]
        (dest/'manifest.json').write_text(json.dumps({'figures':own},indent=2)+'\n')
        print(json.dumps({'rendered':name,'figures':len(own)}),flush=True)
    names=list(run['sessions']);fig,axes=plt.subplots(1,2,figsize=(13,5),constrained_layout=True)
    x=np.arange(len(names));labels=['baseline_180k','clean_high_budget','supported_poisson']
    for j,method in enumerate(labels):
        axes[0].bar(x+(j-1)*.25,[run['sessions'][n][method]['topology']['components'] for n in names],width=.25,label=method)
        axes[1].bar(x+(j-1)*.25,[run['sessions'][n][method]['camera_probes']['audit']['surface_coverage_on_valid_depth']*100 for n in names],width=.25,label=method)
    for ax in axes:ax.set_xticks(x,names,rotation=20);ax.grid(axis='y',alpha=.2)
    axes[0].set(ylabel='Disconnected components (log scale)',yscale='log');axes[0].legend(fontsize=8)
    axes[1].set(ylabel='Same-capture audit depth coverage (%)',ylim=(0,100))
    fig.suptitle('Fewer fragments and more coverage do NOT establish correct geometry')
    plot=a.assets/'continuity_metrics.png';fig.savefig(plot,dpi=170);plt.close(fig)
    records.append({'file':plot.name,'sha256':sha256_file(plot),'actual_data':True})
    verified=0;geometry_checks={}
    for n,s in run['sessions'].items():
        for filename,record in s['outputs'].items():
            if sha256_file(a.run/n/filename)!=record['sha256']:raise ValueError('Reconstruction changed')
            verified+=1
        geometry_checks[n]={}
        for filename in ['clean_high_budget.ply','supported_poisson_full.ply','supported_poisson.ply']:
            mesh=o3d.io.read_triangle_mesh(str(a.run/n/filename))
            vertices=np.asarray(mesh.vertices);triangles=np.asarray(mesh.triangles)
            if not np.isfinite(vertices).all() or not len(triangles):raise ValueError('Invalid mesh geometry')
            if triangles.min()<0 or triangles.max()>=len(vertices):raise ValueError('Triangle index out of range')
            geometry_checks[n][filename]={'finite_vertices':True,'vertices':len(vertices),'triangles':len(triangles)}
    for path,h in run['inputs_sha256'].items():
        if sha256_file(Path(path))!=h:raise ValueError('Input changed after rendering')
    for path,h in display_inputs.items():
        if sha256_file(Path(path))!=h:raise ValueError('Previous display changed after rendering')
    (a.assets/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'figures':records,
        'source_manifest_sha256':sha256_file(a.run/'manifest.json'),'renderer_sha256':sha256_file(Path(__file__)),
        'all_inputs_unchanged':True,'source_map_replaced':False,'navigation_ready':False,
        'previous_display_inputs_sha256':display_inputs,
        'outputs_verified':verified,'geometry_checks':geometry_checks,
        'sessions':{n:{k:v for k,v in s.items() if k!='outputs'} for n,s in run['sessions'].items()}},indent=2)+'\n')


if __name__=='__main__':main()
