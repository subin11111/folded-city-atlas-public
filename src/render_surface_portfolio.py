#!/usr/bin/env python3
"""Render actual model alternatives from identical calibrated cameras.

Images are reconstruction diagnostics, not AI-generated illustration. The source
view may itself supply texture, so source-view similarity is not a heldout test.
"""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from export_rtabmap_db import load_pose_csv
from optimize_session import inverse, sha256_file, utc_now
from surface_model_utils import decode_frame

ROOT = Path(__file__).resolve().parents[1]
METHODS = [('pointcloud', 'Selected point cloud'), ('tsdf_5cm', 'TSDF 5cm'),
           ('tsdf_3cm', 'TSDF 3cm'), ('ball_pivoting', 'Ball pivoting'),
           ('photo_textured', 'TSDF + photo texture'), ('depth_sheet_textured', 'RGB-D photo sheets'),
           ('hybrid', 'Texture + detail points')]


def load_representation(o3d, renderer, directory, selected_cloud, method, layers=None):
    renderer.scene.clear_geometry()
    if method=='pointcloud':
        cloud = o3d.io.read_point_cloud(str(selected_cloud))
        material = o3d.visualization.rendering.MaterialRecord(); material.shader='defaultUnlit'; material.point_size=3.;material.sRGB_color=True
        renderer.scene.add_geometry('pointcloud', cloud, material)
    else:
        filename = 'photo_textured' if method=='hybrid' else method
        model_path=(layers/(filename+'.glb')) if layers is not None else directory/(filename+'.glb')
        model = o3d.io.read_triangle_model(str(model_path))
        for material in model.materials:
            material.shader='defaultUnlit'
        renderer.scene.add_model('model', model)
        if method=='hybrid':
            path = directory/'detail_overlay.ply'
            cloud = o3d.io.read_point_cloud(str(path))
            if len(cloud.points):
                material = o3d.visualization.rendering.MaterialRecord(); material.shader='defaultUnlit'; material.point_size=2.;material.sRGB_color=True
                renderer.scene.add_geometry('detail_overlay', cloud, material)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run', type=Path, required=True)
    p.add_argument('--assets', type=Path, required=True)
    p.add_argument('--layers', type=Path, required=True)
    p.add_argument('--sessions', nargs='+')
    p.add_argument('--resume', action='store_true')
    a=p.parse_args()
    import open3d as o3d
    run=a.run.resolve(); assets=a.assets.resolve()
    if assets.exists() and not a.resume:
        raise FileExistsError(assets)
    assets.mkdir(parents=True, exist_ok=True)
    pointer=json.loads((ROOT/'config/current_map.json').read_text()); selected_path=ROOT/pointer['manifest']
    if sha256_file(selected_path)!=pointer['manifest_sha256']:
        raise ValueError('Selected map changed')
    selected=json.loads(selected_path.read_text()); cfg=json.loads((ROOT/'config/student_union_surface_portfolio.json').read_text())
    width,height=540,720
    renderer=o3d.visualization.rendering.OffscreenRenderer(width,height)
    renderer.scene.set_background([.06,.07,.085,1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(
        o3d.visualization.rendering.ColorGrading.Quality.ULTRA,
        o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    records=[]
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',20)
    for name in a.sessions or selected['sessions']:
        out=assets/name
        if out.exists():
            if not a.resume:
                raise FileExistsError(out)
            records.extend(json.loads((out/'manifest.json').read_text())['figures']); continue
        out.mkdir(); directory=run/name
        summary=json.loads((directory/'summary.json').read_text())
        session=selected['sessions'][name]
        poses=load_pose_csv(selected_path.parent/session['trajectory'])
        con=sqlite3.connect(f'file:{selected["database_integrity"][name]["database"]}?mode=ro',uri=True)
        # One source-texture camera plus one unused camera if available. Neither is
        # independent sensing, and metrics are not used to reselect camera poses.
        primary=next(n for n in cfg['representative_frames'][name] if n in poses)
        unused=sorted(set(poses)-set(summary['texture']['camera_frames']))
        probe=min(unused,key=lambda n:abs(n-primary)) if unused else primary
        views=[('source_view',primary),('unused_texture_view',probe)]
        stair_node={'floor2':1078,'bridge_12_sub':108}.get(name)
        if stair_node in poses:
            views.append(('stairs_view',stair_node))
        frames={label:decode_frame(con,node,session['preprocessing'],poses[node]) for label,node in views}
        images={label:{} for label,node in views}
        for method,title in METHODS:
            load_representation(o3d,renderer,directory,selected_path.parent/session['cloud'],method,a.layers/name)
            for label,node in views:
                frame=frames[label];k=frame['k_rgb'].copy()
                k[0]*=width/frame['rgb'].shape[1];k[1]*=height/frame['rgb'].shape[0]
                renderer.setup_camera(k,inverse(frame['world_optical']),width,height)
                rendered=np.asarray(renderer.render_to_image()).copy()
                file=out/(label+'_'+method+'.png');Image.fromarray(rendered).save(file)
                images[label][method]=rendered
                records.append({'file':str(file.relative_to(assets)), 'sha256':sha256_file(file),
                    'session':name,'method':method,'camera_node':node,'view_role':label,
                    'camera_matrix':frame['world_optical'].tolist(),'synthetic':False})
        for label,node in views:
            original=cv2.resize(frames[label]['rgb'],(width,height),interpolation=cv2.INTER_AREA)
            # Contact sheet preserves exact rendered pixels, only adding labels.
            panel=Image.new('RGB',(width*4,(height+55)*2+45),'white');draw=ImageDraw.Draw(panel)
            for col,(title,im) in enumerate([('Original photograph',original)]+
                                          [(title,images[label][method]) for method,title in METHODS]):
                x=(col%4)*width;y=45+(col//4)*(height+55)
                panel.paste(Image.fromarray(im),(x,y+40));draw.text((x+8,y+10),title,fill='black',font=font)
            note=f'{name} #{node} | {label} | identical camera / no mesh pose fitting | source RGB is not independent truth'
            draw.text((8,10),note,fill='black',font=font);file=out/(label+'_comparison.png');panel.save(file)
            records.append({'file':str(file.relative_to(assets)), 'sha256':sha256_file(file),
                            'session':name,'view_role':label,'camera_node':node,'synthetic':False})
        # A consistent 3D oblique view across methods to expose holes and extra surfaces.
        cloud=o3d.io.read_point_cloud(str(selected_path.parent/session['cloud']))
        bounds=cloud.get_axis_aligned_bounding_box();center=bounds.get_center();extent=np.linalg.norm(bounds.get_extent())
        oblique=[]
        for method,title in METHODS:
            load_representation(o3d,renderer,directory,selected_path.parent/session['cloud'],method,a.layers/name)
            renderer.setup_camera(55.,center,center+np.array([.65,-.9,.8])*extent,[0.,0.,1.])
            im=np.asarray(renderer.render_to_image()).copy();oblique.append((title,im))
        panel=Image.new('RGB',(width*4,(height+55)*2+45),'white');draw=ImageDraw.Draw(panel)
        for col,(title,im) in enumerate(oblique):
            x=(col%4)*width;y=45+(col//4)*(height+55)
            panel.paste(Image.fromarray(im),(x,y+40));draw.text((x+8,y+10),title,fill='black',font=font)
        draw.text((8,10),name+' | same oblique camera; original selected world frame',fill='black',font=font)
        file=out/'oblique_comparison.png';panel.save(file)
        records.append({'file':str(file.relative_to(assets)),'sha256':sha256_file(file),'session':name,'view_role':'oblique','synthetic':False})
        con.close()
        (out/'manifest.json').write_text(json.dumps({'figures':[r for r in records if r['session']==name]},indent=2)+'\n')
        print(json.dumps({'rendered':name,'calibrated_cameras':views,'figures':len([r for r in records if r['session']==name])}),flush=True)
    all_summaries={n:json.loads((run/n/'summary.json').read_text()) for n in selected['sessions'] if (run/n/'summary.json').exists()}
    report={'created_utc':utc_now(),'source_run':str(run),'display_layers':str(a.layers.resolve()),
        'source_sha256':sha256_file(Path(__file__)),'selected_manifest_sha256':pointer['manifest_sha256'],
        'figures':records,'sessions':all_summaries,'source_map_replaced':False,'navigation_ready':False,
        'render_settings':{'width':width,'height':height,'source_RGB_resized_for_contact_sheet_only':True,
            'texture_atlas_uses_native_RGB':True,'tone_mapping':'LINEAR','shader':'defaultUnlit',
            'point_sizes_px':{'reference':3.,'detail':2.},'point_colours_are_sRGB':True,
            'vertex_colour_check':'Synthetic sRGB=0.5 patch: flag false rendered188, flag true126; point flag true used for fair colour comparison.'},
        'limitations':['Source view is an upper-bound appearance check, not heldout.',
            'Unused-texture view is same capture/pose chain, not fresh independent accuracy.',
            'No OCR or surveyed dimensions; legibility remains a human comparison.',
            'Quadric simplification applies to display copies, full-resolution surface PLYs remain unchanged.']}
    (assets/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    del renderer
    print(json.dumps({'complete':True,'figures':len(records),'assets':str(assets)}),flush=True)


if __name__=='__main__':
    main()
