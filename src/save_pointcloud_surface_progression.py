#!/usr/bin/env python3
"""Save real point-cloud -> geometry -> photo-texture figures at fixed cameras.

This records the existing selected map and display bundle, not new optimization.
No GUI window is closed or changed and no existing figures are overwritten.
"""
import argparse
import io
import json
from pathlib import Path
import sqlite3

import numpy as np
from PIL import Image,ImageDraw,ImageFont

from export_rtabmap_db import load_pose_csv
from optimize_session import inverse,sha256_file,utc_now
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]
STAGES=[('pointcloud','1. 최적화된 점군'),('surface','2. 표면 메시 · 회색 기하'),
        ('textured','3. 원본 사진 텍스처')]


def contact_sheet(images,titles,note,font):
    width,height=images[0].size
    if any(im.size!=(width,height) for im in images):raise ValueError('Matched image sizes required')
    panel=Image.new('RGB',(width*len(images),height+90),'white');draw=ImageDraw.Draw(panel)
    draw.text((12,10),note,font=font,fill='#243244')
    for index,(im,title) in enumerate(zip(images,titles)):
        draw.text((index*width+12,49),title,font=font,fill='#111827');panel.paste(im,(index*width,90))
    return panel


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--assets',type=Path,required=True)
    a=p.parse_args()
    if a.assets.exists():raise FileExistsError(a.assets)
    a.assets.mkdir(parents=True)
    import open3d as o3d
    pointer_path=ROOT/'config/current_map.json';pointer=json.loads(pointer_path.read_text());mp=ROOT/pointer['manifest']
    display_path=ROOT/'config/current_display.json';display=json.loads(display_path.read_text());dp=ROOT/display['manifest']
    if sha256_file(mp)!=pointer['manifest_sha256'] or sha256_file(dp)!=display['manifest_sha256']:
        raise ValueError('Selected map/display manifest changed')
    selected=json.loads(mp.read_text());preserved=json.loads(dp.read_text());bundle=dp.parent
    if preserved['selected_manifest_sha256']!=pointer['manifest_sha256']:raise ValueError('Map/display version differs')
    hashes={str(path):sha256_file(path) for path in [pointer_path,mp,display_path,dp,Path(__file__),
        ROOT/'src/surface_model_utils.py',ROOT/'src/export_rtabmap_db.py']}
    names=list(selected['sessions']);figures=[];cameras={}
    font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',20)
    korean={'all':'전체 통합 지도','outside':'외부','lobby':'1층 로비','bridge_out1f':'외부–로비 브리지',
        'floor2':'2층·메인 계단','bridge_12_sub':'서브 계단'}
    def renderer(width,height):
        r=o3d.visualization.rendering.OffscreenRenderer(width,height)
        r.scene.set_background([.065,.08,.105,1.])
        r.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(
            o3d.visualization.rendering.ColorGrading.Quality.ULTRA,
            o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
        return r
    # One renderer avoids Open3D/Filament resource teardown conflicts between
    # multiple offscreen contexts. All saved stages have the same resolution.
    overview=renderer(900,720);calibrated=overview
    def save(image,path,**metadata):
        if path.exists():raise FileExistsError(path)
        image.save(path);Image.open(path).verify()
        figures.append({'file':str(path.relative_to(a.assets)),'sha256':sha256_file(path),**metadata})
    def scene(r,scope,stage):
        r.scene.clear_geometry();targets=names if scope=='all' else [scope]
        if stage=='pointcloud':
            paths=[mp.parent/selected['combined_cloud']] if scope=='all' else [mp.parent/selected['sessions'][scope]['cloud']]
            for index,path in enumerate(paths):
                hashes.setdefault(str(path),sha256_file(path))
                pc=o3d.io.read_point_cloud(str(path));mat=o3d.visualization.rendering.MaterialRecord()
                mat.shader='defaultUnlit';mat.point_size=3.;mat.sRGB_color=True
                r.scene.add_geometry('cloud_'+str(index),pc,mat)
        else:
            for n in targets:
                path=bundle/n/('preserved_geometry.ply' if stage=='surface' else 'preserved_textured.glb')
                h=sha256_file(path)
                if h!=preserved['sessions'][n]['outputs'][path.name]['sha256']:raise ValueError('Display file changed')
                hashes[str(path)]=h
                if stage=='surface':
                    mesh=o3d.io.read_triangle_mesh(str(path));mesh.compute_vertex_normals()
                    mesh.vertex_colors=o3d.utility.Vector3dVector(np.full((len(mesh.vertices),3),.55))
                    mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultLit';mat.sRGB_color=True
                    r.scene.add_geometry(n,mesh,mat)
                    r.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.])
                else:
                    model=o3d.io.read_triangle_model(str(path))
                    for material in model.materials:material.shader='defaultUnlit'
                    r.scene.add_model(n,model)
    # Camera fitting comes only from the frozen selected cloud, shared by all stages.
    for scope in ['all',*names]:
        out=a.assets/scope;out.mkdir();targets=names if scope=='all' else [scope]
        lows=[selected['sessions'][n]['bounds_min_m'] for n in targets]
        highs=[selected['sessions'][n]['bounds_max_m'] for n in targets]
        low=np.min(lows,axis=0);high=np.max(highs,axis=0);centre=(low+high)/2
        extent=float(np.linalg.norm(high-low))
        views={'oblique':(centre+np.array([.7,-.9,.8])*extent*.62,[0,0,1])}
        if scope=='all':views['top']=(centre+np.array([0.,0.,.85])*extent,[0,1,0])
        overview_images={view:[] for view in views}
        source_frames={}
        if scope!='all':
            record=selected['sessions'][scope];pose_path=mp.parent/record['trajectory'];hashes[str(pose_path)]=sha256_file(pose_path)
            poses=load_pose_csv(pose_path);db=Path(selected['database_integrity'][scope]['database'])
            hashes[str(db)]=sha256_file(db)
            if hashes[str(db)]!=selected['database_integrity'][scope]['sha256']:raise ValueError('Original DB changed')
            con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
            ids={'source':preserved['sessions'][scope]['camera_nodes']['source']}
            if scope in ['floor2','bridge_12_sub']:ids['stairs']=preserved['sessions'][scope]['camera_nodes']['stairs']
            source_frames={view:decode_frame(con,node,record['preprocessing'],poses[node]) for view,node in ids.items()}
            con.close()
        source_images={view:[] for view in source_frames}
        for stage,title in STAGES:
            scene(overview,scope,stage)
            for view,(eye,up) in views.items():
                overview.setup_camera(55.,centre,eye,up)
                im=Image.fromarray(np.asarray(overview.render_to_image()).copy());overview_images[view].append(im)
                cameras[f'{scope}/{view}']={'type':'fixed_overview','centre':centre.tolist(),'eye':eye.tolist(),'up':up,'vertical_fov_degrees':55.,'width':900,'height':720}
                if scope=='all':save(im,out/(view+'_'+stage+'.png'),scope=scope,view=view,stage=stage)
            if source_frames:
                scene(calibrated,scope,stage)
                for view,f in source_frames.items():
                    ih,iw=f['rgb'].shape[:2];scale=min(900/iw,720/ih)
                    k=f['k_rgb'].copy();k[0]*=scale;k[1]*=scale
                    k[0,2]+=(900-iw*scale)/2;k[1,2]+=(720-ih*scale)/2
                    calibrated.setup_camera(k,inverse(f['world_optical']),900,720)
                    source_images[view].append(Image.fromarray(np.asarray(calibrated.render_to_image()).copy()))
                    cameras[f'{scope}/{view}']={'type':'calibrated_source','node':f['id'],'K':k.tolist(),
                        'world_optical':f['world_optical'].tolist(),'width':900,'height':720,
                        'viewport_policy':'Preserve native RGB pixel aspect/focal ratio; expand viewport horizontally, no anisotropic image stretching'}
        for view,images in {**overview_images,**source_images}.items():
            note=korean[scope]+' · '+view+' · 동일 카메라 / 포즈 고정 / 새 최적화 아님'
            panel=contact_sheet(images,[t for _,t in STAGES],note,font)
            save(panel,out/(view+'_progression.png'),scope=scope,view=view,stage='three_stage_comparison')
        print(json.dumps({'saved':scope,'comparison_views':len(views)+len(source_frames)}),flush=True)
    for path,h in hashes.items():
        if sha256_file(Path(path))!=h:raise ValueError('Source changed after rendering: '+path)
    report={'created_utc':utc_now(),'figures':figures,'cameras':cameras,'inputs_sha256':hashes,
        'all_original_inputs_unchanged':True,'source_map_replaced':False,'navigation_ready':False,
        'selected_map_manifest_sha256':pointer['manifest_sha256'],'display_manifest_sha256':display['manifest_sha256'],
        'stages':['Selected optimized point cloud','Full 5cm TSDF display in neutral grey','Same surface with depth-checked native RGB texture'],
        'sessions':{n:{'candidate':selected['sessions'][n]['candidate'],'points':selected['sessions'][n]['point_count'],
            'surface_triangles':preserved['sessions'][n]['topology']['triangles']} for n in names},
        'limitations':['Actual model renders, not screenshots of native RViz/Open3D window chrome.',
            'Point shader and grey surface lighting differ intentionally; cameras/world bounds are identical.',
            'Selected point cloud -> existing full 5cm TSDF derivative, not raw -> newly optimized mapping.',
            'Steps 2/3 use the same retained triangle geometry; step 3 only changes appearance.',
            'Source photograph views may supply the texture; not independent validation.',
            'Holes and existing registration errors remain; no hole filling or image generation.',
            'Initial dual-renderer attempt completed PNGs but aborted on Filament teardown; retained under outputs/pointcloud_to_surface_render_attempt1. Final uses one renderer.']}
    (a.assets/'manifest.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    overview.scene.clear_geometry()
    print(json.dumps({'complete':True,'images':len(figures),'inputs_unchanged':True}),flush=True)


if __name__=='__main__':main()
