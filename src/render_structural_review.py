#!/usr/bin/env python3
"""Fixed source/overview/planar-region comparisons, actual renders and paired depth."""
import argparse
import json
from pathlib import Path
import sqlite3

import numpy as np
from PIL import Image,ImageDraw,ImageFont

from audit_display_depth import depth_image,compare_depths
from export_rtabmap_db import load_pose_csv
from optimize_session import inverse,sha256_file,utc_now
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--before-bundle',type=Path,help='Compare against a conservative structural bundle instead of colour-only mesh')
    p.add_argument('--before-title',default=None);p.add_argument('--after-title',default=None)
    p.add_argument('--assets',type=Path,required=True);p.add_argument('--sessions',nargs='+');p.add_argument('--resume',action='store_true');a=p.parse_args()
    a.assets.mkdir(parents=True,exist_ok=True)
    if (a.assets/'manifest.json').exists() and not a.resume:raise FileExistsError(a.assets)
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());ptr=json.loads((ROOT/'config/current_map.json').read_text());mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text())
    prior=json.loads((a.assets/'manifest.json').read_text()) if a.resume else {};figures=prior.get('figures',[]);reports=prior.get('sessions',{})
    renderer=o3d.visualization.rendering.OffscreenRenderer(700,700);renderer.scene.set_background([.055,.065,.08,1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',19)
    for n in a.sessions or run['sessions']:
        if n in reports:continue
        s=selected['sessions'][n];dest=a.assets/n;dest.mkdir(exist_ok=True)
        if a.before_bundle:
            before_path=a.before_bundle/n/'clean_structural.ply'
            if not before_path.exists():before_path=a.before_bundle/n/'structural.ply'
            before=o3d.io.read_triangle_mesh(str(before_path))
        else:
            before_path=ROOT/'outputs/student_union_appearance_v2'/n/'appearance_fixed.ply';before=o3d.io.read_triangle_mesh(str(before_path))
            flag=np.load(before_path.parent/'inferred_faces.npy');before.remove_triangles_by_mask(flag)
        after_path=a.bundle/n/'clean_structural.ply'
        if not after_path.exists():after_path=a.bundle/n/'structural.ply'
        after=o3d.io.read_triangle_mesh(str(after_path))
        poses=load_pose_csv(mp.parent/s['trajectory']);con=sqlite3.connect(f'file:{selected["database_integrity"][n]["database"]}?mode=ro',uri=True)
        probes=json.loads((ROOT/'outputs/student_union_preserved_display_v1'/n/'summary.json').read_text())['camera_nodes']
        frames={label:decode_frame(con,node,s['preprocessing'],poses[node]) for label,node in probes.items()};con.close()
        centre=(np.asarray(s['bounds_min_m'])+s['bounds_max_m'])/2;extent=np.linalg.norm(np.asarray(s['bounds_max_m'])-s['bounds_min_m'])
        cameras={'overview':{'centre':centre.tolist(),'eye':(centre+np.array([.7,-.9,.8])*extent*.55).tolist(),'up':[0,0,1]}}
        walls=[p for p in run['sessions'][n]['patches'] if abs(np.asarray(p['basis'])[2,2])<.12]
        if walls:
            camera_positions=np.array([p[:3,3] for p in poses.values()])
            for wall_index,wall in enumerate(sorted(walls,key=lambda p:-p['cells'])[:4]):
                c=np.array(wall['centre']);normal=np.asarray(wall['basis'])[:,2];size=np.max(wall['shape'])*.06
                nearby=camera_positions[np.linalg.norm(camera_positions-c,axis=1)<7.]
                if len(nearby) and np.median((nearby-c)@normal)<0:normal=-normal
                cameras[f'wall_{wall_index+1}']={'centre':c.tolist(),'eye':(c+normal*np.clip(size*.35,3.,5.)+[0,0,.2]).tolist(),'up':[0,0,1]}
        for label,f in frames.items():
            if label=='audit':continue
            k=f['k_rgb'].copy();ih,iw=f['rgb'].shape[:2];scale=min(700/iw,700/ih);k[:2]*=scale;k[0,2]+=(700-iw*scale)/2;k[1,2]+=(700-ih*scale)/2
            cameras[label]={'K':k.tolist(),'world_optical':f['world_optical'].tolist(),'node':f['id']}
        images={label:[] for label in cameras}
        new_texture=a.bundle/n/'clean_textured.glb'
        if not new_texture.exists():new_texture=a.bundle/n/'structural_textured.glb'
        before_texture=a.before_bundle/n/'clean_textured.glb' if a.before_bundle else ROOT/'outputs/student_union_appearance_v2'/n/'balanced_only.glb'
        if a.before_bundle and not before_texture.exists():before_texture=a.before_bundle/n/'structural_textured.glb'
        methods=[(a.before_title or ('이전 · 보수적 재구성' if a.before_bundle else '이전 · 색만 보정'),before_texture),
            (a.after_title or ('강화 구조면 후보' if a.before_bundle else '새 구조 패치 + 연속 텍스처'),new_texture),('새 구조 · 회색 기하',None)]
        for title,path in methods:
            renderer.scene.clear_geometry()
            if path:
                model=o3d.io.read_triangle_model(str(path))
                for material in model.materials:material.shader='defaultUnlit'
                renderer.scene.add_model('map',model)
            else:
                mesh=o3d.geometry.TriangleMesh(after);mesh.compute_vertex_normals();mesh.vertex_colors=o3d.utility.Vector3dVector(np.full((len(mesh.vertices),3),.6))
                mat=o3d.visualization.rendering.MaterialRecord();mat.shader='defaultLit';mat.sRGB_color=True
                renderer.scene.add_geometry('map',mesh,mat);renderer.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS,[.4,-.5,-1.])
            for label,c in cameras.items():
                if 'K' in c:renderer.setup_camera(np.asarray(c['K']),inverse(np.asarray(c['world_optical'])),700,700)
                else:renderer.setup_camera(55.,c['centre'],c['eye'],c['up'])
                images[label].append((title,np.asarray(renderer.render_to_image()).copy()))
        checks={}
        for label,f in frames.items():checks[label]={'node':f['id'],**compare_depths(depth_image(o3d,before,f),depth_image(o3d,after,f),f['depth'])}
        for label,items in images.items():
            panel=Image.new('RGB',(2100,785),'white');draw=ImageDraw.Draw(panel)
            draw.text((10,10),f'{n} / {label} · 같은 카메라 · 표시용 구조 재구성 / SLAM 포즈 불변 / 실측·길찾기 승인 아님',font=font,fill='#172538')
            for i,(title,pixels) in enumerate(items):draw.text((700*i+10,47),title,font=font,fill='#172538');panel.paste(Image.fromarray(pixels),(700*i,85))
            path=dest/f'{label}.png';panel.save(path);figures.append({'file':str(path.relative_to(a.assets)),'sha256':sha256_file(path),'camera':cameras[label]})
        reports[n]={'paired_depth':checks,'replaced_faces':run['sessions'][n]['replaced_original_faces'],'patches':len(run['sessions'][n]['patches'])}
        (a.assets/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':reports,'figures':figures,'renderer_sha256':sha256_file(Path(__file__)),
            'before_bundle':str(a.before_bundle) if a.before_bundle else 'colour_only',
            'selected_map_unchanged':True,'modeled_display_not_navigation':True,'structural_manifest_sha256':sha256_file(a.bundle/'manifest.json')},indent=2)+'\n')
        print(json.dumps({'rendered':n,'figures':len(images),'source_depth_before_after':[checks['source']['same_pixels_before']['median_m'],checks['source']['same_pixels_after']['median_m']]}),flush=True)


if __name__=='__main__':main()
