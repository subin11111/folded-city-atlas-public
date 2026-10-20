#!/usr/bin/env python3
"""Fixed-camera repair evidence, paired depth audit and sampled boundary topology."""
import argparse
import json
from pathlib import Path
import sqlite3

import cv2
import numpy as np
from PIL import Image,ImageDraw,ImageFont

from audit_display_depth import depth_image,compare_depths
from export_rtabmap_db import load_pose_csv
from optimize_session import inverse,sha256_file,utc_now
from surface_model_utils import decode_frame

ROOT=Path(__file__).resolve().parents[1]


def boundary_edges(mesh):
    t=np.asarray(mesh.triangles);edges=np.concatenate([t[:,[0,1]],t[:,[1,2]],t[:,[2,0]]])
    edges.sort(1);_,counts=np.unique(edges,axis=0,return_counts=True)
    return int(np.sum(counts==1))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path,default=ROOT/'outputs/student_union_appearance_v1')
    p.add_argument('--assets',type=Path,required=True);p.add_argument('--sessions',nargs='+');p.add_argument('--resume',action='store_true');a=p.parse_args()
    a.assets.mkdir(parents=True,exist_ok=True)
    if (a.assets/'manifest.json').exists() and not a.resume:raise FileExistsError(a.assets/'manifest.json')
    import open3d as o3d
    run=json.loads((a.bundle/'manifest.json').read_text());ptr=json.loads((ROOT/'config/current_map.json').read_text())
    mp=ROOT/ptr['manifest'];selected=json.loads(mp.read_text());preserved=ROOT/'outputs/student_union_preserved_display_v1'
    prior=json.loads((a.assets/'manifest.json').read_text()) if a.resume else {}
    records=prior.get('sessions',{});figures=prior.get('figures',[])
    renderer=o3d.visualization.rendering.OffscreenRenderer(540,720)
    renderer.scene.set_background([.06,.075,.095,1.])
    renderer.scene.view.set_color_grading(o3d.visualization.rendering.ColorGrading(
        o3d.visualization.rendering.ColorGrading.Quality.ULTRA,o3d.visualization.rendering.ColorGrading.ToneMapping.LINEAR))
    font=ImageFont.truetype('/usr/share/fonts/truetype/nanum/NanumGothic.ttf',18)
    for n in a.sessions or run['sessions']:
        if n in records:continue
        dest=a.assets/n;dest.mkdir(exist_ok=True);s=selected['sessions'][n]
        ps=json.loads((preserved/n/'summary.json').read_text());poses=load_pose_csv(mp.parent/s['trajectory'])
        con=sqlite3.connect(f'file:{selected["database_integrity"][n]["database"]}?mode=ro',uri=True)
        frames={label:decode_frame(con,node,s['preprocessing'],poses[node]) for label,node in ps['camera_nodes'].items() if label!='audit'};con.close()
        before=o3d.io.read_triangle_mesh(str(preserved/n/'preserved_geometry.ply'))
        after=o3d.io.read_triangle_mesh(str(a.bundle/n/'appearance_fixed.ply'))
        t0=np.asarray(before.triangles);t1=np.asarray(after.triangles)
        records[n]={'original_vertices_exactly_unchanged':bool(np.array_equal(np.asarray(before.vertices),np.asarray(after.vertices))),
            'original_faces_exactly_unchanged':bool(np.array_equal(t0,t1[:len(t0)])),
            'before_boundary_edges':boundary_edges(before),'after_boundary_edges':boundary_edges(after),'cameras':{}}
        pictures={label:[('실제 원본 사진',cv2.resize(f['rgb'],(540,720)))] for label,f in frames.items()}
        methods=[('이전 · 사진 조각 텍스처',preserved/n/'preserved_textured.glb'),
            ('보수 · 노출 보정 연속 RGB',a.bundle/n/'appearance_fixed.glb'),
            ('분홍 = 추정 보수면',a.bundle/n/'inferred_marked.glb')]
        for title,path in methods:
            renderer.scene.clear_geometry();model=o3d.io.read_triangle_model(str(path))
            for material in model.materials:material.shader='defaultUnlit'
            renderer.scene.add_model('map',model)
            for label,f in frames.items():
                k=f['k_rgb'].copy();k[0]*=540/f['rgb'].shape[1];k[1]*=720/f['rgb'].shape[0]
                renderer.setup_camera(k,inverse(f['world_optical']),540,720)
                pictures[label].append((title,np.asarray(renderer.render_to_image()).copy()))
        for label,f in frames.items():
            z0=depth_image(o3d,before,f);z1=depth_image(o3d,after,f)
            records[n]['cameras'][label]={'node':f['id'],**compare_depths(z0,z1,f['depth'])}
            panel=Image.new('RGB',(2160,800),'white');draw=ImageDraw.Draw(panel)
            draw.text((10,8),f'{n} #{f["id"]} · 같은 카메라 · 보수는 표시 전용 / 미촬영 영역과 중복 벽은 남을 수 있음',font=font,fill='#172538')
            for i,(title,pixels) in enumerate(pictures[label]):
                draw.text((i*540+10,42),title,font=font,fill='#172538');panel.paste(Image.fromarray(pixels),(i*540,80))
            path=dest/f'{label}_comparison.png';panel.save(path)
            figures.append({'file':str(path.relative_to(a.assets)),'sha256':sha256_file(path),'session':n,'node':f['id']})
        print(json.dumps({'reviewed':n,'boundaries':[records[n]['before_boundary_edges'],records[n]['after_boundary_edges']]}),flush=True)
        (a.assets/'manifest.json').write_text(json.dumps({'created_utc':utc_now(),'sessions':records,'figures':figures,
            'selected_manifest_sha256':ptr['manifest_sha256'],'repair_manifest_sha256':sha256_file(a.bundle/'manifest.json'),
            'renderer_sha256':sha256_file(Path(__file__)),'scope':'Fixed calibrated views, not an exact reproduction of the user screenshot or a desktop screenshot.',
            'selected_map_changed':False},indent=2)+'\n')


if __name__=='__main__':main()
